#!/usr/bin/env python3
"""Thin, predictable wrapper around the `tea` Gitea CLI for issue tracking.

Why: tea's raw interface trips up agents -- trailing-dash stdin for
descriptions, output format varying by subcommand, index as string in
JSON vs int as input, `show` hidden behind a bare positional.

Every subcommand prints JSON to stdout (errors are an {"error": ...}
object on stdout with non-zero exit, so output can be piped freely).
Bodies are given with --body or piped on stdin. tea receives them via
--description-file - (quoting never matters) except comments, which
only accept a flag -- so a comment rides argv: visible in ps, and the kernel
will not exec an argument that reaches MAX_ARG_STRLEN bytes. Gitea accepts a
comment that large (measured: 131071 bytes landed on #465, and 131072 is the
first size that will not exec), so that kernel cap is the real limit here and
the guard below it is not hiding a stricter server-side one.

tea's own output is inconsistent: `issue create` answers JSON but leaves the
labels it just set out of the answer, while `issue edit`, `issue close/reopen`
and `comments add` ignore -o entirely and print a rendered card. So every write
here proves itself by re-reading: new answers {"ok":..., "confirmed":...,
"issue":{...}}, edit/close/open answer {"ok":..., "indexes":[...],
"confirmed":..., "issues":[{index,state,title,labels,url,closed_at,
body_chars}, ...]}, comment answers the {author,created,chars} of the comment it
found again -- or adds "predates_attempt" when the only match is older than the
send, because that is somebody else's identical text and not proof.
confirmed:false means no proof came back: --no-confirm skips the re-read on
purpose, and any other reason is written to stderr. Past CONFIRM_MAX indexes the
proof is one walk of the issue list rather than one re-read per index, because a
bulk sweep is exactly when a silent typo costs the most and "no proof" is the
worst answer.

Every tea call gets TEA_TIMEOUT seconds. A timeout is never "nothing happened":
it says the request may have reached the server, because retrying a write that
did land is how duplicate comments get made.

"labels" is a JSON list of names in every command's output. It has to be: tea's
ls renders them as one space-joined string, and real label names contain spaces
("Status/Need More Info"), so that string cannot be split by whitespace -- the
wrapper splits it against the repo's own label list instead (tea labels ls, all
of its pages: tea's own page is 30 rows). Where the truth is not known the field
is null, not an empty list.

Bodies and comments are MARKDOWN -- the web UI renders them, and headings,
bullets and ``` fences are what make a ticket skimmable. Neither path escapes
anything, so structure survives either way; --body in a shell one-liner just
makes prose paragraphs the easy thing to type. For anything longer than a
sentence, write the body as a markdown file and pipe it:

  utils/tea.py new --title "..." < tmp/issue.md      # or a <<'MD' heredoc

Usage:
  utils/tea.py new --title "..." [--body "..." | < body.md] [--labels a,b]
                       [--milestone m] [--no-confirm]
  utils/tea.py ls [--state all|open|closed] [--keyword str] [--limit N]
                       [--all | --page N]
                       (--all walks every page; a bare page is one page, and the
                        server caps a page at 50 rows however big --limit is)
  utils/tea.py show INDEX [--comments]
                       (--comments off answers "comments": null, because tea's
                        own --comments=false answers [] like a quiet issue)
  utils/tea.py comment INDEX [--body "..." | < body.md] [--no-confirm]
  utils/tea.py close INDEX [INDEX...] [--no-confirm]
                       (a repeated INDEX is written once and re-read once)
  utils/tea.py open INDEX [INDEX...] [--no-confirm]
  utils/tea.py edit INDEX [INDEX...] [--title ...] [--body ...]
                          [--close | --open]
                          [--labels a,b] [--remove-labels a,b] [--no-confirm]
                          (an empty pipe is not a body; a blank --body needs
                           --allow-empty-body; several indexes are for labels
                           and state only -- --title/--body takes one)
"""

import argparse
import json
import os
import subprocess
import sys
import time
import traceback

TEA = "tea"

# tea's ls JSON omits url/body/created unless asked; index comes back as str.
LS_FIELDS = "index,state,title,url,author,labels,created,updated"
# Gitea clamps one page to this many rows whatever --limit asks for, so
# `--limit 500` is still a 50-row page and not "everything".
SERVER_PAGE_CAP = 50
LS_MAX_PAGES = 100      # --all guard: 100 pages x cap is far past this repo
BODY_MAX_CHARS = 1 << 20  # past this it is not an issue description (/dev/zero, ...)
# The kernel's per-argument exec limit (32 pages). Comments have no file/stdin
# path in tea, so they hit it.
MAX_ARG_STRLEN = 131072
# After a write, re-read this many indexes one at a time; past that one walk of
# the issue list is both cheaper and more honest than answering with no proof.
CONFIRM_MAX = 10
# tea talks to a network. Without this a stalled connection hangs the caller
# forever, which is worse than a failure -- and on a write it must be reported
# as "unknown whether it landed", not as "nothing happened".
TEA_TIMEOUT = 120
# tea labels ls defaults to a 30-row page, which would silently truncate the
# label vocabulary (the thing the ls label string is split against) as soon as
# the repo has 31 labels.
LABEL_PAGE = 50
LABEL_MAX_PAGES = 20


def run_tea(args, stdin_text=None, allow_fail=False):
    # Never inherit our stdin: an unexpected tea prompt would hang the caller.
    # subprocess.run forbids stdin+input together, so feed via input= only.
    argv = [TEA, *args]
    try:
        if stdin_text is None:
            proc = subprocess.run(argv, capture_output=True, text=True,
                                  stdin=subprocess.DEVNULL, timeout=TEA_TIMEOUT)
        else:
            proc = subprocess.run(argv, capture_output=True, text=True,
                                  input=stdin_text, timeout=TEA_TIMEOUT)
    except subprocess.TimeoutExpired:
        # subprocess.run kills the child, but the request may already have
        # reached Gitea, so this is "unknown", never "nothing happened".
        msg = (f"tea {' '.join(args)} gave no answer within {TEA_TIMEOUT}s -- "
               f"the request may have reached the server anyway, so check "
               f"before retrying it")
        if allow_fail:
            print(f"note: {msg}", file=sys.stderr)
            return None
        fail(msg)
    except OSError as e:
        # execve itself can refuse the call (E2BIG when an argument goes over
        # the kernel's per-argument limit); an uncaught OSError here would print
        # a traceback and break the JSON-error contract every caller relies on.
        fail(f"could not run tea: {e}")
    if proc.returncode != 0:
        if allow_fail:
            # The caller carries on without the data, so a null in the JSON must
            # not be a mystery: the reason goes to stderr, stdout stays JSON.
            print(f"note: tea {' '.join(args)} failed: "
                  f"{proc.stderr.strip() or 'exit ' + str(proc.returncode)}",
                  file=sys.stderr)
            return None
        fail(proc.stderr.strip() or f"tea {' '.join(args)} failed "
             f"(exit {proc.returncode})")
    return proc.stdout


def parse_json(out):
    try:
        return json.loads(out)
    except json.JSONDecodeError:
        fail(f"could not parse tea output as JSON:\n{out}")


def fail(msg):
    print(json.dumps({"error": msg}))
    sys.exit(1)


def check_index(idx):
    # isdigit() alone accepts unicode digits like "١٢" -> 12.
    if not (idx.isascii() and idx.isdigit() and int(idx) > 0):
        fail(f"issue index must be a positive integer, got {idx!r}")
    return int(idx)


def check_limit(n):
    n = int(n)
    if n < 1:
        raise argparse.ArgumentTypeError("--limit must be >= 1")
    return n


def check_page(n):
    n = int(n)
    if n < 1:
        raise argparse.ArgumentTypeError("--page must be >= 1")
    return n


def check_body(text, what):
    # A system-boundary check, and an early one: a NUL byte makes subprocess
    # raise ValueError and a piped binary file (sys.stdin decodes with
    # surrogateescape, so it arrives as lone surrogates) makes it raise
    # UnicodeEncodeError -- both from inside run_tea, which for new/edit is
    # after tea has already been spawned. Better a plain JSON error naming the
    # file than a traceback and an empty stdout next to a write that ran.
    if "\0" in text:
        fail(f"{what} contains a NUL byte -- tea cannot send that")
    try:
        text.encode()
    except UnicodeEncodeError as e:
        fail(f"{what} is not utf-8 text ({e}) -- it looks like a binary file "
             f"was piped in")


def read_pipe():
    # Bounded: an unbounded read() over /dev/zero OOMs before we can complain.
    data = sys.stdin.read(BODY_MAX_CHARS + 1)
    if len(data) > BODY_MAX_CHARS:
        fail(f"body is over {BODY_MAX_CHARS} chars -- that is not an issue description")
    return data


def get_body(args):
    # Unlike edit, new/comment are pointless without a body, so a blank one is
    # an error rather than a thing the caller might have meant.
    if args.body is not None and not sys.stdin.isatty():
        piped = read_pipe()
        if piped.strip():
            fail("body given twice: pass --body or pipe markdown, not both")
        body = args.body
    elif args.body is not None:
        body = args.body
    elif sys.stdin.isatty():
        fail("no body: pass --body or pipe markdown on stdin")
    else:
        body = read_pipe()
    if not body.strip():
        fail("body is empty")
    check_body(body, "body")
    return body


_label_vocab = None  # [(name, its words)], longest name first, once per run


def label_vocab():
    """Every label the repo knows, longest first, each pre-split into the words
    it contains. tea renders an issue's labels as one space-joined string and
    real names hold spaces ("Status/Need More Info"), so the vocabulary is the
    only thing that can split that string back into names. Walking pages matters
    because tea's own default page is 30 rows: a 31st label would otherwise go
    missing with no hint, and a missing multi-word label shreds every row that
    carries it."""
    global _label_vocab
    if _label_vocab is None:
        names, junk, page, explained = [], 0, 1, False
        while page <= LABEL_MAX_PAGES:
            out = run_tea(["labels", "ls", "-o", "json",
                           "--limit", str(LABEL_PAGE), "--page", str(page)],
                          allow_fail=True)
            if out is None:
                explained = True          # run_tea already said why
                break
            try:
                data = json.loads(out)
            except json.JSONDecodeError:
                data = None
            if not isinstance(data, list):
                print("note: tea labels ls did not answer a list, so labels are "
                      "split on spaces and a name containing a space will look "
                      "like two", file=sys.stderr)
                explained = True
                break
            for entry in data:
                name = entry.get("name") if isinstance(entry, dict) else entry
                if isinstance(name, str) and name.strip():
                    names.append(name)
                else:
                    junk += 1
            if len(data) < LABEL_PAGE:
                break
            page += 1
        if junk:
            print(f"note: tea labels ls answered {junk} unreadable label(s) -- a "
                  f"multi-word one among them will not be recognized",
                  file=sys.stderr)
        if page > LABEL_MAX_PAGES:
            print(f"note: stopped reading the label list after "
                  f"{LABEL_MAX_PAGES} pages -- a multi-word label past that gets "
                  f"split wrong", file=sys.stderr)
        if not names and not explained:
            print("note: no label vocabulary, so labels are split on spaces and "
                  "a name containing a space will look like two", file=sys.stderr)
        _label_vocab = sorted(((n, n.split()) for n in names),
                              key=lambda entry: len(entry[1]), reverse=True)
    return _label_vocab


def split_labels(text, index=None):
    """Greedy longest match over the vocabulary, so "Status/Need More Info
    Priority/High" gives two names and not five. The joined form is genuinely
    ambiguous if one label name is a word-run inside another ("A B C" being
    "A B"+"C" or "A"+"B C") -- no label here does that, and one that would is a
    naming decision, not something a matcher can recover. What the vocabulary
    cannot explain is reported instead of quietly invented."""
    vocab = label_vocab()
    words = text.split()
    out, i = [], 0
    while i < len(words):
        for name, parts in vocab:
            if words[i:i + len(parts)] == parts:
                out.append(name)
                i += len(parts)
                break
        else:
            out.append(words[i])
            i += 1
    known = {name for name, _ in vocab}
    unknown = [n for n in out if n not in known]
    if vocab and unknown:
        tag = f"issue {index}: " if index is not None else ""
        print(f"note: {tag}labels not on the repo's own label list: "
              f"{', '.join(unknown)} -- the vocabulary is stale, or tea joined "
              f"something it should not have", file=sys.stderr)
    return out


def labels_of(d, index=None):
    """tea's two label shapes (one space-joined string from ls, a list of objects
    from show/create) as one list of names. Notes on stderr for what it cannot
    read and never fails, because callers reach here after a write too -- a hard
    exit there would hide the fact that the write landed."""
    labels = d.get("labels")
    if isinstance(labels, list):
        names = []
        for entry in labels:
            name = entry.get("name") if isinstance(entry, dict) else entry
            if isinstance(name, str) and name.strip():
                names.append(name)
            else:
                # Never die over one junk entry: ls/show would then answer
                # nothing at all about an issue that is right there.
                print(f"note: ignoring an unreadable label entry {entry!r}",
                      file=sys.stderr)
        return names
    if isinstance(labels, str) and labels.strip():
        return split_labels(labels, index)
    return []


def norm_issue(d):
    # ls and single-issue JSON have different shapes; unify the common keys. The
    # payload can be anything tea answers (a bare int, a list), and this runs
    # after writes too, so a wrong shape is a JSON error rather than a TypeError
    # next to a write that already landed.
    if not isinstance(d, dict):
        fail(f"unexpected tea payload, not an object: {str(d)[:120]}")
    if "index" not in d:
        fail(f"unexpected tea payload, no index: {str(d)[:120]}")
    try:
        d["index"] = int(d["index"])
    except (TypeError, ValueError):
        fail(f"unexpected tea payload, index is not a number: {d['index']!r}")
    if "id" in d:
        del d["id"]  # db id, not the issue index
    if "user" in d:
        d["author"] = d.pop("user")
    # A joined string cannot be split by its caller, and a diff between the two
    # shapes shows differences that are not there, so callers get names.
    d["labels"] = labels_of(d, d["index"])
    return d


def confirm_issue(index):
    """What the server holds after a write, in the fields that show the write
    landed. `tea issue edit` and `tea issue close` ignore -o and print a
    rendered card, so there is no payload to echo and re-reading is the only
    proof. None means the re-read failed or came back unreadable -- and the
    write already ran by then, so this must never take the process down and never
    claim proof it does not have; it says why on stderr instead. It deliberately
    does not go through norm_issue, whose errors end the process.
    """
    out = run_tea(["issue", str(index), "-o", "json", "--comments=false"],
                  allow_fail=True)
    if out is None:
        return None                     # run_tea already said why
    try:
        d = json.loads(out)
        if not isinstance(d, dict) or "index" not in d:
            raise ValueError(f"no index in {out[:60]!r}")
        # state and title are in every real payload; a re-read that answers with
        # an index and nothing else proves nothing and must not say otherwise.
        if "state" not in d or "title" not in d:
            raise ValueError(f"payload has no state/title: {out[:60]!r}")
        seen = int(d["index"])
        if seen != index:
            raise ValueError(f"answered index {seen} for a re-read of {index}")
        return {"index": seen, "state": d.get("state"),
                "title": d.get("title"), "labels": labels_of(d, seen),
                "url": d.get("url"), "closed_at": d.get("closedAt"),
                "body_chars": len(d.get("body") or "")}
    except (ValueError, TypeError, KeyError) as e:
        print(f"note: re-read of issue {index} came back unreadable ({e}) -- "
              f"the write itself already ran", file=sys.stderr)
        return None


def split_requested(text):
    """A --labels/--remove-labels value as a set of names (tea's own format is
    comma-separated, so a label name with a comma in it cannot be set this way
    even though it can be read)."""
    return {t.strip() for t in (text or "").split(",") if t.strip()}


def note_unapplied_labels(issue, want, gone, batch):
    """tea ignores a label name it does not know -- a typo'd --labels exits 0 and
    changes nothing. The re-read already knows the truth, so diff it out loud.
    Takes the wanted/dropped sets rather than the parsed args: not every
    subcommand has both flags, and reading a missing attribute here would crash
    after the write already landed."""
    have = set(issue["labels"])
    missing = sorted(want - have)
    leftover = sorted(gone & have)
    tag = f"issue {issue['index']}: " if batch else ""
    if missing:
        print(f"note: {tag}not on the issue after the write: {', '.join(missing)}"
              f" -- tea silently ignores a label name it does not know (see "
              f"`tea labels ls` for the real spellings)", file=sys.stderr)
    if leftover:
        print(f"note: {tag}still on the issue after --remove-labels: "
              f"{', '.join(leftover)}", file=sys.stderr)


def confirm_comment(index, body, sent_at):
    """`tea comments add` ignores -o and prints a card, so the comment has to be
    found again to be believed. Compared with all runs of whitespace collapsed,
    which covers Gitea trimming the edges and any CRLF that came in on the pipe;
    the last match wins, as our comment is the newest of an equal pair. A match
    older than the moment we sent it belongs to somebody else who typed the same
    thing, and saying so is cheap -- re-posting a comment that looked like it
    failed is exactly how that happens. None plus a stderr note means the write
    already ran and only the proof is missing."""
    out = run_tea(["issue", str(index), "-o", "json", "--comments"],
                  allow_fail=True)
    if out is None:
        return None                     # run_tea already said why
    try:
        d = json.loads(out)
        comments = d.get("comments") if isinstance(d, dict) else None
        if not isinstance(comments, list):
            raise ValueError(f"no comments in {out[:60]!r}")
        want = " ".join(body.split())
        found = None
        for c in comments:
            if (isinstance(c, dict) and isinstance(c.get("body"), str)
                    and " ".join(c["body"].split()) == want):
                found = c
        if found is None:
            print(f"note: issue {index} has no comment matching the text that "
                  f"was posted -- it may still have landed, check "
                  f"`utils/tea.py show {index} --comments`", file=sys.stderr)
            return None
        created = found.get("created")
        if _predates(created, sent_at):
            print(f"note: the matching comment on issue {index} is dated "
                  f"{created!r}, before this attempt -- it may be an older "
                  f"comment with the same text, so it is not counted as proof; "
                  f"`show {index} --comments` will settle it", file=sys.stderr)
            return {"author": found.get("author"), "created": created,
                    "chars": len(found["body"]), "predates_attempt": True}
        return {"author": found.get("author"), "created": created,
                "chars": len(found["body"])}
    except (ValueError, TypeError, KeyError) as e:
        print(f"note: re-read of issue {index}'s comments came back unreadable "
              f"({e}) -- the comment itself already ran", file=sys.stderr)
        return None


def _predates(created, sent_at):
    """True when Gitea dated the match clearly before we sent it. Both sides are
    "%Y-%m-%dT%H:%M:%SZ", so comparing the strings compares the instants; 120s
    of slack because this clock and the server's clock are not one clock."""
    return (isinstance(created, str)
            and created < time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                        time.gmtime(sent_at - 120)))


def confirm_many(indexes):
    """Proof for a write that touched several indexes. Few of them: re-read each
    one. Past CONFIRM_MAX: walk the issue list once instead, which costs fewer
    calls than one re-read per index and beats the alternative of answering with
    no proof at all -- which is what a bulk sweep most needs, since that is when
    a silent typo costs the most. Returns (issues, confirmed); an index that is
    simply not in the list gets named, not dropped."""
    if len(indexes) <= CONFIRM_MAX:
        issues = [confirm_issue(i) for i in indexes]
        return issues, all(i is not None for i in issues)
    want = set(indexes)
    found, page = {}, 1
    while True:
        rows = ls_rows("all", None, SERVER_PAGE_CAP, page)
        for row in rows:
            if row["index"] in want:
                # The list carries no closedAt and no body, so those two say
                # "not asked" here rather than looking like "absent".
                found[row["index"]] = {"index": row["index"],
                                       "state": row.get("state"),
                                       "title": row.get("title"),
                                       "labels": row["labels"],
                                       "url": row.get("url"),
                                       "closed_at": None, "body_chars": None}
        if len(rows) < SERVER_PAGE_CAP:
            break
        if page >= LS_MAX_PAGES:
            print(f"note: stopped walking the issue list after {LS_MAX_PAGES} "
                  f"pages -- some of the {len(want)} indexes may be further on",
                  file=sys.stderr)
            break
        page += 1
    missing = sorted(want - set(found))
    if missing:
        print(f"note: not in the issue list after the write: "
              f"{', '.join(str(i) for i in missing)} -- check with ls (a pull "
              f"request shares its number with an issue)", file=sys.stderr)
    return [found[i] for i in indexes if i in found], not missing


def cmd_new(args):
    body = get_body(args)
    tea_args = ["issue", "create", "-o", "json", "--title", args.title,
                "--description-file", "-"]
    if args.labels:
        tea_args += ["--labels", args.labels]
    if args.milestone:
        tea_args += ["--milestone", args.milestone]
    out = run_tea(tea_args, stdin_text=body)
    d = norm_issue(parse_json(out))
    # tea's create payload does not fill in the labels it just set: #466 was
    # created with two and answered "labels": []. So the labels in this answer
    # come from a re-read, like every other write here -- which is also what
    # catches a typo'd --labels on a brand new issue. Without the re-read the
    # field is null, because [] is exactly the lie the re-read exists to avoid.
    issue = None if args.no_confirm else confirm_issue(d["index"])
    if issue is not None:
        d["labels"] = issue["labels"]
        note_unapplied_labels(issue, split_requested(args.labels), set(), False)
    else:
        d["labels"] = None
    print(json.dumps({"ok": True, "index": d["index"],
                      "confirmed": issue is not None, "issue": d}))


def ls_rows(state, keyword, limit, page):
    tea_args = ["issue", "ls", "-o", "json", "--fields", LS_FIELDS,
                "--state", state, "--limit", str(limit), "--page", str(page)]
    if keyword:
        tea_args += ["--keyword", keyword]
    data = parse_json(run_tea(tea_args))
    # Iterating a dict or a bare {} yields nothing, so a payload that is not a
    # list would answer "no issues" -- the complete-looking-empty answer this
    # wrapper exists to avoid.
    if not isinstance(data, list):
        fail(f"tea issue ls answered {type(data).__name__}, not a list: "
             f"{str(data)[:120]}")
    return [norm_issue(d) for d in data]


def cmd_ls(args):
    # A page is one page: callers that forget --page keep getting the newest
    # slice and see a complete-looking list (that cost us 11 issues once), so
    # --all walks the pages and a full single page says so on stderr. The server
    # clamps a page to SERVER_PAGE_CAP, so a big --limit is not "everything".
    if args.all and args.page is not None:
        fail("--all and --page are mutually exclusive")
    step = min(args.limit, SERVER_PAGE_CAP)
    if args.limit > step:
        print(f"note: the server caps a page at {SERVER_PAGE_CAP} rows, so "
              f"--limit {args.limit} fetches {step} -- pass --all for the "
              f"whole list", file=sys.stderr)
    if not args.all:
        page = args.page or 1
        rows = ls_rows(args.state, args.keyword, args.limit, page)
        if len(rows) >= step:
            print(f"note: page full at {len(rows)} items -- pass --all for the "
                  f"whole list (or --page {page + 1})", file=sys.stderr)
        print(json.dumps(rows))
        return
    out, page = [], 1
    while True:
        rows = ls_rows(args.state, args.keyword, args.limit, page)
        out += rows
        if len(rows) < step:
            break
        if page >= LS_MAX_PAGES:
            print(f"note: stopped after {LS_MAX_PAGES} pages ({len(out)} items) "
                  f"-- raise LS_MAX_PAGES if there really are more",
                  file=sys.stderr)
            break
        page += 1
    print(json.dumps(out))


def cmd_show(args):
    # tea ignores --fields here and returns the full object; norm_issue unifies.
    tea_args = ["issue", str(args.index), "-o", "json"]
    # tea prompts interactively about comments unless told either way -- and
    # with --comments=false it still answers comments: [], exactly what an issue
    # with no discussion answers, so "not asked" has to be said explicitly.
    tea_args.append("--comments" if args.comments else "--comments=false")
    d = norm_issue(parse_json(run_tea(tea_args)))
    if not args.comments:
        d["comments"] = None
    print(json.dumps(d))


def cmd_comment(args):
    # tea has no --description-file for comments: the body rides argv, which is
    # both visible in ps and capped by the kernel at MAX_ARG_STRLEN per argument.
    # Say that in words rather than letting execve answer "argument list too long".
    body = get_body(args)
    if len(body.encode()) >= MAX_ARG_STRLEN:
        fail(f"comment is {len(body)} chars ({len(body.encode())} bytes); tea "
             f"passes a comment as one argv string and the kernel will not exec "
             f"an argument of {MAX_ARG_STRLEN} bytes (largest that works is "
             f"{MAX_ARG_STRLEN - 1}) -- split it into two comments, piped: "
             f"utils/tea.py comment {args.index} < part.md")
    # The send time is taken before the call so the proof can tell our comment
    # from an older one with the same text.
    sent_at = time.time()
    run_tea(["comments", "add", str(args.index), "--description", body,
             "-o", "json"])
    # No payload comes back from that call, so the proof is finding the comment
    # again -- one extra read, which is why --no-confirm exists here too.
    comment = None if args.no_confirm else confirm_comment(args.index, body,
                                                           sent_at)
    # A match that predates the attempt is reported but not believed, so the
    # caller still sees what is on the issue without being told it is ours.
    print(json.dumps({"ok": True, "index": args.index,
                      "confirmed": (comment is not None
                                    and not comment.get("predates_attempt")),
                      "comment": comment}))


def cmd_state(cmd, args):
    # dict.fromkeys keeps the order and drops a repeated index, so the tea call
    # and every re-read happen once per issue.
    indexes = list(dict.fromkeys(args.indexes))
    run_tea(["issue", cmd, *[str(i) for i in indexes], "-o", "json"])
    # tea renders a card instead of a payload, so confirming means re-reading:
    # one re-read per index while there are few, one walk of the list past
    # CONFIRM_MAX. --no-confirm still means "I know, skip it".
    issues, confirmed = None, False
    if not args.no_confirm:
        issues, confirmed = confirm_many(indexes)
    print(json.dumps({"ok": True, "indexes": indexes,
                      "confirmed": confirmed, "issues": issues}))


def cmd_edit(args):
    # Piped stdin is a body even without --body, same as new/comment -- but an
    # EMPTY pipe means "no body given", not "clear the description". Devnull and
    # empty pipes are what a caller uses when it only wants labels; taking them
    # as a blank body once wiped the descriptions of 49 issues. A blank --body is
    # the same write wearing a different hat (usually a "$(cat typoed.path)" that
    # expanded to nothing), so it needs its own opt-in.
    body = args.body
    if not sys.stdin.isatty():
        piped = read_pipe()
        if piped.strip():
            if body is not None:
                fail("body given twice: pass --body or pipe markdown, not both")
            body = piped
            # A description is the one thing here that a stray pipe rewrites, so
            # name it: a labels edit should not carry a description silently.
            print("note: sending piped stdin as the new description",
                  file=sys.stderr)
    if body is not None:
        check_body(body, "body")
    if body is not None and not body.strip() and not args.allow_empty_body:
        fail("blank --body would clear the description -- pass "
             "--allow-empty-body if that is really what you mean")
    if args.close and args.is_open:
        fail("--close and --open contradict each other")
    if not any([args.title is not None, body is not None, args.close,
                args.is_open, args.labels, args.remove_labels]):
        fail("nothing to change: pass --title/--body/--close/--open/--labels")
    # tea issue edit takes several indexes, which is what a labels sweep over a
    # pile of issues wants (one call instead of one per issue). Title and body
    # are the exception: the same text landing on every listed issue is almost
    # always a slip, and the damage is not obvious later.
    indexes = list(dict.fromkeys(args.indexes))
    if len(indexes) > 1 and (args.title is not None or body is not None):
        fail("--title/--body take one index -- several indexes are for "
             "--labels/--remove-labels/--close/--open (piped stdin counts as a "
             "--body)")
    if args.title is not None and not args.title.strip():
        print("note: --title \"\" asks tea to unset the title, which Gitea may "
              "not accept", file=sys.stderr)
    want = split_requested(args.labels)
    gone = split_requested(args.remove_labels)
    edited = any([args.title is not None, body is not None, args.labels,
                  args.remove_labels])
    if edited:
        tea_args = ["issue", "edit", *[str(i) for i in indexes], "-o", "json"]
        if args.title is not None:
            tea_args += ["--title", args.title]
        if body is not None:
            tea_args += ["--description-file", "-"]
        if args.labels:
            tea_args += ["--add-labels", args.labels]
        if args.remove_labels:
            tea_args += ["--remove-labels", args.remove_labels]
        if run_tea(tea_args, stdin_text=body, allow_fail=True) is None:
            # One typo'd index among many makes tea exit non-zero while still
            # applying the good ones, so "the edit failed" alone would read as
            # though nothing changed -- and the fix for that is not to re-run the
            # whole sweep. Ask what actually carries the change and say it.
            if len(indexes) > 1:
                applied, _ = confirm_many(indexes)
                landed = [i["index"] for i in applied or [] if i
                          and want <= set(i["labels"])
                          and not gone & set(i["labels"])]
                print(f"note: the change is on: "
                      f"{', '.join(str(i) for i in landed) or 'nothing confirmed'}"
                      f" -- tea applies the indexes it can and still exits "
                      f"non-zero, so re-run only the rest", file=sys.stderr)
            fail(f"the edit did not go through for all of "
                 f"{', '.join(str(i) for i in indexes)}")
    # tea issue edit has no state flags, so --close/--open are a second call
    # after the first one already went out. If it fails, say what is already
    # applied instead of an error that reads as though nothing happened.
    if args.close or args.is_open:
        verb = "close" if args.close else "reopen"
        if run_tea(["issue", verb, *[str(i) for i in indexes], "-o", "json"],
                   allow_fail=True) is None:
            fail(f"the {verb} did not land"
                 + (" -- the title/body/labels part did, so re-run only the "
                    f"{verb}" if edited else ""))
    # tea issue edit ignores -o and prints its rendered card, so there is no
    # payload to echo: re-read the issues and hand back the fields that prove the
    # write landed. A labels-only edit that matched nothing used to answer ok
    # (tea silently ignores an unknown label), and confirmed:false says that this
    # answer carries no proof.
    issues, confirmed = None, False
    if not args.no_confirm:
        issues, confirmed = confirm_many(indexes)
        for issue in issues or []:
            if issue is not None:
                note_unapplied_labels(issue, want, gone, len(indexes) > 1)
    print(json.dumps({"ok": True, "indexes": indexes,
                      "confirmed": confirmed, "issues": issues}))


class JsonParser(argparse.ArgumentParser):
    """argparse would print usage on stderr and exit 2; keep the one contract
    this wrapper has with its callers: every failure is a JSON object."""

    def error(self, message):
        fail(f"{self.prog}: {message}")


def main():
    p = JsonParser(prog="utils/tea.py",
                   description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("new", help="create an issue")
    sp.add_argument("--title", required=True)
    sp.add_argument("--body", help="markdown; omit to read stdin")
    sp.add_argument("--labels")
    sp.add_argument("--milestone")
    sp.add_argument("--no-confirm", action="store_true",
                    help="skip the re-read that fills in the real labels")
    sp.set_defaults(fn=cmd_new)

    sp = sub.add_parser("ls", help="list issues (JSON array)")
    sp.add_argument("--state", default="open",
                    choices=["open", "closed", "all"])
    sp.add_argument("--keyword")
    sp.add_argument("--limit", type=check_limit, default=30,
                    help=f"rows per page (the server caps a page at "
                         f"{SERVER_PAGE_CAP} regardless)")
    sp.add_argument("--page", type=check_page, default=None, help="1-based page")
    sp.add_argument("--all", action="store_true",
                    help="walk every page and print one combined list")
    sp.set_defaults(fn=cmd_ls)

    sp = sub.add_parser("show", help="show one issue with body")
    sp.add_argument("index", type=check_index)
    sp.add_argument("--comments", action="store_true")
    sp.set_defaults(fn=cmd_show)

    sp = sub.add_parser("comment", help="add a comment")
    sp.add_argument("index", type=check_index)
    sp.add_argument("--body", help="markdown; omit to read stdin")
    sp.add_argument("--no-confirm", action="store_true",
                    help="skip the re-read that finds the comment again")
    sp.set_defaults(fn=cmd_comment)

    for name, tcmd in (("close", "close"), ("open", "reopen")):
        sp = sub.add_parser(name, help=f"{name} issues")
        sp.add_argument("indexes", nargs="+", type=check_index)
        sp.add_argument("--no-confirm", action="store_true",
                        help="skip the re-read that proves the change landed")
        sp.set_defaults(fn=lambda a, t=tcmd: cmd_state(t, a))

    sp = sub.add_parser("edit", help="edit title/body/state/labels")
    sp.add_argument("indexes", nargs="+", type=check_index,
                    help="several indexes are for labels/state only")
    sp.add_argument("--title")
    sp.add_argument("--body", help="new body (a piped file works too)")
    sp.add_argument("--allow-empty-body", action="store_true",
                    help="let --body \"\" clear the description")
    sp.add_argument("--no-confirm", action="store_true",
                    help="skip the re-read that proves the change landed")
    sp.add_argument("--labels", help="comma-separated label names to add")
    sp.add_argument("--remove-labels", help="comma-separated label names to drop")
    sp.add_argument("--close", action="store_true")
    sp.add_argument("--open", dest="is_open", action="store_true")
    sp.set_defaults(fn=cmd_edit)

    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    try:
        main()
        # Flush inside the try: with a big list the write can succeed only as
        # far as the pipe holds, and the BrokenPipeError then surfaces here
        # rather than as "Exception ignored while flushing sys.stdout".
        sys.stdout.flush()
    except BrokenPipeError:
        # `ls --all | head -1` closes stdout under us. Point what is left at
        # /dev/null so the interpreter's shutdown flush cannot add a second
        # traceback to a run whose JSON already went out.
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        sys.exit(1)
    except Exception as e:
        # The one promise this wrapper makes is that stdout is JSON. A payload
        # shape nobody predicted should not be able to break it with a traceback:
        # the traceback goes to stderr where it is still readable, and a caller
        # parsing stdout gets an error object. Especially matters after a write,
        # where an empty stdout reads as "nothing happened" and invites a retry.
        traceback.print_exc(file=sys.stderr)
        print(json.dumps({"error": f"utils/tea.py crashed: "
                                   f"{type(e).__name__}: {e}"}))
        sys.exit(1)
