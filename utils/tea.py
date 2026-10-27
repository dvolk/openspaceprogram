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
found again, and only believes it when that match is dated after the send -- a
match dated before it, or carrying no date, is shown with confirmed:false
because it could be somebody else's identical text.
confirmed:false means no proof came back: --no-confirm skips the re-read on
purpose, and any other reason is written to stderr. It means the write is not
known to have taken effect, not that it failed: the state and labels a write was
meant to produce are checked, so a close that tea did not apply says so rather
than confirming the issue still exists. Past CONFIRM_MAX indexes the
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
LS_MAX_PAGES = 100      # 100 pages x cap is far past this repo; guards both
                        # `ls --all` and the bulk proof walk after a write
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


_saw_timeout = False  # set when a tea call gave no answer at all (vs. refusing)


def tea_cmd(args):
    """How to name a tea call in a message. Joining argv raw would paste a whole
    comment body into the error object on stdout -- up to 128 KB of it."""
    return " ".join(a if len(a) <= 60 else f"{a[:40]}... ({len(a)} chars)"
                    for a in args)


def caveat():
    """Appended to any write failure, because a call that timed out is not a
    call that was refused: the difference decides whether a retry duplicates the
    write."""
    return (" -- one of this run's calls went unanswered, so a write may have "
            "reached the server anyway: check before re-running it"
            if _saw_timeout else "")


def run_tea(args, stdin_text=None, allow_fail=False):
    # Never inherit our stdin: an unexpected tea prompt would hang the caller.
    # subprocess.run forbids stdin+input together, so feed via input= only.
    global _saw_timeout
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
        # reached Gitea, so this is "unknown", never "nothing happened". The flag
        # is what lets a caller's {"error": ...} say that instead of implying the
        # server refused it.
        _saw_timeout = True
        msg = f"tea {tea_cmd(args)} gave no answer within {TEA_TIMEOUT}s"
        if allow_fail:
            note(msg + caveat())
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
            note(f"tea {tea_cmd(args)} failed: "
                 f"{proc.stderr.strip() or 'exit ' + str(proc.returncode)}")
            return None
        fail(proc.stderr.strip() or f"tea {tea_cmd(args)} failed "
             f"(exit {proc.returncode})")
    return proc.stdout


def parse_json(out):
    try:
        return json.loads(out)
    except json.JSONDecodeError:
        fail(f"could not parse tea output as JSON:\n{out}")


def fail(msg):
    # If any call in this run went unanswered, say so on the way out: an error
    # object that reads as "the server refused it" invites a retry of a write
    # that may already have landed.
    flush_note_count()
    print(json.dumps({"error": msg + caveat()}))
    sys.exit(1)


_noted = set()  # notes already printed this run, so one can be said once
_NOTE_LIMIT = 40  # ... and a run never says more than this many different ones
_notes_dropped = 0


def note(msg):
    """A stderr note, printed at most once per identical message. Row-by-row
    paths (a list walk over 450 issues) would otherwise bury the answer under
    450 copies of the same complaint, and a note about every row is a note
    nobody reads: past the limit they are counted instead, and the count is said
    when the command is done."""
    global _notes_dropped
    if msg in _noted:
        return
    if len(_noted) >= _NOTE_LIMIT:
        _notes_dropped += 1
        return
    _noted.add(msg)
    print(f"note: {msg}", file=sys.stderr)


def flush_note_count():
    if _notes_dropped:
        print(f"note: {_notes_dropped} more note(s) not printed -- {len(_noted)} "
              f"different ones already were; the JSON answer is unchanged",
              file=sys.stderr)


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
        names, junk, page = [], 0, 1
        # Keep asking until a page comes back empty rather than until a page
        # looks short: the server is known to clamp page sizes (issues pages cap
        # at 50 however big --limit is), and a clamp below LABEL_PAGE would
        # otherwise truncate the vocabulary with no hint at all.
        while page <= LABEL_MAX_PAGES:
            out = run_tea(["labels", "ls", "-o", "json",
                           "--limit", str(LABEL_PAGE), "--page", str(page)],
                          allow_fail=True)
            if out is None:
                break                     # run_tea already said why
            try:
                data = json.loads(out)
            except json.JSONDecodeError:
                data = None
            if not isinstance(data, list):
                note("tea labels ls did not answer a list, so labels are split "
                     "on spaces and a name containing a space will look like two")
                break
            for entry in data:
                name = entry.get("name") if isinstance(entry, dict) else entry
                if isinstance(name, str) and name.strip():
                    names.append(name)
                else:
                    junk += 1
            if not data:
                break
            page += 1
        if junk:
            note(f"tea labels ls answered {junk} unreadable label(s) -- a "
                 f"multi-word one among them will not be recognized")
        if page > LABEL_MAX_PAGES:
            note(f"stopped reading the label list after {LABEL_MAX_PAGES} pages "
                 f"-- a multi-word label past that gets split wrong")
        if not names:
            note("no label vocabulary, so labels are split on spaces and a name "
                 "containing a space will look like two")
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
        note(f"{tag}labels not on the repo's own label list: "
             f"{', '.join(unknown)} -- the vocabulary is stale, or tea joined "
             f"something it should not have")
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
                note(f"ignoring an unreadable label entry {entry!r}")
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
        return issue_proof(seen, d.get("state"), d.get("title"),
                           labels_of(d, seen), d.get("url"), d.get("closedAt"),
                           len(d.get("body") or ""))
    except (ValueError, TypeError, KeyError) as e:
        note(f"re-read of issue {index} came back unreadable ({e}) -- the write "
             f"itself already ran")
        return None


def split_requested(text):
    """A --labels/--remove-labels value as a set of names (tea's own format is
    comma-separated, so a label name with a comma in it cannot be set this way
    even though it can be read)."""
    return {t.strip() for t in (text or "").split(",") if t.strip()}


def issue_proof(index, state, title, labels, url, closed_at, body_chars):
    """The one shape a proof answer has, so the per-index re-read and the list
    walk cannot drift into two different shapes for the same event."""
    return {"index": index, "state": state, "title": title, "labels": labels,
            "url": url, "closed_at": closed_at, "body_chars": body_chars}


def note_unapplied_labels(issue, want, gone, batch):
    """tea ignores a label name it does not know -- a typo'd --labels exits 0 and
    changes nothing. The re-read already knows the truth, so diff it out loud.
    Returns whether the labels now match what was asked, which is what a caller
    means by "the write landed". Takes the sets rather than the parsed args: not
    every subcommand has both flags, and reading a missing attribute here would
    crash after the write already landed."""
    have = set(issue["labels"])
    missing = sorted(want - have)
    leftover = sorted(gone & have)
    tag = f"issue {issue['index']}: " if batch else ""
    ok = True
    if missing:
        note(f"{tag}not on the issue after the write: {', '.join(missing)} -- "
             f"tea silently ignores a label name it does not know (see "
             f"`tea labels ls` for the real spellings)")
        ok = False
    if leftover:
        note(f"{tag}still on the issue after --remove-labels: "
             f"{', '.join(leftover)}")
        ok = False
    # The other way round: the diff above is only trustworthy if the name asked
    # for is one the repo knows. If the label list came back short, a multi-word
    # label reads as several single-word ones and a removal can look successful
    # when it was never possible.
    vocab = {name for name, _ in label_vocab()}
    if vocab:
        unreal = sorted((want | gone) - vocab)
        if unreal:
            note(f"{tag}not on the repo's own label list: {', '.join(unreal)} -- "
                 f"either a name tea quietly ignored, or the label list was read "
                 f"incompletely, so this comparison cannot be trusted")
            ok = False
    return ok


def confirm_comment(index, body, sent_at):
    """`tea comments add` ignores -o and prints a card, so the comment has to be
    found again to be believed. Compared with all runs of whitespace collapsed,
    which covers Gitea trimming the edges and any CRLF that came in on the pipe.
    Of several equal comments the one with the newest date is taken to be ours; a
    match dated before we sent it, or carrying no date at all, is returned but
    not counted as proof -- re-posting a comment that looked like it failed is
    exactly how a borrowed match happens. Returns (comment, believed);
    (None, False) with a stderr note means the write already ran and only the
    proof is missing."""
    out = run_tea(["issue", str(index), "-o", "json", "--comments"],
                  allow_fail=True)
    if out is None:
        return None, False              # run_tea already said why
    try:
        d = json.loads(out)
        comments = d.get("comments") if isinstance(d, dict) else None
        if not isinstance(comments, list):
            raise ValueError(f"no comments in {out[:60]!r}")
        want = " ".join(body.split())
        matches = [c for c in comments
                   if isinstance(c, dict) and isinstance(c.get("body"), str)
                   and " ".join(c["body"].split()) == want]
        if not matches:
            note(f"issue {index} has no comment matching the text that was "
                 f"posted -- it may still have landed, check "
                 f"`utils/tea.py show {index} --comments`")
            return None, False
        # ISO-8601 UTC sorts as text, so the newest date is the max. With no date
        # to sort on, "the last equal one" is the only guess left, and it is a
        # guess that can credit somebody else's older comment as ours.
        dated = [c for c in matches if isinstance(c.get("created"), str)]
        found = max(dated, key=lambda c: c["created"]) if dated else matches[-1]
        created = found.get("created")
        proof = {"author": found.get("author"), "created": created,
                 "chars": len(found["body"])}
        if not isinstance(created, str):
            note(f"the matching comment on issue {index} has no date, so it "
                 f"cannot be told from an older comment with the same text")
            return proof, False
        if _predates(created, sent_at):
            note(f"the matching comment on issue {index} is dated {created!r}, "
                 f"before this attempt -- it may be an older comment with the "
                 f"same text, so it is not counted as proof; `show {index} "
                 f"--comments` will settle it")
            return proof, False
        return proof, True
    except (ValueError, TypeError, KeyError) as e:
        note(f"re-read of issue {index}'s comments came back unreadable ({e}) "
             f"-- the comment itself already ran")
        return None, False


def _predates(created, sent_at):
    """True when Gitea dated the match clearly before we sent it. Both sides are
    "%Y-%m-%dT%H:%M:%SZ", so comparing the strings compares the instants; 120s
    of slack because this clock and the server's clock are not one clock."""
    return (isinstance(created, str)
            and created < time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                        time.gmtime(sent_at - 120)))


def confirm_many(indexes, want=None, gone=None, want_state=None):
    """Proof for a write that touched several indexes. Few of them: re-read each
    one. Past CONFIRM_MAX: walk the issue list once instead, which costs fewer
    calls than one re-read per index and beats the alternative of answering with
    no proof at all -- which is what a bulk sweep most needs, since that is when
    a silent typo costs the most.

    Returns (issues, confirmed), where confirmed means the CHANGE is there, not
    that the issue is there: an index that cannot be found, a state that did not
    move, or labels tea quietly ignored each make it false, each with a note
    saying which. Before this, `close` over eleven issues answered
    confirmed:true while every one of them was still open."""
    want = want or set()
    gone = gone or set()
    if len(indexes) <= CONFIRM_MAX:
        issues = [confirm_issue(i) for i in indexes]
    else:
        issues = walk_issue_list(indexes)
        if issues is None:
            return None, False          # walk_issue_list already said why
    ok = True
    for index, issue in zip(indexes, issues):
        if issue is None:
            ok = False                  # confirm_* already said why
            continue
        if want_state and issue["state"] != want_state:
            note(f"issue {index} is {issue['state']!r} after the write meant to "
                 f"make it {want_state!r}")
            ok = False
        if not note_unapplied_labels(issue, want, gone, len(indexes) > 1):
            ok = False
    return issues, ok


def walk_issue_list(indexes):
    """The bulk proof: one pass over the issue list rather than one re-read per
    index. Runs after the write, so it reports what it could not see and never
    ends the process -- ls_rows would, and that would leave a write with an error
    object instead of an answer. One entry per requested index, null where the
    list had nothing to say."""
    asked = set(indexes)
    found, page = {}, 1
    while page <= LS_MAX_PAGES:
        page_rows = ls_page_soft(page)
        if page_rows is None:
            note(f"the bulk proof stopped after {page - 1} page(s) of the issue "
                 f"list -- {len(asked)} indexes are unproven")
            return None
        rows, seen = page_rows
        for row in rows:
            if row["index"] in asked:
                found[row["index"]] = row
        if not seen:
            break
        page += 1
    else:
        note(f"stopped walking the issue list after {LS_MAX_PAGES} pages -- some "
             f"of the {len(asked)} indexes may be further on")
    missing = sorted(asked - set(found))
    if missing:
        note(f"not in the issue list after the write: "
             f"{', '.join(str(i) for i in missing)} -- check with ls (a pull "
             f"request shares its number with an issue)")
    note("bulk proof came from the issue list, so closed_at and body_chars are "
         "null in it, not absent from the issue")
    return [found.get(i) for i in indexes]


def ls_page_soft(page):
    """One page of the issue list, for the bulk proof only: nothing here may end
    the process, so a bad payload is a note and a None. Returns (rows, rows_seen)
    -- the raw count, so an all-junk page does not read as the end of the list."""
    out = run_tea(_ls_argv("all", None, SERVER_PAGE_CAP, page), allow_fail=True)
    if out is None:
        return None                     # run_tea already said why
    try:
        data = json.loads(out)
    except json.JSONDecodeError:
        note(f"the issue list answered something that is not JSON: {out[:60]!r}")
        return None
    if not isinstance(data, list):
        note(f"the issue list answered {type(data).__name__}, not a list -- no "
             f"bulk proof")
        return None
    rows = []
    for d in data:
        if not isinstance(d, dict) or "index" not in d:
            note(f"ignoring an unreadable row in the issue list: {str(d)[:60]}")
            continue
        try:
            idx = int(d["index"])
        except (TypeError, ValueError):
            note(f"ignoring a row whose index is not a number: {d['index']!r}")
            continue
        # The list carries no closedAt and no body, so those two are null here:
        # "the list was not asked", which the note in walk_issue_list spells out.
        rows.append(issue_proof(idx, d.get("state"), d.get("title"),
                                labels_of(d, idx), d.get("url"), None, None))
    return rows, len(data)


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
    confirmed = False
    if issue is not None:
        d["labels"] = issue["labels"]
        confirmed = note_unapplied_labels(issue, split_requested(args.labels),
                                          set(), False)
    else:
        d["labels"] = None
    print(json.dumps({"ok": True, "index": d["index"],
                      "confirmed": confirmed, "issue": d}))


def _ls_argv(state, keyword, limit, page):
    tea_args = ["issue", "ls", "-o", "json", "--fields", LS_FIELDS,
                "--state", state, "--limit", str(limit), "--page", str(page)]
    if keyword:
        tea_args += ["--keyword", keyword]
    return tea_args


def ls_rows(state, keyword, limit, page):
    # A read, so an unusable payload is a hard error: a partial list would be
    # the complete-looking-empty answer this wrapper exists to prevent. The bulk
    # proof after a write cannot afford that, so it uses ls_page_soft instead.
    data = parse_json(run_tea(_ls_argv(state, keyword, limit, page)))
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
        note(f"the server caps a page at {SERVER_PAGE_CAP} rows, so --limit "
             f"{args.limit} fetches {step} -- pass --all for the whole list")
    if not args.all:
        page = args.page or 1
        rows = ls_rows(args.state, args.keyword, args.limit, page)
        if not rows and page > 1:
            note(f"page {page} is empty -- past the end of the list, not an "
                 f"empty repository")
        elif len(rows) >= step:
            note(f"page full at {len(rows)} items -- pass --all for the whole "
                 f"list (or --page {page + 1})")
        print(json.dumps(rows))
        return
    out, page = [], 1
    while True:
        rows = ls_rows(args.state, args.keyword, args.limit, page)
        out += rows
        if len(rows) < step:
            break
        if page >= LS_MAX_PAGES:
            note(f"stopped after {LS_MAX_PAGES} pages ({len(out)} items) -- "
                 f"raise LS_MAX_PAGES if there really are more")
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
    comment = believed = None
    if not args.no_confirm:
        comment, believed = confirm_comment(args.index, body, sent_at)
    # A match that predates the attempt, or has no date to place it, is reported
    # but not believed, so the caller still sees what is on the issue without
    # being told it is ours.
    print(json.dumps({"ok": True, "index": args.index,
                      "confirmed": bool(believed), "comment": comment}))


def cmd_state(cmd, args):
    # dict.fromkeys keeps the order and drops a repeated index, so the tea call
    # and every re-read happen once per issue.
    indexes = list(dict.fromkeys(args.indexes))
    want_state = "closed" if cmd == "close" else "open"
    if run_tea(["issue", cmd, *[str(i) for i in indexes], "-o", "json"],
               allow_fail=True) is None:
        # One bad index among many makes tea exit non-zero after applying the
        # rest, so an error on its own would read as though nothing moved -- and
        # "re-run the lot" is exactly the wrong next move. Say what each index is
        # now, then fail.
        issues, _ = confirm_many(indexes, want_state=want_state)
        seen = ", ".join(f"{i}={issue['state'] if issue else 'unknown'}"
                         for i, issue in zip(indexes, issues or []))
        fail(f"the {cmd} did not go through for all of "
             f"{', '.join(str(i) for i in indexes)} -- state now: {seen}")
    # tea renders a card instead of a payload, so confirming means re-reading:
    # one re-read per index while there are few, one walk of the list past
    # CONFIRM_MAX. --no-confirm still means "I know, skip it". want_state is what
    # makes the answer mean "it is closed now" rather than "that issue exists".
    issues, confirmed = None, False
    if not args.no_confirm:
        issues, confirmed = confirm_many(indexes, want_state=want_state)
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
    if body is not None:
        check_body(body, "body")
        if args.body is None:
            # A description is the one thing here that a stray pipe rewrites, so
            # say it after the body is known to be sendable: a labels edit should
            # not carry a description silently, and announcing a write that is
            # about to be refused is its own little lie.
            note("sending piped stdin as the new description")
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
        note("--title \"\" asks tea to unset the title, which Gitea may not "
             "accept")
    want = split_requested(args.labels)
    gone = split_requested(args.remove_labels)
    want_state = ("closed" if args.close else "open" if args.is_open else None)
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
            # though nothing changed -- and re-running the whole sweep is the
            # wrong response to that. Ask what actually carries the change.
            if len(indexes) > 1:
                applied, _ = confirm_many(indexes, want, gone)
                landed = [i for i, issue in zip(indexes, applied or [])
                          if issue is not None
                          and want <= set(issue["labels"])
                          and not gone & set(issue["labels"])]
                note(f"end state matches on: "
                     f"{', '.join(str(i) for i in landed) or 'nothing confirmed'}"
                     f" -- tea applies the indexes it can and still exits "
                     f"non-zero, so re-run only the rest. For --remove-labels "
                     f"this cannot tell a removal from a label that was never "
                     f"there, so read the notes above")
            fail(f"the edit did not go through for all of "
                 f"{', '.join(str(i) for i in indexes)}")
    # tea issue edit has no state flags, so --close/--open are a second call
    # after the first one already went out. If it fails, say what is already
    # applied instead of an error that reads as though nothing happened.
    if args.close or args.is_open:
        verb = "close" if args.close else "reopen"
        if run_tea(["issue", verb, *[str(i) for i in indexes], "-o", "json"],
                   allow_fail=True) is None:
            # Same reason as in cmd_state: with several indexes some of them may
            # have moved before tea refused, so the answer names the states
            # rather than implying none did.
            seen_now, _ = confirm_many(indexes, want, gone, want_state)
            states = ", ".join(f"{i}={issue['state'] if issue else 'unknown'}"
                               for i, issue in zip(indexes, seen_now or []))
            fail(f"the {verb} did not go through for all of "
                 f"{', '.join(str(i) for i in indexes)} -- state now: {states}"
                 + (" -- the title/body/labels part did, so re-run only the "
                    f"{verb}" if edited else ""))
    # tea issue edit ignores -o and prints its rendered card, so there is no
    # payload to echo: re-read the issues and hand back the fields that prove the
    # write landed. A labels-only edit that matched nothing used to answer ok
    # (tea silently ignores an unknown label), and confirmed:false says that this
    # answer carries no proof.
    issues, confirmed = None, False
    if not args.no_confirm:
        issues, confirmed = confirm_many(indexes, want, gone, want_state)
        if args.title is not None:
            for issue in issues or []:
                if issue is not None and issue["title"] != args.title:
                    note(f"issue {issue['index']} is titled {issue['title']!r}, "
                         f"not {args.title!r}")
                    confirmed = False
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
        flush_note_count()
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
        flush_note_count()
        sys.exit(1)
