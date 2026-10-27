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
found again. confirmed:false means no proof came back: --no-confirm skips the
re-read on purpose, and any other reason is written to stderr.

"labels" is a JSON list of names in every command's output. It has to be: tea's
ls renders them as one space-joined string, and real label names contain spaces
("Status/Need More Info"), so that string cannot be split by whitespace -- the
wrapper splits it against the repo's own label list instead (tea labels ls).

Bodies and comments are MARKDOWN -- the web UI renders them, and headings,
bullets and ``` fences are what make a ticket skimmable. Neither path escapes
anything, so structure survives either way; --body in a shell one-liner just
makes prose paragraphs the easy thing to type. For anything longer than a
sentence, write the body as a markdown file and pipe it:

  utils/tea.py new --title "..." < tmp/issue.md      # or a <<'MD' heredoc

Usage:
  utils/tea.py new --title "..." [--body "..." | < body.md] [--labels a,b]
                       [--no-confirm]
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
CONFIRM_MAX = 10        # re-read this many indexes after a state change, no more


def run_tea(args, stdin_text=None, allow_fail=False):
    # Never inherit our stdin: an unexpected tea prompt would hang the caller.
    # subprocess.run forbids stdin+input together, so feed via input= only.
    argv = [TEA, *args]
    try:
        if stdin_text is None:
            proc = subprocess.run(argv, capture_output=True, text=True,
                                  stdin=subprocess.DEVNULL)
        else:
            proc = subprocess.run(argv, capture_output=True, text=True,
                                  input=stdin_text)
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
    return body


_label_vocab = None  # repo label names, fetched at most once per run


def label_vocab():
    """Every label the repo knows, longest name first. tea renders an issue's
    labels as one space-joined string and real names hold spaces
    ("Status/Need More Info"), so the vocabulary is the only thing that can
    split that string back into names. Empty means we could not ask."""
    global _label_vocab
    if _label_vocab is None:
        names = []
        out = run_tea(["labels", "ls", "-o", "json"], allow_fail=True)
        if out is not None:
            try:
                data = json.loads(out)
            except json.JSONDecodeError:
                data = None
            if isinstance(data, list):
                names = [d["name"] for d in data
                         if isinstance(d, dict) and isinstance(d.get("name"), str)]
            else:
                print("note: tea labels ls did not answer a list, so labels are "
                      "split on spaces and a name containing a space will look "
                      "like two", file=sys.stderr)
        _label_vocab = sorted(names, key=len, reverse=True)
    return _label_vocab


def split_labels(text):
    """Greedy longest match over the vocabulary, so "Status/Need More Info
    Priority/High" gives two names and not five."""
    vocab = label_vocab()
    words = text.split()
    out, i = [], 0
    while i < len(words):
        for name in vocab:
            parts = name.split()
            if words[i:i + len(parts)] == parts:
                out.append(name)
                i += len(parts)
                break
        else:
            out.append(words[i])
            i += 1
    return out


def norm_issue(d):
    # ls and single-issue JSON have different shapes; unify the common keys.
    if "index" not in d:
        fail(f"unexpected tea payload, no index: {d}")
    d["index"] = int(d["index"])
    if "id" in d:
        del d["id"]  # db id, not the issue index
    if "user" in d:
        d["author"] = d.pop("user")
    # ls gives one space-joined string, show/create a list of objects. Both
    # become a list of names: a joined string cannot be split by its caller,
    # and a diff between the two shapes shows differences that are not there.
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
        d["labels"] = names
    elif isinstance(labels, str) and labels.strip():
        d["labels"] = split_labels(labels)
    else:
        d["labels"] = []
    return d


def confirm_issue(index):
    """What the server holds after a write, in the fields that show the write
    landed. `tea issue edit` and `tea issue close` ignore -o and print a
    rendered card, so there is no payload to echo and re-reading is the only
    proof. None means the re-read failed or came back unreadable -- and the
    write already ran by then, so this must never take the process down; it says
    why on stderr instead."""
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
        d = norm_issue(d)
        return {"index": d["index"], "state": d.get("state"),
                "title": d.get("title"), "labels": d.get("labels"),
                "url": d.get("url"), "closed_at": d.get("closedAt"),
                "body_chars": len(d.get("body") or "")}
    except (ValueError, TypeError, KeyError) as e:
        print(f"note: re-read of issue {index} came back unreadable ({e}) -- "
              f"the write itself already ran", file=sys.stderr)
        return None


def note_unapplied_labels(issue, want="", gone="", batch=False):
    """tea ignores a label name it does not know -- a typo'd --labels exits 0 and
    changes nothing. The re-read already knows the truth, so diff it out loud.
    Takes the wanted/dropped lists rather than the parsed args: not every
    subcommand has both flags, and reading a missing attribute here would crash
    after the write already landed."""
    have = set(issue["labels"])
    want = {l.strip() for l in want.split(",") if l.strip()}
    gone = {l.strip() for l in gone.split(",") if l.strip()}
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


def confirm_comment(index, body):
    """`tea comments add` ignores -o and prints a card, so the comment has to be
    found again to be believed. Compared with the edges stripped because Gitea
    trims them; the last match wins, as our comment is the newest of any equal
    pair. None plus a stderr note means the write already ran and only the
    proof is missing."""
    out = run_tea(["issue", str(index), "-o", "json", "--comments"],
                  allow_fail=True)
    if out is None:
        return None                     # run_tea already said why
    try:
        d = json.loads(out)
        comments = d.get("comments") if isinstance(d, dict) else None
        if not isinstance(comments, list):
            raise ValueError(f"no comments in {out[:60]!r}")
        want = body.strip()
        found = None
        for c in comments:
            if (isinstance(c, dict) and isinstance(c.get("body"), str)
                    and c["body"].strip() == want):
                found = c
        if found is None:
            print(f"note: issue {index} has no comment matching the text that "
                  f"was posted -- it may still have landed, check "
                  f"`utils/tea.py show {index} --comments`", file=sys.stderr)
            return None
        return {"author": found.get("author"), "created": found.get("created"),
                "chars": len(found["body"])}
    except (ValueError, TypeError, KeyError) as e:
        print(f"note: re-read of issue {index}'s comments came back unreadable "
              f"({e}) -- the comment itself already ran", file=sys.stderr)
        return None


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
    # created with two and answered "labels": []. So the labels (and the state)
    # in this answer come from a re-read, like every other write here -- which
    # is also what catches a typo'd --labels on a brand new issue.
    issue = None if args.no_confirm else confirm_issue(d["index"])
    if issue is not None:
        d["labels"] = issue["labels"]
        note_unapplied_labels(issue, args.labels or "")
    print(json.dumps({"ok": True, "index": d["index"],
                      "confirmed": issue is not None, "issue": d}))


def ls_page(args, page):
    tea_args = ["issue", "ls", "-o", "json", "--fields", LS_FIELDS,
                "--state", args.state, "--limit", str(args.limit),
                "--page", str(page)]
    if args.keyword:
        tea_args += ["--keyword", args.keyword]
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
        rows = ls_page(args, page)
        if len(rows) >= step:
            print(f"note: page full at {len(rows)} items -- pass --all for the "
                  f"whole list (or --page {page + 1})", file=sys.stderr)
        print(json.dumps(rows))
        return
    out, page = [], 1
    while True:
        rows = ls_page(args, page)
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
    run_tea(["comments", "add", str(args.index), "--description", body,
             "-o", "json"])
    # No payload comes back from that call, so the proof is finding the comment
    # again -- one extra read, which is why --no-confirm exists here too.
    comment = None if args.no_confirm else confirm_comment(args.index, body)
    print(json.dumps({"ok": True, "index": args.index,
                      "confirmed": comment is not None, "comment": comment}))


def cmd_state(cmd, args):
    # dict.fromkeys keeps the order and drops a repeated index, so the tea call
    # and every re-read happen once per issue.
    indexes = list(dict.fromkeys(args.indexes))
    run_tea(["issue", cmd, *[str(i) for i in indexes], "-o", "json"])
    # Same reason as cmd_edit: tea renders a card instead of a payload, so
    # confirming means re-reading -- one extra call per index, which a bulk
    # change should not pay without being told (hence --no-confirm, and the
    # CONFIRM_MAX ceiling).
    issues, confirmed = None, False
    if not args.no_confirm:
        if len(indexes) <= CONFIRM_MAX:
            issues = [confirm_issue(i) for i in indexes]
            confirmed = all(i is not None for i in issues)
        else:
            print(f"note: {len(indexes)} indexes changed without re-reading "
                  f"them (over {CONFIRM_MAX}) -- pass --no-confirm to mean it "
                  f"from the start, or check with ls", file=sys.stderr)
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
             "--labels/--remove-labels/--close/--open")
    if any([args.title is not None, body is not None, args.labels,
            args.remove_labels]):
        tea_args = ["issue", "edit", *[str(i) for i in indexes], "-o", "json"]
        if args.title is not None:
            tea_args += ["--title", args.title]
        if body is not None:
            tea_args += ["--description-file", "-"]
        if args.labels:
            tea_args += ["--add-labels", args.labels]
        if args.remove_labels:
            tea_args += ["--remove-labels", args.remove_labels]
        run_tea(tea_args, stdin_text=body)
    # tea issue edit has no state flags; close/reopen are their own subcommands.
    if args.close:
        run_tea(["issue", "close", *[str(i) for i in indexes], "-o", "json"])
    if args.is_open:
        run_tea(["issue", "reopen", *[str(i) for i in indexes], "-o", "json"])
    # tea issue edit ignores -o and prints its rendered card, so there is no
    # payload to echo: re-read the issues and hand back the fields that prove the
    # write landed. A labels-only edit that matched nothing used to answer ok
    # (tea silently ignores an unknown label), and confirmed:false says that this
    # answer carries no proof.
    issues, confirmed = None, False
    if not args.no_confirm:
        if len(indexes) <= CONFIRM_MAX:
            issues = [confirm_issue(i) for i in indexes]
            confirmed = all(i is not None for i in issues)
            for issue in issues:
                if issue is not None:
                    note_unapplied_labels(issue, args.labels or "",
                                          args.remove_labels or "",
                                          len(indexes) > 1)
        else:
            print(f"note: {len(indexes)} indexes changed without re-reading "
                  f"them (over {CONFIRM_MAX}) -- pass --no-confirm to mean it "
                  f"from the start, or check with ls", file=sys.stderr)
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
