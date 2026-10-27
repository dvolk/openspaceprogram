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
will not exec an argument that reaches MAX_ARG_STRLEN bytes.

tea's own output is inconsistent: `issue create` answers JSON, while `issue
edit` and `issue close/reopen` ignore -o entirely and print a rendered card. So
the writes here prove themselves by re-reading the issue: edit answers
{"ok":..., "confirmed":..., "issue":{index,state,title,labels,url,closed_at,
body_chars}}, close/open the same per index (up to CONFIRM_MAX of them).
confirmed:false means no proof came back: --no-confirm skips the re-read on
purpose, and any other reason is written to stderr.

Bodies and comments are MARKDOWN -- the web UI renders them, and headings,
bullets and ``` fences are what make a ticket skimmable. Neither path escapes
anything, so structure survives either way; --body in a shell one-liner just
makes prose paragraphs the easy thing to type. For anything longer than a
sentence, write the body as a markdown file and pipe it:

  utils/tea.py new --title "..." < tmp/issue.md      # or a <<'MD' heredoc

Usage:
  utils/tea.py new --title "..." [--body "..." | < body.md] [--labels a,b]
  utils/tea.py ls [--state all|open|closed] [--keyword str] [--limit N]
                       [--all | --page N]
                       (--all walks every page; a bare page is one page, and the
                        server caps a page at 50 rows however big --limit is)
  utils/tea.py show INDEX [--comments]
                       (--comments off answers "comments": null, because tea's
                        own --comments=false answers [] like a quiet issue)
  utils/tea.py comment INDEX [--body "..." | < body.md]
  utils/tea.py close INDEX [INDEX...] [--no-confirm]
  utils/tea.py open INDEX [INDEX...] [--no-confirm]
  utils/tea.py edit INDEX [--title ...] [--body ...] [--close | --open]
                          [--labels a,b] [--remove-labels a,b] [--no-confirm]
                          (an empty pipe is not a body; a blank --body needs
                           --allow-empty-body)
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


def norm_issue(d):
    # ls and single-issue JSON have different shapes; unify the common keys.
    if "index" not in d:
        fail(f"unexpected tea payload, no index: {d}")
    d["index"] = int(d["index"])
    if "id" in d:
        del d["id"]  # db id, not the issue index
    if "user" in d:
        d["author"] = d.pop("user")
    if isinstance(d.get("labels"), list):
        d["labels"] = ",".join(l["name"] for l in d["labels"])
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


def note_unapplied_labels(args, held):
    """tea ignores a label name it does not know -- a typo'd --labels exits 0 and
    changes nothing. The re-read already knows the truth, so diff it out loud."""
    have = {l for l in (held or "").split(",") if l}
    want = {l for l in (args.labels or "").split(",") if l}
    gone = {l for l in (args.remove_labels or "").split(",") if l}
    missing = sorted(want - have)
    leftover = sorted(gone & have)
    if missing:
        print(f"note: not on the issue after the edit: {', '.join(missing)} -- "
              f"tea silently ignores a label name it does not know (see "
              f"`tea labels ls` for the real spellings)", file=sys.stderr)
    if leftover:
        print(f"note: still on the issue after --remove-labels: "
              f"{', '.join(leftover)}", file=sys.stderr)


def cmd_new(args):
    body = get_body(args)
    tea_args = ["issue", "create", "-o", "json", "--title", args.title,
                "--description-file", "-"]
    if args.labels:
        tea_args += ["--labels", args.labels]
    if args.milestone:
        tea_args += ["--milestone", args.milestone]
    out = run_tea(tea_args, stdin_text=body)
    print(json.dumps(norm_issue(parse_json(out))))


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
    print(json.dumps({"ok": True, "index": args.index}))


def cmd_state(cmd, args):
    run_tea(["issue", cmd, *[str(i) for i in args.indexes], "-o", "json"])
    # Same reason as cmd_edit: tea renders a card instead of a payload, so
    # confirming means re-reading -- one extra call per index, which a bulk
    # change should not pay without being told (hence --no-confirm, and the
    # CONFIRM_MAX ceiling).
    issues, confirmed = None, False
    if not args.no_confirm:
        if len(args.indexes) <= CONFIRM_MAX:
            issues = [confirm_issue(i) for i in args.indexes]
            confirmed = all(i is not None for i in issues)
        else:
            print(f"note: {len(args.indexes)} indexes changed without re-reading "
                  f"them (over {CONFIRM_MAX}) -- pass --no-confirm to mean it "
                  f"from the start, or check with ls", file=sys.stderr)
    print(json.dumps({"ok": True, "indexes": args.indexes,
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
    if any([args.title is not None, body is not None, args.labels,
            args.remove_labels]):
        tea_args = ["issue", "edit", str(args.index), "-o", "json"]
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
        run_tea(["issue", "close", str(args.index), "-o", "json"])
    if args.is_open:
        run_tea(["issue", "reopen", str(args.index), "-o", "json"])
    # tea issue edit ignores -o and prints its rendered card, so there is no
    # payload to echo: re-read the issue and hand back the fields that prove the
    # write landed. A labels-only edit that matched nothing used to answer ok
    # (tea silently ignores an unknown label), and confirmed:false says that this
    # answer carries no proof.
    issue = None if args.no_confirm else confirm_issue(args.index)
    if issue is not None:
        note_unapplied_labels(args, issue["labels"])
    print(json.dumps({"ok": True, "index": args.index,
                      "confirmed": issue is not None, "issue": issue}))


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
    sp.set_defaults(fn=cmd_comment)

    for name, tcmd in (("close", "close"), ("open", "reopen")):
        sp = sub.add_parser(name, help=f"{name} issues")
        sp.add_argument("indexes", nargs="+", type=check_index)
        sp.add_argument("--no-confirm", action="store_true",
                        help="skip the re-read that proves the change landed")
        sp.set_defaults(fn=lambda a, t=tcmd: cmd_state(t, a))

    sp = sub.add_parser("edit", help="edit title/body/state/labels")
    sp.add_argument("index", type=check_index)
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
