#!/usr/bin/env python3
"""Thin, predictable wrapper around the `tea` Gitea CLI for issue tracking.

Why: tea's raw interface trips up agents -- trailing-dash stdin for
descriptions, output format varying by subcommand, index as string in
JSON vs int as input, `show` hidden behind a bare positional.

Every subcommand prints JSON to stdout (errors are an {"error": ...}
object on stdout with non-zero exit, so output can be piped freely).
Bodies are given with --body or piped on stdin. tea receives them via
--description-file - (quoting never matters) except comments, which
only accept a flag.

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
  utils/tea.py comment INDEX [--body "..." | < body.md]
  utils/tea.py close INDEX [INDEX...]
  utils/tea.py open INDEX [INDEX...]
  utils/tea.py edit INDEX [--title ...] [--body ...] [--close | --open]
                          [--labels a,b] [--remove-labels a,b]
                          (an empty pipe is not a body; a blank --body needs
                           --allow-empty-body)
"""

import argparse
import json
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


def run_tea(args, stdin_text=None):
    # Never inherit our stdin: an unexpected tea prompt would hang the caller.
    # subprocess.run forbids stdin+input together, so feed via input= only.
    if stdin_text is None:
        proc = subprocess.run([TEA, *args], capture_output=True, text=True,
                              stdin=subprocess.DEVNULL)
    else:
        proc = subprocess.run([TEA, *args], capture_output=True, text=True,
                              input=stdin_text)
    if proc.returncode != 0:
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
    return [norm_issue(d) for d in parse_json(run_tea(tea_args))]


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
    # tea prompts interactively about comments unless told either way.
    tea_args.append("--comments" if args.comments else "--comments=false")
    print(json.dumps(norm_issue(parse_json(run_tea(tea_args)))))


def cmd_comment(args):
    body = get_body(args)
    run_tea(["comments", "add", str(args.index), "--description", body,
             "-o", "json"])
    print(json.dumps({"ok": True, "index": args.index}))


def cmd_state(cmd, args):
    run_tea(["issue", cmd, *[str(i) for i in args.indexes], "-o", "json"])
    print(json.dumps({"ok": True, "indexes": args.indexes}))


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
    print(json.dumps({"ok": True, "index": args.index}))


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
        sp.set_defaults(fn=lambda a, t=tcmd: cmd_state(t, a))

    sp = sub.add_parser("edit", help="edit title/body/state/labels")
    sp.add_argument("index", type=check_index)
    sp.add_argument("--title")
    sp.add_argument("--body", help="new body (a piped file works too)")
    sp.add_argument("--allow-empty-body", action="store_true",
                    help="let --body \"\" clear the description")
    sp.add_argument("--labels", help="comma-separated label names to add")
    sp.add_argument("--remove-labels", help="comma-separated label names to drop")
    sp.add_argument("--close", action="store_true")
    sp.add_argument("--open", dest="is_open", action="store_true")
    sp.set_defaults(fn=cmd_edit)

    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
