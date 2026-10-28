#!/usr/bin/env python3
"""Issue tracker wrapper: Gitea's REST API directly, with one JSON contract.

The name is historical. This used to wrap the `tea` CLI, and most of that
version's machinery existed only because tea is a terminal program: it printed a
rendered card instead of a payload for every write, so each write had to be
re-read to be believed; it space-joined label names that contain spaces; it took
a comment body as one argv string, under the kernel's 128 KB per-argument cap;
it hid paging behind a page clamp. Talking to /api/v1 removes all of that, and
the proof of a write is the object the server returns.

Why keep a wrapper at all: the API still has sharp edges worth standing on --
label writes need ids and separate endpoints (PATCH silently ignores label
fields on this server, measured 2026-10-27), a page is capped at 50 rows and
only X-Total-Count says so, the issue and pull request numbers share one space,
bodies are markdown that a shell will happily eat, and every answer needs to be
the same shape for a caller that is a script and not a human.

Contract, unchanged from the tea version:
- every subcommand prints JSON on stdout; every failure is {"error": ...} there
  with non-zero exit, so output can be piped freely;
- explanations that are not failures go to stderr as "note: ...", deduped and
  capped so a 450-row listing cannot bury its own answer;
- a write answers {"ok": true, "confirmed": bool, ...} where confirmed means the
  server said it did the thing -- not that the issue exists. It is not a promise
  that nothing failed: when the proof is missing, the reason is on stderr;
- a request that got no answer is never reported as "nothing happened": retrying
  a write that landed is how duplicate comments get made.

Config comes from tea's own config (~/.config/tea/config.yml: the default login
gives the server url and token, and the git remote gives owner/repo). Set
TEAWRAP_BASE and TEAWRAP_TOKEN to point it somewhere else -- tests do, at a mock
server, so a test run can never touch the real tracker.

Bodies and comments are MARKDOWN -- the web UI renders them, and headings,
bullets and ``` fences are what make a ticket skimmable. For anything longer
than a sentence, write the body as a markdown file and pipe it:

  utils/tea.py new --title "..." < tmp/issue.md      # or a <<'MD' heredoc

Usage:
  utils/tea.py new --title "..." [--body "..." | < body.md] [--labels a,b]
                       [--milestone m]
                       (the create answer carries the labels it just set, so
                        there is nothing to re-read)
  utils/tea.py ls [--state all|open|closed] [--keyword str] [--limit N]
                       [--all | --page N]
                       (a page is a page; --all walks them, and X-Total-Count
                        makes both the page size and the total exact. If a page
                        partway through --all fails, the issues already read are
                        still printed and the exit code is 1 -- losing nine good
                        pages over a bad tenth is how a list comes to look
                        complete and empty)
  utils/tea.py show INDEX [--comments]
                       (--comments off answers "comments": null, because an empty
                        list would look like an issue nobody discussed)
  utils/tea.py comment INDEX [--body "..." | < body.md]
  utils/tea.py close INDEX [INDEX...]
  utils/tea.py open INDEX [INDEX...]
                       (a repeated INDEX is written once)
  utils/tea.py edit INDEX [INDEX...] [--title ...] [--body ...]
                          [--close | --open] [--labels a,b] [--remove-labels a,b]
                          (an empty pipe is not a body; a blank --body needs
                           --allow-empty-body; several indexes are for labels
                           and state only -- --title/--body takes one)
  utils/tea.py labels            (the repo's label list, names and ids)

Labels within one name group ("Priority/") are exclusive on this server, so
`edit N --labels Priority/High` on an issue that holds Priority/Low moves the
priority rather than stacking a second one; the label that came off is said on
stderr. `--labels` and `--remove-labels` naming the same label is an error
instead of a coin flip.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import traceback
import urllib.error
import urllib.parse
import urllib.request

# The network is the thing that hangs. No request is allowed to wait forever:
# a hung caller learns nothing, and on a write it cannot even tell whether the
# write happened. TEAWRAP_TIMEOUT exists so a test can watch that path instead
# of waiting half a minute for it.
HTTP_TIMEOUT = int(os.environ.get("TEAWRAP_TIMEOUT", "30"))
# Gitea's own page cap; asking for more gets 50 back and no warning.
SERVER_PAGE_CAP = 50
LS_MAX_PAGES = 200      # guards `ls --all`; 200 x 50 is far past this repo
BODY_MAX_CHARS = 1 << 20  # past this it is not an issue description (/dev/zero, ...)
CONFIG_PATH = os.path.expanduser("~/.config/tea/config.yml")


class Tracker:
    """Server url, token and repo slug, from tea's config and the git remote.

    tea's config is a small YAML document; PyYAML is used when it is installed
    and a two-level scan handles it when it is not, so the wrapper keeps working
    on a machine that has no YAML library.
    """

    def __init__(self):
        base = os.environ.get("TEAWRAP_BASE")
        token = os.environ.get("TEAWRAP_TOKEN")
        if not base or not token:
            url, tok = self._login()
            base = base or url
            token = token or tok
        self.base = base.rstrip("/")
        self.token = token
        self.owner, self.repo = self._slug()

    @staticmethod
    def _login():
        try:
            with open(CONFIG_PATH, encoding="utf-8") as fh:
                text = fh.read()
        except OSError as e:
            fail(f"cannot read {CONFIG_PATH}: {e} -- or point TEAWRAP_BASE and "
                 f"TEAWRAP_TOKEN at a server")
        logins = _logins_from_yaml(text)
        if not logins:
            fail(f"no login in {CONFIG_PATH} -- `tea login add`, or "
                 f"TEAWRAP_BASE and TEAWRAP_TOKEN")
        login = next((l for l in logins if l.get("default")), logins[0])
        if not login.get("url") or not login.get("token"):
            fail(f"{CONFIG_PATH} login {login.get('name')!r} has no url/token")
        return login["url"], login["token"]

    @staticmethod
    def _slug():
        proc = subprocess.run(["git", "remote", "get-url", "origin"],
                              capture_output=True, text=True, timeout=10)
        url = proc.stdout.strip()
        # https://host/owner/repo(.git) or git@host:owner/repo(.git)
        m = re.search(r"[:/]([^/:]+)/([^/:]+?)(?:\.git)?/?$", url)
        if proc.returncode or not m:
            fail("could not read owner/repo from `git remote get-url origin` "
                 f"({url!r}) -- run this inside the repo")
        return m.group(1), m.group(2)

    def call(self, method, path, params=None, payload=None):
        """One API call -> (status, parsed body or None, headers).

        HTTP errors come back as a value rather than an exception: several
        callers need the status and the server's message, and one place that
        formats them is one place that cannot forget the timeout caveat.
        """
        url = f"{self.base}/api/v1{path}"
        if params:
            url += "?" + urllib.parse.urlencode(params)
        data = None
        headers = {"Authorization": f"token {self.token}",
                   "Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers,
                                     method=method)
        try:
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT) as resp:
                return resp.status, _json_or_none(resp.read()), dict(resp.headers)
        except urllib.error.HTTPError as e:
            return e.code, _json_or_none(e.read()), dict(e.headers)
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            # The request may already have reached Gitea, so this is "unknown",
            # never "nothing happened".
            fail(f"{method} {path} got no answer ({e}) -- if that was a write it "
                 f"may have reached the server, so check before retrying it")

    def api(self, method, path, params=None, payload=None, want=(200, 201)):
        """Like call(), but anything unexpected is a hard error."""
        status, body, headers = self.call(method, path, params, payload)
        if status not in want:
            fail(f"{method} {path} -> HTTP {status}: {_why(body)}")
        return body, headers

    def repo_path(self, rest=""):
        return f"/repos/{self.owner}/{self.repo}{rest}"


def _logins_from_yaml(text):
    """The logins: list of tea's config, without assuming PyYAML exists."""
    try:
        import yaml
        data = yaml.safe_load(text) or {}
        logins = data.get("logins") or []
        return [l for l in logins if isinstance(l, dict)]
    except ImportError:
        pass
    logins, current, in_logins = [], None, False
    for line in text.splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        if indent == 0:
            in_logins = line.strip().rstrip(":") == "logins"
            continue
        if not in_logins:
            continue
        stripped = line.strip()
        if stripped.startswith("- "):
            current = {}
            logins.append(current)
            stripped = stripped[2:]
            indent = len(line) - len(line.lstrip("- "))
        elif current is None:
            continue
        m = re.match(r"([A-Za-z0-9_\-]+):\s*(.*)$", stripped)
        if m and current is not None:
            key, val = m.group(1), m.group(2).strip().strip('"')
            current[key] = {"true": True, "false": False, "": None,
                            "null": None}.get(val.lower(), val)
    return logins


def _json_or_none(raw):
    if not raw:
        return None
    try:
        return json.loads(raw.decode("utf-8", "replace"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None


def _why(body):
    """What the server said about an error, in one line, without the body of a
    request that may have been a whole ticket."""
    if isinstance(body, dict):
        for key in ("message", "errors"):
            if body.get(key):
                return str(body[key])[:300]
    return (str(body)[:300] if body else "no message from the server")


_notes = []          # notes this run has printed, so one can be said once
_note_limit = 40     # ... and a run never says more than this many
_notes_dropped = 0


def note(msg):
    """A stderr note, printed at most once per identical message. A note about
    every row of a 450-row listing is a note nobody reads, so past the limit
    they are counted instead and the count is said at the end."""
    global _notes_dropped
    if msg in _notes:
        return
    if len(_notes) >= _note_limit:
        _notes_dropped += 1
        return
    _notes.append(msg)
    print(f"note: {msg}", file=sys.stderr)


def flush_note_count():
    if _notes_dropped:
        print(f"note: {_notes_dropped} more note(s) not printed -- {len(_notes)} "
              f"different ones already were; the JSON answer is unchanged",
              file=sys.stderr)


def fail(msg):
    flush_note_count()
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
    if n > SERVER_PAGE_CAP:
        raise argparse.ArgumentTypeError(
            f"--limit cannot be past the server's page cap of "
            f"{SERVER_PAGE_CAP} -- use --all for the whole list")
    return n


def check_page(n):
    n = int(n)
    if n < 1:
        raise argparse.ArgumentTypeError("--page must be >= 1")
    return n


def check_body(text, what):
    """Refuse locally what tea could not have sent, before touching the network.
    A NUL byte survives reading but not the JSON encode, and a body this size is
    a mistake (or /dev/zero) rather than a description."""
    if "\x00" in text:
        fail(f"{what} contains a NUL byte")
    try:
        text.encode("utf-8")
    except UnicodeEncodeError as e:
        fail(f"{what} cannot be sent as UTF-8: {e}")
    if len(text) > BODY_MAX_CHARS:
        note(f"{what} is {len(text)} characters -- past what a ticket usually "
             f"means; sending it anyway")


def read_pipe():
    # Bounded read: a misconfigured caller streaming /dev/zero would otherwise be
    # swallowed whole before anything says so.
    text = sys.stdin.read(BODY_MAX_CHARS + 1)
    if len(text) > BODY_MAX_CHARS:
        fail(f"piped body is longer than {BODY_MAX_CHARS} characters -- that is "
             f"not a description (a mistaken /dev/zero, a binary file); aborting "
             f"rather than sending it")
    return text


def get_body(args):
    body = args.body
    if body is None and not sys.stdin.isatty():
        body = read_pipe()
    if body is None:
        fail("no body: pass --body or pipe markdown on stdin")
    check_body(body, "body")
    return body


def norm_issue(d, body_chars=True):
    """Gitea's issue object -> the shape this wrapper has always answered.
    `index` not `number`, no db id, author rather than a user object. A shape
    nobody predicted is a JSON error, not a traceback next to a write that
    already ran."""
    if not isinstance(d, dict) or "number" not in d:
        fail(f"the server answered an issue with something else: {str(d)[:160]}")
    out = {"index": d["number"], "state": d.get("state"), "title": d.get("title"),
           "url": d.get("html_url"),
           "author": (d.get("user") or {}).get("login"),
           "labels": [l.get("name") for l in d.get("labels") or []
                      if isinstance(l, dict)],
           "created": d.get("created_at"), "updated": d.get("updated_at")}
    if body_chars:
        out["closed_at"] = d.get("closed_at")
        out["body_chars"] = len(d.get("body") or "")
    return out


def issue_proof(d):
    """What a write answers about an issue: the fields that show the change."""
    return norm_issue(d, body_chars=True)


class Labels:
    """The repo's label list: names to ids and back, fetched once per run.

    Label writes take ids and their own endpoints -- PATCH on this Gitea ignores
    `labels`, `add_labels` and `remove_labels` without saying so, which is the
    same silent-ignore class of bug the tea wrapper used to re-read its way out
    of. Resolving names here also means a typo'd --labels is a hard error before
    anything is written, instead of an exit 0 that changed nothing.
    """

    def __init__(self, tracker):
        self.tracker = tracker
        self._by_name = None
        self._by_id = None

    def by_name(self):
        """name -> the whole label object. The whole object because a label write
        has to know whether a label is exclusive and in which group, and only the
        label itself says so."""
        if self._by_name is None:
            rows, page = [], 1
            while page <= LS_MAX_PAGES:
                part, headers = self.tracker.api(
                    "GET", self.tracker.repo_path("/labels"),
                    {"limit": SERVER_PAGE_CAP, "page": page})
                if not isinstance(part, list):
                    fail(f"the label list came back {type(part).__name__}")
                rows += [l for l in part if isinstance(l, dict)]
                if not part or len(part) < SERVER_PAGE_CAP:
                    break
                page += 1
            self._by_name = {(l.get("name") or ""): l for l in rows
                             if l.get("id") is not None}
            self._by_name.pop("", None)
            total = headers.get("X-Total-Count")
            if total is not None and int(total) != len(self._by_name):
                note(f"the label list answered {len(self._by_name)} of the "
                     f"{total} labels the server counts -- a name past what was "
                     f"read will look unknown")
            if not self._by_name:
                note("the repo has no labels at all, so any --labels is unknown")
        return self._by_name

    def by_id(self):
        if self._by_id is None:
            self._by_id = {l["id"]: l for l in self.by_name().values()}
        return self._by_id

    def ids(self, names, what):
        """Resolve names to ids; unknown names are an error, not a no-op."""
        vocab = self.by_name()
        unknown = [n for n in names if n not in vocab]
        if unknown:
            fail(f"{', '.join(unknown)} not in the repo's label list -- "
                 f"spellings are: {', '.join(sorted(vocab))}")
        return [vocab[n]["id"] for n in names]

    def names(self, ids):
        inverse = {i: n for n, l in self.by_name().items()
                   for i in [l["id"]]}
        return [inverse.get(i, f"id {i}") for i in ids]


def exclusive_group(label):
    """The group an exclusive label belongs to: measured against this server,
    the name before the '/' (Priority/Low and Priority/High fight, Priority/Low
    and Status/Need More Info do not -- and they differ in colour, so colour is
    not the group)."""
    return str(label.get("name") or "").split("/")[0]


def rival_ids(labels, on_issue, want_ids):
    """Labels already on the issue that would block the asked-for ones. An
    exclusive group keeps one member, so `--labels Priority/High` on an issue
    that holds Priority/Low cannot land unless Low comes off in the same write:
    asking the server to hold both loses (measured: it keeps one of them, and
    which one it keeps is not the caller's choice)."""
    if not want_ids:
        return set()
    wanted = [labels.by_id().get(i) for i in want_ids]
    groups = {exclusive_group(l) for l in wanted
              if l and l.get("exclusive")}
    if not groups:
        return set()
    return {l.get("id") for l in on_issue
            if l.get("exclusive") and l.get("id") not in set(want_ids)
            and exclusive_group(l) in groups}


def label_ids_for_edit(tracker, labels, args):
    want = [t.strip() for t in (args.labels or "").split(",") if t.strip()]
    gone = [t.strip() for t in (args.remove_labels or "").split(",")
            if t.strip()]
    if not want and not gone:
        return None, None
    both = sorted(set(want) & set(gone))
    if both:
        # Better to say so than to pick a side: the write would end with the
        # label on, which is not what someone writing both lines meant.
        fail(f"{', '.join(both)} asked for in both --labels and "
             f"--remove-labels -- say which one you mean")
    return labels.ids(want, "--labels"), labels.ids(gone, "--remove-labels")


def cmd_new(args):
    tracker = Tracker()
    labels = Labels(tracker)
    body = get_body(args)
    want = [t.strip() for t in (args.labels or "").split(",") if t.strip()]
    payload = {"title": args.title, "body": body,
               "labels": labels.ids(want, "--labels")}
    if args.milestone is not None:
        payload["milestone"] = milestone_id(tracker, args.milestone)
    # 201 is the create answer and it carries the labels it just set, so this
    # needs no re-read -- the thing the tea version could not do.
    d, _ = tracker.api("POST", tracker.repo_path("/issues"), payload=payload,
                       want=(201,))
    d = norm_issue(d)
    missing = [n for n in want if n not in d["labels"]]
    if missing:
        note(f"created issue {d['index']} without: {', '.join(missing)} -- the "
             f"server took the label list and answered without these")
    print(json.dumps({"ok": True, "index": d["index"],
                      "confirmed": not missing, "issue": d}))


def milestone_id(tracker, title):
    rows, _ = tracker.api("GET", tracker.repo_path("/milestones"),
                          {"state": "all", "limit": SERVER_PAGE_CAP})
    names = [m.get("title") for m in rows or [] if isinstance(m, dict)]
    if title not in names:
        fail(f"milestone {title!r} not in this repo -- milestones: "
             f"{', '.join(n for n in names if n) or '(none)'}")
    return next(m["id"] for m in rows if m.get("title") == title)


def cmd_ls(args):
    tracker = Tracker()
    if args.all and args.page is not None:
        fail("--all and --page are mutually exclusive")
    params = {"state": args.state, "type": "issues", "limit": args.limit}
    if args.keyword:
        params["q"] = args.keyword
    if not args.all:
        params["page"] = args.page or 1
        rows, headers = tracker.api("GET", tracker.repo_path("/issues"), params)
        if not isinstance(rows, list):
            fail(f"the issue list came back {type(rows).__name__}, not a list")
        out = [norm_issue(d, body_chars=False) for d in rows]
        total = headers.get("X-Total-Count")
        page = args.page or 1
        if total is not None:
            total = int(total)
            past = page * args.limit
            if not out and past > total:
                note(f"page {page} is empty -- past the end of {total} matching "
                     f"issues, not an empty repository")
            elif past < total or (page - 1) * args.limit + len(out) < total:
                note(f"{len(out)} of {total} matching issues on this page -- "
                     f"pass --all for the whole list (or --page {page + 1})")
        elif len(rows) >= args.limit:
            note(f"page full at {len(rows)} items and the server gave no total "
                 f"-- pass --all for the whole list")
        print(json.dumps(out))
        return
    out, page = [], 1
    while page <= LS_MAX_PAGES:
        status, rows, headers = tracker.call(
            "GET", tracker.repo_path("/issues"), {**params, "page": page})
        if status != 200 or not isinstance(rows, list):
            why = (f"HTTP {status}: {_why(rows)}" if status != 200
                   else f"came back {type(rows).__name__}, not a list")
            if not out:
                # Nothing read: a hard error, because an empty list here looks
                # like an empty repository.
                fail(f"the issue list did not answer ({why})")
            # Pages already read are real issues. Losing them because a later
            # page failed is the old tea wrapper's bug, so print what was read
            # and let the exit code and the note say that it is short.
            print(json.dumps(out))
            note(f"the list is short: {len(out)} issues from {page - 1} "
                 f"page(s), then page {page} failed ({why}) -- the exit code "
                 f"says these are not all of them")
            flush_note_count()
            sys.exit(1)
        out += [norm_issue(d, body_chars=False) for d in rows]
        total = headers.get("X-Total-Count")
        if len(rows) < args.limit:
            break
        if total is not None and len(out) >= int(total):
            break
        page += 1
    if page > LS_MAX_PAGES:
        note(f"stopped after {LS_MAX_PAGES} pages ({len(out)} items) -- raise "
             f"LS_MAX_PAGES if there really are more")
    print(json.dumps(out))


def cmd_show(args):
    tracker = Tracker()
    d = get_issue(tracker, args.index)
    if args.comments:
        d["comments"] = [comment_out(c) for c in list_comments(tracker,
                                                               args.index)]
    else:
        # An empty list is what a quiet issue answers, so "not asked" has to be
        # said rather than left as [].
        d["comments"] = None
    print(json.dumps(d))


def get_issue(tracker, index):
    """One issue, with the 404 explained: issue and pull request numbers share
    one space, and this endpoint only answers issues."""
    status, body, _ = tracker.call("GET", tracker.repo_path(f"/issues/{index}"))
    if status == 404:
        fail(f"no issue {index} -- the number may be a pull request, or the "
             f"issue may be in another repo (numbers are shared)")
    if status not in (200,):
        fail(f"GET issue {index} -> HTTP {status}: {_why(body)}")
    out = norm_issue(body)
    out["body"] = body.get("body")
    return out


def list_comments(tracker, index):
    out, page = [], 1
    while page <= LS_MAX_PAGES:
        rows, headers = tracker.api("GET",
                                    tracker.repo_path(f"/issues/{index}/comments"),
                                    {"limit": SERVER_PAGE_CAP, "page": page})
        if not isinstance(rows, list):
            fail(f"the comment list came back {type(rows).__name__}, not a list")
        out += rows
        if len(rows) < SERVER_PAGE_CAP:
            break
        page += 1
    return out


def comment_out(c):
    return {"id": c.get("id"), "author": (c.get("user") or {}).get("login"),
            "body": c.get("body"), "created": c.get("created_at")}


def cmd_comment(args):
    tracker = Tracker()
    body = get_body(args)
    # `tea comments add` printed a card and had to be found again to be believed.
    # This answers the comment itself, with the server's id and date, so the
    # proof is the response and an identical older comment cannot be mistaken
    # for ours.
    c, _ = tracker.api("POST", tracker.repo_path(f"/issues/{args.index}/comments"),
                       payload={"body": body}, want=(201,))
    proof = comment_out(c)
    print(json.dumps({"ok": True, "index": args.index,
                      "confirmed": proof["id"] is not None,
                      "comment": proof}))


def _edit_one(tracker, labels, index, args, want_ids, gone_ids):
    """One issue's edit. Returns (proof, ok); a refused write is reported for
    this index rather than ending the run, because a sweep over many issues
    usually has one bad index among good ones, and the good ones have landed.

    Labels are set by reading the issue, working out the set it should end with,
    and PUTting that whole set. Add-then-remove calls would be two calls and are
    wrong here: this repo's Priority/* labels are exclusive (measured live), so
    adding Low to an issue holding High and then removing High ends with neither.
    The end state is then read back, because a label write on this server can
    answer yes without having done it.
    """
    payload = {}
    if args.title is not None:
        payload["title"] = args.title
    if args.body is not None:
        payload["body"] = args.body
    if args.close:
        payload["state"] = "closed"
    elif args.is_open:
        payload["state"] = "open"
    issue = None
    landed = ""
    before_names = None
    rival_names = set()
    if payload:
        status, issue, _ = tracker.call("PATCH",
                                        tracker.repo_path(f"/issues/{index}"),
                                        payload=payload)
        if status == 404:
            note(f"issue {index}: no such issue -- the number may be a pull "
                 f"request, or the issue may be in another repo (numbers are "
                 f"shared)")
            return None, False
        # 201 for an update, measured on this server: it is Gitea saying "here is
        # the object", not a second create.
        if status not in (200, 201):
            note(f"issue {index}: the title/body/state write was refused "
                 f"(HTTP {status}: {_why(issue)})")
            return None, False
        landed = ", ".join(k for k in ("title", "body", "state") if k in payload)
    if want_ids or gone_ids:
        status, current, _ = tracker.call("GET",
                                          tracker.repo_path(f"/issues/{index}"))
        if status != 200:
            note(f"issue {index}: could not read its labels back (HTTP {status}: "
                 f"{_why(current)})"
                 + (f" -- the {landed} part did land" if landed else ""))
            # Even so, say what the part that did land looks like: "nothing
            # happened" would be a lie about a state change the caller made.
            return proof_of(_read_issue(tracker, index) if landed else None), False
        on_issue = [l for l in current.get("labels") or [] if isinstance(l, dict)]
        before_names = {l.get("name") for l in on_issue}
        rivals = rival_ids(labels, on_issue, want_ids)
        # Dropped on purpose so the asked-for label can land; not a surprise.
        rival_names = {labels.names([i])[0] for i in rivals}
        have_ids = {l.get("id") for l in on_issue}
        final = sorted((have_ids - set(gone_ids) - rivals) | set(want_ids))
        if rivals:
            note(f"issue {index}: {', '.join(sorted(rival_names))} came off to "
                 f"let the asked-for label in -- one label per group")
        status, answered, _ = tracker.call(
            "PUT", tracker.repo_path(f"/issues/{index}/labels"),
            payload={"labels": final})
        # 200 answers the label list; 204 means yes-with-no-body (both seen from
        # this server), and the read at the end is what is believed either way.
        if status not in (200, 204):
            note(f"issue {index}: setting labels refused (HTTP {status}: "
                 f"{_why(answered)})"
                 + (f" -- the {landed} part did land" if landed else ""))
            return proof_of(_read_issue(tracker, index) if landed else None), False
    # What an edit did is believed from a read, not from the write answers: this
    # server has answered a label write with yes and nothing done (measured).
    fresh = _read_issue(tracker, index)
    if fresh is None:
        note(f"issue {index}: took the write but would not read back -- not "
             f"confirming")
        return proof_of(issue), False
    proof = issue_proof(fresh)
    ok = True
    if args.title is not None and proof["title"] != args.title:
        note(f"issue {index} is titled {proof['title']!r}, not {args.title!r}")
        ok = False
    want_state = "closed" if args.close else "open" if args.is_open else None
    if want_state and proof["state"] != want_state:
        note(f"issue {index} is {proof['state']!r} after the write meant to "
             f"make it {want_state!r}")
        ok = False
    have = set(proof["labels"])
    want_names = set(labels.names(want_ids or []))
    gone_names = set(labels.names(gone_ids or []))
    missing = sorted(want_names - have)
    leftover = sorted(gone_names & have)
    if missing:
        note(f"issue {index}: not on the issue after the write: "
             f"{', '.join(missing)}")
        ok = False
    if leftover:
        note(f"issue {index}: still on the issue after --remove-labels: "
             f"{', '.join(leftover)}")
        ok = False
    if before_names is not None:
        # A label that came off without being asked for and without being a rival
        # of something asked for: the ask landed, so this is not a failure, but
        # it is the server doing something the caller did not write out.
        lost = sorted(before_names - have - gone_names - rival_names)
        if lost:
            note(f"issue {index}: came off the issue although it was not asked "
                 f"for: {', '.join(lost)}")
    return proof, ok


def _read_issue(tracker, index):
    """An issue read that returns None instead of ending the run, for the paths
    that run after something was already written: losing the proof of a change
    that landed is worse than an uncertain answer, and a sweep over 40 indexes
    must not die on the 41st read."""
    status, body, _ = tracker.call("GET", tracker.repo_path(f"/issues/{index}"))
    if status != 200 or not isinstance(body, dict) or "number" not in body:
        return None
    return body


def proof_of(issue):
    """The short answer shape this tool prints, or None for 'nothing to show'."""
    if not isinstance(issue, dict) or "number" not in issue:
        return None
    return issue_proof(issue)


def _edit_many(tracker, labels, indexes, args, want_ids, gone_ids):
    """Apply the same change to several indexes. The server answers each one, so
    the proof is per index and a bad index does not hide what the good ones did.
    A total failure is an error; a partial one is an answer with
    confirmed:false, because "nothing happened" would be a lie."""
    proofs, oks = [], []
    for index in indexes:
        proof, ok = _edit_one(tracker, labels, index, args, want_ids, gone_ids)
        proofs.append(proof)
        oks.append(ok)
    if not any(oks) and all(p is None for p in proofs):
        fail("no index ended up with the change -- see the notes above, and "
             "note that part of an edit can land even when the whole of it did "
             "not")
    print(json.dumps({"ok": True, "indexes": indexes,
                      "confirmed": all(oks), "issues": proofs}))


def cmd_edit(args):
    tracker = Tracker()
    labels = Labels(tracker)
    # Piped stdin is a body even without --body, same as new/comment -- but an
    # EMPTY pipe means "no body given", not "clear the description". Devnull and
    # empty pipes are what a caller uses when it only wants labels; taking them
    # as a blank body once wiped the descriptions of 49 issues. A blank --body is
    # the same write wearing a different hat (usually a "$(cat typoed.path)" that
    # expanded to nothing), so it needs its own opt-in.
    body = args.body
    if body is None and not sys.stdin.isatty():
        piped = read_pipe()
        if piped.strip():
            if body is not None:
                fail("body given twice: pass --body or pipe markdown, not both")
            body = piped
            check_body(body, "body")
            note("sending piped stdin as the new description")
    if body is not None and not body.strip() and not args.allow_empty_body:
        fail("blank --body would clear the description -- pass "
             "--allow-empty-body if that is really what you mean")
    if args.close and args.is_open:
        fail("--close and --open contradict each other")
    if not any([args.title is not None, body is not None, args.close,
                args.is_open, args.labels, args.remove_labels]):
        fail("nothing to change: pass --title/--body/--close/--open/--labels")
    indexes = list(dict.fromkeys(args.indexes))
    if len(indexes) > 1 and (args.title is not None or body is not None):
        fail("--title/--body take one index -- several indexes are for "
             "--labels/--remove-labels/--close/--open (piped stdin counts as a "
             "--body)")
    if args.title is not None and not args.title.strip():
        note("--title \"\" asks Gitea to unset the title, which it may not "
             "accept")
    args.body = body
    want_ids, gone_ids = label_ids_for_edit(tracker, labels, args)
    _edit_many(tracker, labels, indexes, args, want_ids, gone_ids)


def cmd_state(cmd, args):
    tracker = Tracker()
    labels = Labels(tracker)
    indexes = list(dict.fromkeys(args.indexes))   # order kept, each written once
    args.title = args.body = args.labels = args.remove_labels = None
    args.close = cmd == "close"
    args.is_open = cmd == "open"
    args.allow_empty_body = True
    _edit_many(tracker, labels, indexes, args, None, None)


def cmd_labels(args):
    tracker = Tracker()
    labels = Labels(tracker)
    # exclusive matters to whoever picks the label: within one name group only
    # one of these can be on an issue at a time.
    print(json.dumps([{"name": n, "id": l["id"], "exclusive": bool(l.get("exclusive"))}
                      for n, l in sorted(labels.by_name().items())]))


class JsonParser(argparse.ArgumentParser):
    """argparse would print usage on stderr and exit 2; keep the one contract
    this wrapper has with its callers: every failure is a JSON object."""

    def error(self, message):
        fail(message)


def main():
    p = JsonParser(description="Gitea issue tracker wrapper (JSON in, JSON out)")
    sp = p.add_subparsers(dest="cmd", required=True)

    new = sp.add_parser("new", help="create an issue")
    new.add_argument("--title", required=True)
    new.add_argument("--body", help="markdown; or pipe it on stdin")
    new.add_argument("--labels", help="comma-separated label names")
    new.add_argument("--milestone", help="milestone title")
    new.set_defaults(fn=cmd_new)

    ls = sp.add_parser("ls", help="list issues")
    ls.add_argument("--state", default="open",
                    choices=["open", "closed", "all"])
    ls.add_argument("--keyword", help="search title and description")
    ls.add_argument("--limit", type=check_limit, default=30)
    ls.add_argument("--page", type=check_page, default=None)
    ls.add_argument("--all", action="store_true", help="walk every page")
    ls.set_defaults(fn=cmd_ls)

    show = sp.add_parser("show", help="one issue, body included")
    show.add_argument("index", type=check_index)
    show.add_argument("--comments", action="store_true")
    show.set_defaults(fn=cmd_show)

    cm = sp.add_parser("comment", help="add a comment")
    cm.add_argument("index", type=check_index)
    cm.add_argument("--body", help="markdown; or pipe it on stdin")
    cm.set_defaults(fn=cmd_comment)

    for name, fn in (("close", cmd_state), ("open", cmd_state)):
        s = sp.add_parser(name, help=f"{name} one or more issues")
        s.add_argument("indexes", nargs="+", type=check_index)
        s.set_defaults(fn=fn, cmd=name)

    ed = sp.add_parser("edit", help="change title/body/state/labels")
    ed.add_argument("indexes", nargs="+", type=check_index)
    ed.add_argument("--title")
    ed.add_argument("--body", help="markdown; or pipe it on stdin")
    ed.add_argument("--allow-empty-body", action="store_true",
                    help="let --body \"\" clear the description")
    ed.add_argument("--labels", help="comma-separated label names to add")
    ed.add_argument("--remove-labels", help="comma-separated label names to drop")
    ed.add_argument("--close", action="store_true")
    ed.add_argument("--open", dest="is_open", action="store_true")
    ed.set_defaults(fn=cmd_edit, cmd="edit")

    lb = sp.add_parser("labels", help="the repo's labels, with ids")
    lb.set_defaults(fn=cmd_labels)

    args = p.parse_args()
    if args.cmd == "close" or args.cmd == "open":
        args.fn(args.cmd, args)
    else:
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
        # shape nobody predicted should not be able to break it with a traceback
        # next to a write that already landed: the traceback goes to stderr where
        # it is still readable, and a caller parsing stdout gets an error object.
        traceback.print_exc(file=sys.stderr)
        print(json.dumps({"error": f"utils/tea.py crashed: "
                                   f"{type(e).__name__}: {e}"}))
        flush_note_count()
        sys.exit(1)
