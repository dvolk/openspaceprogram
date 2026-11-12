#!/usr/bin/env python3
"""Fetch the raw asteroid sources into utils/rss/asteroids/raw/ and stop.

This is the ONLY thing in the set that touches the network, and it never refetches
what it already has (cf. utils/rss/html/ for the planets): once raw/ is populated,
parse.py derives the dataset offline, so the corpus is reviewable in a diff and a
laptop on a train can regenerate res/systems.

Sources (also in urls.txt):
  https://en.wikipedia.org/wiki/List_of_exceptional_asteroids
      -> raw/exceptional_asteroids.html   (physical character: diameter, dimensions,
         mass + its stated precision, rotation, spectral type, class, discovery)
  https://ssd-api.jpl.nasa.gov/sbdb.api?sstr=<obj>&discovery=1
      -> raw/sbdb/<slug>.json             (orbital elements at an epoch, spkid, orbit
         class, discovery block -- the rail data the wiki table does not carry)
  https://minorplanetcenter.net/Extended_Files/mpcorb_extended.json.gz
      -> raw/mpcorb/mpcorb_extended.json.gz  (1.58M orbits WITH absolute magnitude H,
         the uncertainty parameter U, arc/opposition counts and a dynamical class.
         NOT COMMITTED -- 175 MB; the extract lands in asteroids.json. See mporb.py.)
  https://en.wikipedia.org/wiki/List_of_Solar_System_objects_by_size
      -> raw/objects_by_size.html         (stated radius + mass per body, split by
         class, for the bodies the exceptional list leaves size-less: the dwarf
         planets, the big TNOs, the Trojans, the centaurs. See sizes.py.)
  https://en.wikipedia.org/wiki/List_of_trans-Neptunian_objects
      -> raw/tno_list.html                (~1050 rows of D (km) plus a, e, i, q, Q --
         the Kuiper belt population, which the other four sources size almost not at
         all). See sizes.tno_rows().

JPL's SBDB API has NO physical parameters (probe: ?phys_par=1 returns an error
object), so the two sources are complementary rather than redundant -- and neither
one alone can build a body. See parse.py's report for what is still missing.

MPCORB's own header asks that the datafile be redistributed with its header and with
attribution to the Minor Planet Center. We commit only a per-row extract of orbital
facts, and every consumer of it says where it came from.
"""
import argparse
import json
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mporb  # noqa: E402  -- knows where the MPCORB cache lives
import parse  # noqa: E402  -- the offline parser, reused here to list what to fetch

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = parse.RAW
SBDB_DIR = parse.SBDB_DIR

WIKI_URL = "https://en.wikipedia.org/wiki/List_of_exceptional_asteroids"
SIZES_URL = "https://en.wikipedia.org/wiki/List_of_Solar_System_objects_by_size"
TNO_URL = "https://en.wikipedia.org/wiki/List_of_trans-Neptunian_objects"
MPCORB_URL = "https://minorplanetcenter.net/Extended_Files/mpcorb_extended.json.gz"
SBDB_URL = "https://ssd-api.jpl.nasa.gov/sbdb.api"
# JPL's SSD-API terms of use ask for a User-Agent that says who is calling.
USER_AGENT = "openspaceprogram-rss/1.0 (game world data; https://git.oxfordfun.com/dvolk/openspaceprogram)"
SBDB_DELAY_S = 1.1   # one call per object; stay polite to a shared NASA service


def get(url, path, refresh, binary=True):
    """Fetch url into path unless path already exists (or refresh). Returns (bytes, fetched)."""
    if os.path.exists(path) and not refresh:
        with open(path, "rb") as f:
            return f.read(), False
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            body = resp.read()
    except urllib.error.HTTPError as e:
        raise SystemExit("fetch: %s -> HTTP %d: %s" % (url, e.code, e.reason))
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        raise SystemExit("fetch: %s -> %s" % (url, e))
    if len(body) < 200:
        raise SystemExit("fetch: %s -> %d bytes, too small to be the page" % (url, len(body)))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(body)
    return body, True


def fetch_wiki(refresh):
    body, got = get(WIKI_URL, parse.WIKI_HTML, refresh)
    print("%s %s (%d bytes)" % ("fetched" if got else "cached ", WIKI_URL, len(body)))
    return body.decode("utf-8", "replace")


def fetch_size_tables(refresh):
    """The two wiki tables that carry a SIZE for bodies the first page leaves blank."""
    for url, path in ((SIZES_URL, parse.SIZES_HTML), (TNO_URL, parse.TNO_HTML)):
        body, got = get(url, path, refresh)
        print("%s %s (%d bytes)" % ("fetched" if got else "cached ", url, len(body)))


def fetch_mpcorb(refresh):
    """The 175 MB MPCORB extended JSON. Cached, gitignored, and only read by mporb.py,
    which streams it -- nothing else in the tree ever holds it in memory."""
    body, got = get(MPCORB_URL, mporb.MPCORB_GZ, refresh)
    print("%s %s (%d bytes)" % ("fetched" if got else "cached ", MPCORB_URL, len(body)))


def sbdb_candidates(query):
    """Queries to try for one wiki label. SBDB accepts "1 Ceres" but REJECTS
    "(127546) 2002 XU93" ("one more more unexpected characters detected" -- the
    parenthesised number is only allowed with a named body), so fall back to the
    bare number and then the bare designation."""
    out = [query]
    m = re.match(r"^\(?\s*(\d+)\s*\)?\s*(.*)$", query.strip())
    if m:
        number, rest = m.group(1), m.group(2).strip()
        if rest:
            out.append(rest)          # "2002 XU93" / "Ceres"
        out.append(number)            # "127546"
    seen, uniq = set(), []
    for q in out:
        if q and q not in seen:
            seen.add(q)
            uniq.append(q)
    return uniq


def fetch_sbdb(obj, refresh):
    """One SBDB record, cached at raw/sbdb/<slug-of-wiki-label>.json.

    Returns (dict, fetched, resolved_query); (None, False, None) if every candidate
    query failed or every answer was a DIFFERENT object -- a miss is reported and the
    run continues, because one bad row in a 259-object sweep should not lose the other
    258. Rejecting a wrong answer here rather than logging it later is the point: the
    cache file is named for the wiki row, so a wrong object in it is a trap that outlives
    the run.
    """
    query = obj["query"]
    slug = parse.slug(query)
    path = os.path.join(SBDB_DIR, slug + ".json")
    if os.path.exists(path) and not refresh:
        with open(path, encoding="utf-8") as f:
            return json.load(f), False, query
    last = None
    for q in sbdb_candidates(query):
        url = SBDB_URL + "?" + urllib.parse.urlencode({"sstr": q, "discovery": "1"})
        try:
            body, _ = get(url, path, False)
        except SystemExit as e:      # get() raises SystemExit on HTTP/network error
            last = "%s" % e
            time.sleep(SBDB_DELAY_S)
            continue
        try:
            data = json.loads(body)
        except ValueError as e:
            last = "%s is not JSON (%s)" % (url, e)
            os.remove(path)
            continue
        if "object" not in data:
            # SBDB answers a bad query with {"message":...,"code":"..."}, and get()
            # has already written it, so a typo would otherwise sit in the cache
            # forever.
            os.remove(path)
            last = "%s -> %s" % (url, data.get("message", data))
            continue
        if not parse.sbdb_identity_ok(obj, data["object"]):
            os.remove(path)
            last = ("%s -> %s (%s), which is not %s"
                    % (q, data["object"].get("des"), data["object"].get("fullname"), query))
            continue
        return data, True, q
    print("MISS %-28s %s" % (query, last))
    return None, False, None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--refresh", action="store_true",
                    help="re-download even what is already cached")
    ap.add_argument("--only", metavar="A,B",
                    help="just these SBDB queries (wiki row names/designations)")
    ap.add_argument("--limit", type=int, metavar="N",
                    help="fetch at most N SBDB records")
    ap.add_argument("--skip-sbdb", action="store_true", help="only the wiki page")
    ap.add_argument("--skip-sizes", action="store_true",
                    help="skip the two size tables (objects-by-size, TNO list)")
    ap.add_argument("--skip-mporb", action="store_true",
                    help="skip the 175 MB MPCORB download")
    ap.add_argument("--list", action="store_true",
                    help="print the object list and exit (no SBDB calls)")
    args = ap.parse_args()

    html = fetch_wiki(args.refresh)
    objs = parse.wiki_objects(html)
    print("wiki tables name %d distinct bodies" % len(objs))

    if args.list:
        for o in objs:
            print("%-28s %s" % (o["query"], ",".join(o["sections"])))
        return 0

    if not args.skip_sizes:
        fetch_size_tables(args.refresh)
    if not args.skip_mporb:
        fetch_mporb(args.refresh)
    if args.skip_sbdb:
        return 0

    wanted = None
    if args.only:
        wanted = {s.strip() for s in args.only.split(",") if s.strip()}
        known = {o["query"] for o in objs}
        unknown = sorted(wanted - known)
        if unknown:
            print("note: not named by the wiki page, querying anyway: %s" % ", ".join(unknown))
            objs += [{"query": q, "sections": ["cli"]} for q in unknown]

    got = cached = 0
    misses = []
    for i, o in enumerate(objs):
        if wanted is not None and o["query"] not in wanted:
            continue
        if args.limit is not None and got + cached >= args.limit:
            print("--limit %d reached" % args.limit)
            break
        data, fetched, used = fetch_sbdb(o, args.refresh)
        if data is None:
            misses.append(o["query"])
            continue
        got += fetched
        cached += not fetched
        obj = data["object"]
        if not parse.sbdb_identity_ok(o, obj):
            # Only reachable for a cache file written before the guard existed.
            print("MISMATCH %-28s SBDB des %s (%s)"
                  % (o["query"], obj.get("des"), obj.get("fullname")))
        print("%-28s %-8s %s%s" % (o["query"], obj.get("spkid", "?"),
                                   obj.get("fullname", "?"),
                                   "" if used == o["query"] else "  [via %s]" % used))
        if fetched:
            time.sleep(SBDB_DELAY_S)
    print("sbdb: %d fetched, %d already cached, %d missed" % (got, cached, len(misses)))
    if misses:
        print("missed: %s" % ", ".join(misses))
    return 0


if __name__ == "__main__":
    sys.exit(main())
