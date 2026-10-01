#!/usr/bin/env python3
# Scrape the KSP wiki Parts page into one CSV per part category table.
# Fixes <img> unit icons and <figure> part-image cells before pandas.read_html;
# page is cached at tmp/parts_wiki.html. Output: <out>/<heading>.csv + index.json.
import argparse
import io
import json
import os
import re
import urllib.request

import pandas as pd
from lxml import etree
from lxml import html

URL = "https://wiki.kerbalspaceprogram.com/wiki/Parts"
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CACHE = os.path.join(REPO, "tmp", "parts_wiki.html")
OUT = os.path.join(HERE, "parts_wiki")


def load_html(html_arg, refresh):
    if html_arg:
        with open(html_arg, encoding="utf-8") as f:
            return f.read()
    if not refresh and os.path.exists(CACHE):
        with open(CACHE, encoding="utf-8") as f:
            return f.read()
    body = urllib.request.urlopen(URL, timeout=60).read()
    os.makedirs(os.path.dirname(CACHE), exist_ok=True)
    with open(CACHE, "wb") as f:
        f.write(body)
    with open(CACHE, encoding="utf-8") as f:
        return f.read()


def replace_with_text(parent, el, text):
    i = parent.index(el)
    parent.remove(el)
    span = etree.Element("span")
    span.text = text
    parent.insert(i, span)


def prep(doc):
    """Fix wiki quirks in the DOM; return (serialized html, per-table meta)."""
    hstack = {2: "", 3: "", 4: ""}
    tables = []
    for el in doc.iter():
        tag = el.tag
        if tag in ("h1", "h2", "h3", "h4"):
            lvl = int(tag[1])
            hstack[lvl] = el.text_content().strip()
            for l in range(lvl + 1, 5):
                hstack[l] = ""
        elif tag == "table":
            path = " > ".join(h for h in (hstack[2], hstack[3], hstack[4]) if h)
            is_part = any(c.text_content().strip() == "Part"
                          for c in el.xpath(".//th"))
            note = None
            trs = el.xpath(".//tr")
            if trs:
                c = trs[0].xpath("./*")[0]
                if c.tag == "th" and int(c.get("colspan") or 1) >= 3:
                    txt = re.sub(r"\s+", " ", c.text_content()).strip()
                    if " " in txt:    # a sentence, not a column group
                        note = txt
                        c.text = ""
                        for child in list(c):
                            c.remove(child)
            tables.append({"path": path, "is_part": is_part, "note": note})

    for fig in doc.xpath("//figure"):
        name = None
        a = fig.xpath(".//a[contains(@href, '/wiki/File:')]")
        if a:
            name = a[0].get("href").split("/wiki/File:")[-1]
        else:
            src = fig.xpath(".//img/@src")
            if src:
                name = src[0].split("/thumb/")[-1].split("/")[0]
        if name:
            replace_with_text(fig.getparent(), fig, name)
        else:
            fig.getparent().remove(fig)

    for img in doc.xpath("//img[@alt]"):
        replace_with_text(img.getparent(), img, img.get("alt"))

    return html.tostring(doc, encoding="unicode"), tables


def slugify(s):
    s = s.lower().replace("&", " and ")
    return re.sub(r"[^a-z0-9]+", "_", s).strip("_")


def flatten(cols):
    """MultiIndex columns -> 'Group / leaf' names, wiki footnote refs dropped."""
    out, seen = [], set()
    for c in cols:
        parts = [str(x) for x in c] if isinstance(c, tuple) else [str(c)]
        parts = [p for p in parts if not p.startswith("Unnamed")]
        parts = list(dict.fromkeys(parts))
        name = re.sub(r"\s*\[Note ?\d+\]|\[\d+\]", "", " / ".join(parts) or "Unnamed")
        name = re.sub(r"\s+", " ", name).strip()
        base, i = name, 2
        while name in seen:
            name = f"{base}_{i}"
            i += 1
        seen.add(name)
        out.append(name)
    return out


def clean_cell(v):
    if v is None or (isinstance(v, float) and pd.isna(v)):
        return ""
    s = str(v)
    for ch in ("\u2009", "\u202f", "\xa0"):
        s = s.replace(ch, " ")
    s = re.sub(r"\s*\[Note ?\d+\]", "", s)
    s = re.sub(r"(?<=\S)\[\d+\]", "", s)
    return re.sub(r"\s+", " ", s).strip()


def main():
    ap = argparse.ArgumentParser(
        description="Scrape the KSP wiki's Parts page into one CSV per part table")
    ap.add_argument("--refresh", action="store_true",
                    help="re-download the page even if the cache exists")
    ap.add_argument("--html", help="parse this local HTML file instead of the cache")
    ap.add_argument("--out", default=OUT, help=f"output directory (default {OUT})")
    args = ap.parse_args()

    doc = html.fromstring(load_html(args.html, args.refresh))
    ser, tables = prep(doc)
    frames = pd.read_html(io.StringIO(ser))
    assert len(frames) == len(tables), f"table count mismatch: {len(frames)} != {len(tables)}"

    os.makedirs(args.out, exist_ok=True)
    part = [(t, df) for t, df in zip(tables, frames) if t["is_part"]]
    index = {}
    for i, (t, df) in enumerate(part, 1):
        cols = flatten(df.columns)
        data = df.map(clean_cell)
        data.columns = cols
        if "Part" in cols:
            data = data[data["Part"].astype(str).str.strip() != ""]
        fname = "__".join(slugify(seg) for seg in t["path"].split(" > ")) + ".csv"
        data.to_csv(os.path.join(args.out, fname), index=False)
        index[fname] = {"heading": t["path"], "note": t["note"],
                        "rows": len(data), "columns": cols}
        print(f"[{i:2d}/{len(part)}] {fname:55s} {len(data):3d} rows"
              + (f"  note: {t['note']}" if t["note"] else ""))

    with open(os.path.join(args.out, "index.json"), "w") as f:
        json.dump(index, f, indent=2)
    print(f"\nwrote {len(part)} CSVs + index.json to {args.out}")


if __name__ == "__main__":
    main()
