#!/usr/bin/env bash
# Build a binary .deb for osp into dist/ -- the .deb counterpart of make_appimage.sh.
#   ./utils/make_deb.sh [DISTDIR] [OSP_BIN]
# Version comes from src/version.h (make version / git describe), so it matches `osp --version`.
set -euo pipefail
cd "$(dirname "$0")/.."

DISTDIR=${1:-dist}
OSP_BIN=${2:-build/linux-v2-znver3/release/osp}

# dpkg-buildpackage needs debian/ at the source root; copy from utils/ and drop on exit.
cleanup() { rm -rf debian; }
trap cleanup EXIT
rm -rf debian
cp -a utils/debian debian

# Debian allows one '-'; map the rest to '~': v0.0.1-38-gX -> 0.0.1~38~gX-1.
VER=$(sed -n 's/^#define VERSION "\(.*\)"/\1/p' src/version.h)
if [ -z "$VER" ]; then
    echo "error: no version string (src/version.h missing? run 'make version' first)" >&2
    exit 1
fi
case "$VER" in *-dirty*) echo "note: version '$VER' marks a dirty tree" >&2;; esac
# sed, not bash ${var//-/~} -- that expands '~' to $HOME and mangles the version.
DEBVER=$(printf '%s' "$VER" | sed -e 's/^v//' -e 's/-/~/g')
DEBVER="${DEBVER}-1"

# Refresh the changelog's first entry (dpkg-buildpackage reads the version from it).
{
    echo "openspaceprogram (${DEBVER}) unstable; urgency=low"
    echo
    echo "  * Initial Debian packaging."
    echo
    echo " -- Denis Volk <denis.volk@gmail.com>  $(date -R)"
} > debian/changelog

# Binary-only, unsigned. OSP_BIN via env (rules uses ?=); artifacts land in the parent dir.
export OSP_BIN
ARCH=$(dpkg --print-architecture)
dpkg-buildpackage -us -uc -b
BASE="../openspaceprogram_${DEBVER}_${ARCH}"
# Collect .deb + dpkg byproducts (dbgsym, .changes, .buildinfo) into dist/.
for kind in .deb .ddeb .changes .buildinfo; do
    if [ -f "${BASE}${kind}" ]; then
        mkdir -p "$DISTDIR"
        mv "${BASE}${kind}" "$DISTDIR/"
    fi
done

ls -lh "$DISTDIR/openspaceprogram_${DEBVER}_${ARCH}.deb"
