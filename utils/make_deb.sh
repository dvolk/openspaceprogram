#!/usr/bin/env bash
# Build a binary .deb for osp and land it in dist/ (gitignored, uploaded
# manually) -- the .deb counterpart of make_appimage.sh.
#
#   ./utils/make_deb.sh [DISTDIR] [OSP_BIN]
#     DISTDIR  where the .deb (and byproducts) land (default: dist)
#     OSP_BIN  the release binary to package (default: the v2/znver3 build).
#              The Makefile's `deb` target passes $(LINUX_BIN), so a non-
#              default MARCH/MTUNE build isn't silently packaged stale.
#
# The Debian version is derived from the built game's version string
# (src/version.h, set by `make version` from git describe), so the .deb
# version always matches `osp --version`. dpkg-buildpackage reads the
# version from debian/changelog first, so regenerate it before building.
set -euo pipefail
cd "$(dirname "$0")/.."

DISTDIR=${1:-dist}
OSP_BIN=${2:-build/linux-v2-znver3/release/osp}

# The canonical packaging lives in utils/debian/ (kept out of the repo root).
# dpkg-buildpackage needs debian/ at the source root, so copy it in for the
# build and remove it on exit (success, error, or Ctrl-C) -- tree stays clean.
cleanup() { rm -rf debian; }
trap cleanup EXIT
rm -rf debian
cp -a utils/debian debian

# 1. Derive a valid Debian version from the game's version string.
#    git describe gives e.g. "v0.0.1-38-g35f4de0". Debian versions allow at
#    most one '-' (the revision separator) and use '~' to mean "sorts before",
#    so: strip a leading 'v', turn every '-' into '~', append revision '-1'
#    -> 0.0.1~38~g35f4de0-1.
VER=$(sed -n 's/^#define VERSION "\(.*\)"/\1/p' src/version.h)
if [ -z "$VER" ]; then
    echo "error: no version string (src/version.h missing? run 'make version' first)" >&2
    exit 1
fi
case "$VER" in *-dirty*) echo "note: version '$VER' marks a dirty tree" >&2;; esac
# sed does no tilde expansion, so '~' stays literal (bash's ${var//-/~} would
# expand the '~' to $HOME and mangle the version).
DEBVER=$(printf '%s' "$VER" | sed -e 's/^v//' -e 's/-/~/g')
DEBVER="${DEBVER}-1"

# 2. Regenerate the changelog's first entry with that version + today's date
#    (the committed entry is a snapshot; this keeps the .deb in sync with git).
{
    echo "openspaceprogram (${DEBVER}) unstable; urgency=low"
    echo
    echo "  * Initial Debian packaging."
    echo
    echo " -- Denis Volk <denis.volk@gmail.com>  $(date -R)"
} > debian/changelog

# 3. Build the binary .deb (binary-only, unsigned). The binary to package is
#    handed to debian/rules via OSP_BIN (rules uses `?=`, so the env wins).
#    dpkg-buildpackage emits the artifacts in the PARENT of the source dir
#    (the repo's parent); their names are deterministic from the version + arch.
export OSP_BIN
ARCH=$(dpkg --print-architecture)
dpkg-buildpackage -us -uc -b
BASE="../openspaceprogram_${DEBVER}_${ARCH}"
# Move the .deb + the dpkg byproducts (dbgsym .ddeb, .changes, .buildinfo)
# into dist/ rather than leaving them in the user's home dir or deleting them.
for kind in .deb .ddeb .changes .buildinfo; do
    if [ -f "${BASE}${kind}" ]; then
        mkdir -p "$DISTDIR"
        mv "${BASE}${kind}" "$DISTDIR/"
    fi
done

# 4. Report.
ls -lh "$DISTDIR/openspaceprogram_${DEBVER}_${ARCH}.deb"
