#!/usr/bin/env bash
# Stage an FHS AppDir for osp and pack it with appimagetool.
#
#   ./utils/make_appimage.sh <LINUX_BIN> <VERSION> <DISTDIR> [APPIMAGETOOL]
#
# AppDir layout (resdir::root() walks up from usr/bin and finds
# usr/share/openspaceprogram/res/):
#   AppDir/
#     AppRun -> usr/bin/osp
#     openspaceprogram.desktop
#     openspaceprogram.png
#     usr/bin/osp
#     usr/share/openspaceprogram/res/
#     usr/share/doc/{LICENSE.md,README.md}
#
# No library bundling (libGL / X11 / Pulse / libstdc++ all host) -- see the
# Makefile's appimage target for why.
set -euo pipefail

LINUX_BIN=${1:?usage: make_appimage.sh LINUX_BIN VERSION DISTDIR [APPIMAGETOOL]}
VER=${2:?}
DISTDIR=${3:?}
APPIMAGETOOL=${4:-tmp/appimagetool-x86_64.AppImage}

if [ ! -x "$LINUX_BIN" ]; then
    echo "error: game binary not found/executable: $LINUX_BIN" >&2
    exit 1
fi
if [ ! -x "$APPIMAGETOOL" ]; then
    echo "fetching appimagetool -> $APPIMAGETOOL" >&2
    mkdir -p "$(dirname "$APPIMAGETOOL")"
    curl -fsSL -o "$APPIMAGETOOL" \
        https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage
    chmod +x "$APPIMAGETOOL"
fi

STAGE=tmp/appimage/AppDir
rm -rf tmp/appimage
mkdir -p "$STAGE/usr/bin" "$STAGE/usr/share/openspaceprogram" "$STAGE/usr/share/doc"
cp "$LINUX_BIN" "$STAGE/usr/bin/osp"
cp -r res "$STAGE/usr/share/openspaceprogram/res"
cp LICENSE.md release/README.md "$STAGE/usr/share/doc/"
cp release/openspaceprogram.desktop "$STAGE/"
cp release/openspaceprogram.png "$STAGE/"
ln -s usr/bin/osp "$STAGE/AppRun"
chmod +x "$STAGE/usr/bin/osp" "$STAGE/AppRun"

mkdir -p "$DISTDIR"
OUT="$DISTDIR/osp-$VER-x86_64.AppImage"
rm -f "$OUT"
# APPIMAGE_EXTRACT_AND_RUN: no FUSE needed (containers / many CI images).
ARCH=x86_64 APPIMAGE_EXTRACT_AND_RUN=1 VERSION="$VER" \
    "$(pwd)/$APPIMAGETOOL" "$STAGE" "$OUT"
ls -lh "$OUT"
