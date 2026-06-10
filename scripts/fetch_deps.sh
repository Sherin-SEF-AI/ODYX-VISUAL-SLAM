#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# ODYX :: fetch native dependencies
#
# Pulls pinned, PERMISSIVELY-LICENSED source trees into
#   app/src/main/cpp/third_party/
# and (optionally) the official OpenCV Android SDK into third_party_download/.
#
# Dependencies & licenses (see README "LICENSE INVENTORY"):
#   - Eigen   3.4.0   MPL2   (header-only)
#   - GTSAM   4.2.0   BSD-3
#   - Boost  1.84.0   BSL-1.0 (header subset GTSAM needs; bcp-style copy)
#   - DBoW3  master   BSD-3
#   - RTKLIB 2.4.3-b34 BSD-2
#   - OpenCV 4.10.0   Apache-2.0 (official prebuilt Android SDK; NOT built here)
#
# NO GPL/copyleft VI-SLAM code is fetched. We reimplement algorithms.
#
# Usage:
#   scripts/fetch_deps.sh            # fetch source deps (eigen, gtsam, boost, dbow3, rtklib)
#   scripts/fetch_deps.sh --opencv   # ALSO download + unpack the OpenCV Android SDK
#   scripts/fetch_deps.sh --vocab    # ALSO build/place the ORB vocabulary asset
# ---------------------------------------------------------------------------
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TP="$ROOT/app/src/main/cpp/third_party"
DL="$ROOT/third_party_download"
mkdir -p "$TP" "$DL"

# Pinned versions / refs.
EIGEN_VER="3.4.0"
GTSAM_TAG="4.2.0"
BOOST_VER="1.84.0"
BOOST_USCORE="1_84_0"
DBOW3_REF="b7f72d3174d3486f9bc5d0586cb8f0d31a7f7b1d"   # pinned commit on master
RTKLIB_TAG="v2.4.3-b34"
OPENCV_VER="4.10.0"

DO_OPENCV=0
DO_VOCAB=0
for a in "$@"; do
  case "$a" in
    --opencv) DO_OPENCV=1 ;;
    --vocab)  DO_VOCAB=1 ;;
    --all)    DO_OPENCV=1; DO_VOCAB=1 ;;
    *) echo "unknown arg: $a" >&2; exit 2 ;;
  esac
done

log() { printf '\033[36m[fetch]\033[0m %s\n' "$*"; }

# fetch_tarball URL DEST_DIR STRIP
fetch_tarball() {
  local url="$1" dest="$2" strip="${3:-1}"
  if [ -d "$dest" ] && [ -n "$(ls -A "$dest" 2>/dev/null)" ]; then
    log "exists, skip: $dest"
    return 0
  fi
  mkdir -p "$dest"
  local tmp; tmp="$(mktemp)"
  log "downloading $url"
  curl -fL --retry 3 -o "$tmp" "$url"
  log "extracting -> $dest"
  tar -xf "$tmp" -C "$dest" --strip-components="$strip"
  rm -f "$tmp"
}

# --- Eigen -----------------------------------------------------------------
fetch_tarball \
  "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VER}/eigen-${EIGEN_VER}.tar.gz" \
  "$TP/eigen" 1

# --- GTSAM -----------------------------------------------------------------
if [ ! -d "$TP/gtsam/.git" ] && [ ! -f "$TP/gtsam/CMakeLists.txt" ]; then
  log "cloning GTSAM $GTSAM_TAG"
  git clone --depth 1 --branch "$GTSAM_TAG" https://github.com/borglab/gtsam.git "$TP/gtsam"
else
  log "exists, skip: $TP/gtsam"
fi
# Android portability patch: GTSAM's bundled metis (GKlib) calls backtrace()/
# backtrace_symbols(), which Android's Bionic does not provide even though a stub
# <execinfo.h> exists. Exclude Android from that code path.
GKLIB_ERR="$TP/gtsam/gtsam/3rdparty/metis/GKlib/error.c"
if [ -f "$GKLIB_ERR" ] && ! grep -q '!defined(__ANDROID__)' "$GKLIB_ERR"; then
  sed -i 's/#ifdef HAVE_EXECINFO_H/#if defined(HAVE_EXECINFO_H) \&\& !defined(__ANDROID__)/' "$GKLIB_ERR"
  log "patched GKlib backtrace for Android"
fi

# --- Boost (header subset for GTSAM 4.2 with Boost features mostly OFF) -----
# GTSAM 4.2 still includes a handful of Boost headers even with
# GTSAM_ENABLE_BOOST_SERIALIZATION=OFF (optional, assign, tuple, math/special
# functions in a few spots). We provide the full header-only Boost distribution
# unpacked; only headers are used (no Boost libs are compiled or linked).
fetch_tarball \
  "https://archives.boost.io/release/${BOOST_VER}/source/boost_${BOOST_USCORE}.tar.gz" \
  "$TP/boost" 1
if [ ! -d "$TP/boost/boost" ]; then
  echo "ERROR: boost/boost header dir missing after extract" >&2; exit 1
fi
log "Boost headers at $TP/boost/boost (header-only; no Boost libs linked)"

# --- DBoW3 -----------------------------------------------------------------
if [ ! -d "$TP/DBoW3/.git" ] && [ ! -f "$TP/DBoW3/CMakeLists.txt" ]; then
  log "cloning DBoW3"
  git clone https://github.com/rmsalinas/DBow3.git "$TP/DBoW3"
  ( cd "$TP/DBoW3" && git checkout -q "$DBOW3_REF" || log "WARN: pinned DBoW3 ref unavailable, using master HEAD" )
else
  log "exists, skip: $TP/DBoW3"
fi

# --- RTKLIB ----------------------------------------------------------------
if [ ! -d "$TP/rtklib/.git" ] && [ ! -f "$TP/rtklib/src/rtklib.h" ]; then
  log "cloning RTKLIB $RTKLIB_TAG"
  git clone --depth 1 --branch "$RTKLIB_TAG" https://github.com/tomojitakasu/RTKLIB.git "$TP/rtklib" \
    || git clone --depth 1 https://github.com/tomojitakasu/RTKLIB.git "$TP/rtklib"
else
  log "exists, skip: $TP/rtklib"
fi

# --- OpenCV Android SDK (optional) -----------------------------------------
if [ "$DO_OPENCV" -eq 1 ]; then
  OCV_ZIP="$DL/opencv-${OPENCV_VER}-android-sdk.zip"
  OCV_DIR="$DL/OpenCV-android-sdk"
  if [ ! -d "$OCV_DIR" ]; then
    log "downloading OpenCV $OPENCV_VER Android SDK"
    curl -fL --retry 3 -o "$OCV_ZIP" \
      "https://github.com/opencv/opencv/releases/download/${OPENCV_VER}/opencv-${OPENCV_VER}-android-sdk.zip"
    log "unzipping OpenCV SDK"
    unzip -q "$OCV_ZIP" -d "$DL"
  else
    log "exists, skip: $OCV_DIR"
  fi
  JNI_DIR="$OCV_DIR/sdk/native/jni"
  if [ -f "$JNI_DIR/OpenCVConfig.cmake" ]; then
    log "OpenCV JNI cmake dir: $JNI_DIR"
    log "Set in gradle.properties:  odyx.opencv.sdk=$JNI_DIR"
  else
    echo "ERROR: OpenCVConfig.cmake not found under $JNI_DIR" >&2; exit 1
  fi
fi

# --- ORB vocabulary asset (optional) ---------------------------------------
if [ "$DO_VOCAB" -eq 1 ]; then
  ASSETS="$ROOT/app/src/main/assets"
  mkdir -p "$ASSETS"
  VOC="$ASSETS/orbvoc.dbow3"
  if [ -f "$VOC" ]; then
    log "exists, skip: $VOC"
  else
    # DBoW3 ships an ORB vocabulary (orbvoc.dbow3) at its repo ROOT.
    # It is BSD-licensed alongside DBoW3. Copy it into assets.
    SRC="$TP/DBoW3/orbvoc.dbow3"
    if [ -f "$SRC" ]; then
      cp "$SRC" "$VOC"
      log "copied DBoW3 ORB vocabulary -> $VOC"
    else
      log "WARN: $SRC not present. Build one with tools/build_vocab (see README)"
      log "      or run the in-repo DBoW3 demo to generate orbvoc.dbow3."
    fi
  fi
fi

log "done. Next:"
log "  1) scripts/fetch_deps.sh --opencv   (if not done)  then set odyx.opencv.sdk"
log "  2) scripts/fetch_deps.sh --vocab"
log "  3) scripts/env_check.sh"
log "  4) ./gradlew :app:assembleDebug"
