#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Cross-compile the Boost libraries GTSAM 4.2 requires for Android arm64-v8a.
#
# GTSAM 4.2 hard-requires compiled Boost components (serialization, system,
# filesystem, thread, date_time, timer, chrono, regex, program_options). We
# build them static with the NDK clang and install into
#   third_party/boost-android/{include,lib}
# which the native CMake then points GTSAM at (and links into libodyx.so).
#
# Usage: scripts/build_boost_android.sh [ndk_dir] [api]
# ---------------------------------------------------------------------------
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BOOST_SRC="$ROOT/app/src/main/cpp/third_party/boost"
PREFIX="$ROOT/app/src/main/cpp/third_party/boost-android"
NDK="${1:-${ANDROID_NDK_ROOT:-$HOME/Android/Sdk/ndk/26.3.11579264}}"
API="${2:-26}"
JOBS="$(nproc)"

TOOLS="$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin"
CXX="$TOOLS/aarch64-linux-android${API}-clang++"
AR="$TOOLS/llvm-ar"
RANLIB="$TOOLS/llvm-ranlib"
[ -x "$CXX" ] || { echo "ERROR: NDK clang not found: $CXX"; exit 1; }

if [ -f "$PREFIX/lib/libboost_serialization.a" ]; then
  echo "[boost] already built at $PREFIX"; exit 0
fi

cd "$BOOST_SRC"
echo "[boost] bootstrapping b2 (host)..."
./bootstrap.sh --with-libraries=serialization,system,filesystem,thread,date_time,timer,chrono,regex,program_options >/tmp/boost_bootstrap.log 2>&1

# NDK clang toolset for b2.
cat > "$BOOST_SRC/user-config-android.jam" <<EOF
using clang : android
:
"$CXX"
:
<archiver>"$AR"
<ranlib>"$RANLIB"
;
EOF

echo "[boost] building static libs for arm64 (api $API)..."
./b2 -j"$JOBS" \
  --user-config="$BOOST_SRC/user-config-android.jam" \
  toolset=clang-android target-os=android \
  architecture=arm address-model=64 \
  link=static runtime-link=shared threading=multi variant=release \
  --layout=system \
  --with-serialization --with-system --with-filesystem --with-thread \
  --with-date_time --with-timer --with-chrono --with-regex --with-program_options \
  cxxflags="-fPIC -std=c++17 -frtti -fexceptions" \
  cflags="-fPIC" \
  --prefix="$PREFIX" install

echo "[boost] installed components:"
ls "$PREFIX/lib"/libboost_*.a
echo "[boost] done -> $PREFIX"
