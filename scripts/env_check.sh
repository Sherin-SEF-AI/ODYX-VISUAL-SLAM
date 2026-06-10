#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# ODYX :: environment check
#
# Verifies that the host has everything required to cross-compile the native
# estimator (GTSAM / DBoW3 / RTKLIB) for arm64-v8a and assemble the APK.
# Prints a PASS/FAIL summary; exits non-zero if any hard requirement is missing.
# ---------------------------------------------------------------------------
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Minimum versions.
NDK_MIN_MAJOR=26          # r26+
CMAKE_MIN="3.22.1"        # ships with AGP 8.x; GTSAM needs >=3.0
ok=1

red()   { printf '\033[31m%s\033[0m\n' "$*"; }
green() { printf '\033[32m%s\033[0m\n' "$*"; }
yellow(){ printf '\033[33m%s\033[0m\n' "$*"; }

note()  { printf '  %s\n' "$*"; }
pass()  { green  "PASS  $*"; }
fail()  { red    "FAIL  $*"; ok=0; }
warn()  { yellow "WARN  $*"; }

ver_ge() { # ver_ge A B  -> true if A >= B (dotted)
  [ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -n1)" = "$2" ]
}

echo "== ODYX environment check =="
echo "root: $ROOT"
echo

# --- Android SDK -----------------------------------------------------------
SDK="${ANDROID_SDK_ROOT:-${ANDROID_HOME:-}}"
if [ -z "$SDK" ] && [ -f "$ROOT/local.properties" ]; then
  SDK="$(grep -E '^sdk\.dir=' "$ROOT/local.properties" | cut -d= -f2- || true)"
fi
if [ -n "$SDK" ] && [ -d "$SDK" ]; then
  pass "Android SDK: $SDK"
else
  fail "Android SDK not found (set ANDROID_SDK_ROOT or local.properties sdk.dir)"
fi

# --- Android NDK -----------------------------------------------------------
NDK="${ANDROID_NDK_ROOT:-${ANDROID_NDK_HOME:-}}"
if [ -z "$NDK" ] && [ -n "${SDK:-}" ] && [ -d "$SDK/ndk" ]; then
  # pick the highest installed NDK
  NDK="$(find "$SDK/ndk" -maxdepth 1 -mindepth 1 -type d | sort -V | tail -n1)"
fi
if [ -n "$NDK" ] && [ -f "$NDK/source.properties" ]; then
  NDK_REV="$(grep -E 'Pkg.Revision' "$NDK/source.properties" | cut -d= -f2- | tr -d ' ')"
  NDK_MAJOR="${NDK_REV%%.*}"
  if [ "${NDK_MAJOR:-0}" -ge "$NDK_MIN_MAJOR" ]; then
    pass "Android NDK r${NDK_MAJOR} ($NDK_REV): $NDK"
  else
    fail "Android NDK r${NDK_MAJOR} too old; need r${NDK_MIN_MAJOR}+"
  fi
  export ANDROID_NDK_ROOT="$NDK"
  TOOLCHAIN="$NDK/build/cmake/android.toolchain.cmake"
  [ -f "$TOOLCHAIN" ] && note "toolchain: $TOOLCHAIN" || fail "android.toolchain.cmake missing under NDK"
else
  fail "Android NDK not found (set ANDROID_NDK_ROOT, or install via sdkmanager 'ndk;26.x')"
fi

# --- CMake -----------------------------------------------------------------
if command -v cmake >/dev/null 2>&1; then
  CM_VER="$(cmake --version | head -n1 | awk '{print $3}')"
  if ver_ge "$CM_VER" "$CMAKE_MIN"; then
    pass "cmake $CM_VER"
  else
    warn "cmake $CM_VER < $CMAKE_MIN (AGP-bundled cmake is fine; standalone may be old)"
  fi
else
  warn "cmake not on PATH (AGP will use the SDK-bundled cmake)"
fi

# --- Ninja -----------------------------------------------------------------
if command -v ninja >/dev/null 2>&1; then
  pass "ninja $(ninja --version)"
else
  warn "ninja not on PATH (SDK cmake bundles one; ExternalProject prefers it)"
fi

# --- Build tools used by fetch_deps.sh -------------------------------------
for t in git curl unzip tar; do
  if command -v "$t" >/dev/null 2>&1; then pass "$t present"; else fail "$t missing (needed by fetch_deps.sh)"; fi
done

# --- Java ------------------------------------------------------------------
if command -v java >/dev/null 2>&1; then
  JV="$(java -version 2>&1 | head -n1)"
  pass "java: $JV"
else
  fail "java not found (need JDK 17+ for AGP 8.x)"
fi

# --- OpenCV SDK ------------------------------------------------------------
OCV="$(grep -E '^odyx\.opencv\.sdk=' "$ROOT/gradle.properties" 2>/dev/null | cut -d= -f2- || true)"
OCV="${ODYX_OPENCV_SDK:-$OCV}"
if [ -n "$OCV" ] && [ -f "$OCV/OpenCVConfig.cmake" ]; then
  pass "OpenCV Android SDK: $OCV"
elif [ -n "$OCV" ]; then
  fail "odyx.opencv.sdk set ($OCV) but OpenCVConfig.cmake not found there"
else
  warn "OpenCV SDK not set yet (run fetch_deps.sh, then set odyx.opencv.sdk in gradle.properties)"
fi

# --- Fetched native sources ------------------------------------------------
TP="$ROOT/app/src/main/cpp/third_party"
for d in eigen gtsam DBoW3 rtklib; do
  if [ -d "$TP/$d" ] && [ -n "$(ls -A "$TP/$d" 2>/dev/null)" ]; then
    pass "third_party/$d fetched"
  else
    warn "third_party/$d not fetched yet (run scripts/fetch_deps.sh)"
  fi
done

echo
if [ "$ok" -eq 1 ]; then
  green "== environment OK =="
  exit 0
else
  red "== environment has blocking issues (see FAIL lines) =="
  exit 1
fi
