# ODYX - On-Device Monocular Visual-Inertial SLAM with GNSS Fusion

**Package:** `ai.deepmost.odyx` · **Label:** ODYX

ODYX is an on-device, monocular **visual-inertial SLAM** system with **GNSS fusion**. Using the
phone's camera + IMU + GNSS, it estimates a globally-georeferenced, drift-corrected **6-DoF
trajectory** and a sparse **3D map** in real time, and exposes that pose stream through a
`PoseProvider` interface so other in-house apps (LANYX, FERYX) consume ODYX **instead of ARCore**.

ODYX is the in-house localization core that **replaces ARCore for pose**: it does **not** use
ARCore. It computes VIO itself from raw Camera2 frames + raw `SensorManager` IMU + raw
`GnssMeasurement`/`GnssNavigationMessage`. The estimator is a **GTSAM factor-graph fixed-lag
smoother** fusing:

- **visual features + IMU preintegration** → local metric VIO, and
- **GNSS** → global ENU anchoring + drift correction, in two selectable modes:
  - **LOOSE** (default): per-fix GPS position factors (robust), and
  - **TIGHT** (advanced): raw pseudorange + Doppler factors via RTKLIB.

It also performs **online camera-IMU temporal (`td`) + extrinsic calibration**, **rolling-shutter
compensation**, **DBoW3 loop closure** with pose-graph optimization, and **relocalization** on
tracking loss.

> **Status.** ODYX has been **built, installed, and verified running on a real device** (Samsung
> Galaxy A17, arm64, Android 13): GTSAM/Boost/OpenCV/DBoW3/RTKLIB cross-compiled into `libodyx.so`;
> the capture→time-sync→front-end pipeline is verified live (camera on the REALTIME clock, 200 Hz
> uncalibrated IMU, raw multi-constellation GNSS, KLT feature tracking, ORB vocabulary loaded, no
> crashes). The build was non-trivial - see the **exact, reproduced** steps in §3, especially the
> **Boost cross-compile** (GTSAM 4.2 requires the *compiled* Boost libraries, not headers) and the
> GKlib/`noCompress` fixes. The remaining work is **VIO tuning against real motion** (the estimator
> initializes only with translational excitation) - see **Prototype scope & limitations**.

---

## 1. Architecture (three layers - the boundary is kept clean)

```
A. Android / Kotlin I/O      app/src/main/kotlin/ai/deepmost/odyx/
   camera2 · imu · gnss · time-base unification · GL render · Compose UI ·
   PoseProvider · record/replay · foreground service · DataStore/persistence

B. JNI bridge                app/src/main/cpp/jni/odyx_jni.cpp
   thread-safe push of {frame, imu, gnss} down · pull of {pose, map, telemetry} up
   (thin - validates inputs, holds NO estimation logic)

C. Native estimator (C++17)  app/src/main/cpp/{sync,frontend,init,backend,gnss,loop,map,reloc,core}
   sensor-sync buffers · KLT front-end · VINS-style initializer · GTSAM fixed-lag
   smoother (+online td/extrinsics) · GNSS coupling · loop closure · map · reloc
```

Native modules:

| dir          | role |
|--------------|------|
| `common/`    | POD types, time conventions, math (SO3/geodesy), config, logging, snapshots |
| `sync/`      | thread-safe IMU ring (never-drop) + latest-wins frame slot + GNSS queues; IMU-interval lookup |
| `frontend/`  | FAST/GFTT grid detect, KLT track, fwd/bwd check, RANSAC, keyframe selection, KF-only ORB |
| `init/`      | vision-only SfM → gyro-bias calib → linear VI alignment (scale/gravity/velocity) → gravity refine |
| `backend/`   | GTSAM `IncrementalFixedLagSmoother`; `CombinedImuFactor` + visual ExpressionFactors with online `td`/`T_bc`; marginalization |
| `gnss/`      | ENU coarse-to-fine alignment + `GnssCoupling` (loose GPS factors / tight RTKLIB SPP) |
| `loop/`      | async DBoW3 detect → PnP/essential verify → GTSAM pose-graph optimize |
| `map/`       | bounded keyframe DB + landmark store + trajectory + loop edges (snapshot copied out) |
| `reloc/`     | DBoW3 query + PnP relocalization on tracking loss |
| `core/`      | `Estimator` orchestrator (owns the VIO thread) + the M0 GTSAM smoke test |

### Threading model
- **Camera thread** → unify timestamp → push to native frame slot (latest-wins).
- **IMU thread (~200 Hz)** → push **every** sample (preintegration needs the full stream).
- **GNSS thread** → fixes + raw measurements + ephemeris.
- **Native VIO thread** → front-end + sliding-window optimization (owns the estimator).
- **Native loop-closure thread** → DBoW + verify + pose-graph optimize; commits under lock.
- **UI thread** → pulls pose/telemetry/map snapshots at ~15-60 Hz via double-buffered copies.

---

## 2. Device requirements
- **arm64-v8a**, ARCore-class device (good camera + 6-axis IMU at ~200 Hz).
- **API 26+** (minSdk 26). **Raw GNSS (TIGHT mode) needs API 27+** and a phone whose
  GNSS chipset exposes `GnssMeasurement`/`GnssNavigationMessage` (most flagships since ~2018).
- Steady 30 fps back camera. `SENSOR_INFO_TIMESTAMP_SOURCE` and
  `SENSOR_ROLLING_SHUTTER_SKEW` are read at runtime.

---

## 3. Build

### 3.1 Toolchain
- **Android NDK r26+**, Android SDK (compileSdk 35), **JDK 17**, CMake 3.22+, Ninja, Git.
- Verify with:
  ```bash
  scripts/env_check.sh
  ```

### 3.2 Fetch + prepare native dependencies (permissive-license sources)
```bash
scripts/fetch_deps.sh --all          # Eigen, GTSAM, Boost SRC, DBoW3, RTKLIB, OpenCV SDK, ORB vocab
                                      # (also applies the GKlib Android backtrace patch)
# point Gradle at the OpenCV SDK it printed (gradle.properties or ~/.gradle/gradle.properties):
#   odyx.opencv.sdk=/abs/path/OpenCV-android-sdk/sdk/native/jni

# Install Eigen so GTSAM's find_package(Eigen3 CONFIG) resolves (header-only, seconds):
cmake -S app/src/main/cpp/third_party/eigen -B /tmp/eigen-build \
      -DCMAKE_INSTALL_PREFIX=$PWD/app/src/main/cpp/third_party/eigen-install
cmake --install /tmp/eigen-build

# Cross-compile the Boost libraries GTSAM 4.2 REQUIRES (NOT header-only - see §3.4):
scripts/build_boost_android.sh        # -> third_party/boost-android/{include,lib}
```

### 3.3 Build the APK
```bash
gradle wrapper --gradle-version 8.11.1   # once, if the wrapper jar is missing
./gradlew :app:assembleDebug             # first run cross-compiles GTSAM (~20-40 min)
adb install -r app/build/outputs/apk/debug/app-debug.apk
```
The ORB vocabulary (49 MB) is bundled in `assets/` and stored **uncompressed**
(`androidResources { noCompress += "dbow3" }`) - large *compressed* assets cannot be opened by
`AssetManager`.

### 3.4 The GTSAM cross-compile (highest-risk step - actual reproduced notes)
GTSAM (tag `4.2`, version string `4.2a9`) is cross-compiled from source by a CMake `ExternalProject`
in `app/src/main/cpp/CMakeLists.txt`. The **working flag set**:

```
-DCMAKE_MAKE_PROGRAM=<AGP's ninja>          # REQUIRED: sub-build doesn't inherit it -> "cannot find Ninja"
-DGTSAM_WITH_TBB=OFF  -DGTSAM_BUILD_TESTS=OFF  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
-DGTSAM_BUILD_UNSTABLE=ON                    # IncrementalFixedLagSmoother lives in gtsam_unstable
-DGTSAM_USE_SYSTEM_EIGEN=ON  -DEIGEN3_INCLUDE_DIR=<eigen-install/include/eigen3>
-DEigen3_DIR=<eigen-install/share/eigen3/cmake>  -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH
-DGTSAM_BUILD_STATIC_LIBRARY=ON  -DBUILD_SHARED_LIBS=OFF
-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF          # CRITICAL: march=native breaks cross-compile
-DBOOST_ROOT=<boost-android>  -DBoost_USE_STATIC_LIBS=ON  -DBoost_NO_SYSTEM_PATHS=ON
-DBoost_NO_BOOST_CMAKE=ON  -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH
```
The NDK toolchain (`android.toolchain.cmake`, `ANDROID_ABI`, `ANDROID_PLATFORM`, `ANDROID_STL`) is
forwarded into the sub-build so GTSAM compiles identically to `libodyx.so`. `gtsam_unstable`,
`gtsam`, and the bundled `metis-gtsam` static libs are linked into `libodyx.so` (in that order).

**Boost handling - the crux.** GTSAM 4.2 **hard-requires the compiled Boost component libraries**
(`serialization system filesystem thread date_time timer chrono regex`) - the
`GTSAM_USE_BOOST_FEATURES`/`GTSAM_ENABLE_BOOST_SERIALIZATION` "header-only" flags do **not** exist in
4.2. So Boost must be **cross-compiled for Android**: `scripts/build_boost_android.sh` runs Boost's
`b2` with the NDK clang toolset, producing static `libboost_*.a` in `third_party/boost-android/lib`,
which GTSAM finds (with the `CMAKE_FIND_ROOT_PATH_MODE_*=BOTH` overrides, since the NDK toolchain
otherwise restricts `find_*` to the sysroot) and which are linked into `libodyx.so`.

**Other reproduced fixes** (all scripted/patched in-repo):
- GTSAM's bundled **metis (GKlib) calls `backtrace()`**, absent from Android Bionic - `fetch_deps.sh`
  patches `GKlib/error.c` to exclude Android.
- **RTKLIB**: only `decode_frame` is needed; pulling `rcvraw.c` drags in receiver-format decoders we
  don't use → `third_party/rtklib_rcv_stubs.c` stubs them (never called).
- **OpenCV** is linked from the **prebuilt** official Android SDK (statically pulled into `libodyx.so`).
- **DBoW3** is built in-tree (static).

---

## 4. Calibration workflow
1. **Camera intrinsics** - easiest: **in-app checkerboard calibration** (Settings ▸ *CHECKERBOARD
   CALIB*): point at a 9×6 inner-corner board from varied angles, capture 8+ views, tap CALIBRATE -
   OpenCV `calibrateCamera` runs natively and writes `camera_intrinsics.yaml` (`calibrated: true`),
   clearing the UNCALIBRATED warning live. Alternatively edit `assets/calib/camera_intrinsics.yaml`
   by hand (Kalibr/OpenCV), or rely on the device's `LENS_INTRINSIC_CALIBRATION` when reported.
2. **IMU noise** - `assets/calib/imu_noise.yaml`. Use the **in-app Allan-variance logger**
   (Sessions ▸ *RECORD 2 min STATIC*): place the phone perfectly still, record ≥2 min; it writes a
   real `imu_noise.yaml` into the app files dir (which overrides the bundled default).
3. **Cam-IMU extrinsics + td** - `assets/calib/cam_imu_extrinsics.yaml` from **Kalibr**. These are
   only *seeds*: ODYX refines `td` and (optionally) `T_bc` **online** in the smoother.
4. **GNSS lever arm** - `assets/calib/gnss_lever_arm.yaml` (antenna position in the body frame).

While running on defaults, the SLAM HUD and Telemetry show a prominent **UNCALIBRATED** warning.

---

## 5. GNSS modes (Settings ▸ GNSS, or `setGnssMode`)
- **OFF** - VIO-only. Accumulates drift; the telemetry "drift proxy" is N/A.
- **LOOSE** (default, robust) - each fused fix becomes a GPS **position factor** (antenna position
  via lever arm), weighted by the reported accuracy, after the local↔ENU transform is estimated.
  Best general-purpose choice; works on any GNSS phone.
- **TIGHT** (advanced) - raw `GnssMeasurement` + `GnssNavigationMessage` are decoded by **RTKLIB**
  into per-satellite positions/clocks; a single-point-position (pseudorange + Doppler) solution
  feeds position factors with receiver clock states. Use when you need maximum global accuracy and
  have a **raw-GNSS-capable phone (API 27+)**. **Degrades automatically to LOOSE** when raw
  measurements or ephemeris are unavailable (shown as `TIGHT(->LOOSE)`).

**Local↔ENU alignment** is an online coarse-to-fine 4-DoF estimate (yaw + 3D translation; ENU
origin = first good fix), GVINS-style logic reimplemented from first principles. On GNSS dropout
(tunnels) the transform is held and VIO continues; it re-anchors on reacquisition. Loop closures
re-anchor the transform too.

---

## 6. Record / Replay + offline validation
- **Record** (Sessions ▸ ● REC): writes a synchronized raw session in **EuRoC/ASL layout** -
  `mav0/cam0/data/<ns>.png` + `cam0/data.csv`, `imu0/data.csv`, `gnss0/data.csv`, `calib.txt`.
  Recording is a *tap* on the same unified-timestamp stream the estimator consumes, so it never
  perturbs the live estimate.
- **Replay** (Sessions ▸ REPLAY): feeds a recorded session **deterministically through the SAME
  native estimator** for tuning + regression.
- **Export** (Sessions ▸ EXPORT): zips the session (EuRoC/ASL) for offline analysis.
- **Trajectory log** (Sessions ▸ LOG TRAJ): writes the live pose stream to **TUM-format**
  files (`trajectory_local_*.tum`, `trajectory_enu_*.tum`) under the app files dir, ready to feed
  straight into `evo_ape`/`evo_traj` against the oracle trajectories below. The reconstructed sparse
  map is also persisted (`maps/map_last.odyxmap`) on stop.
- **Offline validation against oracles**: run **ORB-SLAM3** and/or **VINS-Fusion** on the SAME
  recorded EuRoC/ASL session **externally** (on a workstation) and compare trajectories (e.g. with
  `evo`). These oracles are run **outside ODYX and are never linked into it** (they are GPL - see
  the license inventory). They are ground-truth references only.

---

## 7. Coordinate frames & time conventions
**Frames**
- **Body (`b`) = IMU frame.** The estimator's `Pose3` state is `T_wb` (body in world).
- **Camera (`c`).** `T_bc` (camera in body) from extrinsics; refined online.
- **Local world (`w`)** - metric, **gravity-aligned** (z up, gravity ≈ `(0,0,-g)`); origin at init.
- **Global ENU** - East-North-Up at the first good GNSS fix (ECEF origin). `p_enu = Rz(yaw)·p_w + t`.

**Time (priority zero).** Every sensor stream is reduced to ONE monotonic nanosecond clock -
Android `elapsedRealtimeNanos` - **before** crossing JNI (`time/TimeBase.kt`):
- IMU `SensorEvent.timestamp` is already on that base.
- Camera `SENSOR_TIMESTAMP`: if `SENSOR_INFO_TIMESTAMP_SOURCE == REALTIME`, same base; if `UNKNOWN`,
  the uptime→realtime offset is added (documented in `TimeBase`).
- GNSS uses `Location.getElapsedRealtimeNanos()` (API 29+) when available.

The **residual** camera↔IMU offset that survives this reduction is the online state **`td`**,
estimated in the smoother (`t_imu_aligned = t_cam + td`). The time-base layer removes only the
*clock-base* difference; `td` handles the remaining latency/skew.

---

## 8. LICENSE INVENTORY

| Dependency | Version | License | Linked into `libodyx.so`? | How obtained |
|-----------|---------|---------|---------------------------|--------------|
| **GTSAM** | 4.2 (4.2a9) | BSD-3-Clause | yes (static, +gtsam_unstable, +metis) | source, cross-compiled in-tree |
| **Eigen** | 3.4.0 | MPL-2.0 | yes (header-only) | source, installed for config |
| **Boost** | 1.84.0 | BSL-1.0 | yes (static libs, **cross-compiled for arm64**) | source + `build_boost_android.sh` |
| **OpenCV** | 4.10.0 | Apache-2.0 | yes (official prebuilt Android SDK, static) | prebuilt SDK |
| **DBoW3** | master | BSD-3-Clause | yes (static) | source |
| **RTKLIB** | 2.4.3-b34 | BSD-2-Clause | yes (subset, static) | source |
| AndroidX / Compose / Material3 | see catalog | Apache-2.0 | - (Kotlin) | Maven |
| Play Services Location | 21.3.0 | Android SDK Terms | - (Kotlin) | Maven |
| Timber, Accompanist | see catalog | Apache-2.0 | - (Kotlin) | Maven |

**No GPL / copyleft code is linked, copied, vendored, or pasted.** In particular ORB-SLAM2/3,
VINS-Mono, VINS-Fusion, GVINS, OpenVINS, and Kimera-VIO are **not** used as code. Their *published
algorithms* (KLT tracking, sliding-window smoothing, IMU preintegration, GVINS-style coarse-to-fine
GNSS init, online `td` estimation) are **reimplemented from first principles** on the permissive
libraries above. ORB-SLAM3 / VINS-Fusion may be run **externally** as ground-truth oracles
(Section 6) but are never linked.

---

## 9. Prototype scope & limitations (deliberate)
- **Monocular** - no stereo baseline on a single phone, so absolute scale comes from inertial
  observability (needs translational excitation to initialize) and from GNSS.
- **Sparse map** - point-feature landmarks, not dense reconstruction.
- **CPU / NEON front-end** - KLT + FAST/GFTT on the CPU; no GPU feature extraction.
- **Accuracy is bounded without aggregation** - a single handheld pass drifts between GNSS anchors;
  global accuracy improves with GNSS quality and loop closures.

All **advanced features are fully implemented, not stubbed**: online `td` + extrinsic calibration,
tightly-coupled raw GNSS, DBoW3 loop closure with pose-graph optimization, rolling-shutter
compensation, and relocalization.

### Graceful degradation
- No raw GNSS → auto **LOOSE**. No GNSS at all → **VIO-only** with a drift indicator.
- Uncalibrated → documented **defaults** + prominent UI warning.
- Tracking loss → **relocalize**; if impossible, start a new map segment and stitch on relocalize.
- Thermal pressure → drop target feature count / frame stride (surfaced in Telemetry).
- Optimizer divergence / NaN → clean window reset; never emit garbage (pose carries validity + cov).

---

## 10. Repository layout
```
ODYX-VSLAM/
├── settings.gradle.kts · build.gradle.kts · gradle.properties · gradle/libs.versions.toml
├── scripts/            env_check.sh · fetch_deps.sh
├── app/
│   ├── build.gradle.kts  (externalNativeBuild CMake, abi arm64-v8a)
│   └── src/main/
│       ├── AndroidManifest.xml · assets/calib/*.yaml · res/…
│       ├── cpp/         CMakeLists.txt · common sync frontend init backend gnss loop map reloc core jni · third_party/
│       └── kotlin/ai/deepmost/odyx/
│           camera · imu · gnss · time · core · pose · session · allan · config · service · ui/{slam,map,telemetry,sessions,settings,common,theme}
└── README.md
```
