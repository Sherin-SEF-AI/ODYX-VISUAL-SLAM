# ODYX assets

- `calib/*.yaml` — default calibration templates (see each file). Replace with
  real per-device calibration; the in-app Allan logger writes a real
  `imu_noise.yaml` into the app files dir, which overrides the bundled one.

- `orbvoc.dbow3` — the ORB bag-of-words vocabulary for loop closure /
  relocalization. **Not committed** (it is large and BSD-licensed alongside
  DBoW3). Populate it with:

      scripts/fetch_deps.sh --vocab

  which copies DBoW3's bundled `orbvoc.dbow3` here. Without it, loop closure and
  relocalization are disabled (VIO still runs); the log shows
  "no ORB vocabulary asset".
