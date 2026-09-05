# PX4-Autopilot — Vetri2425 rover fork

Custom PX4 firmware for a **CubeOrangePlus differential-drive marking rover**.
Safety-critical C/C++. This file is the only preloaded context.

## Fork facts (verified 2026-08-05)

- **Repo:** `Vetri2425/PX4-Autopilot`, branch `main`.
- **Diverged from upstream at** `92fa89d7` (`ci(mavros): remove MAVROS integration test suite`, 2026-05-14). Fork tree is post-`v1.18.0-alpha1`.
- **31 commits above baseline.** HEAD == `origin/main` == `06309e41a7` (`feat(logger): log wheel-encoder fusion debug topics on rover`).
- **But CI builds against `v1.16.2`** — `build_rover.yml` checks out stock upstream `v1.16.2` and overlays 30 fork files (as of 2026-09-05, UNCOMMITTED — see below). So all overlay files must be edited as `v1.16.2`-stock + minimal diff, NOT against the fork's own newer tree (base-discipline rule — see `.claude/memory/patches.md`).

## Build — CI only

```sh
make format                           # run on changed C/C++ before commit; CI enforces check_format
git push origin main                  # auto-triggers build_rover.yml
```

`.github/workflows/build_rover.yml` (ubuntu-22.04) is the **only** build path and the
canonical artifact to flash: checkout stock `v1.16.2` → fetch NuttX tags → checkout fork
`main` into `fork_patches/` → overlay 30 files → `make cubepilot_cubeorangeplus_rover`.
See `.claude/memory/build.md` for the gh-CLI push → watch → download flow.

⚠ **There is no local build script.** `Tools/local_build_rover.sh` was reverted on
2026-08-05 and never reached `origin/main` — see "Repo state" below. Do not reference it.
Building in this tree directly is what caused the ekf2 CMakeLists drift; if you need a
local build, work in a separate clean `v1.16.2` checkout, never in this working tree.

## Commit / PR rules

- Conventional commits, topic scope: `type(scope): description`. Use `/commit`, `/pr` skills.
- **No Claude attribution** — no `Co-Authored-By`, no "Generated with" footer.

## Patch domains (30-file overlay)

| Scope | Files | Purpose |
|-------|-------|---------|
| `boards` | `rover.px4board` | Enables ROVER_DIFFERENTIAL, ROBOCLAW, EKF2_WHEEL_ENCODER; disables FW/MC/VTOL |
| `rover_differential` | `RoverDifferential.{cpp,hpp}`, `module.yaml`, `DifferentialVelControl/*`, `DifferentialAttControl/{cpp,hpp}` | RD_TANK_MODE, IK signs, disarm guard, OFFBOARD signed-speed + hold-yaw, `RD_YAW_RATE_FF` yaw-rate feedforward (2026-09-05, uncommitted) |
| `roboclaw` | `Roboclaw.{cpp,hpp}`, `module.yaml` | QPPS velocity (opcodes 35/36), UART/raw/baud fixes, creep deadband |
| `land_detector` | `RoverLandDetector.cpp` | Always-landed (coupled to mission_block) |
| `navigator` | `mission_block.cpp` | Rover waypoint-acceptance bypass (coupled to land_detector) |
| `ekf2` | 13 files incl. `EKF/aid_sources/wheel_encoder/` | Wheel-encoder body-frame velocity fusion (default OFF) |
| `msg` | `RoverAttitudeSetpoint.msg`, `DifferentialVelocitySetpoint.msg` | Additive `yaw_rate_feedforward` field (2026-09-05, uncommitted); `DifferentialVelocitySetpoint.msg` re-created from v1.16.2 stock (fork-main deleted it) |

⚠ **DifferentialPosControl is NOT overlaid** — fork copy needs `RoverSpeedSetpoint.msg` absent in v1.16.2. Stock v1.16.2 PosControl is used.

⚠ **DifferentialAttControl.{cpp,hpp} are v1.16.2-stock RE-ANCHORED, not fork-main's copy** — fork-main's version belongs to the post-`v1.16.2` `DifferentialDriveModes` mode-dispatch refactor (depends on `DifferentialOffboardMode`, `DifferentialAutoMode`, `DifferentialManualMode`, none ever overlaid, none built). See `.claude/memory/patches.md` "Yaw-rate feedforward" entry.

⛔ **`RD_YAW_RATE_FF` must stay 0 on the `segment` tracking profile.** The Jetson companion does not send a rate feedforward there — `rpp_controller_node.py` sets `yaw_rate_body = segment_yaw_rate_gain · θ_e` (gain **1.5**), a proportional heading-error *feedback* on the same error `RO_YAW_P` (**1.5**) already closes. Applying it sums two P gains into `3.0 · θ_e`, doubling the heading-loop gain on a loop already documented at near-zero phase margin. Only the `smooth`/arc profile sends a true `κ·v` feedforward (`yaw_rate_feedback_gain` defaults 0 there), so that is the only profile where raising `RD_YAW_RATE_FF` is meaningful — and it needs a measured A/B.

⚠ **`trajectory_setpoint.yaw` is deliberately ignored on the velocity path.** Bearing stays `atan2(vy,vx)`. The reverse 180° flip in `DifferentialVelControl` is only valid on a direction-of-travel heading; applying it to an explicit attitude command spins the rover 180° away from the request. Every current companion publisher sends `yaw = atan2(velocity)` anyway (`twist_to_setpoint_node.py`, `spin_in_place_test.py`), so honouring it would be a no-op today and a trap tomorrow.

## Repo state (2026-08-05 cleanup)

Working tree was reset to `origin/main`. Discarded, all recoverable:

- **Commit `4152220472`** (`chore(build)`: local build script + CLAUDE.md rover rewrite) — dropped
  intentionally, along with its backup branch. **No backup remains.** The commit is unreferenced;
  `git cherry-pick 4152220472` works only while it survives in the reflog (~90 days from
  2026-08-05, sooner if `git gc` runs). Treat the local build script as gone.
- **Uncommitted loiter/stop work** — `DifferentialAutoMode.cpp` (+17, LOITER hold-position
  setpoint) and `DifferentialPosControl.{cpp,hpp}` (+33, zero-speed stop branch). Not in the
  overlay, so it never affected CI. Patch saved in the scratchpad backup (volatile).
- **ekf2 CMakeLists drift** — reverted; `gps_checks.cpp` + `zero_innovation_heading_update.cpp`
  restored (v1.16.2-correct). A stale `.git/index.lock` from 2026-06-12 was blocking all git
  writes for ~7 weeks and was removed.

## Memory index (`.claude/memory/`) — load ONLY on demand

⚠ Do NOT read these at session start. Open one **only** when the current task needs it or the user asks.

- `build.md` — CI/gh-CLI build & flash workflow, base-discipline rule.
- `patches.md` — full overlay map, per-patch detail, interactions, v1.16.2 compat rules.
- `runtime.md` — hardware wiring, RoboClaw, EKF/GPS fix, NED↔ENU, OFFBOARD safety params, open issues.
- `progress.md` — chronological patch + CI-build log, milestones.
- `integration.md` — **cross-project bridge**: how PX4_DXP (Jetson ROS2 + FastAPI + MAVROS) sits on top of this firmware.
