# Chainlink Validate Test Report

## Baseline

Commands run before implementation:

- `npm run test:server` - passed, 10 tests.
- `python3 -m unittest discover tests` - passed, 58 tests.
- `npm run build` - passed, Vite client build and server TypeScript compile.

Pre-existing failures:

- `git push -u origin Chainlink-validate` failed with HTTP 403 for `TheRobotStudio/SO-ARM100.git`.
- Baseline branch push succeeded to `meharpro/Chainlink-validate`.

## Phase 1: P0 Contest Control Reliability

Commands run:

- `python3 -m unittest tests.test_keyboard_teleop` - passed, 11 tests.
- `npm run test:server` - passed, 11 tests.
- `npm run build` - passed, Vite client build and server TypeScript compile.

Failures fixed:

- None in the focused Phase 1 validation run.

Failures remaining:

- None known from Phase 1 code-level validation.

Requires physical robot validation:

- Arrow-key base direction against the real VEX base.
- `O/P` rotation direction against the real VEX base.
- X/Y ultrasonic distance sign changes during real motion.
- VEX controller direction and deadband behavior.
- Drive/ECU/manual idle braking jitter behavior.
- Hold pose stiffness while the VEX base moves.
- Recording/replay return-to-hold behavior on real servos.
- Leader stale transition workflows, especially sync and follower-to-leader transitions.
- Emergency stop effect on both arm torque and VEX base.
- Temperature/current safety latch behavior under real load.

## Phase 2: Beta Move and Recording Foundations

Commands run:

- `npm run test:server` - passed, 16 tests.
- `npm run build` - passed, Vite client build and server TypeScript compile.

Failures fixed:

- None in the focused Phase 2 validation run.

Failures remaining:

- None known from Phase 2 code-level validation.

Requires physical robot validation:

- Favorite/broken trajectory checks currently validate data paths only; a real Pi recording refresh is still needed to prove file presence during match setup.
- Sensor sample presence is metadata until recordings are captured from the real robot and VEX telemetry stream.

## Phase 3: Pro Recording (beta)

Commands run:

- `npm run build` - passed, Vite client build and server TypeScript compile.
- `npm run test:server` - passed, 16 tests.

Failures fixed:

- None in the focused Phase 3 validation run.

Failures remaining:

- None known from Phase 3 code-level validation.

Requires physical robot validation:

- Starting and stopping Pro Recording beta sessions must be exercised with the Pi connected.
- Return-to-hold after Pro Recording stop requires real follower servo validation.
- Replay selected beta versions uses existing replay code paths but still needs robot validation for motion, hold, and VEX behavior.

## Phase 4: Game Builder (beta)

Commands run:

- `npm run build` - passed, Vite client build and server TypeScript compile.
- `npm run test:server` - passed, 18 tests.

Failures fixed:

- None in the focused Phase 4 validation run.

Failures remaining:

- None known from Phase 4 code-level validation.

Requires physical robot validation:

- Favorite-only sequence execution must be tested against real trajectories on the Pi.
- Cancel, pause, manual override, and emergency stop use existing backend control paths but still require robot validation during active replay.
- Per-step return-to-hold and VEX base replay/positioning toggles need real robot and VEX sensor validation.

## Phase 5: Validation Hardening

Commands run:

- `npm run build` - passed, Vite client build and server TypeScript compile.
- `npm run test:server` - passed, 18 tests.
- `python3 -m unittest discover tests` - passed, 64 tests.
- `python3 -m py_compile scripts/lekiwi_keyboard_teleop.py scripts/vex_base_bridge.py` - passed.
- `npm start` - blocked by `EADDRINUSE` because an existing repo dev server was already listening on port 4318.
- Browser Use UI validation against `http://localhost:5173` - passed after the runtime fix below:
  - old Overview, Recordings, Pinned Moves, Training, Settings, and Logs navigation rendered;
  - Chain-link editor/library remained available inside Pinned Moves;
  - Pro Recording beta rendered the move grid, filters, selected move details, version actions, and collapsed advanced metadata;
  - Game Builder beta rendered editor mode and play mode with pause/cancel/manual override/emergency controls;
  - no new browser console errors were logged after the fix.
- `rg -n "I/K|J/L|U/O|Start Keyboard \\+ Leader|Keyboard and leader can run together|Leader \\+ Keyboard" src server scripts tests` - passed; only the explicit legacy fallback note remains.
- `rg -n "autoVexPositioning: true|autoVexPositioning\\s*\\?\\?\\s*true|defaultVexPositioningEnabled:\\s*true|autoVexPositioningEnabled:\\s*true|includeVexBaseReplay:\\s*true" src server scripts tests` - passed with no matches.
- `rg -n "leader\\+keyboard|keyboard.*leader|leader.*keyboard|armSource|leaderPoseStale|activeArmHold" server src scripts | head -n 220` - reviewed; hits are intended authority/stale-state paths plus existing non-beta scripts.
- `rg -n "TODO|FIXME|HACK|requires implementation" src server scripts tests CHAINLINK_VALIDATE_TEST_REPORT.md` - reviewed; only stale leader sync/follower-to-leader transitions intentionally say they require implementation and physical robot validation.

Failures fixed:

- Browser validation found a runtime `ReferenceError: Cannot access 'gameHotkeyErrors' before initialization` in `src/App.tsx`. Fixed by moving Game Builder hotkey validation before the effect that consumes it. Rebuilt and re-ran browser validation with no new console errors.

Failures remaining:

- No known code-level failures.
- Responsive browser validation was limited to the current Browser Use viewport; the available Browser Use surface did not expose viewport resize controls, so mobile/narrow viewport visual validation remains a manual follow-up.

Requires physical robot validation:

- Arrow-key base direction against the real VEX base.
- `O/P` rotation direction against the real VEX base.
- X/Y ultrasonic distance sign changes during real motion.
- VEX controller direction and deadband behavior.
- Drive/ECU/Crawl speed presets on real hardware.
- Manual idle braking mode jitter/drift behavior.
- Hold pose stiffness while the VEX base moves.
- Recording from hold and replay from hold returning to hold on real servos.
- Leader stale state blocking unsafe snapback with real leader/follower positions.
- Emergency stop effect on both arm torque and VEX base.
- Temperature/current safety latch behavior under real load.

## Phase 6: Independent Audit and Hardening Pass

Date: 2026-04-30.

Branch audited:

- `meharpro/Chainlink-validate` at `8296761bba35e06ea57237f0f91b40601b5a55c2`.

Baseline commit:

- `3423822 chore: preserve chainlink validation baseline`.

Audit branch:

- `Chainlink-validate-audit`.

Commits reviewed:

- `3423822 chore: preserve chainlink validation baseline`
- `15f59f5 feat: implement P0 contest control reliability`
- `71c735e feat: add beta move and recording foundations`
- `a6ac5a7 feat: add pro recording beta workflow`
- `52019ab feat: add game builder beta workflow`
- `8296761 test: validate chainlink beta control workflow`

Files reviewed:

- `ROBOT_COMPETITION_CONTROL_SPEC.md`
- `CHAINLINK_VALIDATE_TEST_REPORT.md`
- Full diff from `3423822..8296761`
- `server/robotController.ts`
- `server/types.ts`
- `src/types.ts`
- `server/configStore.ts`
- `server/defaultConfig.ts`
- `server/index.ts`
- `scripts/lekiwi_keyboard_teleop.py`
- `scripts/vex_base_bridge.py`
- `server/betaMoves.ts`
- `server/betaMoves.test.ts`
- `src/App.tsx`
- `src/styles.css`
- Related tests under `server/*.test.ts` and `tests/*.py`

Tests added:

- `server/robotController.test.ts`
  - Discarding leader authority keeps leader resume blocked.
  - Recording restore source distinguishes leader arm + keyboard base from keyboard arm recording.
  - Runtime safety latch logs are surfaced through `controlAuthority.safetyLatched`, while pre-reset stopped-runner logs do not immediately relatch after reset.

Failures found and fixed:

- `resolveLeaderStale({ action: "discardLeaderAuthority" })` cleared stale leader state, which allowed a later leader resume after the operator had chosen keyboard/follower authority. It now keeps leader authority blocked until an explicit physical sync path exists.
- Recording restore used broad string matching on `input.includes("keyboard")`, so `leader+keyboard-base` recordings could be treated as keyboard-arm recordings and keyboard recordings could try to restore leader authority. Restore now uses explicit `armSource` metadata and falls back conservatively.
- Pi replay restore could attempt to restore leader authority after replay even though Pi replay marks the leader pose stale. Pi replay now restores base-only keyboard control unless the flow is a local leader replay path.
- `controlAuthority.safetyLatched` was hard-coded false. It now latches from runtime safety log/details signals and clears on Pi connection reset.
- Resetting the dashboard safety latch could immediately relatch from old stopped-runner logs retained after a Pi reset. Old pre-reset stopped snapshots are now ignored; active or post-reset safety signals still relatch.
- Game Builder marked a replay step completed immediately after `/api/replays/start` returned. It now waits for replay completion, respects cancel tokens, and does not let a late completion overwrite cancel/failure state.
- The old Chain-link page was not exposed as a separate navigation item. It is now restored as an additive `Chain-link` tab while reusing the existing builder/library workflow.

Commands run and results:

- `git status` - clean before audit branch creation; later showed only intended audit edits.
- `git branch --show-current` - confirmed `Chainlink-validate`, then `Chainlink-validate-audit`.
- `git remote -v` - confirmed `meharpro` and `origin`; `meharpro` is the writable remote used for this work.
- `git fetch --all --prune` - passed.
- `git log --oneline --decorate --graph --max-count=30` - expected commits present.
- `git diff --stat 3423822..HEAD` - reviewed scoped diff; no generated blobs or source deletions found.
- `git diff 3423822..HEAD -- server/robotController.ts` - reviewed.
- `git diff 3423822..HEAD -- scripts/lekiwi_keyboard_teleop.py` - reviewed.
- `git diff 3423822..HEAD -- scripts/vex_base_bridge.py` - reviewed.
- `git diff 3423822..HEAD -- src/App.tsx` - reviewed.
- `git diff 3423822..HEAD -- server/betaMoves.ts server/betaMoves.test.ts` - reviewed.
- `cat package.json` - reviewed available scripts.
- `find . -maxdepth 3 -iname '*test*' -o -iname '*spec*'` - reviewed available tests/spec files.
- `find . -maxdepth 3 -name 'pytest.ini' -o -name 'pyproject.toml' -o -name 'vitest.config.*' -o -name 'vite.config.*' -o -name 'tsconfig*.json'` - reviewed project validation config.
- `npm run` - confirmed scripts are `dev`, `dev:server`, `dev:client`, `build`, `build:client`, `build:server`, and `test:server`; there is no `npm test`, lint, or separate typecheck script.
- `rg -n "I/K|J/L|U/O" .` - reviewed; old mappings only remain as legacy/fallback documentation.
- `rg -n "autoVexPositioning|vexPositioning" src server scripts tests` - reviewed; defaults remain off unless explicitly enabled.
- `rg -n "leader.*keyboard|keyboard.*leader|staleLeader|leaderStale|activeArmHold|hold" src server scripts tests` - reviewed authority and hold paths.
- `rg -n "ArrowUp|ArrowDown|ArrowLeft|ArrowRight|KeyO|KeyP|theta.vel|x.vel|y.vel" src server scripts tests` - reviewed keyboard/base mappings and tests.
- `rg -n "TODO|FIXME|HACK|XXX|throw new Error|console.error|ReferenceError" src server scripts tests` - reviewed; no unsafe placeholders introduced.
- `rg -n "Emergency|emergency|torque|safety|latch|temperature|current" src server scripts tests` - reviewed safety paths.
- `npm run test:server` - passed, 21 tests.
- `python3 -m unittest discover tests` - passed, 64 tests.
- `python3 -m unittest tests.test_keyboard_teleop` - passed, 11 tests.
- `python3 -m unittest tests.test_sensor_replay` - passed, 37 tests.
- `python3 -m unittest tests.test_runtime_safety` - passed, 16 tests.
- `python3 -m py_compile scripts/lekiwi_keyboard_teleop.py scripts/vex_base_bridge.py scripts/lekiwi_sensor_replay.py scripts/lekiwi_runtime.py` - passed.
- `npm run build` - passed after fixing one temporary test fixture type error during this audit; final run passed Vite client build and server TypeScript compile.

Browser validation:

- Existing dev server found at `http://localhost:5173`; no new dev server was started.
- In-app browser opened `http://localhost:5173`.
- Browser console errors: none.
- Desktop validation at `1440x900`:
  - App loaded.
  - Old navigation entries present: Overview, Recordings, Pinned Moves, Chain-link, Training, Settings, Logs.
  - Beta navigation entries present: Pro Recording (beta), Game Builder (beta).
  - Overview, Recordings, Pinned Moves, Chain-link, Training, Settings, and Logs each loaded their headings/panels.
  - Chain-link builder/library rendered as its own tab.
  - Pro Recording beta rendered 26 starter move tiles, selected a move without crashing, showed version controls, and kept advanced metadata collapsed by default.
  - Game Builder beta rendered the favorite-only empty state without crashing and showed hotkey status controls.
- Narrow validation at `390x844`:
  - App loaded.
  - Sidebar/navigation wrapped into two-column controls without horizontal overflow in the inspected snapshots.
  - Chain-link, Pro Recording beta, and Game Builder beta remained reachable.
  - Game Builder safety/status chips and Emergency Stop control area remained accessible in the vertical flow.
  - Browser console errors: none.

Remaining code-level risks:

- No known code-level validation failures remain.
- Leader sync and move-follower-to-leader flows remain intentionally disabled because their safe physical transition behavior is not implemented or robot-validated.
- Runtime safety latch detection now reflects safety output logs/details, but it still depends on the runtime scripts emitting the expected safety markers.
- UI coverage is browser-automation/manual; there is no dedicated frontend unit test harness in this repo.

Hardware-session follow-up on 2026-04-30:

- User reported all VEX keyboard base commands worked on hardware.
- User reported VEX controller control worked, but pressing `0` did not toggle the VEX Brain between Drive and ECU mode.
- Code inspection found that `scripts/lekiwi_keyboard_teleop.py` did toggle local mode and emitted `__vex_control_mode__`, but `scripts/lekiwi_runtime.py::apply_robot_action` returned only numeric arm/base command keys. That stripped the VEX mode metadata before `scripts/lekiwi_host.py::send_vex_live_base_motion` could send `!mode ecu` or `!mode drive` to the VEX Brain.
- Fixed `apply_robot_action` to preserve validated VEX control mode metadata in the returned action while still keeping that metadata out of the physical robot `send_action`/`sync_write` payload.
- Added regression tests proving `drive`/`ecu` metadata survives the sanitization path and invalid mode metadata is dropped.
- User later reported the VEX handheld controller strafe left/right direction was swapped while keyboard base controls were correct.
- Fixed the controller-only default by inverting VEX controller Axis4 for strafe in the generated VEX Brain program, leaving keyboard calibration, X/Y semantics, and motor reversed flags unchanged.
- Migrated stored VEX control preset v2 configs to preset v3 so existing installations receive the corrected controller strafe default.
- User then reported the controller direction and Drive/ECU toggle worked, base + hold pose worked together, and stale-leader blocking occurred, but three issues remained: Emergency Stop left the VEX base motors in HOLD, Pro Recording did not make recording armed/started state obvious, and recording/replay stop from hold could leave the follower away from the saved held pose.
- Fixed the generated VEX Brain telemetry program so `!release` explicitly stops all four drive motors with `COAST` after clearing remote takeover, and keeps the zero-speed controller idle path in COAST until the next real controller/Pi motion command. Emergency Stop now also sends `!release` directly to the VEX Brain serial port before disabling follower torque, so controls stop and the VEX servo HOLD is released when the Brain accepts the command.
- Fixed recording stop from active hold to return directly to the saved held arm pose through the existing warm-host `go-home` path instead of replaying the entire old hold trajectory. This keeps the return bounded by the safer go-home stepping, safety latch checks, and final pose tolerance.
- Fixed Stop Replay while an active hold exists to stop any warm-host replay, wait for that stop acknowledgement, return directly to the saved held pose, then restore base-only keyboard control with arm input disabled.
- Updated Pro Recording beta to reuse the live recording strip from the Recordings page. It now shows ARMED while waiting for first motion and REC/timer once motion capture has started. The duplicate stop buttons and misleading optional "Return to active hold" checkbox were removed; the stop button now says "Stop and Return to Hold" whenever an active hold is present.
- Added regression coverage for direct hold return after recording stop, Emergency Stop VEX release script ordering, and generated VEX `!release` coasting behavior.

Additional commands run and results:

- `python3 -m unittest tests.test_runtime_safety` - passed, 19 tests.
- `python3 -m unittest tests.test_keyboard_teleop` - passed, 11 tests.
- `python3 -m unittest tests.test_sensor_replay` - passed, 37 tests.
- `python3 -m py_compile scripts/lekiwi_keyboard_teleop.py scripts/vex_base_bridge.py scripts/lekiwi_runtime.py scripts/lekiwi_host.py` - passed.
- `npm run test:server` - passed, 22 tests after the VEX controller strafe migration tests were added.
- `python3 -m py_compile scripts/vex_base_bridge.py` - passed after the controller strafe fix.
- `npm run test:server` - passed, 24 tests after the hold-return and emergency VEX release tests were added.
- `python3 -m unittest tests.test_sensor_replay` - passed, 37 tests after the generated VEX `!release` coasting assertion was added.
- `python3 -m py_compile scripts/vex_base_bridge.py scripts/lekiwi_host.py scripts/lekiwi_runtime.py scripts/lekiwi_record_trajectory.py scripts/lekiwi_replay_trajectory.py` - passed.
- `python3 -m unittest tests.test_runtime_safety tests.test_keyboard_teleop` - passed, 30 tests.
- `python3 -m unittest discover tests` - passed, 67 tests.
- `npm run build` - passed after the Pro Recording UI changes.
- In-app browser reload of `http://localhost:5173/` - passed; Pro Recording beta opened, move/version controls rendered, the removed return-to-hold checkbox was absent, and browser console error count was 0.

Hardware-session safety follow-up on 2026-04-30:

- User reported that starting Pro Recording after a held pose hit a runtime safety latch before recording: `arm_shoulder_lift` stalled with an 11.17 degree position error, no meaningful motion for 0.58s, and 767mA current over the 650mA stall threshold. The runtime correctly disabled all follower-arm torque.
- Follow-up state inspection found the dashboard could still attempt Start Recording/Live Control after this latch, and an errored recorder snapshot could continue to advertise arm authority as `recording`.
- Fixed the server to treat the runtime safety latch as a motion-command lockout. Start Recording, Live Control, Keyboard/Base Control, hotkey host arming, replay, go-home, calibration, training capture, and policy evaluation now fail before issuing Pi commands while the latch is active.
- Fixed control-authority reporting so safety latch reports arm/base as `none`, and errored/stale host snapshots no longer claim active recording, keyboard, leader, or VEX live-base authority.
- Fixed foreground transient SSH failures to surface as `lastError` instead of being logged as ignorable noise.
- Updated the UI to show a runtime safety latch banner and disable robot-start actions while the latch is active. Stop, Emergency Stop, and Reset Pi Connections remain available.
- Added regression tests for blocked recording before Pi commands, errored recorder authority cleanup, and foreground transient SSH error visibility.
- `npm run test:server` - passed, 27 tests.
- `npm run build:server` - passed.
- `npm run build` - passed.
- In-app browser reload of `http://localhost:5173/` - passed; Overview, Pro Recording, and Game Builder navigation were present and browser console error count was 0.

Physical robot validation checklist:

- Verify `ArrowUp` drives forward and decreases Y distance.
- Verify `ArrowDown` drives backward and increases Y distance.
- Verify `ArrowLeft` strafes left and decreases X distance.
- Verify `ArrowRight` strafes right and increases X distance.
- Verify `O` and `P` rotate in opposite, documented directions.
- Verify manual keyboard calibration signs affect manual keyboard control only, not sensor replay.
- Verify VEX controller secondary control and deadband behavior.
- Verify Drive/ECU/Crawl speed presets on real hardware.
- Verify manual idle braking modes do not create jitter or drift.
- Verify replay/preposition HOLD behavior on VEX motors.
- Verify recording from hold returns to hold.
- Verify replay and pinned moves return to hold when expected.
- Verify keyboard arm does not resume after recording/replay while hold is active.
- Verify leader stale state blocks unsafe leader-follower resume with physically separated leader/follower poses.
- Verify gripper/claw movement marks leader stale on real follower motion.
- Verify Emergency Stop + Torque Off disables follower torque and stops VEX base motion.
- Verify temperature/current/stall safety latches stop unsafe behavior under real load.

Final audit conclusion:

- Code-level validation passed.
- Physical robot validation is still required before claiming the robot works.
