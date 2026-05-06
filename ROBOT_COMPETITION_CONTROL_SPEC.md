# Robot Competition Control Specification

This document turns the current robot-control discussion into a technical
source of truth for the repository. It is intentionally detailed because the
main risk is not one missing button. The main risk is ambiguous control
authority between the leader arm, keyboard arm controls, VEX base controls,
recording, replay, and safety logic.

The immediate target is a reliable contest workflow under a short deadline. The
design priority is:

1. Prevent two systems from commanding the same arm at the same time.
2. Keep the VEX base usable while the arm is held, recorded, or replayed.
3. Make recording and replay repeatable enough for the ECU fuse and container
   tasks.
4. Keep every safety latch stricter than operator convenience.
5. Move experimental tools out of the critical path unless they are actively
   needed.

## Repository Context

The current application is a local Mac dashboard backed by an Express server
and Pi-side Python helpers.

- UI entry point: `src/App.tsx`
- Shared dashboard types: `src/types.ts` and `server/types.ts`
- Backend routes: `server/index.ts`
- Backend orchestration: `server/robotController.ts`
- Persistent UI config: `.lekiwi-ui/config.json`
- Pi host process: `scripts/lekiwi_host.py`
- Pi recording process: `scripts/lekiwi_record_trajectory.py`
- Pi replay process: `scripts/lekiwi_replay_trajectory.py`
- Keyboard and optional leader relay: `scripts/lekiwi_keyboard_teleop.py`
- Runtime safety, sensor, torque, and power helpers: `scripts/lekiwi_runtime.py`
  and `scripts/lekiwi_power.py`
- VEX Brain bridge: `scripts/vex_base_bridge.py`
- Sensor-aware VEX positioning and replay logic: `scripts/lekiwi_sensor_replay.py`
- VEX Brain reference docs: `Optional/VEX_V5_Base_Brain/README.md`
- Existing dashboard overview docs: `CONTROL_UI.md`

The current default hardware assumptions are:

- Pi host: `rawr.local`
- Pi fallback host: `10.42.0.1`
- Hotspot SSID: `rawr-hotspot`
- Pi trajectory directory: `/home/pi/lekiwi-trajectories`
- VEX telemetry slot: 8
- VEX replay slot: 7
- VEX inertial sensor port: 4
- VEX drive motors:
  - front right: port 1, reversed
  - front left: port 2
  - rear right: port 9, reversed
  - rear left: port 10
- VEX controller axes:
  - axis3: forward/back
  - axis4: strafe
  - axis1: turn

## Vocabulary

Follower arm:
The powered SO-101 arm on the robot. This is the arm that actually moves pieces.

Leader arm:
The Mac-connected SO leader arm used as a teleoperation input device.

Arm keyboard:
The keyboard keys that command follower arm joints:

- `Q/A`: shoulder pan
- `W/S`: shoulder lift
- `E/D`: elbow
- `R/F` and `Y/H`: wrist flex
- `T/G`: wrist roll
- `Z/X`, `C/V`, and `B/N`: gripper

Base keyboard:
The keyboard keys that command the VEX base:

- `ArrowUp/ArrowDown`: forward/back
- `ArrowLeft/ArrowRight`: left/right strafe
- `O/P`: rotate left/right
- `0`: toggle Drive/ECU speed
- `Esc`: stop keyboard capture

The current repository still has the older `I/K`, `J/L`, and `U/O` keyboard
mapping in `scripts/lekiwi_keyboard_teleop.py`. The beta requirement is to move
the contest-facing base controls to arrow keys plus `O/P` rotation because that
is easier to operate under pressure and avoids depending on analog VEX
controller values.

VEX controller:
The VEX handheld controller connected to the VEX Brain. It can drive the base
independently of the arm, as long as the VEX Brain telemetry program is running
and no replay/preposition command is temporarily taking over the base.

VEX Brain:
The base controller. It streams base velocity, wheel encoder positions, inertial
rotation, inertial heading, inertial rate, pose epoch, and mixer version to the
Pi over USB serial.

Hand-guide:
A recording mode where follower arm torque is disabled and the operator moves
the follower arm directly by hand. The recorder stores observed follower arm
positions instead of sending powered arm commands.

Home:
A saved arm pose used as a known reference and optional replay start/end pose.
Home is not the same thing as a hold pose. Home is a general safe reference.

Hold arm pose:
A pinned recording whose final arm pose becomes the active hold pose. While a
hold is active, the robot should be able to move the base while the arm stays at
that pose. Other Pi pinned moves should return to that pose after they run.

Pinned move:
A saved recording exposed as a dashboard button and optional keyboard hotkey.

Chain-link:
A saved ordered sequence of recordings that can be replayed back to back, with
optional confirmation between blocks.

Auto VEX positioning:
The sensor-assisted preposition step that tries to align the VEX base to the
recording start using the inertial sensor plus ultrasonic X/Y readings.

## Core Control Rules

### Rule 1: one arm authority at a time

The follower arm must have exactly one active arm authority:

- leader arm
- keyboard arm
- hand-guide observation
- replay
- go-home command
- hold pose
- none

The leader arm and keyboard arm controls must not both command the follower arm
as peers. The current code can let keyboard arm keys temporarily override the
leader target, then the leader target takes over again when keyboard arm keys
are released. That causes the follower arm to jump back toward the leader arm
position. This is the exact behavior that must be avoided.

Keyboard and leader can both be connected to the system, but only one of them
can own the arm at a time.

### Rule 2: arm authority is separate from base authority

The VEX base can be active while the arm is controlled by the leader, keyboard,
hold, or replay, because the base and arm are different hardware paths.

Valid combinations:

- leader arm + VEX controller base
- leader arm + keyboard base
- keyboard arm + VEX controller base
- keyboard arm + keyboard base
- hold arm pose + VEX controller base
- hold arm pose + keyboard base
- arm-only replay + stationary base
- arm replay + VEX base replay, when explicitly requested

Invalid or risky combinations:

- leader arm + keyboard arm as simultaneous arm authorities
- hand-guide follower arm + powered follower arm commands
- VEX controller manual drive + Pi VEX replay command at the exact same time
- VEX controller manual drive + Pi VEX preposition command at the exact same time
- auto VEX positioning when objects block the ultrasonic sensors

### Rule 3: VEX positioning is optional and should be off by default

Auto VEX positioning is useful only when the X/Y ultrasonic sensors have a clear
view of the same surfaces that existed at recording time. It is not reliable
when circular pieces, dowels, fuses, hands, containers, or other objects block
the left or front ultrasonic sensors.

For contest use, auto VEX positioning should remain available as a checkbox but
should be unchecked by default. The operator should intentionally enable it only
for positions where the sensor references are known to be clear and repeatable.

### Rule 4: safety latches override all controls

If the safety latch trips, everything else loses authority. The arm should stop,
torque should be disabled, and the operator should inspect the robot before
restarting the Pi host.

The current runtime has the right philosophy:

- hard temperature limit: 50.0 C
- over-temperature confirmation: 2 samples and 0.4 seconds
- stall detection based on command error, lack of motion, and current evidence
- gripper soft force hold before full fault when closing current is high
- recommended torque caps per arm motor

This behavior should stay strict. The project already had a servo smoke event,
so safety limits are not cosmetic.

### Rule 5: recording should not restart more of the system than necessary

Starting and stopping a recording currently replaces the live host with the
recorder, then may restore keyboard control afterwards. This can take several
seconds and creates too much mode churn.

The preferred future design is a warm Pi host that can receive commands:

- start recording to path and label
- stop recording and save
- start replay
- stop replay
- capture home
- go home
- zero VEX gyro

The current warm-host replay path already points in this direction. Recording
should eventually use the same model.

## Current Dashboard Behavior

### Overview

The Overview page has quick actions for live control, process reset, hotkeys,
home, emergency stop, calibration, power telemetry, sensors, and logs.

Start Keyboard + Leader:

- Checks the leader arm status.
- Connects to the Pi.
- Ensures remote helpers exist.
- Optionally syncs and runs VEX telemetry.
- Writes torque limits to the Pi.
- Starts the Pi host.
- Starts the local keyboard teleop relay.
- Passes the leader port when the leader is connected.
- Falls back to keyboard-only arm control when the leader is unavailable.

Important correction:
This mode should not mean "keyboard arm and leader arm can both drive the arm at
the same time." It should mean "keyboard base is available and the selected arm
source is available." If leader arm is the arm source, keyboard arm keys should
be ignored. If keyboard arm is the arm source, leader arm input should not be
polled or should be ignored.

Stop Control:

- Stops the local teleop process.
- Stops the Pi host.
- Clears active arm hold.
- Clears replay-control restore state.

Reset Pi Connections:

- Closes auxiliary SSH/SFTP sessions.
- Resets host, replay, calibration, dataset capture, and policy-eval SSH
  connections.
- Stops local teleop.
- Clears cached reachability and active hold state.
- This exists because too many simultaneous SSH sessions to the Pi can leave the
  dashboard in a broken state.

Start Keyboard + Leader Backup:

- Starts a keyboard backup control session.
- Still allows leader input when leader is connected.
- This should be renamed or reworked because the name implies the same conflict:
  keyboard and leader together are fine for the whole system, but not for the
  arm authority path.

Arm Hotkeys:

- Starts a command-ready Pi host without a teleop stream.
- Intended to make pinned Pi replays start quickly.
- The label is confusing because page-level hotkeys are already active whenever
  the page is focused and the user is not typing. A better label is "Arm Replay
  Hotkeys" or "Prime Pi for Pins."

Set Arm Home:

- Requires or creates a command-ready Pi host.
- Sends a `capture-home` warm-host command.
- Saves the current follower pose as home.

Go Home:

- Requires a saved home pose.
- Stops live teleop so it does not fight the home command.
- Sends a `go-home` warm-host command.
- Moves the follower arm to saved home without using the VEX Brain.

Emergency Stop + Torque Off:

- Stops teleop, replay, host, calibration, dataset capture, and policy eval.
- Runs the stop-all recovery script on the Pi.
- Clears active arm hold.
- This is the correct panic path and should stay visually separate from normal
  Stop Control.

Calibration:

- Pi follower calibration follows the LeRobot style calibration prompts.
- Mac leader calibration does the same for the leader arm.
- UI can send `c`, Enter, or stop to either calibration process.
- Individual follower servo calibration exists from the power/torque panel.

Power telemetry:

- Reads motor-side voltage, current, power, and temperature.
- Estimates battery percentage from voltage.
- Current defaults estimate empty at 9.6 V and full at 12.6 V.
- This matches the hardware reality that the motor control path is only useful
  inside a limited voltage range.

Sensor status:

- Gyro card shows VEX inertial rotation/heading state.
- Sensor 1 (X) is the left-mounted ultrasonic sensor for strafe/reference X.
- Sensor 2 (Y) is the front-mounted ultrasonic sensor for forward/reference Y.
- Sensor feed logs are parsed from `[sensor-status]` lines emitted by Pi
  processes.

### Recordings

Recording label:

- Stored in the trajectory metadata and used as the display name.

Recording mode:

- Auto: currently uses leader if connected, otherwise keyboard.
- Leader + Keyboard: currently means leader can provide arm input and keyboard
  can provide arm/base input.
- Keyboard only: uses keyboard arm and base input, with no leader arm.
- Hand-guide: disables follower arm torque and records observed follower poses.

Required correction:
"Leader + Keyboard" should be renamed to "Leader arm + keyboard/VEX base" and
must not allow keyboard arm keys to fight leader arm input. Keyboard base keys
can stay available.

Start Recording:

- Starts a Pi-side recorder.
- For leader/keyboard modes, starts local keyboard teleop into the recorder.
- For hand-guide mode, disables follower arm torque and records immediately.
- Records a JSON trajectory on the Pi.
- Records follower arm state, base state, VEX motor state, inertial state,
  pose epoch, ultrasonic distances, and optionally X/Y derived positions.

Stop Recording:

- Stops local teleop.
- Stops the recorder and saves trajectory JSON.
- Refreshes the recording list from the Pi when possible.
- If it was a keyboard recording, it currently restores keyboard backup control.

Required correction:
If an arm hold is active, Stop Recording should return the follower arm to the
active hold pose and restore base-only keyboard/VEX control. It should not
restore leader arm authority or free keyboard arm authority unless the operator
explicitly asks for that.

Zero VEX Gyro:

- Sends a command to reset VEX inertial rotation/heading origin.
- Important because the inertial sensor does not know the contest field's true
  zero after power cycles or program restarts.

Save Start as Home:

- Reads the first valid arm pose from the selected recording.
- Saves it as the global home pose.

Replay Selected Move:

- Can target the Pi follower or the leader arm.
- Pi replay can be arm-only or include VEX base motion.
- Pi replay can optionally use auto VEX positioning before arm replay.
- Replay speed is critical because contest tasks must be completed quickly.
- Hold Final currently defaults to 0.5 seconds. This should remain adjustable
  and should be evaluated per move.

Replay on Leader:

- This was experimental and is not useful for the contest critical path.
- Keep it hidden, secondary, or clearly marked as experimental.
- Do not rely on it for actual field performance.

Servo recording adjustments:

- Current editor allows changing six arm joint values at one timestamp.
- This is too slow when leader recordings create hundreds of samples.
- It is useful for tiny corrections, but not for reshaping full trajectories.
- A timeline/3D editor is the correct long-term direction.

Trim recording:

- Lets the operator keep only the useful action window.
- This is useful and should remain prominent because accidental lead-in and
  lead-out time wastes contest time.

Duplicate recording:

- Useful before recapturing ultrasonic X/Y streams or making destructive edits.

Replay + Recapture X/Y:

- Replays the whole recording and overwrites the ultrasonic stream with live
  sensor readings.
- Useful only when the base/sensor view is repeatable.

### Pins

Pinned moves:

- Store the trajectory path, replay target, speed, hold-final time, home mode,
  base replay choice, VEX replay mode, auto VEX positioning options, hotkey, and
  hold toggle status.

Hotkeys:

- Fire when the dashboard page is focused.
- Do not fire while typing in a field.
- Should be used for fast contest operation.

Hold arm pose toggle:

- A pinned Pi follower move can become a hold toggle.
- When turned on, the backend reads the final arm pose of that recording.
- It then replays the hold recording as arm-only with no VEX positioning.
- The final pose becomes `activeArmHold`.
- Other Pi pinned moves should return to that held pose after they run.
- When toggled off, the active hold is cleared.

Required correction:
When hold is active, the restored control mode after replay should be base-only
with arm input disabled. This lets the operator line up the VEX base without the
keyboard arm or leader arm accidentally moving the follower arm out of the hold
pose.

Chain-links:

- A chain-link is an ordered list of recording replay blocks.
- Each block stores its own replay options.
- Blocks can be reordered by drag/drop.
- Blocks can require confirmation between steps.
- Home dots show before/after home behavior.

Contest recommendation:
Chain-link is promising, but only after single recordings are reliable. Use
manual pinned hotkeys first. Use chain-links only for sequences that have been
tested repeatedly and can safely fail between confirmed steps.

### Training

Training is not part of the immediate contest plan.

Available modes:

- Leader
- Free-teach
- Leader as follower (Mac)

This can stay in the app, but it should not distract from recording, pins, hold
pose, and base alignment until the contest workflow is reliable.

### Settings

Settings include:

- hotspot and Pi connection details
- Pi/Mac project paths
- host runtime settings
- torque limits
- VEX telemetry/replay slots
- VEX inertial port
- VEX motor ports and reversal
- VEX controller axes and inversion
- VEX drive tuning
- safer servo mode

Important current behavior:

- Legacy LeKiwi base control is forced off.
- The VEX Brain/controller is the base drive path.
- Safer servo mode keeps wrist roll continuous and clamps other joints.

## Control Authority Matrix

This table is the desired behavior, not just current behavior.

| Arm authority | Base authority | Allowed | Notes |
| --- | --- | --- | --- |
| Leader arm | VEX controller | Yes | Good normal teleop mode. |
| Leader arm | Keyboard base | Yes | Keyboard base keys can move VEX base while leader owns arm. |
| Keyboard arm | VEX controller | Yes | Good fallback and recording mode. |
| Keyboard arm | Keyboard base | Yes | Single keyboard can control arm and base. |
| Leader arm | Keyboard arm | No | This causes target reversion and arm jumps. |
| Hold pose | VEX controller | Yes | Needed for row alignment. |
| Hold pose | Keyboard base | Yes | Needed for row alignment from the Mac. |
| Hold pose | Keyboard arm | No by default | Arm should stay locked until recording starts or hold is turned off. |
| Hold pose | Leader arm | No | Release the hold before giving authority back to the leader. |
| Replay | None/stationary base | Yes | Default safe replay. |
| Replay | VEX replay | Yes, explicit | Only when "Replay VEX base with this move" is enabled. |
| Replay | VEX controller | No | Controller should not fight Pi replay. |
| Auto VEX positioning | VEX controller | No | Positioning owns the base during correction pulses. |
| Hand-guide | Base hold | Yes | Follower arm torque is off; base should hold idle. |
| Hand-guide | Powered arm control | No | Torque-off observation and powered command conflict. |
| Calibration | Any other control | No | Calibration needs exclusive access. |
| Emergency stop | Any | Stops all | Safety wins. |

## Desired Control State Model

The backend should eventually expose explicit authority state instead of
inferring everything from process names and metadata.

Recommended model:

```ts
type ArmAuthority =
  | "none"
  | "leader"
  | "keyboard"
  | "handGuide"
  | "holdPose"
  | "replay"
  | "goHome"
  | "calibration"
  | "emergencyStop";

type BaseAuthority =
  | "none"
  | "vexController"
  | "keyboard"
  | "vexReplay"
  | "vexPreposition"
  | "hold";

interface ControlAuthorityState {
  arm: ArmAuthority;
  base: BaseAuthority;
  activeHoldId: string | null;
  activeHoldName: string | null;
  leaderConnected: boolean;
  keyboardCaptureActive: boolean;
  vexTelemetryActive: boolean;
  safetyLatched: boolean;
}
```

The UI should display this as the operator-facing truth:

- Arm: Leader, Keyboard, Held, Recording, Replay, Calibration, or Stopped
- Base: VEX Controller, Keyboard, Replay, Positioning, Held, or Stopped
- Hold: Off or active hold name
- Safety: Clear or latched

## Critical UI Restructure

### Overview should become a control cockpit

Group Overview into:

1. System Health
   - Pi reachable
   - Leader connected
   - VEX Brain connected
   - VEX telemetry active
   - Gyro state
   - X/Y ultrasonic state
   - safety latch status
   - battery and temperature summary

2. Live Control
   - Arm source selector: Leader or Keyboard
   - Base source selector: VEX Controller or Keyboard
   - Start Live Control
   - Stop Control
   - Reset Pi Connections

3. Hold and Home
   - active hold status
   - Go Home
   - Set Home
   - clear hold

4. Safety
   - Emergency Stop + Torque Off
   - torque limit preset
   - hottest servo
   - peak current motor

5. Calibration
   - Pi follower calibration
   - Mac leader calibration
   - individual servo calibration

Rename confusing buttons:

- "Start Keyboard + Leader" -> "Start Live Control"
- "Start Keyboard + Leader Backup" -> "Start Keyboard/Base Control"
- "Arm Hotkeys" -> "Prime Pi for Pinned Moves"
- "Replay on Leader" -> "Replay on Leader (Experimental)"
- "Speed position fixing" -> "VEX positioning speed"

### Recording page should focus on fast repeatable move creation

Top area:

- Recording name
- Recording source:
  - Leader arm
  - Keyboard arm
  - Hand-guide follower
  - Keyboard from active hold
- Start Recording
- Stop Recording
- active hold indicator
- "return to hold after stop" indicator

Replay settings should be secondary, not mixed into the recording start flow.

Auto VEX positioning should be inside a collapsed "VEX positioning assist"
section and unchecked by default.

### Pins page should make hold pins visually distinct

Hold pins are not normal move pins. They define a persistent arm state.

The Pins page should show:

- active hold strip at top
- hold pins in a separate group
- normal action pins in another group
- hotkey conflicts highlighted
- per-pin options hidden behind the three-dot menu

When a hold is active:

- The UI should show "Arm held at <name>".
- It should show "Base control allowed".
- It should show "Arm keyboard disabled until recording or hold off" unless the
  user explicitly starts a keyboard-arm recording.

### Chain-link should stay optional

Chain-link is useful, but it should not become the first dependency for contest
operation. Keep it for after single pins are proven.

Required chain-link behaviors:

- Each block has explicit replay options.
- Each block can disable/enable VEX positioning.
- Each block can decide home behavior.
- The chain can require confirmation after each recording.
- A running chain can be stopped immediately.
- If a block fails, the chain stops and holds the base.
- If an active hold exists, each Pi block returns to hold unless the block says
  otherwise.

## Beta Competition UI Addendum

This addendum incorporates the requested UI direction from the gameplay
training notes. The goal is not to replace the current dashboard immediately.
The goal is to add two new contest-focused tabs that sit beside the older
screens:

- `Pro Recording (beta)`
- `Game Builder (beta)`

The `beta` label is the right approach if it means:

- the old Overview, Recordings, Pins, Chain-link, Training, Settings, and Logs
  pages remain available and unchanged while the new workflow is tested;
- the beta pages reuse the same backend safety rules instead of creating a
  second control system;
- operators understand that these are the new preferred contest workflows, but
  still have the older controls available if the beta workflow exposes a bug;
- beta does not mean relaxed safety. Emergency stop, torque limits, current
  latches, temperature latches, one-arm-authority rules, and reset behavior
  must be identical or stricter in the beta tabs.

The beta tabs should be treated as a cleaner operator layer over capabilities
that already exist or are already planned:

- named recordings;
- pinned moves;
- hotkeys;
- hold arm pose toggles;
- VEX base replay and positioning toggles;
- chain-link style sequencing;
- recording mode selection;
- sensor recording;
- manual override and stop/cancel behavior.

The important design rule is this: the beta pages can include old capabilities,
but they should only show the parts needed for fast competition recording and
gameplay. Training, policy learning, experimental leader replay, and the 3D
trajectory editor should not be on the critical contest surface.

Efficiency priority:

- The beta menus should prioritize speed and low operator effort slightly more
  than the old pages.
- This does not mean reducing safety checks. It means reducing repeated clicks,
  repeated naming, repeated mode setup, and repeated searching through raw
  recordings.
- The most common contest actions should be one click or one hotkey after setup:
  pick a move, record a new version, mark final, add to game plan, run move,
  pause, cancel, and switch base speed.
- Advanced options should be present but collapsed by default.
- The UI should remember the last safe choices per move category, such as
  recording type, sensor recording checkboxes, return-to-hold behavior, and
  playback speed.
- The operator should not need to re-enter the same label, category, hotkey, or
  VEX positioning choice for every version of the same move.

### Competition move library

The current UI makes recordings hard to interpret because recordings are mostly
raw files plus labels. For competition, recordings need to be organized around
the actual game actions the operator needs.

Create a first-class `MoveDefinition` concept. A move is not a recording file.
A move is the named action the team needs during the match. A move can have many
recording versions, and one version can be marked as the favorite/final version.

Recommended move fields:

```ts
interface MoveDefinition {
  id: string;
  label: string;
  category: "general" | "fuseCollection" | "fuseTransfer" | "boardCollection" | "boardTransfer";
  iconKind: "doorLeft" | "doorRight" | "circleBlue" | "circleYellow" | "circleGreen" | "board4" | "board6" | "board8" | "heightLow" | "heightHigh";
  colorToken: string;
  sortIndex: number;
  description: string;
  allowedRecordingTypes: RecordingType[];
  defaultRecordingType: RecordingType;
  defaultIncludeVexBaseReplay: boolean;
  defaultAutoVexPositioning: boolean;
  defaultRecordDistanceSensors: boolean;
  defaultRecordInertialSensor: boolean;
  favoriteVersionId: string | null;
  archived: boolean;
}

interface MoveRecordingVersion {
  id: string;
  moveId: string;
  version: number;
  recordingName: string;
  piTrajectoryPath: string | null;
  localDisplayName: string;
  recordedAtIso: string;
  recordingType: RecordingType;
  playbackSpeed: number;
  includesVexBaseSamples: boolean;
  includesDistanceSensors: boolean;
  includesInertialSensor: boolean;
  autoVexPositioningEnabled: boolean;
  isFavorite: boolean;
  notes: string;
  safetyStatusAtRecord: "clear" | "latched" | "unknown";
}

type RecordingType =
  | "keyboardControl"
  | "leaderArm"
  | "followerHandGuide"
  | "keyboardFromActiveHold";
```

The exact TypeScript names can change, but the separation should not change:
move definitions describe game intent; recording versions describe actual
trajectory files.

Initial move library:

| Category | Move label | Icon | Purpose |
| --- | --- | --- | --- |
| General | Open left beam door | Left door | Open the left beam door mechanism. |
| General | Open right beam door | Right door | Open the right beam door mechanism. |
| Fuse collection | Open claw position for driver to orient bot - blue | Blue circle | Put the claw in the blue fuse approach pose so the driver can line up the base. |
| Fuse collection | Close open claw -> blue slot | Blue circle | Close from the blue approach pose and place into the blue dowel/fuse storage slot. |
| Fuse collection | Open claw position for driver to orient bot - yellow | Yellow circle | Put the claw in the yellow fuse approach pose so the driver can line up the base. |
| Fuse collection | Close open claw -> yellow slot | Yellow circle | Close from the yellow approach pose and place into the yellow dowel/fuse storage slot. |
| Fuse collection | Open claw position for driver to orient bot - green | Green circle | Put the claw in the green fuse approach pose so the driver can line up the base. |
| Fuse collection | Close open claw -> green slot | Green circle | Close from the green approach pose and place into the green dowel/fuse storage slot. |
| Fuse removal/insertion | Low fuse height | Low height | Move the arm to the low fuse insertion/removal height. |
| Fuse removal/insertion | High fuse height | High height | Move the arm to the high fuse insertion/removal height. |
| Fuse removal/insertion | Low fuse height repeat | Low height | Second low-height utility move if the gameplay flow needs a distinct low-height version. |
| Fuse removal/insertion | Green slot -> low fuse height | Green circle + low height | Move a green stored fuse from its slot to the low fuse height. |
| Fuse removal/insertion | Yellow slot -> high fuse height | Yellow circle + high height | Move a yellow stored fuse from its slot to the high fuse height. |
| Fuse removal/insertion | Blue slot -> low fuse height | Blue circle + low height | Move a blue stored fuse from its slot to the low fuse height. |
| Circuit board collection | Open claw 8 inch board height | 8 inch rectangle | Put the claw at the 8 inch board pickup height. |
| Circuit board collection | Close open claw -> 8 inch board slot | 8 inch rectangle | Close from the 8 inch board approach pose and place in the 8 inch board slot. |
| Circuit board collection | Open claw 4 inch board height | 4 inch rectangle | Put the claw at the 4 inch board pickup height. |
| Circuit board collection | Close open claw -> 4 inch board slot | 4 inch rectangle | Close from the 4 inch board approach pose and place in the 4 inch board slot. |
| Circuit board collection | Open claw 6 inch board height | 6 inch rectangle | Put the claw at the 6 inch board pickup height. |
| Circuit board collection | Close open claw -> 6 inch board slot | 6 inch rectangle | Close from the 6 inch board approach pose and place in the 6 inch board slot. |
| Circuit board removal/insertion | 6 inch board height | 6 inch rectangle | Move the arm to the 6 inch board insertion/removal height. |
| Circuit board removal/insertion | 8 inch board height | 8 inch rectangle | Move the arm to the 8 inch board insertion/removal height. |
| Circuit board removal/insertion | 4 inch board height | 4 inch rectangle | Move the arm to the 4 inch board insertion/removal height. |
| Circuit board removal/insertion | 6 inch slot -> 6 inch board height | 6 inch rectangle | Move the 6 inch stored board from its slot to the 6 inch target height. |
| Circuit board removal/insertion | 8 inch slot -> 8 inch board height | 8 inch rectangle | Move the 8 inch stored board from its slot to the 8 inch target height. |
| Circuit board removal/insertion | 4 inch slot -> 4 inch board height | 4 inch rectangle | Move the 4 inch stored board from its slot to the 4 inch target height. |

Notes on the repeated "open claw position for driver to orient bot" entries:

- These should be separate move tiles because the driver needs a quick,
  color-specific target during the match.
- Internally they can share the same underlying hold pose if testing proves one
  row hold is sufficient.
- The UI should still show them as blue/yellow/green so the driver does not
  have to translate "row hold" into a game action under time pressure.

Notes on the repeated low fuse height entry:

- It should not be deleted from the initial library.
- It should be kept as a separate editable tile until the team confirms whether
  both low-height actions can use the same final recording.
- If the duplicates are later proven identical, one can be archived rather than
  destroyed. Archiving preserves the team history and avoids breaking any game
  plan that references it.

### Pro Recording (beta)

`Pro Recording (beta)` should be the main place for creating and improving
competition recordings. It should not look like a raw file browser. It should
look like a move library.

Top-level layout:

1. Category filter bar
   - All
   - General
   - Fuse collection
   - Fuse removal/insertion
   - Circuit board collection
   - Circuit board removal/insertion

2. Move tile grid
   - Use a 4-column grid on normal desktop width.
   - Keep tile sizes stable so the grid does not shift as versions are added.
   - If the viewport is small, collapse to 2 columns or 1 column.
   - The friend suggested a 4x4 tile grid. The best practical version is a
     4-column paged or scrollable grid, because the initial move list has more
     than 16 moves. Do not force the whole library into exactly 16 tiles.
   - Sort by category, then `sortIndex`.
   - Color-code by category and target object.
   - Use small icons:
     - green/yellow/blue circles for fuse/dowel targets;
     - 4 inch, 6 inch, and 8 inch rectangles for board heights;
     - left/right door icons for beam door moves;
     - low/high height markers for fuse insertion heights.
   - Show a checkmark or "final" indicator if the move has a favorited version.
   - Show a warning indicator if the favorite version has missing data, stale
     sensor metadata, or a safety warning.
   - Show the number of versions on the tile.

3. Move management actions
   - Add move
   - Archive/delete move
   - Duplicate move
   - Rename move
   - Change category
   - Change icon/color

Delete behavior should be conservative:

- If a move has no recording versions, delete can remove it.
- If a move has versions, prefer archive.
- If a move is used in a game plan, block deletion until the plan is updated.

Selecting a move opens a move-specific recording menu.

Move-specific recording menu:

1. Header
   - Move label
   - Category
   - Icon/color
   - Favorite/final status
   - Active hold status
   - Pi connection status
   - VEX connection status
   - safety latch status

2. Version list
   - `v1`, `v2`, `v3`, etc.
   - recorded time
   - recording type
   - playback speed
   - VEX base samples present or absent
   - distance sensor samples present or absent
   - inertial samples present or absent
   - favorite selector
   - replay/test button
   - rename/notes
   - duplicate before edit

3. Recording setup
   - Recording type:
     - Keyboard control
     - Leader arm
     - Follower arm hand-guide
     - Keyboard from active hold
   - Record distance sensors checkbox
   - Record inertial sensor checkbox
   - Include VEX base samples checkbox
   - Auto VEX positioning on replay checkbox, default off
   - Return to active hold after stop checkbox, default on when a hold exists
   - Playback speed for the new version

4. Recording actions
   - Start recording
   - Stop recording
   - Stop and return to hold
   - Emergency stop
   - Replay selected version
   - Mark selected version as favorite/final

Recording type behavior:

- Keyboard control:
  - Keyboard can command arm/claw.
  - Keyboard or controller can command the VEX base, if the operator explicitly
    enables base recording.
  - Leader arm input is ignored for arm authority.

- Leader arm:
  - Leader arm commands the follower arm.
  - Keyboard arm keys are ignored.
  - Keyboard VEX base movement is allowed because base authority is separate
    from arm authority.
  - This matches the core rule: leader arm and keyboard arm cannot both command
    the arm, but leader arm and keyboard VEX base can be active together.

- Follower arm hand-guide:
  - Follower torque is off.
  - The operator physically moves the follower arm.
  - The system records observed follower positions.
  - This mode must show a warning until the torque-off position accuracy issue
    is solved.
  - It should not be the default for contest-critical moves.

- Keyboard from active hold:
  - The move starts from the currently active hold pose.
  - Arm keyboard control is enabled only for the recording session.
  - On stop, the arm returns to the same hold pose.
  - This should be the P0 workflow for the fuse row because it avoids the leader
    snapping problem.

Sensor recording behavior:

- Distance sensors and inertial sensor recording should be selectable per move
  version.
- Recording the sensor data is not the same as requiring VEX positioning during
  replay.
- A version can record sensor data for debugging while still replaying with
  auto VEX positioning off.
- If distance sensors are disabled, the replay should not attempt ultrasonic
  alignment.
- If inertial sensor recording is disabled, the replay should not require gyro
  matching.

Favorite/final behavior:

- Each move can have zero or one favorite version.
- Marking a version as favorite means "this is the current competition-ready
  version for this move."
- Marking a favorite adds the completion checkmark to the move tile.
- Favorite status should be easy to change because a better version may be
  recorded later.
- Game Builder should only offer moves with a favorite version.
- If the favorited recording file is missing from the Pi, the move should show
  a broken favorite warning and should not be playable until fixed.

Playback speed behavior:

- Playback speed should be editable per version.
- The default should be conservative.
- Increasing speed should require the operator to intentionally set it.
- Speed changes should not modify the raw trajectory file. They should be
  replay metadata so the original recording remains recoverable.

### Game Builder (beta)

`Game Builder (beta)` should be the competition execution surface. It should
combine the useful parts of pins and chain-link without exposing every advanced
recording setting.

There are two modes:

- Editor mode
- Play mode

Editor mode is for building the match plan before running it.

Editor mode layout:

1. Favorite move palette
   - Only moves with a favorited/final recording appear.
   - Tiles use the same colors and icons as Pro Recording.
   - Tiles show category, hotkey, and favorite version number.
   - Tiles with broken Pi paths, stale files, or safety warnings are disabled.

2. Game sequence list
   - Drag favorite moves into a list.
   - Reorder moves by drag and drop.
   - Duplicate a move in the list if the same move is used multiple times.
   - Remove a move from the list without deleting the recording.
   - Give each step an optional display label.
   - Each step stores the move id and favorite version id at the time it was
     added.

3. Per-step options
   - Playback speed override.
   - Include VEX base replay, if the recording has base samples.
   - Use auto VEX positioning, if the recording has required sensor metadata.
   - Return to active hold after step.
   - Require confirmation after step.
   - Pause after step.

4. Hotkey binding
   - Bind a key to a move.
   - Bind a key to a game sequence step.
   - Highlight conflicts.
   - Prevent unsafe conflicts with emergency stop and core drive controls.
   - Allow changing hotkeys without rerecording a move.

5. VEX base hotkey menu
   - Default to arrow-key translation:
     - `ArrowUp`: forward
     - `ArrowDown`: back
     - `ArrowLeft`: left strafe
     - `ArrowRight`: right strafe
     - `O`: rotate left
     - `P`: rotate right
   - Keep the keys editable after testing, but do not force the operator to
     use the old `I/K`, `J/L`, and `U/O` layout.
   - Set Drive mode speed.
   - Set ECU mode speed.
   - Set the speed-mode toggle key, currently `0`.
   - Show the current base mode clearly.
   - Show a direction-test helper that reports whether each key makes the X/Y
     ultrasonic distance change in the expected direction.

6. Claw and arm hotkey menu
   - Set open claw key.
   - Set close claw key.
   - Set arm keyboard keys if keyboard arm control is active.
   - Keep claw hotkeys separate from base hotkeys.
   - Do not allow claw/arm hotkeys to override leader arm authority unless the
     selected manual mode allows it.

Game plan model:

```ts
interface GamePlan {
  id: string;
  name: string;
  steps: GamePlanStep[];
  createdAtIso: string;
  updatedAtIso: string;
}

interface GamePlanStep {
  id: string;
  moveId: string;
  favoriteVersionId: string;
  labelOverride: string | null;
  hotkey: string | null;
  playbackSpeedOverride: number | null;
  includeVexBaseReplay: boolean;
  autoVexPositioning: boolean;
  returnToActiveHold: boolean;
  requireConfirmationAfter: boolean;
  pauseAfter: boolean;
}
```

Play mode is for operating during the match or during match simulation.

Play mode layout:

1. Large active status strip
   - Pi connection
   - VEX connection
   - safety latch
   - active hold
   - arm authority
   - base authority
   - speed mode, Drive, ECU/Align, or Crawl if enabled

2. Playable sequence list
   - Click a step to run it.
   - Use bound hotkeys to run moves.
   - Show running, completed, skipped, failed, and canceled states.
   - Show the next planned step.

3. Manual override controls
   - Pause current replay.
   - Resume current replay, only if safe.
   - Cancel current replay.
   - Emergency Stop + Torque Off.
   - Switch to manual operation.
   - Clear active hold, if explicitly requested.

4. Manual operation selector
   - Keyboard arm control.
   - Leader arm control.
   - Follower hand-guide only if intentionally selected and safe.
   - Base keyboard control.
   - VEX controller control.

5. VEX speed toggle
   - Drive mode for fast travel.
   - ECU mode for slow alignment.
   - Optional Crawl mode for final alignment after trials prove the values.
   - Visible current speed/mode.
   - Hotkey and click target both supported.

Manual override rules:

- Pause/cancel must be available during every replay.
- Emergency stop must always win.
- Canceling a move should stop arm replay and base replay.
- After cancel, the UI should not silently restart leader or keyboard arm
  control. It should ask the operator to choose the next control mode.
- If hold is active and cancel occurs during a non-hold move, the system can
  offer "return to hold" but should not move without confirmation.
- Manual base movement should be available while the arm is held, because this
  is the core fuse-row strategy.

Mode switching risks:

- Switching from keyboard/follower control back to leader-follower control is
  dangerous if the follower arm moved but the leader arm did not.
- In that case, the leader pose is stale. Re-enabling leader-follower can pull
  the follower back toward the stale leader pose.
- Game Builder and Pro Recording should show "Leader out of sync" whenever the
  follower has been moved by keyboard, replay, hold, or hand-guide after the
  last leader sync.
- Leader-follower control should not resume from that state unless the operator
  explicitly chooses one of these actions:
  - move follower to leader pose;
  - discard leader authority and stay in keyboard/follower mode.
- Claw control has the same authority problem as the rest of the arm. If the
  follower gripper is opened/closed by keyboard while the leader gripper stays
  elsewhere, leader control must be treated as stale until resynced.

Move-to-move path risks:

- A recording usually assumes a start pose.
- The end pose of the previous recording may not equal the start pose of the
  next recording.
- If the next replay starts immediately, the arm can take a direct joint-space
  path that is mechanically valid but bad for the field, the game piece, or the
  containers.
- Every Game Builder step should therefore declare one of these transition
  policies:
  - start only if the robot is already within tolerance of the expected start
    pose;
  - return to the active hold pose before running;
  - run a named transition move first;
  - ask for manual confirmation before continuing.
- The default for contest should be conservative: return to hold or require
  confirmation unless the sequence has already passed repeated trial runs.

End-hold behavior:

- At the end of a recording, the follower motors should keep holding the final
  pose unless the selected mode explicitly says torque off.
- When keyboard control starts after a replay, the arm target should be seeded
  from the current observed follower pose.
- Keyboard base control may start while the arm remains held.
- Keyboard arm control should start only after the operator chooses it, because
  starting free keyboard arm control automatically can break the hold strategy.

Relationship to chain-link:

- Game Builder is the contest-friendly version of chain-link.
- The old Chain-link page can remain for debugging and advanced sequencing.
- Game Builder should use stricter defaults:
  - favorites only;
  - VEX positioning off unless enabled per step;
  - confirmation available between steps;
  - easy cancel/pause;
  - visible hotkeys;
  - no training controls.

### Beta features to exclude from the contest-critical path

The beta tabs should not become cluttered with features that are interesting but
not needed for the immediate competition workflow.

Do not put these on the main beta surfaces:

- model training or policy training;
- automatic camera positioning, unless hardware is added later;
- 3D trajectory editing as a required recording step;
- leader replay target as a normal option;
- advanced raw JSON editing;
- calibration internals that belong on the existing calibration/settings pages;
- every SSH/session reset detail, except a clear "Reset Pi Connections" action
  when the Pi is stuck.

These can still exist behind advanced/debug screens, but they should not slow
down the operator who is trying to create final recordings and run the match.

## Contest Workflow: ECU Fuse/Dowel Row Strategy

The important field setup described in the notes:

- The ECU fuses/dowels are in a horizontal row.
- The row is color-coded, likely blue, yellow, green.
- The pieces are cylindrical dowels, roughly 3 by 1.
- The goal is to hold the claw at a consistent row pickup pose.
- The base moves left/right to line up the claw with each dowel.
- Containers are mounted at the back.
- There are three dowel containers aligned with the claw.
- There are three circuit board containers aligned with the circuit board.

Appendix A details confirmed from the local court PDF
`ontario-skills-scope/2026_-_Robotics_-_Appendix_A_-_2025-09-19 (1).pdf`:

- The fuse holder drawing labels the fuse pieces as `ECU18`.
- `ECU18` fuses are 1 inch diameter wooden dowels.
- Each fuse is 3 inches long.
- There are 12 painted dowel fuses in the full material list.
- The drawing shows colors in a row: Green, Yellow, Blue.
- The fuses are painted for 2 inches of their length.
- The last 1 inch is left bare wood.
- Fuse holes are drilled all the way through.
- Fuse holes are 1 1/8 inch diameter for easy removal.
- The fuse holder base is labeled `FH1`, with a 3/4 plywood piece sized
  3/4 x 3 x 16 in the drawing.
- The `ECU Piece Storage x 3` drawing uses three green storage slots and calls
  for 5/8 inch spacing so 1/2 inch plywood ECU pieces fit loosely.
- Properly functioning circuit boards have a 3 1/2 inch wide green strip along
  the front-facing edge.
- Malfunctioning circuit boards are marked with a large X.

Operational meaning:

- The claw should be held at a consistent dowel-row approach height and depth.
- The base should do most of the left/right alignment work.
- The arm should not be re-aimed from scratch for each fuse.
- The row hold pose should reference the holder geometry, not the current
  ultrasonic reading from a dowel.
- Auto VEX positioning should not use the fuses themselves as sensor targets.

The strategic idea is correct:

1. Use one held arm pose for the row.
2. Move only the base to line up each target.
3. Record each pickup/drop from the same starting arm pose.
4. After each recording stops, return to the same held arm pose.
5. Pin each finished move to a hotkey.
6. Optionally chain them later after individual hotkeys work.

### Recommended immediate contest workflow

Preparation:

1. Start the dashboard.
2. Confirm Pi is reachable.
3. Confirm leader arm status, even if keyboard will be used.
4. Confirm VEX Brain status and telemetry.
5. Confirm the inertial sensor is streaming.
6. Confirm X and Y ultrasonic sensors are streaming, but do not depend on them
   unless they have clear line of sight.
7. Run Pi follower calibration if needed.
8. Run Mac leader calibration if leader will be used.
9. Load safer torque preset.
10. Confirm temperature and current are safe.
11. Set or refresh arm home.
12. Zero VEX gyro at the desired field reference if VEX positioning or replay
    uses heading.

Create the row hold:

1. Move the follower arm to the desired row pre-grasp pose.
2. The claw should line up with the dowel row height and back edge.
3. Record a short arm-only move that ends exactly at this pose, or use a direct
   hold-capture helper once implemented.
4. Pin it as a hold arm pose toggle.
5. Assign a hotkey, for example `G`.
6. Press the hotkey once.
7. Confirm the UI says the hold is active.
8. Confirm base controls still work.
9. Confirm arm keyboard keys do not move the arm while merely lining up the
   base.

Record dowel 1:

1. Turn on the row hold with the hold hotkey.
2. Use VEX controller or keyboard base to align the base with dowel 1.
3. Start keyboard-only recording from active hold.
4. Enable keyboard arm input for the recording only.
5. Close the claw with `X` or the selected gripper-close key.
6. Move the arm to the correct dowel container.
7. Release or place the piece.
8. Stop recording.
9. The system returns to the row hold pose.
10. Replay the recording once to verify it grabs and places correctly.
11. Pin it to a clear hotkey.

Record dowel 2:

1. Keep or reactivate the same row hold.
2. Move the base left/right to line up with dowel 2.
3. Repeat the pickup/drop recording.
4. Stop recording and return to the same hold.
5. Replay and pin.

Record dowel 3:

1. Keep or reactivate the same row hold.
2. Move the base to dowel 3.
3. Repeat the pickup/drop recording.
4. Stop recording and return to hold.
5. Replay and pin.

Optional chain after pins work:

1. Build a chain with the three pinned pickup/drop recordings.
2. Require confirmation after each block at first.
3. Run the chain slowly once.
4. Increase speed only after it succeeds repeatedly.

## Hold Arm Pose Requirements

Hold pose is the most important new workflow.

Required behavior:

1. A hold pin must target the Pi follower.
2. A hold pin must be arm-only.
3. A hold pin must disable auto VEX positioning.
4. The backend must read the final arm pose from the hold recording.
5. When toggled on, the follower replays to that final pose.
6. The final pose is stored as active hold.
7. While active hold exists, base control stays available.
8. While active hold exists, arm input is disabled during normal control.
9. A non-hold Pi pinned move should return to active hold after it finishes.
10. Recording from hold should start at that pose.
11. Stopping a recording from hold should return to that pose.
12. Toggling hold off clears the active hold and allows explicit arm control
    again.
13. Emergency stop clears active hold.
14. Reset Pi connections clears active hold.

Current partial implementation:

- `activeArmHold` exists in `server/robotController.ts`.
- Hold pins read the final pose of a recording.
- Hold pins can replay to the held pose.
- Non-hold Pi pinned moves can return to hold.
- Replay restore can start keyboard control with arm input disabled when hold is
  active.

Remaining important gap:

- Recording stop must preserve and return to active hold.
- Live control labels and modes must make it obvious when arm input is disabled.
- Leader arm should not be reintroduced after a held replay or held recording
  unless explicitly requested.

## VEX Positioning Requirements

Auto VEX positioning should be treated as an assist, not a guaranteed navigation
system.

Current recorded sensor keys:

- `ultrasonic_sensor_1.distance_m`
- `ultrasonic_sensor_2.distance_m`
- `ultrasonic_sensor_1.position_m`
- `ultrasonic_sensor_2.position_m`
- `vex_inertial_rotation.deg`
- `vex_inertial_heading.deg`
- `vex_inertial_rate_z.dps`
- `vex_pose_epoch`
- VEX wheel encoder positions

Current algorithm behavior:

- Uses recorded start state as the target.
- Uses live ultrasonic X/Y and inertial heading/rotation as feedback.
- Corrects heading, X, and Y in staged pulses.
- Uses tolerances and trim tolerances to decide when to accept or abort.
- Aborts arm replay if it cannot align when positioning was required.

Known limitations:

- Ultrasonic sensors see obstacles, not semantic field landmarks.
- Cylindrical pieces can corrupt readings.
- Hands, containers, fuses, and nearby geometry can block sensors.
- If the VEX Brain was restarted, the inertial zero may not match the recording
  unless the gyro zero was marked and reset correctly.
- If the sensor view differs from the recording, the base will correct to the
  wrong physical place.

Required default:

- New recordings and pins should default `autoVexPositioning` to false.
- The UI should keep the checkbox available.
- When enabled, the UI should warn that it requires clear sensor references.

Good uses:

- ECU section where side/front references are stable and unblocked.
- Repeatable alignment against walls or flat field edges.
- Testing sensor behavior.

Bad uses:

- Front pickup area where loose pieces can block sensors.
- Any area where the ultrasonic target is the dowel or object being manipulated.
- Any run where speed matters more than correction accuracy.

## VEX Base Direction, Keyboard, and Braking Requirements

The VEX base direction problem should be handled as a direction-calibration
problem, not as a blind motor-reversal change.

Reported issue:

- VEX controls feel reversed.
- Attempts to reverse them back to normal caused other problems.
- The VEX controller uses analog joystick values, which can introduce deadband,
  inversion, jitter, and driver interpretation issues.
- For contest use, keyboard base control should become the primary manual base
  control path because it is discrete, easier to document, and easier to test.

The current repo has multiple layers that can invert the base:

- motor physical wiring and port placement;
- per-motor `reversed` flags in `server/defaultConfig.ts`;
- VEX controller axis selection and inversion in `settings.vex.controls`;
- keyboard key-to-axis mapping in `scripts/lekiwi_keyboard_teleop.py`;
- the VEX Brain drive mixer generated by `scripts/vex_base_bridge.py`;
- sensor-assisted preposition correction signs in `scripts/lekiwi_sensor_replay.py`.

Repository alignment note:

- `server/defaultConfig.ts` currently sets front-right and rear-right VEX drive
  motors as reversed.
- `scripts/lekiwi_keyboard_teleop.py` currently uses the older letter-key base
  bindings.
- `scripts/vex_base_bridge.py` converts semantic `x.vel`, `y.vel`, and
  `theta.vel` commands into VEX wheel percentages.
- `tests/test_sensor_replay.py` already asserts that X is strafe, Y is
  forward, and turn-only commands use the expected left/right wheel pattern.
- Any implementation should update these existing files intentionally instead
  of building a parallel base-control path that ignores them.

Do not fix direction by flipping random motor reversals. A motor reversal change
can make the base feel correct in one mode while breaking VEX replay or
sensor-assisted positioning. The correct approach is to define the semantic
motion signs once, test them against sensor feedback, and then keep every input
device mapped to that semantic layer.

### Required contest keyboard mapping

Use this as the beta default:

| Key | Operator meaning | Semantic command |
| --- | --- | --- |
| `ArrowUp` | Forward | positive `y.vel` |
| `ArrowDown` | Back | negative `y.vel` |
| `ArrowLeft` | Left strafe | positive `x.vel` |
| `ArrowRight` | Right strafe | negative `x.vel` |
| `O` | Rotate left | negative or calibrated `theta.vel` |
| `P` | Rotate right | positive or calibrated `theta.vel` |
| `0` | Toggle speed preset | Drive/ECU, or cycle through tested presets |
| `Esc` | Stop keyboard capture | zero base command and stop keyboard session |

The X/Y signs above intentionally follow the existing sensor-positioning
behavior:

- Sensor 1 / X is mounted on the left side.
- When the X distance gets smaller, the base has moved left toward the X-side
  reference.
- When the X distance gets bigger, the base has moved right away from the
  X-side reference.
- Therefore `ArrowLeft` should create the same semantic sign used by VEX
  prepositioning when it needs X distance to get smaller.
- Sensor 2 / Y is mounted on the front.
- When the Y distance gets smaller, the base has moved forward toward the
  front reference.
- When the Y distance gets bigger, the base has moved back away from the front
  reference.
- Therefore `ArrowUp` should create the same semantic sign used by VEX
  prepositioning when it needs Y distance to get smaller.

In the current sensor replay tests, the fake base model applies:

```py
state["x"] -= motion["x.vel"] * dt_s
state["y"] -= motion["y.vel"] * dt_s
```

That means positive `x.vel` makes X distance smaller, and positive `y.vel`
makes Y distance smaller. The beta keyboard mapping should match that
convention unless real-field testing proves the physical robot reports the
opposite. If the physical robot is opposite, add an explicit base direction
calibration setting; do not silently change every consumer of `x.vel` and
`y.vel`.

Implementation detail for macOS keyboard capture:

- `scripts/lekiwi_keyboard_teleop.py` currently maps letter virtual key codes
  but not arrow keys.
- The arrow keys need to be added to `MAC_VK_TO_KEY` for Quartz capture and to
  the pynput key handling path.
- macOS virtual key codes are typically:
  - left arrow: `123`
  - right arrow: `124`
  - down arrow: `125`
  - up arrow: `126`
- The implementation should still verify these inside the actual environment
  because keyboard capture can differ between libraries.

### VEX controller role

For the immediate contest workflow, the VEX controller should be secondary.
Keyboard base control should be the default for beta recording and Game Builder
manual control.

Keep the VEX controller available only when:

- the direction calibration test passes;
- joystick deadband is large enough to prevent drift;
- the operator intentionally selects VEX controller base control;
- no replay or preposition command currently owns the base.

If the VEX controller remains inverted after keyboard controls are correct, the
bug is likely in controller axis inversion or joystick interpretation, not in
the robot-wide base convention.

### Direction calibration test

Add a simple test UI in Settings or Game Builder:

1. Put the robot near stable left and front references.
2. Show live X and Y ultrasonic distances.
3. Press and hold `ArrowLeft` briefly.
4. Expected: X distance gets smaller.
5. Press and hold `ArrowRight` briefly.
6. Expected: X distance gets bigger.
7. Press and hold `ArrowUp` briefly.
8. Expected: Y distance gets smaller.
9. Press and hold `ArrowDown` briefly.
10. Expected: Y distance gets bigger.
11. Press `O` briefly.
12. Expected: robot rotates left according to the chosen field/operator
    convention.
13. Press `P` briefly.
14. Expected: robot rotates right according to the chosen field/operator
    convention.

The UI should show pass/fail for each direction. If a direction fails, the fix
should be stored as a single base-direction calibration value:

```ts
interface VexKeyboardDirectionCalibration {
  xSign: 1 | -1;
  ySign: 1 | -1;
  thetaSign: 1 | -1;
  calibratedAtIso: string;
  notes: string;
}
```

This setting should affect manual keyboard base commands only. VEX
prepositioning already has its own progress checks and should continue to stop
when error gets worse or no progress is detected.

### Base braking and jitter strategy

Official VEX V5 Python documentation exposes three motor stopping modes through
`set_stopping`: `BRAKE`, `COAST`, and `HOLD`. The generated VEX Brain code in
`scripts/vex_base_bridge.py` currently uses `HOLD` for idle/hold commands.

Recommended contest behavior:

- Use `HOLD` for short replay/preposition stops where the base must resist
  movement before the next arm action.
- Use `BRAKE` as the candidate manual-drive idle mode if `HOLD` causes
  jittering or buzzing on the mecanum base.
- Avoid `COAST` for precise ECU alignment because the base can drift after the
  operator releases the key.
- Keep deadband active for controller input.
- Keep keyboard input discrete and ramped so the base does not jump from zero
  to full speed instantly.
- Keep command TTLs short so stale Pi commands cannot keep driving the base.
- Deduplicate repeated zero/hold commands so the VEX Brain is not spammed.
- After every replay, preposition, cancel, or emergency stop, send an explicit
  hold/stop command and show the resulting base authority in the UI.

Research source:

- VEX V5 Python API, Motor and Motor Group `set_stopping`: documents `BRAKE`,
  `COAST`, and `HOLD` stop behaviors:
  https://api.vex.com/v5/home/python/Motion/motor_and_motor_group.html

Trial plan:

1. Test manual keyboard base idle with current `HOLD`.
2. If the base jitters, add a setting for manual idle stopping mode:
   `hold`, `brake`, or `coast`.
3. Try `BRAKE` for manual idle and keep `HOLD` for replay/preposition hold.
4. Log which mode was used when a recording or replay was made.
5. Do not change arm hold behavior when testing VEX base braking modes.

### Speed presets

The beta UI should keep speed presets simple and trial-driven.

Minimum presets:

- Drive: fastest tested speed for moving around the field.
- ECU/Align: slower speed for lining up with fuse and circuit board targets.
- Crawl: very slow speed for final centimeter-level adjustment, if testing
  shows the current ECU mode is still too fast.

The `0` key can keep toggling Drive/ECU at first. If three presets are added,
`0` can cycle Drive -> ECU/Align -> Crawl -> Drive, but the UI must always show
the current speed mode clearly.

Suggested trial data to record per preset:

- linear speed;
- turn speed;
- whether braking mode was `hold`, `brake`, or `coast`;
- whether movement starts smoothly;
- whether release causes drift;
- whether the base jitters at rest;
- whether the operator can line up the ECU row consistently.

## VEX Replay Requirements

VEX base motion is recorded in the same trajectory JSON as arm state.

Replay options:

- Arm only: default.
- Arm + VEX base: explicit checkbox.
- VEX replay mode:
  - ECU mode: slower and target/encoder oriented.
  - Drive mode: faster velocity-style replay.

Operator speed mode:

- `0` toggles Drive/ECU speed during live keyboard base control.
- Drive mode should be used for moving around quickly.
- ECU mode should be used for gentle positioning.
- If testing needs a third preset, add Crawl for final alignment.
- The UI should show the current preset at all times.

Contest recommendation:

- Use keyboard base manually for alignment by default.
- Keep the VEX controller available only after direction and deadband tests
  pass.
- Use arm-only replays for pickup/drop moves unless base motion is proven
  repeatable.
- Only use VEX replay for known short shifts that have been tested.

## Hand-Guide Recording Requirements

Hand-guide mode:

- Disables torque on follower arm motors.
- Records observed follower positions.
- Bypasses powered-servo stall detection while torque is disabled.
- Restores follower arm torque at the recorded stop pose if no safety latch
  tripped.

Known problem:

- Position readings while torque is off have been underestimating or
  overestimating some joint positions.
- This can produce bad replay targets.

Debug direction:

1. Record raw bus `Present_Position` values alongside normalized degrees.
2. Compare torque-on and torque-off readings at the same physical pose.
3. Confirm calibration ranges and normalization modes.
4. Confirm no joint is slipping mechanically while torque is off.
5. Confirm follower arm torque restoration does not move the arm before final
   sample is saved.

Contest recommendation:

- Do not depend on hand-guide for the main ECU row workflow unless it is proven
  with repeated replay tests.
- Keyboard-from-hold is more deterministic under time pressure.

## Recording JSON Contract

Trajectory JSON files should continue using this shape:

- `format`: `lekiwi-follower-trajectory`
- `version`: currently 4
- `created_at`
- `recorded_on`: `pi`
- `robot_id`
- `robot_port`
- `loop_hz`
- `label`
- `recording_mode`
- `duration_s`
- `arm_state_keys`
- `base_state_keys`
- `vex_state_keys`
- `vex_motor_state_keys`
- `vex_inertial_state_keys`
- `vex_pose_state_keys`
- `sensor_state_keys`
- `ultrasonic_position_reference`, when available
- `samples`
- `command_samples`

Each sample should preserve:

- timestamp `t_s`
- arm joint state
- base velocity state
- VEX motor encoder state
- VEX inertial state
- VEX pose epoch
- ultrasonic distances
- derived ultrasonic X/Y positions when reference exists

Each command sample should preserve:

- timestamp `t_s`
- commanded arm positions
- commanded base velocities

Future editor requirements:

- Edits should create duplicates or history before overwriting.
- Servo edits should modify a time window or keyframe, not only one sample.
- Trimming should preserve metadata and replay options where possible.
- 3D editor changes should be saved as keyframe edits and resampled to a clean
  trajectory.

## 3D Arm Editor Proposal

The 3D editor idea is possible and worth building after the control path is
stable.

Existing assets:

- `Simulation/SO101/so101_new_calib.urdf`
- `Simulation/SO101/so101_new_calib.xml`
- `Simulation/SO101/assets/*.stl`
- SO101 STL/STEP files in `STL/SO101` and `STEP/SO101`

Goal:

- Display the SO101 arm in the browser.
- Load a trajectory JSON.
- Scrub the timeline.
- Visualize follower arm pose at each sample.
- Show base/sensor state alongside the arm.
- Add keyframes.
- Move joints with sliders or direct manipulators.
- Resample a corrected trajectory.
- Avoid manually editing hundreds of samples through six text boxes.

Recommended technical approach:

1. Use Three.js in the React app.
2. Load or convert the SO101 URDF/STL assets into a browser-friendly model.
3. Map JSON keys to joints:
   - `arm_shoulder_pan.pos`
   - `arm_shoulder_lift.pos`
   - `arm_elbow_flex.pos`
   - `arm_wrist_flex.pos`
   - `arm_wrist_roll.pos`
   - `arm_gripper.pos`
4. Apply joint rotations in degrees.
5. Use the existing recording samples as animation frames.
6. Add a timeline scrubber with keyframe markers.
7. Let the operator edit selected keyframes.
8. Interpolate between edited keyframes.
9. Save edits as a new duplicated recording.

Important caution:

- Mouse cursor control in 3D implies inverse kinematics.
- IK is possible, but it is harder than visualizing and editing joint angles.
- For contest timing, build visualization and keyframe editing first.
- Add IK only after replay and hold workflows are reliable.

## Safety Specification

Safety is mandatory and should remain conservative.

Current torque presets:

- `arm_shoulder_pan`: 810
- `arm_shoulder_lift`: 870
- `arm_elbow_flex`: 790
- `arm_wrist_flex`: 820
- `arm_wrist_roll`: 850
- `arm_gripper`: 1000

Current runtime safety:

- Gripper absolute position is clamped between 0 and 85.
- Safer mode clamps per-command steps for most joints.
- Wrist roll can run in continuous mode.
- Stall detection compares command target, observed position, progress, and
  current evidence.
- Temperature over 50 C confirmed across samples trips the latch.
- Safety trip disables torque on all detected motors.
- The safety latch requires restart/inspection.

Current power telemetry:

- Samples motor voltage/current/temperature.
- Estimates current using motor current raw counts.
- Estimates battery from voltage using 9.6 V empty and 12.6 V full.
- Logs CSV files under `~/lekiwi-power-logs`.
- Prints live summaries with current, voltage, battery, peak motor, and hottest
  motor.

Operator requirements:

- If any motor approaches 50 C, stop and cool down.
- If current repeatedly spikes near the limits, inspect for binding.
- If a servo smokes, do not just lower speed; inspect wiring, load, torque,
  physical obstruction, and calibration.
- Emergency Stop + Torque Off is the correct response to unexpected motion.

## Implementation Plan

### P0: must fix before relying on the robot

1. Enforce one arm authority.
   - Add explicit arm-source selection to live control and recording.
   - If leader is arm source, ignore keyboard arm keys.
   - If keyboard is arm source, do not poll leader actions.
   - If hold is active, disable arm input during restored base control.

2. Rename confusing UI actions.
   - Use names that distinguish arm control from base control.
   - Remove the implication that leader arm and keyboard arm can both command
     the arm.

3. Make auto VEX positioning default off.
   - Keep checkbox available.
   - Persist explicit per-recording choices.
   - Do not silently enable it for new pins or recordings.

4. Finish hold-assisted recording.
   - Recording can start from active hold.
   - Stop recording returns to active hold.
   - Control restore after recording is base-only when hold is active.
   - Active hold status is visible on Overview, Recordings, and Pins.

5. Make pinned hold workflow reliable.
   - Hold pin must be Pi follower only.
   - Hold pin must be arm-only.
   - Hold pin must disable VEX positioning.
   - Non-hold Pi pins return to hold.

6. Keep emergency stop and reset behavior strict.
   - Emergency clears hold and stops everything.
   - Reset Pi Connections clears hold and SSH state.

7. De-emphasize leader replay target.
   - Mark as experimental or move behind advanced settings.

8. Add the beta move library foundation.
   - Seed the competition move list from the General, Fuse, and Circuit Board
     categories above.
   - Store moves separately from recording files.
   - Allow each move to have multiple versions.
   - Allow one favorite/final version per move.
   - Show favorite completion on the move tile.
   - Keep old Recordings and Pins behavior unchanged.

9. Add the minimum Pro Recording beta workflow.
   - Show move tiles by category.
   - Open a move-specific recording menu.
   - Support recording type selection before start.
   - Support distance sensor and inertial sensor recording checkboxes.
   - Allow leader arm recording while keyboard VEX base movement remains
     available.
   - Allow keyboard-from-hold recording for the fuse-row workflow.
   - Save playback speed as version metadata.

10. Add the minimum Game Builder beta workflow.
   - Only show moves that have a favorite version.
   - Allow favorite moves to be dragged or added into a sequence list.
   - Allow each move or step to receive a hotkey.
   - Allow per-step VEX base replay and VEX positioning toggles.
   - Add Play mode with click-to-run, hotkey run, pause, cancel, and emergency
     stop.
   - Keep manual base control available when the arm is held.
   - Keep beta screens efficient: low-click defaults, collapsed advanced
     settings, remembered per-move setup, and visible one-key execution.

11. Fix contest-facing VEX keyboard control semantics.
   - Add arrow-key support to keyboard capture.
   - Default base keyboard control to `ArrowUp/ArrowDown`,
     `ArrowLeft/ArrowRight`, and `O/P` rotation.
   - Match the X/Y sign convention used by VEX prepositioning.
   - Add a direction calibration test that verifies X/Y ultrasonic distance
     changes.
   - Keep VEX controller drive secondary until direction and deadband tests
     pass.

12. Make mode transitions explicit.
   - Mark leader pose stale when follower/claw position changes outside leader
     authority.
   - Require leader sync before returning to leader-follower control.
   - Seed keyboard arm targets from observed follower pose.
   - Hold follower pose at the end of replay/recording unless the operator
     explicitly chooses torque off.

### P1: should implement after P0 works

1. Warm-host recording.
   - Avoid restarting the whole Pi host for every start/stop recording.
   - Add `start-recording` and `stop-recording` warm-host commands.
   - Reduce the 2-4 second start/stop delay.

2. Recording timeline editor.
   - Support window/keyframe edits instead of one timestamp at a time.
   - Support duplicate-before-edit.
   - Show command samples and observed samples separately.

4. Hand-guide diagnostics.
   - Store raw and normalized positions.
   - Identify why torque-off positions are wrong.

5. Beta UI refinement.
   - Add full move add/archive/duplicate editing.
   - Add broken-favorite detection when the Pi trajectory file is missing.
   - Add paged 4-column move grid behavior for small screens and large move
     libraries.
   - Add per-step confirmation and pause controls in Game Builder.
   - Add conflict-proof hotkey editing for base, claw, arm, and move bindings.

6. VEX base jitter tuning.
   - Add manual idle braking mode setting for base control: hold, brake, or
     coast.
   - Trial `BRAKE` for manual idle if `HOLD` jitters.
   - Keep `HOLD` available for replay/preposition stops.
   - Store tested speed presets and braking choices in settings.

### P2: useful after contest-critical flow is stable

1. 3D SO101 trajectory viewer/editor.
2. Chain-link execution hardening.
3. Per-block failure recovery.
4. Advanced VEX positioning profiles by field zone.
5. Base path planner for known field movements.
6. Optional camera-based positioning, if hardware becomes available.
7. Advanced Game Builder analytics, including run timing, failure rate, and
   repeated-match plan comparison.

## Acceptance Tests

These are the practical tests that determine whether the robot is ready.

### Arm authority tests

1. Start live control with leader arm as arm source.
2. Press keyboard arm keys.
3. Follower arm should not move from keyboard arm input.
4. Move leader arm.
5. Follower arm should follow leader.
6. Base keyboard keys should still move the VEX base.

Second test:

1. Start live control with keyboard arm as arm source.
2. Move leader arm physically.
3. Follower arm should not follow leader.
4. Keyboard arm keys should move follower.
5. VEX controller should still move base.

### Hold tests

1. Trigger a hold pin.
2. Confirm follower moves to hold pose.
3. Confirm UI shows active hold.
4. Move base with VEX controller.
5. Arm should stay fixed.
6. Move base with keyboard base keys.
7. Arm should stay fixed.
8. Press keyboard arm keys during normal held/base-only control.
9. Arm should ignore them.
10. Trigger a normal pinned move.
11. It should run and return to hold.
12. Toggle hold off.
13. Arm authority should be available again.

### Recording from hold tests

1. Trigger row hold.
2. Start keyboard-only recording from active hold.
3. Move gripper and arm through a short pickup/drop motion.
4. Stop recording.
5. Follower should return to the active hold pose.
6. Base-only control should be restored.
7. Replay the recording.
8. It should start from the hold pose and return to hold.

### Auto VEX positioning default test

1. Refresh settings and recordings.
2. Select a recording that has no explicit replay options.
3. Auto VEX positioning should be off by default.
4. Pin the move.
5. The pin should also have auto VEX positioning off unless manually enabled.

### Safety tests

1. Load safer torque preset.
2. Confirm power telemetry prints.
3. Confirm temperatures are shown.
4. Confirm current values are shown.
5. Trigger Emergency Stop + Torque Off.
6. Confirm host, teleop, replay, calibration, dataset capture, and policy eval
   stop.
7. Confirm active hold clears.

### VEX tests

1. Run VEX telemetry slot 8.
2. Confirm VEX Brain status is connected and telemetry active.
3. Confirm keyboard base controls use arrow keys plus `O/P`.
4. Press `0` in keyboard control.
5. Confirm Drive/ECU mode toggles.
6. Zero VEX gyro.
7. Confirm inertial rotation/heading reference is reset.
8. Replay an arm-only recording with VEX base disabled.
9. Base should hold.
10. Replay a base-inclusive recording only after intentional enablement.

### VEX direction calibration tests

1. Put the base near stable left and front ultrasonic references.
2. Open the direction calibration helper.
3. Press `ArrowLeft`.
4. Confirm X distance gets smaller.
5. Press `ArrowRight`.
6. Confirm X distance gets bigger.
7. Press `ArrowUp`.
8. Confirm Y distance gets smaller.
9. Press `ArrowDown`.
10. Confirm Y distance gets bigger.
11. Press `O`.
12. Confirm the base rotates left according to the chosen operator convention.
13. Press `P`.
14. Confirm the base rotates right according to the chosen operator convention.
15. If any direction fails, store the correction in one direction calibration
    setting and retest.
16. Do not change motor reversed flags unless the direction calibration proves
    the physical motor configuration itself is wrong.

### VEX braking and speed tests

1. Test manual keyboard idle with current hold behavior.
2. Confirm whether the base jitters or buzzes at rest.
3. If it jitters, test manual idle with brake behavior.
4. Confirm the base does not drift too far after key release.
5. Keep coast disabled for precise ECU alignment unless testing proves it is
   acceptable.
6. Test Drive, ECU/Align, and optional Crawl speed presets.
7. Record which preset gives fastest reliable fuse-row alignment.
8. Confirm the UI always shows the active preset.

### Mode transition tests

1. Start leader-follower control.
2. Move the follower using keyboard arm or replay.
3. Do not move the leader.
4. Confirm the UI marks leader pose stale.
5. Try to resume leader-follower control.
6. Confirm the UI blocks it until sync is explicitly chosen.
7. Sync leader to follower pose.
8. Confirm leader-follower control can resume without follower snapback.
9. Repeat the same test with the gripper/claw opened or closed by keyboard.

### Move transition tests

1. Add two favorite moves to a Game Builder sequence.
2. Make the first move end at a pose different from the second move start.
3. Run the sequence with strict transition checking.
4. Confirm Game Builder pauses or returns to hold before the second move.
5. Add an explicit transition move.
6. Confirm the sequence can run only after the transition policy is satisfied.

### Pro Recording beta tests

1. Open `Pro Recording (beta)`.
2. Confirm the initial move library includes all General, Fuse collection, Fuse
   removal/insertion, Circuit Board collection, and Circuit Board
   removal/insertion moves listed in this spec.
3. Confirm moves are grouped and color-coded by category.
4. Select a move with no versions.
5. Record `v1` with keyboard control.
6. Confirm the move tile shows one version but no final checkmark.
7. Mark `v1` as favorite.
8. Confirm the move tile shows the final/completion checkmark.
9. Record `v2`.
10. Confirm `v1` remains favorite until the operator changes it.
11. Change playback speed on `v2`.
12. Confirm the raw trajectory file is unchanged and speed is stored as
    metadata.
13. Select Leader arm as recording type.
14. Confirm keyboard arm keys are disabled.
15. Confirm keyboard VEX base keys still work if base control is enabled.
16. Disable distance sensor and inertial recording.
17. Confirm replay does not require VEX positioning metadata.
18. Confirm common actions are low-click: new version, stop, replay, mark
    favorite, and add to Game Builder should not require repeated category or
    label entry.

### Game Builder beta tests

1. Open `Game Builder (beta)`.
2. Confirm only moves with favorite versions appear in the move palette.
3. Drag three favorite moves into the sequence list.
4. Reorder the list.
5. Assign hotkeys to each move or step.
6. Confirm hotkey conflicts are blocked or clearly highlighted.
7. Disable VEX positioning on every step.
8. Enter Play mode.
9. Trigger a step by clicking it.
10. Trigger a step by hotkey.
11. Pause a running step.
12. Cancel a running step.
13. Confirm cancel stops arm replay and base replay.
14. Confirm Emergency Stop + Torque Off overrides everything.
15. Activate an arm hold and move the VEX base manually.
16. Confirm the arm stays held while base control remains available.
17. Confirm advanced replay options are collapsed by default.
18. Confirm Play mode prioritizes the sequence, hotkeys, pause/cancel, manual
    override, and speed preset over less-used settings.

## Final Engineering Position

The strongest contest path is not to make every experimental feature work. The
strongest path is:

1. Make arm authority explicit.
2. Make hold pose reliable.
3. Use base movement separately to align the horizontal ECU row.
4. Record short, repeatable arm-only pickup/drop moves from the same hold pose.
5. Pin those moves to hotkeys.
6. Use chain-link only after the pinned moves succeed repeatedly.
7. Keep VEX positioning available but off by default.
8. Keep safety latches strict.
9. Add `Pro Recording (beta)` and `Game Builder (beta)` as safer, clearer
   contest workflows that preserve the old pages while reducing recording and
   replay confusion.
10. Make keyboard VEX base control the primary contest base input until the VEX
    controller inversion/deadband behavior is proven reliable.
11. Treat every move transition and every leader/claw mode switch as a possible
    snapback risk unless the start pose and authority state are verified.

The 3D model editor is also valid and would solve the "hundreds of points and
six text boxes" problem, but it is not the first reliability fix. The first
reliability fix is control authority.
