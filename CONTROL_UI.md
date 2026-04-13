# Robot Arm Control UI

This app gives you a local dashboard on your Mac for:

- starting and stopping the Pi host
- starting and stopping local teleop
- recording exact follower trajectories on the Pi
- replaying saved trajectories on the Pi or on the Mac-attached leader arm
- pinning saved moves to dashboard buttons and keyboard shortcuts
- showing live backend and process output while actions run
- checking whether the leader arm is actually connected on the port used by `teleoperate.py`
- capturing a local training dataset on the Mac by using the leader arm as a follower stand-in

## Start It

From the repo root:

```bash
npm install
npm run dev
```

Open:

```text
http://localhost:5173
```

For a production-style build:

```bash
npm run build
npm start
```

That serves the built UI and backend from:

```text
http://localhost:4318
```

## What The UI Does

`Start Control`

- checks that the leader arm is connected on the exact port configured in `teleoperate.py`
- connects to the Pi on `rawr.local` or `10.42.0.1`
- retries Pi connection 5 times before failing
- if the Pi is still unreachable, the UI tells you that you are probably not on `rawr-hotspot`
- starts the Pi host
- starts the headless local relay helper at `scripts/lekiwi_ui_teleop.py` on your Mac

`Stop Control`

- stops the Mac teleop process
- stops the remote Pi host process

`Emergency Stop + Torque Off`

- stops teleop
- stops replay
- stops the Pi host
- runs the torque-disable recovery block on the Pi

`Start Recording`

- uploads the recording helper to the Pi if needed
- starts the Pi recording host
- starts the same headless local relay helper
- saves trajectories under `/home/pi/lekiwi-trajectories`

`Replay Selected`

- uploads the replay helper to the Pi if needed
- stops live control
- replays the selected saved trajectory on the Pi, or downloads it locally first if you target the leader arm

`Training > Capture Dataset`

- `Leader` and `Free-teach` still capture on the Pi
- `Leader as follower (Mac)` records a local arm-only dataset by using the leader arm as a free-taught follower stand-in
- the Mac-side leader workflow writes directly to the configured Mac dataset path, so the sync button becomes a local metadata refresh instead of a Pi download

`Pin Move to Dashboard`

- saves a dashboard shortcut with replay settings and a hotkey
- saved hotkeys fire while the page is focused and you are not typing into a field

## Saved Settings

UI settings and pinned moves are stored locally in:

```text
.lekiwi-ui/config.json
```

That file includes the Pi password because the backend uses password-based SSH directly from the dashboard.

## Current Defaults

- hotspot SSID: `rawr-hotspot`
- Pi user: `pi`
- Pi host: `rawr.local`
- Pi fallback host: `10.42.0.1`
- Pi password: `password`
- Pi lerobot path: `/home/pi/lerobot`
- Mac lerobot path: `/Users/meharkhanna/lerobot`

You can change all of these from the Settings card in the UI.

## Layout

- the UI uses a fixed sidebar with section buttons instead of one long scrolling page
- the browser window itself should stay locked in place on desktop
- long logs and long recording lists scroll inside their own panel instead
