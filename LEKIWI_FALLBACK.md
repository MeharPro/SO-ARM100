# LeKiwi Fallback

Use this when the stock LeKiwi host crashes in `get_observation()` but the robot still accepts commands.

## Start fallback host

From the Mac:

```bash
export LEKIWI_PI_HOST=10.42.0.1
~/lekiwi_min_host
```

This SSHes into the Pi and starts a command-only LeKiwi host on port `5555`.

If you want the same minimal control path but with the default two camera feeds (`front` and `wrist`) published on port `5556`, run:

```bash
lekiwi-cam-min
```

This keeps motor-state reads out of the loop and only streams camera frames.

## Start raw relay

From the Mac, in a second terminal:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
export LEKIWI_PI_HOST=10.42.0.1
export LEKIWI_LEADER_PORT=/dev/tty.usbmodem5AE60840411
~/lekiwi_raw_relay --remote-host "$LEKIWI_PI_HOST" --print-every 1
```

## What you lose

- No observation stream from the Pi
- No camera frames
- No follower-state feedback to the client
- No stock `LeKiwiClient` teleop path

## What still works

- Leader arm to follower arm command relay
- Keyboard base control in the raw relay (`W/S`, `A/D`, `Z/X`, `R/F`)
- Base watchdog stop in the fallback host

## Notes

- The fallback host is command-only. It does not call `robot.get_observation()`.
- The raw relay sends arm commands plus keyboard-derived base velocity over raw ZMQ.

## Resilient Observation Host

If you want observations back without using the brittle stock LeKiwi host, start:

```bash
export LEKIWI_PI_HOST=10.42.0.1
~/lekiwi_resilient_host
```

This starts an alternate Pi host that:

- reads arm and base state with sync-read first
- falls back to per-motor reads on failure
- uses cached state instead of crashing on a transient packet error
- still publishes observations on port `5556`

By default it runs with cameras disabled to keep the path narrow.

If you want the resilient host with the default two cameras enabled, run:

```bash
lekiwi-cam-res
```

That keeps the arm/base observation fallback logic and adds the `front` and `wrist` camera feeds on port `5556`.

## View the camera feeds

Once either `lekiwi-cam-res` or `lekiwi-cam-min` is running, open the streams on the Mac with:

```bash
lekiwi-cam-view
```

This opens two OpenCV windows: `LeKiwi Front` and `LeKiwi Wrist`.

## Emergency stop and release

If the follower arm stays stiff after a crash or interrupted SSH session, run:

```bash
export LEKIWI_PI_HOST=10.42.0.1
~/lekiwi_stop_all
```

This kills known Pi-side LeKiwi host processes and explicitly disables torque on follower motors `1..9`.

## Gripper overheat recovery

If motor `id=6` (`arm_gripper`) drops off the bus with `RxPacketError] Overheat error!`, treat that as a stall or binding issue first, not just a software fault:

1. Power the follower arm off and let the gripper cool completely.
2. Open the jaw by hand and check that the horn, linkage, and printed parts are not already against a hard stop.
3. From the Mac, once the motor is visible again on the follower arm, run:

```bash
lekiwi-gripper-recovery
```

That SSHes to `pi@rawr.local` or `pi@10.42.0.1`, streams the recovery script to the Pi, and writes lower `Max_Torque_Limit`, `Torque_Limit`, `Protection_Current`, and `Overload_Torque` values so the gripper is less likely to cook itself on the next stall.
