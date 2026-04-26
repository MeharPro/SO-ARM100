import type { StoredConfig } from "./types.js";
import { createDefaultTrainingConfig } from "./trainingUtils.js";

export const defaultConfig: StoredConfig = {
  settings: {
    hotspot: {
      ssid: "rawr-hotspot",
    },
    pi: {
      host: "rawr.local",
      fallbackHost: "10.42.0.1",
      username: "pi",
      password: "password",
      projectDir: "/home/pi/lerobot",
      condaScript: "/home/pi/miniforge3/etc/profile.d/conda.sh",
      robotPort: "/dev/ttyACM0",
      remoteHelperDir: "/home/pi/.lekiwi-ui",
    },
    mac: {
      projectDir: "/Users/meharkhanna/lerobot",
      condaScript: "/Users/meharkhanna/miniforge3/etc/profile.d/conda.sh",
    },
    host: {
      connectionTimeS: 3600,
      robotId: "follow-mobile",
      camerasJson: "{}",
      baseMaxRawVelocity: 6000,
      baseWheelTorqueLimit: 700,
      enableBase: false,
      saferServoMode: true,
      armTorqueLimits: {
        arm_shoulder_pan: 500,
        arm_shoulder_lift: 650,
        arm_elbow_flex: 600,
        arm_wrist_flex: 450,
        arm_wrist_roll: 250,
        arm_gripper: 1000,
      },
    },
    trajectories: {
      remoteDir: "/home/pi/lekiwi-trajectories",
      defaultReplaySpeed: 1,
      defaultHoldFinalS: 0.5,
    },
    vex: {
      telemetrySlot: 8,
      replaySlot: 7,
      autoRunTelemetry: true,
      telemetryProgramName: "Base Telemetry",
      inertial: {
        port: 4,
      },
      motors: {
        frontRight: {
          port: 1,
          reversed: true,
        },
        frontLeft: {
          port: 2,
          reversed: false,
        },
        rearRight: {
          port: 9,
          reversed: true,
        },
        rearLeft: {
          port: 10,
          reversed: false,
        },
      },
      controls: {
        forwardAxis: "axis2",
        strafeAxis: "axis4",
        turnAxis: "axis1",
        invertForward: false,
        invertStrafe: false,
        invertTurn: false,
      },
      tuning: {
        deadbandPercent: 5,
        maxLinearSpeedMps: 0.35,
        maxTurnSpeedDps: 90,
      },
    },
  },
  pinnedMoves: [],
  training: createDefaultTrainingConfig(process.cwd()),
};
