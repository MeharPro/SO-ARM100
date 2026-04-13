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
      armTorqueLimits: {
        arm_shoulder_pan: 1000,
        arm_shoulder_lift: 1000,
        arm_elbow_flex: 1000,
        arm_wrist_flex: 1000,
        arm_wrist_roll: 1000,
        arm_gripper: 1000,
      },
    },
    trajectories: {
      remoteDir: "/home/pi/lekiwi-trajectories",
      defaultReplaySpeed: 1,
      defaultHoldFinalS: 0.5,
    },
  },
  pinnedMoves: [],
  training: createDefaultTrainingConfig(process.cwd()),
};
