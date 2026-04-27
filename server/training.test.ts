import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import test from "node:test";

import { ConfigStore } from "./configStore.js";
import { defaultConfig } from "./defaultConfig.js";
import {
  buildMacTrainingCommand,
  buildPiDatasetCaptureCommand,
} from "./trainingCommands.js";
import {
  createDefaultTrainingProfile,
  normalizeTrainingConfig,
  validateTrainingProfile,
} from "./trainingUtils.js";
import { isTransientRemoteTransportError } from "./transportErrors.js";

test("config store migrates older config files by adding training defaults", () => {
  const tempRoot = fs.mkdtempSync(path.join(os.tmpdir(), "robot-arm-config-"));
  const configDir = path.join(tempRoot, ".lekiwi-ui");
  fs.mkdirSync(configDir, { recursive: true });
  fs.writeFileSync(
    path.join(configDir, "config.json"),
    JSON.stringify({
      settings: defaultConfig.settings,
      pinnedMoves: [],
    }),
  );

  const store = new ConfigStore(defaultConfig, tempRoot);
  const loaded = store.getConfig();

  assert.ok(loaded.training);
  assert.equal(loaded.training.settings.defaultPolicyType, "act");
  assert.ok(loaded.training.profiles.length >= 1);
  assert.ok(loaded.training.selectedProfileId);
});

test("config store defaults legacy pinned moves to Pi replay", () => {
  const tempRoot = fs.mkdtempSync(path.join(os.tmpdir(), "robot-arm-config-"));
  const configDir = path.join(tempRoot, ".lekiwi-ui");
  fs.mkdirSync(configDir, { recursive: true });
  fs.writeFileSync(
    path.join(configDir, "config.json"),
    JSON.stringify({
      settings: defaultConfig.settings,
      pinnedMoves: [
        {
          id: "move-1",
          name: "Legacy move",
          trajectoryPath: "/home/pi/lekiwi-trajectories/legacy.json",
          speed: 1,
          includeBase: false,
          holdFinalS: 0.5,
          keyBinding: "SHIFT+1",
        },
      ],
      training: defaultConfig.training,
    }),
  );

  const store = new ConfigStore(defaultConfig, tempRoot);
  const loaded = store.getConfig();

  assert.equal(loaded.pinnedMoves[0]?.target, "pi");
  assert.equal(loaded.pinnedMoves[0]?.vexReplayMode, "ecu");
});

test("config store upgrades untouched legacy VEX forward axis default", () => {
  const tempRoot = fs.mkdtempSync(path.join(os.tmpdir(), "robot-arm-config-"));
  const configDir = path.join(tempRoot, ".lekiwi-ui");
  fs.mkdirSync(configDir, { recursive: true });
  fs.writeFileSync(
    path.join(configDir, "config.json"),
    JSON.stringify({
      settings: {
        ...defaultConfig.settings,
        vex: {
          ...defaultConfig.settings.vex,
          controlPresetVersion: undefined,
          controls: {
            ...defaultConfig.settings.vex.controls,
            forwardAxis: "axis2",
            strafeAxis: "axis4",
            turnAxis: "axis1",
          },
        },
      },
      pinnedMoves: [],
      training: defaultConfig.training,
    }),
  );

  const store = new ConfigStore(defaultConfig, tempRoot);
  const loaded = store.getConfig();

  assert.equal(loaded.settings.vex.controlPresetVersion, 2);
  assert.equal(loaded.settings.vex.controls.forwardAxis, "axis3");
  assert.equal(loaded.settings.vex.controls.strafeAxis, "axis4");
  assert.equal(loaded.settings.vex.controls.turnAxis, "axis1");
});

test("config store preserves recording replay speed and auto VEX positioning", () => {
  const tempRoot = fs.mkdtempSync(path.join(os.tmpdir(), "robot-arm-config-"));
  const configDir = path.join(tempRoot, ".lekiwi-ui");
  fs.mkdirSync(configDir, { recursive: true });
  fs.writeFileSync(
    path.join(configDir, "config.json"),
    JSON.stringify({
      settings: defaultConfig.settings,
      pinnedMoves: [],
      recordingReplayOptions: {
        "/home/pi/lekiwi-trajectories/slow.json": {
          homeMode: "start",
          speed: 0.4,
          autoVexPositioning: false,
          vexPositioningSpeed: 1.6,
          vexPositioningTimeoutS: 12,
          vexPositioningXyToleranceM: 0.12,
          vexPositioningHeadingToleranceDeg: 4,
          vexPositioningXyTrimToleranceM: 0.75,
          vexPositioningHeadingTrimToleranceDeg: 15,
        },
        "/home/pi/lekiwi-trajectories/legacy.json": {
          homeMode: "end",
        },
      },
      training: defaultConfig.training,
    }),
  );

  const store = new ConfigStore(defaultConfig, tempRoot);
  const loaded = store.getConfig();

  assert.deepEqual(loaded.recordingReplayOptions["/home/pi/lekiwi-trajectories/slow.json"], {
    homeMode: "start",
    speed: 0.4,
    autoVexPositioning: false,
    vexPositioningSpeed: 1.6,
    vexPositioningTimeoutS: 12,
    vexPositioningXyToleranceM: 0.12,
    vexPositioningHeadingToleranceDeg: 4,
    vexPositioningXyTrimToleranceM: 0.75,
    vexPositioningHeadingTrimToleranceDeg: 15,
  });
  assert.deepEqual(loaded.recordingReplayOptions["/home/pi/lekiwi-trajectories/legacy.json"], {
    homeMode: "end",
    speed: defaultConfig.settings.trajectories.defaultReplaySpeed,
    autoVexPositioning: true,
    vexPositioningSpeed: 1,
    vexPositioningTimeoutS: 8,
    vexPositioningXyToleranceM: 0.02,
    vexPositioningHeadingToleranceDeg: 1.5,
    vexPositioningXyTrimToleranceM: 0.05,
    vexPositioningHeadingTrimToleranceDeg: 8.5,
  });
});

test("config store normalizes chain-links with safe confirmation defaults", () => {
  const tempRoot = fs.mkdtempSync(path.join(os.tmpdir(), "robot-arm-config-"));
  const configDir = path.join(tempRoot, ".lekiwi-ui");
  fs.mkdirSync(configDir, { recursive: true });
  fs.writeFileSync(
    path.join(configDir, "config.json"),
    JSON.stringify({
      settings: defaultConfig.settings,
      pinnedMoves: [],
      chainLinks: [
        {
          id: "chain-1",
          name: "Pick sequence",
          items: [
            {
              id: "block-1",
              name: "Approach",
              trajectoryPath: "/home/pi/lekiwi-trajectories/approach.json",
              target: "pi",
              homeMode: "both",
              speed: 0.7,
              autoVexPositioning: false,
              includeBase: false,
              holdFinalS: 0.2,
            },
          ],
        },
      ],
      training: defaultConfig.training,
    }),
  );

  const store = new ConfigStore(defaultConfig, tempRoot);
  const loaded = store.getConfig();

  assert.equal(loaded.chainLinks[0]?.confirmAfterEach, true);
  assert.equal(loaded.chainLinks[0]?.items[0]?.homeMode, "both");
  assert.equal(loaded.chainLinks[0]?.items[0]?.speed, 0.7);
  assert.equal(loaded.chainLinks[0]?.items[0]?.autoVexPositioning, false);
});

test("training profile validation rejects unsupported camera modes", () => {
  const profile = createDefaultTrainingProfile(process.cwd(), "Bad Cameras");
  profile.camerasMode = "{}";

  assert.throws(() => validateTrainingProfile(profile), /cameras mode/i);
});

test("training profile normalization preserves leader-as-follower capture mode", () => {
  const normalized = normalizeTrainingConfig(
    {
      profiles: [
        {
          ...createDefaultTrainingProfile(process.cwd(), "Leader Local"),
          captureMode: "leader-as-follower",
        },
      ],
      selectedProfileId: "leader-local",
    },
    process.cwd(),
  );

  assert.equal(normalized.profiles[0]?.captureMode, "leader-as-follower");
});

test("training commands include Pi-safe ACT defaults", () => {
  const training = normalizeTrainingConfig(defaultConfig.training, process.cwd());
  const profile = training.profiles[0];

  const macCommand = buildMacTrainingCommand(defaultConfig.settings, profile);
  assert.match(macCommand, /--policy\.type=act/);
  assert.match(macCommand, /--policy\.device='mps'/);
  assert.match(macCommand, /--dataset\.root=/);

  const captureCommand = buildPiDatasetCaptureCommand(
    defaultConfig.settings,
    profile,
    "/home/pi/.lekiwi-ui/torque_limits.json",
  );
  assert.match(captureCommand, /--robot-cameras-json 'default'/);
  assert.match(captureCommand, /--dataset-vcodec h264/);
  assert.match(captureCommand, /--dataset-streaming-encoding false/);
  assert.match(captureCommand, /--capture-mode 'leader'/);
});

test("transient Pi SSH transport errors are classified as reset noise", () => {
  const reset = new Error("read ECONNRESET") as Error & { code: string };
  reset.code = "ECONNRESET";

  assert.equal(isTransientRemoteTransportError(reset), true);
  assert.equal(isTransientRemoteTransportError(new Error("Connection lost before handshake")), true);
  assert.equal(isTransientRemoteTransportError(new Error("client-socket disconnected")), true);
  assert.equal(isTransientRemoteTransportError(new Error("Permission denied")), false);
});
