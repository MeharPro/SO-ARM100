import assert from "node:assert/strict";
import test from "node:test";

import { defaultConfig } from "./defaultConfig.js";
import { RobotController } from "./robotController.js";
import type { LeaderStatus, ServiceSnapshot, VexBrainStatus } from "./types.js";

function snapshot(overrides: Partial<ServiceSnapshot> = {}): ServiceSnapshot {
  return {
    label: "Test service",
    state: "idle",
    detail: "Idle",
    pid: null,
    exitCode: null,
    startedAt: null,
    stoppedAt: null,
    mode: null,
    logs: [],
    meta: {},
    ...overrides,
  };
}

const leaderDisconnected: LeaderStatus = {
  teleoperateScriptPath: "",
  connected: false,
  expectedPort: null,
  availablePorts: [],
  message: "Leader unavailable",
};

const vexDisconnected: VexBrainStatus = {
  connected: false,
  telemetryActive: false,
  consolePort: null,
  commPort: null,
  source: null,
  message: "VEX unavailable",
};

const holdHomePosition = {
  capturedAt: "2026-04-30T13:00:00.000Z",
  joints: {
    "arm_shoulder_pan.pos": 10,
    "arm_shoulder_lift.pos": -20,
    "arm_elbow_flex.pos": 30,
    "arm_wrist_flex.pos": -40,
    "arm_wrist_roll.pos": 50,
    "arm_gripper.pos": 60,
  },
};

test("discarding leader authority can be overridden by selecting leader control", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  harness.getState = async () => ({ ok: true });
  harness.markLeaderPoseStale("keyboard follower movement");

  await controller.resolveLeaderStale({ action: "discardLeaderAuthority" });

  assert.equal(harness.leaderPoseStale, true);
  assert.match(harness.leaderPoseStaleReason, /selecting leader again will move the follower/);
  assert.equal(harness.normalizeLiveArmSource("leader", true), "leader");
  assert.equal(harness.leaderPoseStale, false);
  assert.equal(harness.leaderPoseStaleReason, null);
});

test("leader mimic moves leader to the active hold pose from the recorder menu action", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const calls: string[] = [];
  let mimicScript = "";

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.activeArmHold = {
    pinnedMoveId: "hold-open",
    name: "Open claw hold",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-open.json",
    activatedAt: "2026-04-30T13:05:00.000Z",
    homePosition: holdHomePosition,
  };
  harness.ensureLeaderConnected = async () => ({
    teleoperateScriptPath: "",
    connected: true,
    expectedPort: "/dev/tty.usbmodem123",
    availablePorts: ["/dev/tty.usbmodem123"],
    message: "Leader arm detected.",
  });
  harness.macCalibrationRunner.stop = async () => {
    calls.push("mac-calibration-stop");
  };
  harness.localReplayRunner.stop = async () => {
    calls.push("local-replay-stop");
  };
  harness.runLocalLeaderMimicPoseScript = async (script: string) => {
    calls.push("leader-mimic-script");
    mimicScript = script;
  };
  harness.getState = async () => ({ ok: true });

  const state = await controller.mimicLeaderToActiveHold();

  assert.deepEqual(state, { ok: true });
  assert.deepEqual(calls, ["mac-calibration-stop", "local-replay-stop", "leader-mimic-script"]);
  assert.match(mimicScript, /lekiwi_leader_mimic_pose\.py/);
  assert.match(mimicScript, /--robot-port/);
  assert.match(mimicScript, /\/dev\/tty\.usbmodem123/);
  assert.match(mimicScript, /--home-position-json/);
  assert.match(mimicScript, /arm_shoulder_pan\.pos/);
  assert.equal(harness.leaderPoseStale, false);
  assert.equal(harness.leaderPoseStaleReason, null);
});

test("leader mimic lazily captures the active hold pose through the warm host", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const commands: unknown[] = [];
  let mimicScript = "";

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.activeArmHold = {
    pinnedMoveId: "hold-open",
    name: "Open claw hold",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-open.json",
    activatedAt: "2026-04-30T13:05:00.000Z",
    homePosition: null,
  };
  harness.ensureCommandReadyHost = async () => "10.42.0.1";
  harness.writeRemoteHostCommand = async (
    _settings: unknown,
    _host: string,
    command: unknown,
  ) => {
    commands.push(command);
  };
  harness.waitForWarmHostHomeCommandResult = async () => ({
    success: true,
    status: "captured",
    homePosition: holdHomePosition,
  });
  harness.ensureLeaderConnected = async () => ({
    teleoperateScriptPath: "",
    connected: true,
    expectedPort: "/dev/tty.usbmodem123",
    availablePorts: ["/dev/tty.usbmodem123"],
    message: "Leader arm detected.",
  });
  harness.macCalibrationRunner.stop = async () => undefined;
  harness.localReplayRunner.stop = async () => undefined;
  harness.runLocalLeaderMimicPoseScript = async (script: string) => {
    mimicScript = script;
  };
  harness.getState = async () => ({ ok: true });

  await controller.mimicLeaderToActiveHold();

  assert.equal((commands[0] as any).command, "capture-home");
  assert.deepEqual(harness.activeArmHold.homePosition, holdHomePosition);
  assert.match(mimicScript, /arm_shoulder_pan\.pos/);
});

test("leader mimic requires an active hold pose", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  let leaderChecked = false;

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.ensureLeaderConnected = async () => {
    leaderChecked = true;
    return leaderDisconnected;
  };

  await assert.rejects(() => controller.mimicLeaderToActiveHold(), /Turn on an arm hold/);
  assert.equal(leaderChecked, false);
});

test("recording restore source does not confuse keyboard base with keyboard arm", () => {
  const harness = new RobotController() as any;

  assert.equal(
    harness.resolveRecordingRestoreArmSource(
      snapshot({
        mode: "recording",
        meta: { input: "leader+keyboard-base", armSource: "leader" },
      }),
      snapshot({
        mode: "recording",
        meta: { input: "leader+keyboard-base", armSource: "leader" },
      }),
    ),
    "leader",
  );
  assert.equal(
    harness.resolveRecordingRestoreArmSource(
      snapshot({
        mode: "recording",
        meta: { input: "keyboard", armSource: "keyboard" },
      }),
      snapshot({
        mode: "recording",
        meta: { input: "keyboard", armSource: "keyboard" },
      }),
    ),
    "keyboard",
  );
  assert.equal(
    harness.resolveRecordingRestoreArmSource(
      snapshot({
        mode: "recording",
        meta: { input: "free-teach", armSource: "none" },
      }),
      snapshot({ mode: "recording", meta: { input: "free-teach" } }),
    ),
    null,
  );
});

test("runtime safety latch logs are reflected in control authority", () => {
  const harness = new RobotController() as any;
  const host = snapshot({
    state: "running",
    mode: "control",
    logs: [
      "10:00:00 AM [stdout] [safety] latched arm_gripper temperature reached 50.0C (hard limit 50.0C); disabled all motor torque",
    ],
  });

  harness.updateRuntimeSafetyLatchFromSnapshots([host]);
  const authority = harness.buildControlAuthorityState(
    leaderDisconnected,
    vexDisconnected,
    host,
    snapshot(),
    snapshot(),
    snapshot(),
    snapshot(),
  );

  assert.equal(authority.safetyLatched, true);
  const oldStoppedHost = snapshot({
    ...host,
    state: "idle",
    detail: "Pi connection reset.",
    stoppedAt: new Date(Date.now() - 1000).toISOString(),
  });
  harness.resetRuntimeSafetyLatch("test reset");
  harness.updateRuntimeSafetyLatchFromSnapshots([oldStoppedHost]);
  const resetAuthority = harness.buildControlAuthorityState(
    leaderDisconnected,
    vexDisconnected,
    host,
    snapshot(),
    snapshot(),
    snapshot(),
    snapshot(),
  );
  assert.equal(resetAuthority.safetyLatched, false);

  harness.updateRuntimeSafetyLatchFromSnapshots([host]);
  const relatchedAuthority = harness.buildControlAuthorityState(
    leaderDisconnected,
    vexDisconnected,
    host,
    snapshot(),
    snapshot(),
    snapshot(),
    snapshot(),
  );
  assert.equal(relatchedAuthority.safetyLatched, true);
});

test("runtime safety latch ignores stall-only safety logs", () => {
  const harness = new RobotController() as any;
  const host = snapshot({
    state: "running",
    mode: "control",
    logs: [
      "10:00:00 AM [stdout] [safety] latched arm_elbow_flex stalled with 35.78deg position error and no meaningful motion for 0.50s, current 1040mA >= 600mA; disabled all motor torque",
    ],
  });

  harness.updateRuntimeSafetyLatchFromSnapshots([host]);
  const authority = harness.buildControlAuthorityState(
    leaderDisconnected,
    vexDisconnected,
    host,
    snapshot(),
    snapshot(),
    snapshot(),
    snapshot(),
  );

  assert.equal(authority.safetyLatched, false);
  assert.equal(harness.runtimeSafetyLatchReason, null);
});

test("control authority reports VEX controller base when keyboard base is disabled", () => {
  const harness = new RobotController() as any;
  const authority = harness.buildControlAuthorityState(
    leaderDisconnected,
    { ...vexDisconnected, telemetryActive: true },
    snapshot({
      state: "running",
      mode: "control",
      meta: { armSource: "keyboard", baseSource: "vexController" },
    }),
    snapshot({
      state: "running",
      mode: "control",
      meta: { armSource: "keyboard", baseSource: "vexController" },
    }),
    snapshot(),
    snapshot(),
    snapshot(),
  );

  assert.equal(authority.arm, "keyboard");
  assert.equal(authority.base, "vexController");
  assert.equal(authority.keyboardCaptureActive, true);
});

test("runtime safety latch blocks new recording before Pi commands", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  let prepareCalled = false;

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.runtimeSafetyLatched = true;
  harness.runtimeSafetyLatchReason =
    "[safety] latched arm_shoulder_lift temperature reached 50.0C (hard limit 50.0C); disabled all motor torque";
  harness.preparePi = async () => {
    prepareCalled = true;
    return "10.42.0.1";
  };

  await assert.rejects(
    () => controller.startRecording({ label: "blocked", inputMode: "keyboard" }),
    /Start recording is blocked because the runtime arm safety latch is active/,
  );
  assert.equal(prepareCalled, false);
  assert.match(harness.lastError, /runtime arm safety latch is active/);
});

test("quick manual restart clears latch before starting control", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const calls: string[] = [];

  harness.resetPiConnectionsLocked = async (options: unknown) => {
    calls.push("reset");
    assert.deepEqual(options, { stopLocalTeleop: false });
    harness.runtimeSafetyLatched = false;
  };
  harness.startControlLocked = async (payload: unknown, options: unknown) => {
    calls.push(`start:${JSON.stringify(payload)}:${JSON.stringify(options)}`);
    return { ok: "started" };
  };

  const state = await controller.startAgain({ armSource: "keyboard", baseSource: "keyboard" });

  assert.deepEqual(calls, [
    "reset",
    'start:{"armSource":"keyboard","baseSource":"keyboard"}:{"localReadyTimeoutMs":15000,"skipSafetyAssert":true,"skipPiConnectionReset":true}',
  ]);
  assert.deepEqual(state, { ok: "started" });
});

test("live control resets stale Pi connections before preparing the host", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const calls: string[] = [];

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.assertRuntimeSafetyClear = () => {
    calls.push("safety");
  };
  harness.resetPiConnectionsLocked = async (options: unknown) => {
    calls.push(`reset:${JSON.stringify(options)}`);
  };
  harness.getLeaderStatus = async () => leaderDisconnected;
  harness.preparePi = async () => {
    calls.push("prepare");
    return "10.42.0.1";
  };
  harness.ensureRemoteHelpers = async () => undefined;
  harness.syncVexTelemetryProgramOnPi = async () => undefined;
  harness.writeRemoteTorqueLimits = async () => undefined;
  harness.loadKeyboardJointLimitsJson = async () => "{}";
  harness.replayRunner.stop = async () => undefined;
  harness.localReplayRunner.stop = async () => undefined;
  harness.datasetCaptureRunner.stop = async () => undefined;
  harness.localDatasetCaptureRunner.stop = async () => undefined;
  harness.policyEvalRunner.stop = async () => undefined;
  harness.teleopRunner.stop = async () => undefined;
  harness.hostRunner.stop = async () => undefined;
  harness.piCalibrationRunner.stop = async () => undefined;
  harness.macCalibrationRunner.stop = async () => undefined;
  harness.clearRemoteHostCommand = async () => undefined;
  harness.hostRunner.start = async () => undefined;
  harness.waitForHostReady = async () => undefined;
  harness.teleopRunner.start = async () => undefined;
  harness.waitForLocalProcessReady = async () => undefined;
  harness.getState = async () => ({ ok: "started" });

  const state = await controller.startControl({ armSource: "keyboard", baseSource: "keyboard" });

  assert.deepEqual(state, { ok: "started" });
  assert.deepEqual(calls.slice(0, 3), [
    "safety",
    'reset:{"stopLocalTeleop":false,"clearActiveArmHold":false,"clearReplayControlRestore":false,"clearRuntimeSafetyLatch":false}',
    "prepare",
  ]);
});

test("VEX controller base disables keyboard base commands during live control", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  let hostScript = "";
  let teleopScript = "";
  let hostMeta: Record<string, unknown> = {};
  let teleopMeta: Record<string, unknown> = {};

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.assertRuntimeSafetyClear = () => undefined;
  harness.resetPiConnectionsLocked = async () => undefined;
  harness.getLeaderStatus = async () => leaderDisconnected;
  harness.preparePi = async () => "10.42.0.1";
  harness.ensureRemoteHelpers = async () => undefined;
  harness.syncVexTelemetryProgramOnPi = async () => undefined;
  harness.writeRemoteTorqueLimits = async () => undefined;
  harness.loadKeyboardJointLimitsJson = async () => "{}";
  harness.replayRunner.stop = async () => undefined;
  harness.localReplayRunner.stop = async () => undefined;
  harness.datasetCaptureRunner.stop = async () => undefined;
  harness.localDatasetCaptureRunner.stop = async () => undefined;
  harness.policyEvalRunner.stop = async () => undefined;
  harness.teleopRunner.stop = async () => undefined;
  harness.hostRunner.stop = async () => undefined;
  harness.piCalibrationRunner.stop = async () => undefined;
  harness.macCalibrationRunner.stop = async () => undefined;
  harness.clearRemoteHostCommand = async () => undefined;
  harness.hostRunner.start = async (
    script: string,
    _connection: unknown,
    _mode: string,
    meta: Record<string, unknown>,
  ) => {
    hostScript = script;
    hostMeta = meta;
  };
  harness.waitForHostReady = async () => undefined;
  harness.teleopRunner.start = async (
    script: string,
    _mode: string,
    meta: Record<string, unknown>,
  ) => {
    teleopScript = script;
    teleopMeta = meta;
  };
  harness.waitForLocalProcessReady = async () => undefined;
  harness.getState = async () => ({ ok: "started" });

  const state = await controller.startControl({ armSource: "keyboard", baseSource: "vexController" });

  assert.deepEqual(state, { ok: "started" });
  assert.match(hostScript, /--release-vex-controller-base true/);
  assert.doesNotMatch(hostScript, /--vex-live-base-control true/);
  assert.match(teleopScript, /--disable-base-input/);
  assert.equal(hostMeta.baseSource, "vexController");
  assert.equal(hostMeta.keyboardBaseInput, "disabled");
  assert.equal(hostMeta.vexLiveBaseControl, false);
  assert.equal(teleopMeta.baseSource, "vexController");
  assert.equal(teleopMeta.keyboardBaseInput, "disabled");
});

test("manual Pi reset is rejected while a robot command is starting", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  harness.exclusiveOperationInFlight = true;

  await assert.rejects(
    () => controller.resetPiConnections(),
    /robot command is still starting/,
  );
});

test("background incomplete JSON output during a reset is not surfaced as a command error", () => {
  const harness = new RobotController() as any;
  harness.suppressRemoteTransportErrorsUntil = Date.now() + 10000;

  harness.noteBackgroundRemoteError(new SyntaxError("Unexpected end of JSON input"));
  harness.noteBackgroundRemoteError(
    new SyntaxError("Expected ',' or '}' after property value in JSON at position 196608 (line 1 column 196609)"),
  );
  harness.noteBackgroundRemoteError(
    new SyntaxError("Unterminated string in JSON at position 32768 (line 1 column 32769)"),
  );

  assert.equal(harness.lastError, null);
  assert.equal(harness.activityLog.some((line: string) => line.includes("Unexpected end of JSON input")), false);
  assert.equal(harness.activityLog.some((line: string) => line.includes("position 196608")), false);
  assert.equal(harness.activityLog.some((line: string) => line.includes("position 32768")), false);
});

test("recording detail is parsed from downloaded JSON instead of SSH stdout", async () => {
  const harness = new RobotController() as any;
  const calls: string[] = [];

  harness.validateRemoteRecordingPath = async () => {
    calls.push("validate");
    return "/home/pi/lekiwi-trajectories/detail.json";
  };
  harness.downloadRemoteRecordingText = async () => {
    calls.push("download");
    return JSON.stringify({
      duration_s: 1.25,
      arm_state_keys: ["arm_shoulder_pan.pos"],
      base_state_keys: ["x.vel"],
      sensor_state_keys: ["ultrasonic_sensor_1.distance_m"],
      samples: [
        {
          t_s: 0.5,
          state: {
            "arm_shoulder_pan.pos": 12,
            "x.vel": 0.1,
            "ultrasonic_sensor_1.distance_m": 0.3,
          },
        },
      ],
      command_samples: [
        {
          t_s: 0.5,
          action: {
            "arm_shoulder_pan.pos": 13,
            "x.vel": 0.2,
          },
        },
      ],
    });
  };

  const detail = await harness.readRecordingDetail(
    defaultConfig.settings,
    "10.42.0.1",
    "/home/pi/lekiwi-trajectories/detail.json",
  );

  assert.deepEqual(calls, ["validate", "download"]);
  assert.equal(detail.path, "/home/pi/lekiwi-trajectories/detail.json");
  assert.equal(detail.durationS, 1.25);
  assert.equal(detail.sampleCount, 1);
  assert.equal(detail.commandSampleCount, 1);
  assert.equal(detail.timelineSource, "commands");
  assert.equal(detail.samples[0].values["ultrasonic_sensor_1.distance_m"], 0.3);
  assert.equal(detail.commandSamples[0].values["x.vel"], 0.2);
});

test("warm host replay remains available for VEX base recordings during live control", () => {
  const harness = new RobotController() as any;
  harness.hostRunner.getSnapshot = () => snapshot({ state: "running", mode: "control" });

  assert.equal(
    harness.canUseWarmHostReplay({
      target: "pi",
      includeBase: true,
    }),
    true,
  );
});

test("quick manual restart does not relatch from stale teleop safety logs", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  let startCalled = false;

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.runtimeSafetyLatched = true;
  harness.runtimeSafetyLatchReason =
    "[safety] latched arm_shoulder_lift temperature reached 50.0C (hard limit 50.0C); disabled all motor torque";
  harness.hostRunner.resetConnection = () => undefined;
  harness.replayRunner.resetConnection = () => undefined;
  harness.piCalibrationRunner.resetConnection = () => undefined;
  harness.datasetCaptureRunner.resetConnection = () => undefined;
  harness.policyEvalRunner.resetConnection = () => undefined;
  harness.closeActiveRemoteClients = () => 0;
  harness.teleopRunner.getSnapshot = () =>
    snapshot({
      state: "running",
      mode: "control",
      logs: [
        "10:00:00 AM [stdout] [safety] latched arm_shoulder_lift temperature reached 50.0C (hard limit 50.0C); disabled all motor torque",
      ],
    });
  harness.startControlLocked = async (_payload: unknown, options: { skipSafetyAssert?: boolean }) => {
    if (!options.skipSafetyAssert) {
      harness.assertRuntimeSafetyClear("Live control");
    }
    startCalled = true;
    return { ok: "started" };
  };

  const state = await controller.startAgain({ armSource: "keyboard", baseSource: "keyboard" });

  assert.equal(startCalled, true);
  assert.deepEqual(state, { ok: "started" });
});

test("errored recording process does not keep arm authority", () => {
  const harness = new RobotController() as any;
  const erroredRecorder = snapshot({
    state: "error",
    mode: "recording",
    detail: "Connection lost before handshake",
    meta: { recordingInputMode: "keyboard", armSource: "keyboard" },
  });

  const authority = harness.buildControlAuthorityState(
    leaderDisconnected,
    vexDisconnected,
    erroredRecorder,
    snapshot(),
    snapshot(),
    snapshot(),
    snapshot(),
  );

  assert.equal(authority.arm, "none");
  assert.equal(authority.base, "none");
});

test("foreground transient SSH errors become visible backend errors", () => {
  const harness = new RobotController() as any;

  harness.noteError(new Error("read ECONNRESET"));

  assert.match(harness.lastError, /Pi SSH connection dropped/);
});

test("warm host commands use the running host stdin before opening SSH", async () => {
  const harness = new RobotController() as any;
  let sentInput: string | null = null;
  let sentLabel: string | null = null;

  harness.hostRunner.getSnapshot = () =>
    snapshot({ state: "running", mode: "control", meta: { host: "10.42.0.1" } });
  harness.hostRunner.sendInput = (input: string, label: string) => {
    sentInput = input;
    sentLabel = label;
    return true;
  };
  harness.execRemoteScript = async () => {
    throw new Error("warm host command should not open a fresh SSH session");
  };

  await harness.writeRemoteHostCommand(defaultConfig.settings, "10.42.0.1", {
    command: "stop-replay",
  });

  assert.ok(sentInput);
  assert.equal(JSON.parse(sentInput).command, "stop-replay");
  assert.equal(sentLabel, "warm-host stop-replay");
});

test("warm host command channel failure does not fall back to fresh SSH", async () => {
  const harness = new RobotController() as any;

  harness.hostRunner.getSnapshot = () =>
    snapshot({ state: "running", mode: "control", meta: { host: "10.42.0.1" } });
  harness.hostRunner.sendInput = () => false;
  harness.execRemoteScript = async () => {
    throw new Error("should not open a fresh SSH session while warm host owns the Pi");
  };

  await assert.rejects(
    () =>
      harness.writeRemoteHostCommand(defaultConfig.settings, "10.42.0.1", {
        command: "stop-replay",
      }),
    /running Pi host command channel is unavailable/,
  );
});

test("state treats a running warm host as reachable and clears stale Pi errors", async () => {
  const harness = new RobotController() as any;

  harness.lastError =
    "Could not restore leader control: the Pi is not reachable on 10.42.0.1 after 5 attempts.";
  harness.findReachablePiHost = async () => {
    throw new Error("state should not probe SSH while the warm host is running");
  };
  harness.getLeaderStatus = async () => leaderDisconnected;
  harness.hostRunner.getSnapshot = () =>
    snapshot({ state: "running", mode: "control", meta: { host: "10.42.0.1" } });

  const state = await harness.getState();

  assert.equal(state.piReachable, true);
  assert.equal(state.resolvedPiHost, "10.42.0.1");
  assert.equal(state.lastError, null);
});

test("VEX 0 speed toggle persists into restored keyboard teleop sessions", async () => {
  const harness = new RobotController() as any;

  harness.hostRunner.getSnapshot = () =>
    snapshot({ state: "running", mode: "control", meta: { host: "10.42.0.1" } });
  harness.teleopRunner.getSnapshot = () =>
    snapshot({
      state: "running",
      mode: "control",
      meta: { armSource: "leader" },
      logs: [
        "2:00:00 PM [stdout] Leader arm + keyboard base teleop is live. VEX base mode: DRIVE.",
        "2:01:00 PM [stdout] VEX base mode switched to ECU speed.",
      ],
    });
  harness.getLeaderStatus = async () => leaderDisconnected;

  const state = await harness.getState();
  const script = harness.buildKeyboardTeleopScript(defaultConfig.settings, "10.42.0.1", "{}");

  assert.equal(state.controlAuthority.speedPreset, "ecu");
  assert.match(script, /--initial-vex-control-mode ecu/);
});

test("command-ready hold host starts with VEX live base control enabled", async () => {
  const harness = new RobotController() as any;
  let hostScript = "";
  const hostMeta: Record<string, unknown> = {};

  harness.hostRunner.getSnapshot = () => snapshot();
  harness.preparePi = async () => "10.42.0.1";
  harness.ensureRemoteHelpers = async () => undefined;
  harness.syncVexTelemetryProgramOnPi = async () => undefined;
  harness.writeRemoteTorqueLimits = async () => undefined;
  harness.stopRobotExclusiveProcesses = async () => undefined;
  harness.clearRemoteHostCommand = async () => undefined;
  harness.hostRunner.start = async (
    script: string,
    _connection: unknown,
    _mode: string,
    meta: Record<string, unknown>,
  ) => {
    hostScript = script;
    Object.assign(hostMeta, meta);
  };
  harness.waitForHostReady = async () => undefined;

  const host = await harness.ensureCommandReadyHost(
    defaultConfig.settings,
    "return to held arm pose after recording",
  );

  assert.equal(host, "10.42.0.1");
  assert.match(hostScript, /--vex-live-base-control true/);
  assert.equal(hostMeta.vexLiveBaseControl, true);
});

test("remote state refresh skips auxiliary SSH work while live host owns the Pi", async () => {
  const harness = new RobotController() as any;
  const calls: string[] = [];

  harness.lastRecordingRefresh = 0;
  harness.lastPowerLogSync = 0;
  harness.lastTrainingMetadataRefresh = 0;
  harness.fetchRecordings = async () => {
    calls.push("fetch-recordings");
    return [];
  };
  harness.syncRemotePowerLogs = async () => {
    calls.push("sync-power");
  };
  harness.getVexBrainStatus = async () => {
    calls.push("vex-status");
  };
  harness.refreshTrainingArtifacts = async () => {
    calls.push("training");
  };

  harness.startRemoteStateRefresh(
    defaultConfig.settings,
    "10.42.0.1",
    snapshot({ state: "running", mode: "control", meta: { host: "10.42.0.1" } }),
    snapshot(),
    snapshot(),
    defaultConfig.training,
  );
  await new Promise((resolve) => setTimeout(resolve, 10));

  assert.deepEqual(calls, []);
  assert.equal(harness.remoteStateRefreshInFlight, false);
});

test("hold pins replay follower without live SSH pose preload", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const config = structuredClone(defaultConfig);
  const holdPin = {
    id: "hold-pin",
    name: "Hold Pin",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-pin.json",
    target: "pi",
    vexReplayMode: "ecu",
    homeMode: "none",
    speed: 1,
    autoVexPositioning: true,
    vexPositioningSpeed: 1,
    vexPositioningTimeoutS: 8,
    vexPositioningXyToleranceM: 0.02,
    vexPositioningHeadingToleranceDeg: 1.5,
    vexPositioningXyTrimToleranceM: 0.05,
    vexPositioningHeadingTrimToleranceDeg: 8.5,
    includeBase: true,
    holdFinalS: 0.5,
    keyBinding: "H",
    holdArmPose: true,
  };
  const commandReadyReasons: string[] = [];
  let replayArgs: unknown[] | null = null;

  config.pinnedMoves = [holdPin as any];
  harness.configStore.getConfig = () => config;
  harness.preparePi = async () => {
    throw new Error("hold activation should not open a fresh SSH session");
  };
  harness.readRecordingEndHomePosition = async () => {
    throw new Error("hold activation should not read a remote pose");
  };
  harness.ensureCommandReadyHost = async (_settings: unknown, reason: string) => {
    commandReadyReasons.push(reason);
    return "10.42.0.1";
  };
  harness.startReplayLocked = async (...args: unknown[]) => {
    replayArgs = args;
    return { ok: true };
  };

  const state = await controller.triggerPinnedMove("hold-pin");

  assert.deepEqual(state, { ok: true });
  assert.deepEqual(commandReadyReasons, ["arm hold replay"]);
  assert.equal(harness.activeArmHold?.pinnedMoveId, "hold-pin");
  assert.equal(harness.activeArmHold?.name, "Hold Pin");
  assert.equal(harness.activeArmHold?.homePosition, null);
  assert.ok(replayArgs);
  assert.equal((replayArgs[0] as any).target, "pi");
  assert.equal((replayArgs[0] as any).homeMode, "none");
  assert.equal((replayArgs[0] as any).includeBase, false);
  assert.equal((replayArgs[0] as any).autoVexPositioning, false);
  assert.equal((replayArgs[0] as any).startTimeS, 0);
  assert.equal(replayArgs[1], null);
  assert.deepEqual(replayArgs[2], { returnToActiveHold: false });
});

test("duplicate hold pin triggers are ignored while the first trigger is in flight", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const config = structuredClone(defaultConfig);
  const holdPin = {
    id: "hold-live",
    name: "Hold Live",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
    target: "pi",
    vexReplayMode: "ecu",
    homeMode: "none",
    speed: 1,
    autoVexPositioning: false,
    vexPositioningSpeed: 1,
    vexPositioningTimeoutS: 8,
    vexPositioningXyToleranceM: 0.02,
    vexPositioningHeadingToleranceDeg: 1.5,
    vexPositioningXyTrimToleranceM: 0.05,
    vexPositioningHeadingTrimToleranceDeg: 8.5,
    includeBase: false,
    holdFinalS: 0.5,
    keyBinding: "8",
    holdArmPose: true,
  };
  let releaseCommandReady: () => void = () => undefined;
  const commandReadyStarted = new Promise<void>((resolve) => {
    harness.ensureCommandReadyHost = async () => {
      resolve();
      await new Promise<void>((release) => {
        releaseCommandReady = release;
      });
      return "10.42.0.1";
    };
  });

  config.pinnedMoves = [holdPin as any];
  harness.configStore.getConfig = () => config;
  harness.startReplayLocked = async () => ({ ok: "first" });
  harness.getState = async () => ({ ok: "current" });

  const first = controller.triggerPinnedMove("hold-live");
  await commandReadyStarted;

  const duplicate = await controller.triggerPinnedMove("hold-live");
  assert.deepEqual(duplicate, { ok: "current" });

  releaseCommandReady();
  assert.deepEqual(await first, { ok: "first" });
});

test("keyboard arm input overrides active hold and restores keyboard arm authority", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const config = structuredClone(defaultConfig);
  const replayStops: string[] = [];
  let restoreArgs: unknown[] | null = null;

  harness.configStore.getConfig = () => config;
  harness.activeArmHold = {
    pinnedMoveId: "hold-live",
    name: "Hold Live",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
    activatedAt: "2026-05-04T14:00:00.000Z",
    homePosition: holdHomePosition,
    restoreArmSource: "leader",
  };
  harness.deriveWarmHostReplaySnapshot = () => snapshot({ state: "idle" });
  harness.replayRunner.stop = async (reason: string) => {
    replayStops.push(reason);
  };
  harness.localReplayRunner.stop = async (reason: string) => {
    replayStops.push(reason);
  };
  harness.restoreControlAfterHoldRelease = async (...args: unknown[]) => {
    restoreArgs = args;
  };
  harness.getState = async () => ({
    activeArmHold: harness.activeArmHold,
    keyboardOverrideArmHold: harness.keyboardOverrideArmHold,
  });

  const state = await controller.overrideActiveArmHoldWithKeyboard();

  assert.equal(harness.activeArmHold, null);
  assert.equal(harness.keyboardOverrideArmHold?.pinnedMoveId, "hold-live");
  assert.equal(harness.keyboardOverrideArmHold?.homePosition, holdHomePosition);
  assert.deepEqual(restoreArgs, [config.settings, "Hold Live", "keyboard"]);
  assert.deepEqual(replayStops, [
    "Keyboard arm control is overriding the hold.",
    "Keyboard arm control is overriding the hold.",
  ]);
  assert.deepEqual(state, {
    activeArmHold: null,
    keyboardOverrideArmHold: harness.keyboardOverrideArmHold,
  });
});

test("enter returns keyboard override hold by replaying the held recording", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const config = structuredClone(defaultConfig);
  const holdPin = {
    id: "hold-live",
    name: "Hold Live",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
    target: "pi",
    vexReplayMode: "ecu",
    homeMode: "none",
    speed: 1,
    autoVexPositioning: false,
    vexPositioningSpeed: 1,
    vexPositioningTimeoutS: 8,
    vexPositioningXyToleranceM: 0.02,
    vexPositioningHeadingToleranceDeg: 1.5,
    vexPositioningXyTrimToleranceM: 0.05,
    vexPositioningHeadingTrimToleranceDeg: 8.5,
    includeBase: false,
    holdFinalS: 0.5,
    keyBinding: "8",
    holdArmPose: true,
  };
  const commandReadyReasons: string[] = [];
  let replayArgs: unknown[] | null = null;

  config.pinnedMoves = [holdPin as any];
  harness.configStore.getConfig = () => config;
  harness.keyboardOverrideArmHold = {
    pinnedMoveId: "hold-live",
    name: "Hold Live",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
    activatedAt: "2026-05-04T14:00:00.000Z",
    homePosition: holdHomePosition,
    restoreArmSource: "leader",
  };
  harness.ensureCommandReadyHost = async (_settings: unknown, reason: string) => {
    commandReadyReasons.push(reason);
    return "10.42.0.1";
  };
  harness.startReplayLocked = async (...args: unknown[]) => {
    replayArgs = args;
    return { ok: true };
  };

  const state = await controller.returnToKeyboardOverrideArmHold();

  assert.deepEqual(state, { ok: true });
  assert.equal(harness.keyboardOverrideArmHold, null);
  assert.equal(harness.activeArmHold?.pinnedMoveId, "hold-live");
  assert.equal(harness.activeArmHold?.restoreArmSource, "leader");
  assert.deepEqual(commandReadyReasons, ["return to held arm pose"]);
  assert.ok(replayArgs);
  assert.equal((replayArgs[0] as any).target, "pi");
  assert.equal((replayArgs[0] as any).homeMode, "none");
  assert.equal((replayArgs[0] as any).includeBase, false);
  assert.equal((replayArgs[0] as any).autoVexPositioning, false);
  assert.equal((replayArgs[0] as any).startTimeS, 0);
  assert.equal(replayArgs[1], null);
  assert.deepEqual(replayArgs[2], { returnToActiveHold: false });
});

test("hold hotkey from keyboard override restores leader instead of rearming hold", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const config = structuredClone(defaultConfig);
  const holdPin = {
    id: "hold-live",
    name: "Hold Live",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
    target: "pi",
    vexReplayMode: "ecu",
    homeMode: "none",
    speed: 1,
    autoVexPositioning: false,
    vexPositioningSpeed: 1,
    vexPositioningTimeoutS: 8,
    vexPositioningXyToleranceM: 0.02,
    vexPositioningHeadingToleranceDeg: 1.5,
    vexPositioningXyTrimToleranceM: 0.05,
    vexPositioningHeadingTrimToleranceDeg: 8.5,
    includeBase: false,
    holdFinalS: 0.5,
    keyBinding: "8",
    holdArmPose: true,
  };
  let restoreArgs: unknown[] | null = null;

  config.pinnedMoves = [holdPin as any];
  harness.configStore.getConfig = () => config;
  harness.keyboardOverrideArmHold = {
    pinnedMoveId: "hold-live",
    name: "Hold Live",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
    activatedAt: "2026-05-04T14:00:00.000Z",
    homePosition: holdHomePosition,
    restoreArmSource: "keyboard",
  };
  harness.restoreControlAfterHoldRelease = async (...args: unknown[]) => {
    restoreArgs = args;
  };
  harness.startReplayLocked = async () => {
    throw new Error("Suspended hold hotkey should not replay the hold.");
  };
  harness.getState = async () => ({ ok: true });

  const state = await controller.triggerPinnedMove("hold-live");

  assert.deepEqual(state, { ok: true });
  assert.equal(harness.activeArmHold, null);
  assert.equal(harness.keyboardOverrideArmHold, null);
  assert.deepEqual(restoreArgs, [config.settings, "Hold Live", "leader"]);
});

test("hold pins override live leader control and restore base-only hold", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const config = structuredClone(defaultConfig);
  const stops: string[] = [];
  const teleopStarts: unknown[][] = [];
  let teleopMeta: Record<string, unknown> = { armSource: "leader" };
  let replayCommand: unknown[] | null = null;

  config.pinnedMoves = [
    {
      id: "hold-live",
      name: "Hold Live",
      trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
      target: "pi",
      vexReplayMode: "ecu",
      homeMode: "none",
      speed: 1,
      autoVexPositioning: false,
      vexPositioningSpeed: 1,
      vexPositioningTimeoutS: 8,
      vexPositioningXyToleranceM: 0.02,
      vexPositioningHeadingToleranceDeg: 1.5,
      vexPositioningXyTrimToleranceM: 0.05,
      vexPositioningHeadingTrimToleranceDeg: 8.5,
      includeBase: false,
      holdFinalS: 0.5,
      keyBinding: "H",
      holdArmPose: true,
    } as any,
  ];
  harness.configStore.getConfig = () => config;
  harness.ensureCommandReadyHost = async () => "10.42.0.1";
  harness.hostRunner.getSnapshot = () =>
    snapshot({
      state: "running",
      mode: "control",
      meta: {
        host: "10.42.0.1",
        input: "leader+keyboard-base",
        armSource: "leader",
        vexLiveBaseControl: true,
      },
    });
  harness.teleopRunner.getSnapshot = () =>
    snapshot({ state: "running", mode: "control", meta: teleopMeta });
  harness.teleopRunner.stop = async (reason: string) => {
    stops.push(reason);
  };
  harness.teleopRunner.start = async (...args: unknown[]) => {
    teleopStarts.push(args);
    teleopMeta = args[2] as Record<string, unknown>;
  };
  harness.waitForLocalProcessReady = async () => undefined;
  harness.replayRunner.stop = async () => undefined;
  harness.localReplayRunner.stop = async () => undefined;
  harness.stopWarmHostReplay = async () => undefined;
  harness.piCalibrationRunner.stop = async () => undefined;
  harness.macCalibrationRunner.stop = async () => undefined;
  harness.datasetCaptureRunner.stop = async () => undefined;
  harness.localDatasetCaptureRunner.stop = async () => undefined;
  harness.policyEvalRunner.stop = async () => undefined;
  harness.sendWarmHostReplay = async (...args: unknown[]) => {
    replayCommand = args;
  };
  harness.getState = async () => ({ ok: true });

  const state = await controller.triggerPinnedMove("hold-live");

  assert.deepEqual(state, { ok: true });
  assert.ok(stops.includes("Restoring base-only keyboard control during replay."));
  assert.equal(stops.includes("Replay overrides live control until it completes."), false);
  assert.equal(teleopStarts.length, 1);
  assert.match(teleopStarts[0][0] as string, /--disable-arm-input/);
  assert.equal((teleopStarts[0][2] as any).armSource, "none");
  assert.equal((teleopStarts[0][2] as any).armInput, "disabled");
  assert.ok(replayCommand);
  assert.equal((replayCommand[2] as any).homeMode, "none");
  assert.equal((replayCommand[2] as any).includeBase, false);
  assert.equal(harness.activeArmHold?.restoreArmSource, "leader");
  assert.equal(harness.replayControlRestore?.restoreArmSource, "none");
  assert.equal(harness.replayControlRestore?.returnToActiveHold, false);
});

test("live Pi replay without hold restores previous leader control", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const config = structuredClone(defaultConfig);

  config.pinnedMoves = [
    {
      id: "move-live",
      name: "Move Live",
      trajectoryPath: "/home/pi/lekiwi-trajectories/move-live.json",
      target: "pi",
      vexReplayMode: "ecu",
      homeMode: "none",
      speed: 1,
      autoVexPositioning: false,
      vexPositioningSpeed: 1,
      vexPositioningTimeoutS: 8,
      vexPositioningXyToleranceM: 0.02,
      vexPositioningHeadingToleranceDeg: 1.5,
      vexPositioningXyTrimToleranceM: 0.05,
      vexPositioningHeadingTrimToleranceDeg: 8.5,
      includeBase: false,
      holdFinalS: 0.5,
      keyBinding: "M",
      holdArmPose: false,
    } as any,
  ];
  harness.configStore.getConfig = () => config;
  harness.hostRunner.getSnapshot = () =>
    snapshot({
      state: "running",
      mode: "control",
      meta: {
        host: "10.42.0.1",
        input: "leader+keyboard-base",
        armSource: "leader",
        vexLiveBaseControl: true,
      },
    });
  harness.teleopRunner.getSnapshot = () =>
    snapshot({ state: "running", mode: "control", meta: { armSource: "leader" } });
  harness.teleopRunner.stop = async () => undefined;
  harness.replayRunner.stop = async () => undefined;
  harness.localReplayRunner.stop = async () => undefined;
  harness.stopWarmHostReplay = async () => undefined;
  harness.piCalibrationRunner.stop = async () => undefined;
  harness.macCalibrationRunner.stop = async () => undefined;
  harness.datasetCaptureRunner.stop = async () => undefined;
  harness.localDatasetCaptureRunner.stop = async () => undefined;
  harness.policyEvalRunner.stop = async () => undefined;
  harness.sendWarmHostReplay = async () => undefined;
  harness.getState = async () => ({ ok: true });

  await controller.triggerPinnedMove("move-live");

  assert.equal(harness.replayControlRestore?.restoreArmSource, "leader");
  assert.equal(harness.replayControlRestore?.returnToActiveHold, false);
});

test("live Pi replay with active hold replays the hold recording after completion", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const config = structuredClone(defaultConfig);
  const replayCommands: unknown[][] = [];
  const restoredSessions: unknown[][] = [];
  const teleopStarts: unknown[][] = [];
  const holdReturns: string[] = [];
  let teleopMeta: Record<string, unknown> = { armSource: "leader" };

  config.pinnedMoves = [
    {
      id: "hold-live",
      name: "Hold Live",
      trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
      target: "pi",
      vexReplayMode: "ecu",
      homeMode: "none",
      speed: 1,
      autoVexPositioning: false,
      vexPositioningSpeed: 1,
      vexPositioningTimeoutS: 8,
      vexPositioningXyToleranceM: 0.02,
      vexPositioningHeadingToleranceDeg: 1.5,
      vexPositioningXyTrimToleranceM: 0.05,
      vexPositioningHeadingTrimToleranceDeg: 8.5,
      includeBase: false,
      holdFinalS: 0.5,
      keyBinding: "H",
      holdArmPose: true,
    } as any,
    {
      id: "move-live",
      name: "Move Live",
      trajectoryPath: "/home/pi/lekiwi-trajectories/move-live.json",
      target: "pi",
      vexReplayMode: "ecu",
      homeMode: "none",
      speed: 1,
      autoVexPositioning: false,
      vexPositioningSpeed: 1,
      vexPositioningTimeoutS: 8,
      vexPositioningXyToleranceM: 0.02,
      vexPositioningHeadingToleranceDeg: 1.5,
      vexPositioningXyTrimToleranceM: 0.05,
      vexPositioningHeadingTrimToleranceDeg: 8.5,
      includeBase: false,
      holdFinalS: 0.5,
      keyBinding: "M",
      holdArmPose: false,
    } as any,
  ];
  harness.activeArmHold = {
    pinnedMoveId: "hold-live",
    name: "Hold Live",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
    activatedAt: "2026-04-30T13:05:00.000Z",
    homePosition: holdHomePosition,
  };
  harness.configStore.getConfig = () => config;
  harness.hostRunner.getSnapshot = () =>
    snapshot({
      state: "running",
      mode: "control",
      meta: {
        host: "10.42.0.1",
        input: "leader+keyboard-base",
        armSource: "leader",
        vexLiveBaseControl: true,
      },
    });
  harness.teleopRunner.getSnapshot = () =>
    snapshot({ state: "running", mode: "control", meta: teleopMeta });
  harness.ensureCommandReadyHost = async () => "10.42.0.1";
  harness.teleopRunner.stop = async () => undefined;
  harness.replayRunner.stop = async () => undefined;
  harness.localReplayRunner.stop = async () => undefined;
  harness.stopWarmHostReplay = async () => undefined;
  harness.piCalibrationRunner.stop = async () => undefined;
  harness.macCalibrationRunner.stop = async () => undefined;
  harness.datasetCaptureRunner.stop = async () => undefined;
  harness.localDatasetCaptureRunner.stop = async () => undefined;
  harness.policyEvalRunner.stop = async () => undefined;
  harness.sendWarmHostReplay = async (...args: unknown[]) => {
    replayCommands.push(args);
  };
  harness.returnToActiveArmHoldPose = async (_settings: unknown, actionLabel: string, reason: string) => {
    holdReturns.push(`${actionLabel}:${reason}`);
    return "10.42.0.1";
  };
  harness.startKeyboardControlSession = async (...args: unknown[]) => {
    restoredSessions.push(args);
  };
  harness.teleopRunner.start = async (...args: unknown[]) => {
    teleopStarts.push(args);
    teleopMeta = args[2] as Record<string, unknown>;
  };
  harness.waitForLocalProcessReady = async () => undefined;
  harness.getState = async () => ({ ok: true });

  await controller.triggerPinnedMove("move-live");

  assert.equal(replayCommands.length, 1);
  assert.equal((replayCommands[0][2] as any).trajectoryPath, "/home/pi/lekiwi-trajectories/move-live.json");
  assert.equal((replayCommands[0][2] as any).homeMode, "none");
  assert.equal(teleopStarts.length, 1);
  assert.match(teleopStarts[0][0] as string, /--disable-arm-input/);
  assert.equal((teleopStarts[0][2] as any).armSource, "none");
  assert.equal((teleopStarts[0][2] as any).activeArmHold, "Hold Live");
  assert.equal(harness.replayControlRestore?.restoreArmSource, "none");
  assert.equal(harness.replayControlRestore?.returnToActiveHold, true);

  harness.maybeRestoreControlAfterReplay(
    config.settings,
    snapshot(),
    snapshot(),
    snapshot({
      state: "idle",
      stoppedAt: new Date().toISOString(),
      meta: { transport: "warm-host" },
    }),
  );
  await harness.replayControlRestoreInFlight;

  assert.deepEqual(holdReturns, ["restore held arm pose after replay:after replay"]);
  assert.equal(restoredSessions.length, 0);
  assert.equal(teleopStarts.length, 1);
});

test("returning to active hold waits for the newly queued hold recording replay", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const commands: unknown[][] = [];
  let commandQueued = false;
  let waitPolls = 0;
  const now = new Date();
  const timeLabel = now.toLocaleTimeString("en-US", {
    hour12: true,
    hour: "numeric",
    minute: "2-digit",
    second: "2-digit",
  });
  const previousLogs = [
    `${timeLabel} [stdout] [replay] start path=/home/pi/lekiwi-trajectories/other.json`,
    `${timeLabel} [stdout] [replay] complete path=/home/pi/lekiwi-trajectories/other.json`,
  ];
  const holdStartLog = `${timeLabel} [stdout] [replay] start path=/home/pi/lekiwi-trajectories/hold-live.json`;
  const holdCompleteLog = `${timeLabel} [stdout] [replay] complete path=/home/pi/lekiwi-trajectories/hold-live.json`;

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.activeArmHold = {
    pinnedMoveId: "hold-live",
    name: "Hold Live",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
    activatedAt: "2026-04-30T13:05:00.000Z",
    homePosition: holdHomePosition,
  };
  harness.ensureCommandReadyHost = async () => "10.42.0.1";
  harness.teleopRunner.stop = async () => undefined;
  harness.hostRunner.getSnapshot = () => {
    if (!commandQueued) {
      return snapshot({ state: "running", mode: "control", logs: previousLogs });
    }
    waitPolls += 1;
    if (waitPolls === 1) {
      return snapshot({ state: "running", mode: "control", logs: previousLogs });
    }
    if (waitPolls === 2) {
      return snapshot({ state: "running", mode: "control", logs: [...previousLogs, holdStartLog] });
    }
    return snapshot({
      state: "running",
      mode: "control",
      logs: [...previousLogs, holdStartLog, holdCompleteLog],
    });
  };
  harness.sendWarmHostReplay = async (...args: unknown[]) => {
    commands.push(args);
    commandQueued = true;
  };

  const host = await harness.returnToActiveArmHoldPose(
    defaultConfig.settings,
    "restore held arm pose after replay",
    "after replay",
  );

  assert.equal(host, "10.42.0.1");
  assert.equal(commands.length, 1);
  assert.equal((commands[0][2] as any).trajectoryPath, "/home/pi/lekiwi-trajectories/hold-live.json");
  assert.equal((commands[0][2] as any).homeMode, "none");
  assert.equal((commands[0][2] as any).includeBase, false);
  assert.ok(waitPolls >= 3);
});

test("released hold hotkey restores remembered leader control locally", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const config = structuredClone(defaultConfig);
  const teleopStops: string[] = [];
  let teleopStart: unknown[] | null = null;
  let warmReplayStopped = false;

  config.pinnedMoves = [
    {
      id: "hold-live",
      name: "Hold Live",
      trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
      target: "pi",
      vexReplayMode: "ecu",
      homeMode: "none",
      speed: 1,
      autoVexPositioning: false,
      vexPositioningSpeed: 1,
      vexPositioningTimeoutS: 8,
      vexPositioningXyToleranceM: 0.02,
      vexPositioningHeadingToleranceDeg: 1.5,
      vexPositioningXyTrimToleranceM: 0.05,
      vexPositioningHeadingTrimToleranceDeg: 8.5,
      includeBase: false,
      holdFinalS: 0.5,
      keyBinding: "H",
      holdArmPose: true,
    } as any,
  ];
  harness.activeArmHold = {
    pinnedMoveId: "hold-live",
    name: "Hold Live",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-live.json",
    activatedAt: "2026-04-30T13:05:00.000Z",
    homePosition: holdHomePosition,
    restoreArmSource: "leader",
  };
  harness.configStore.getConfig = () => config;
  harness.hostRunner.getSnapshot = () =>
    snapshot({
      state: "running",
      mode: "keyboard-control",
      meta: { host: "10.42.0.1", armSource: "none", activeArmHold: "Hold Live" },
    });
  harness.getActiveHostAddress = () => "10.42.0.1";
  harness.getLeaderStatus = async () => ({
    teleoperateScriptPath: "",
    connected: true,
    expectedPort: "/dev/tty.usbmodem123",
    availablePorts: ["/dev/tty.usbmodem123"],
    message: "Leader arm detected.",
  });
  harness.teleopRunner.stop = async (reason: string) => {
    teleopStops.push(reason);
  };
  harness.teleopRunner.start = async (...args: unknown[]) => {
    teleopStart = args;
  };
  harness.waitForLocalProcessReady = async () => undefined;
  harness.replayRunner.stop = async () => undefined;
  harness.localReplayRunner.stop = async () => undefined;
  harness.stopWarmHostReplay = async () => {
    warmReplayStopped = true;
  };
  harness.getState = async () => ({ ok: true });

  const state = await controller.triggerPinnedMove("hold-live");

  assert.deepEqual(state, { ok: true });
  assert.equal(harness.activeArmHold, null);
  assert.deepEqual(teleopStops, ["Restoring leader control after arm hold release."]);
  assert.equal(warmReplayStopped, false);
  assert.ok(teleopStart);
  assert.match(teleopStart[0] as string, /--leader-port '\/dev\/tty\.usbmodem123'/);
  assert.match(teleopStart[0] as string, /--arm-source 'leader'/);
  assert.equal(teleopStart[1], "keyboard-control");
  assert.equal((teleopStart[2] as any).armSource, "leader");
  assert.equal((teleopStart[2] as any).activeArmHoldReleased, "Hold Live");
});

test("completed warm replay restores leader control when no hold is active", async () => {
  const harness = new RobotController() as any;
  const sessions: unknown[][] = [];

  harness.replayControlRestore = {
    requestedAtMs: Date.now() - 1000,
    target: "pi",
    trajectoryPath: "/home/pi/lekiwi-trajectories/move-live.json",
    restoreArmSource: "leader",
    returnToActiveHold: false,
  };
  harness.preparePi = async () => "10.42.0.1";
  harness.startKeyboardControlSession = async (...args: unknown[]) => {
    sessions.push(args);
  };

  harness.maybeRestoreControlAfterReplay(
    defaultConfig.settings,
    snapshot(),
    snapshot(),
    snapshot({
      state: "idle",
      stoppedAt: new Date().toISOString(),
      meta: { transport: "warm-host" },
    }),
  );
  await harness.replayControlRestoreInFlight;

  assert.equal(sessions.length, 1);
  assert.deepEqual(sessions[0][4], {
    allowLeader: true,
    disableArmInput: false,
    armSource: "leader",
  });
});

test("recording stop from active hold replays the hold recording", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const commands: unknown[] = [];
  const restoredSessions: unknown[] = [];
  const waits: unknown[][] = [];

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.hostRunner.getSnapshot = () =>
    snapshot({
      state: "running",
      mode: "recording",
      meta: { input: "keyboard", armSource: "keyboard" },
    });
  harness.teleopRunner.getSnapshot = () => snapshot({ state: "running", mode: "recording" });
  harness.teleopRunner.stop = async () => undefined;
  harness.hostRunner.stop = async () => undefined;
  harness.findReachablePiHost = async () => "10.42.0.1";
  harness.fetchRecordings = async () => [];
  harness.ensureCommandReadyHost = async () => "10.42.0.1";
  harness.writeRemoteHostCommand = async (_settings: unknown, _host: string, command: unknown) => {
    commands.push(command);
  };
  harness.waitForWarmHostReplayStopped = async (...args: unknown[]) => {
    waits.push(args);
  };
  harness.startKeyboardControlSession = async (...args: unknown[]) => {
    restoredSessions.push(args);
  };
  harness.getState = async () => ({ ok: true });
  harness.activeArmHold = {
    pinnedMoveId: "hold-open",
    name: "Open claw hold",
    trajectoryPath: "/home/pi/lekiwi-trajectories/hold-open.json",
    activatedAt: "2026-04-30T13:05:00.000Z",
    homePosition: holdHomePosition,
  };

  await controller.stopRecording();

  assert.equal(commands.length, 1);
  const command = commands[0] as { command: string; trajectoryPath: string; homeMode: string; includeBase: boolean };
  assert.equal(command.command, "replay");
  assert.equal(command.trajectoryPath, "/home/pi/lekiwi-trajectories/hold-open.json");
  assert.equal(command.homeMode, "none");
  assert.equal(command.includeBase, false);
  assert.equal(waits.length, 1);
  assert.equal(restoredSessions.length, 1);
  assert.deepEqual((restoredSessions[0] as unknown[])[4], {
    allowLeader: false,
    disableArmInput: true,
  });
});

test("emergency stop script releases VEX base hold and cuts telemetry before torque-off", () => {
  const harness = new RobotController() as any;
  const script = harness.buildStopAllScript(defaultConfig.settings);

  assert.match(script, /for command in \(b"!release\\n", b"!telemetry off\\n", b"!telemetry off\\n"\):/);
  assert.match(script, /accepted_ports\.append\(port\)/);
  assert.match(script, /Sent VEX base shutdown commands/);
  assert.ok(script.indexOf('b"!release\\n"') < script.indexOf("bus.disable_torque"));
  assert.ok(script.indexOf('b"!telemetry off\\n"') < script.indexOf("bus.disable_torque"));
});
