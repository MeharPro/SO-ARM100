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

test("discarding leader authority keeps leader resume blocked", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  harness.getState = async () => ({ ok: true });
  harness.markLeaderPoseStale("keyboard follower movement");

  await controller.resolveLeaderStale({ action: "discardLeaderAuthority" });

  assert.equal(harness.leaderPoseStale, true);
  assert.match(harness.leaderPoseStaleReason, /leader authority discarded/);
  assert.throws(
    () => harness.assertLeaderPoseFresh("Leader arm control"),
    /Leader arm control is blocked because the leader pose is stale/,
  );
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
      "10:00:00 AM [stdout] [safety] latched arm_gripper stalled; disabled all motor torque",
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

test("runtime safety latch blocks new recording before Pi commands", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  let prepareCalled = false;

  harness.configStore.getConfig = () => structuredClone(defaultConfig);
  harness.runtimeSafetyLatched = true;
  harness.runtimeSafetyLatchReason =
    "[safety] latched arm_shoulder_lift stalled; disabled all motor torque";
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

test("recording stop from active hold returns directly to saved hold pose", async () => {
  const controller = new RobotController();
  const harness = controller as any;
  const commands: unknown[] = [];
  const restoredSessions: unknown[] = [];

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
  harness.waitForWarmHostHomeCommandResult = async () => ({
    success: true,
    status: "ok",
    homePosition: holdHomePosition,
  });
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
  const command = commands[0] as { command: string; requestId: string; homePosition: unknown };
  assert.deepEqual(command, {
    command: "go-home",
    requestId: command.requestId,
    homePosition: holdHomePosition,
  });
  assert.match(command.requestId, /^return-hold-/);
  assert.equal(restoredSessions.length, 1);
  assert.deepEqual((restoredSessions[0] as unknown[])[4], {
    allowLeader: false,
    disableArmInput: true,
  });
});

test("emergency stop script releases VEX base hold before torque-off", () => {
  const harness = new RobotController() as any;
  const script = harness.buildStopAllScript(defaultConfig.settings);

  assert.match(script, /handle\.write\(b"!release\\n"\)/);
  assert.match(script, /Sent VEX base release/);
  assert.ok(script.indexOf('b"!release\\n"') < script.indexOf("bus.disable_torque"));
});
