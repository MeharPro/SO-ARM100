import assert from "node:assert/strict";
import test from "node:test";

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
