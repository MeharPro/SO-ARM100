import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import test from "node:test";

import {
  createGamePlanStepFromFavorite,
  createDefaultMoveDefinitions,
  favoriteTrajectoryIsBroken,
  favoriteVersionForMove,
  gameBuilderEligibleFavorites,
  gameBuilderHotkeyErrors,
  nextMoveVersionNumber,
  withFavoriteVersion,
} from "./betaMoves.js";
import { ConfigStore } from "./configStore.js";
import { defaultConfig } from "./defaultConfig.js";
import type { MoveRecordingVersion } from "./types.js";

const EXPECTED_STARTER_MOVES = [
  ["general-open-left-beam-door", "General", "Open left beam door"],
  ["general-open-right-beam-door", "General", "Open right beam door"],
  ["fuse-collection-open-claw-blue", "Fuse collection", "Open claw position for driver to orient bot - blue"],
  ["fuse-collection-close-blue-slot", "Fuse collection", "Close open claw -> blue slot"],
  ["fuse-collection-open-claw-yellow", "Fuse collection", "Open claw position for driver to orient bot - yellow"],
  ["fuse-collection-close-yellow-slot", "Fuse collection", "Close open claw -> yellow slot"],
  ["fuse-collection-open-claw-green", "Fuse collection", "Open claw position for driver to orient bot - green"],
  ["fuse-collection-close-green-slot", "Fuse collection", "Close open claw -> green slot"],
  ["fuse-transfer-low-height", "Fuse removal/insertion", "Low fuse height"],
  ["fuse-transfer-high-height", "Fuse removal/insertion", "High fuse height"],
  ["fuse-transfer-low-height-repeat", "Fuse removal/insertion", "Low fuse height repeat"],
  ["fuse-transfer-green-slot-low-height", "Fuse removal/insertion", "Green slot -> low fuse height"],
  ["fuse-transfer-yellow-slot-high-height", "Fuse removal/insertion", "Yellow slot -> high fuse height"],
  ["fuse-transfer-blue-slot-low-height", "Fuse removal/insertion", "Blue slot -> low fuse height"],
  ["board-collection-open-claw-8-inch", "Circuit Board collection", "Open claw 8 inch board height"],
  ["board-collection-close-8-inch-slot", "Circuit Board collection", "Close open claw -> 8 inch board slot"],
  ["board-collection-open-claw-4-inch", "Circuit Board collection", "Open claw 4 inch board height"],
  ["board-collection-close-4-inch-slot", "Circuit Board collection", "Close open claw -> 4 inch board slot"],
  ["board-collection-open-claw-6-inch", "Circuit Board collection", "Open claw 6 inch board height"],
  ["board-collection-close-6-inch-slot", "Circuit Board collection", "Close open claw -> 6 inch board slot"],
  ["board-transfer-6-inch-height", "Circuit Board removal/insertion", "6 inch board height"],
  ["board-transfer-8-inch-height", "Circuit Board removal/insertion", "8 inch board height"],
  ["board-transfer-4-inch-height", "Circuit Board removal/insertion", "4 inch board height"],
  ["board-transfer-6-inch-slot-height", "Circuit Board removal/insertion", "6 inch slot -> 6 inch board height"],
  ["board-transfer-8-inch-slot-height", "Circuit Board removal/insertion", "8 inch slot -> 8 inch board height"],
  ["board-transfer-4-inch-slot-height", "Circuit Board removal/insertion", "4 inch slot -> 4 inch board height"],
] as const;

const CATEGORY_BY_LABEL = {
  General: "general",
  "Fuse collection": "fuseCollection",
  "Fuse removal/insertion": "fuseTransfer",
  "Circuit Board collection": "boardCollection",
  "Circuit Board removal/insertion": "boardTransfer",
} as const;

function tempStoreWithConfig(config: unknown): ConfigStore {
  const tempRoot = fs.mkdtempSync(path.join(os.tmpdir(), "robot-arm-beta-config-"));
  const configDir = path.join(tempRoot, ".lekiwi-ui");
  fs.mkdirSync(configDir, { recursive: true });
  fs.writeFileSync(path.join(configDir, "config.json"), JSON.stringify(config));
  return new ConfigStore(defaultConfig, tempRoot);
}

function version(
  moveId: string,
  id: string,
  versionNumber: number,
  trajectoryPath: string | null,
): MoveRecordingVersion {
  return {
    id,
    moveId,
    versionNumber,
    trajectoryPath,
    displayName: `v${versionNumber}`,
    recordedAtIso: "2026-04-29T12:00:00.000Z",
    recordingType: "keyboardControl",
    playbackSpeed: 1,
    vexBaseSamplesPresent: false,
    distanceSensorSamplesPresent: true,
    inertialSamplesPresent: true,
    autoVexPositioningEnabled: false,
    isFavorite: false,
    notes: "",
    safetyStatus: "unknown",
  };
}

test("starter move library contains every competition move with stable ids and categories", () => {
  const moves = createDefaultMoveDefinitions();
  assert.equal(moves.length, EXPECTED_STARTER_MOVES.length);

  for (const [id, categoryLabel, label] of EXPECTED_STARTER_MOVES) {
    const move = moves.find((item) => item.id === id);
    assert.ok(move, `missing starter move ${id}`);
    assert.equal(move.label, label);
    assert.equal(move.category, CATEGORY_BY_LABEL[categoryLabel]);
    assert.equal(move.defaultVexPositioningEnabled, false);
    assert.equal(move.defaultVexReplaySetting.includeBaseReplay, false);
    assert.equal(move.defaultSensorRecordingSettings.includeVexBaseSamples, false);
  }
});

test("recording version numbers increment per move", () => {
  const moveId = "fuse-transfer-low-height";
  const versions = [
    version(moveId, "v1", 1, "/tmp/v1.json"),
    version(moveId, "v2", 2, "/tmp/v2.json"),
    version("general-open-left-beam-door", "other", 9, "/tmp/other.json"),
  ];

  assert.equal(nextMoveVersionNumber(versions, moveId), 3);
  assert.equal(nextMoveVersionNumber(versions, "board-transfer-4-inch-height"), 1);
});

test("favorite selection allows one favorite version per move and updates completion state", () => {
  const moves = createDefaultMoveDefinitions();
  const moveId = "fuse-transfer-low-height";
  const versions = [
    version(moveId, "low-v1", 1, "/tmp/low-v1.json"),
    version(moveId, "low-v2", 2, "/tmp/low-v2.json"),
    version("fuse-transfer-high-height", "high-v1", 1, "/tmp/high-v1.json"),
  ];

  const first = withFavoriteVersion(moves, versions, moveId, "low-v1");
  assert.equal(first.moves.find((move) => move.id === moveId)?.favoriteVersionId, "low-v1");
  assert.equal(first.versions.filter((item) => item.moveId === moveId && item.isFavorite).length, 1);

  const second = withFavoriteVersion(first.moves, first.versions, moveId, "low-v2");
  assert.equal(second.moves.find((move) => move.id === moveId)?.favoriteVersionId, "low-v2");
  assert.equal(second.versions.find((item) => item.id === "low-v1")?.isFavorite, false);
  assert.equal(favoriteVersionForMove(second.moves.find((move) => move.id === moveId)!, second.versions)?.id, "low-v2");
});

test("broken favorite trajectory is detected", () => {
  const moves = createDefaultMoveDefinitions();
  const moveId = "fuse-transfer-low-height";
  const versions = [version(moveId, "low-v1", 1, "/tmp/missing-low-v1.json")];
  const favorite = withFavoriteVersion(moves, versions, moveId, "low-v1");
  const move = favorite.moves.find((item) => item.id === moveId)!;

  assert.equal(favoriteTrajectoryIsBroken(move, favorite.versions, () => false), true);
  assert.equal(favoriteTrajectoryIsBroken(move, favorite.versions, () => true), false);
});

test("legacy config still loads old recordings and pins while adding beta move defaults", () => {
  const store = tempStoreWithConfig({
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
    chainLinks: [],
    recordingReplayOptions: {
      "/home/pi/lekiwi-trajectories/legacy.json": {
        homeMode: "none",
        speed: 1,
      },
    },
    training: defaultConfig.training,
  });

  const loaded = store.getConfig();
  assert.equal(loaded.pinnedMoves[0]?.name, "Legacy move");
  assert.equal(loaded.recordingReplayOptions["/home/pi/lekiwi-trajectories/legacy.json"]?.autoVexPositioning, false);
  assert.equal(loaded.moveDefinitions.length, EXPECTED_STARTER_MOVES.length);
  assert.deepEqual(loaded.moveRecordingVersions, []);
});

test("game builder only exposes favorites and defaults VEX positioning off", () => {
  const moves = createDefaultMoveDefinitions();
  const moveId = "fuse-transfer-low-height";
  const versions = [
    version(moveId, "low-v1", 1, "/tmp/low-v1.json"),
    version("fuse-transfer-high-height", "high-v1", 1, "/tmp/high-v1.json"),
  ];
  const favorite = withFavoriteVersion(moves, versions, moveId, "low-v1");
  const eligible = gameBuilderEligibleFavorites(
    favorite.moves,
    favorite.versions,
    (trajectoryPath) => trajectoryPath === "/tmp/low-v1.json",
  );

  assert.deepEqual(eligible.map((item) => item.move.id), [moveId]);
  assert.equal(eligible[0]?.broken, false);

  const step = createGamePlanStepFromFavorite(eligible[0].move, eligible[0].favorite);
  assert.equal(step.autoVexPositioning, false);
  assert.equal(step.includeVexBaseReplay, false);
  assert.equal(step.returnToActiveHold, true);
  assert.equal(step.transitionPolicy, "returnToActiveHoldFirst");
});

test("game builder hotkey conflicts and protected drive keys are detected", () => {
  const moves = createDefaultMoveDefinitions();
  const move = moves.find((item) => item.id === "fuse-transfer-low-height")!;
  const favorite = version(move.id, "low-v1", 1, "/tmp/low-v1.json");
  const first = {
    ...createGamePlanStepFromFavorite(move, favorite),
    id: "step-1",
    hotkey: "A",
  };
  const duplicate = {
    ...createGamePlanStepFromFavorite(move, favorite),
    id: "step-2",
    hotkey: "a",
  };
  const protectedKey = {
    ...createGamePlanStepFromFavorite(move, favorite),
    id: "step-3",
    hotkey: "ArrowUp",
  };

  const errors = gameBuilderHotkeyErrors([first, duplicate, protectedKey]);
  assert.match(errors["step-1"], /Conflicts/);
  assert.match(errors["step-2"], /Conflicts/);
  assert.match(errors["step-3"], /Protected/);
});
