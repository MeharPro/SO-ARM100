import crypto from "node:crypto";

import type {
  BetaRecordingType,
  GamePlan,
  GamePlanStep,
  GamePlanStepTransitionPolicy,
  MoveCategory,
  MoveDefinition,
  MoveIconKind,
  MoveRecordingSafetyStatus,
  MoveRecordingVersion,
  VexReplayMode,
} from "./types.js";

export const MOVE_CATEGORY_LABELS: Record<MoveCategory, string> = {
  general: "General",
  fuseCollection: "Fuse collection",
  fuseTransfer: "Fuse removal/insertion",
  boardCollection: "Circuit Board collection",
  boardTransfer: "Circuit Board removal/insertion",
};

const CATEGORY_SORT_ORDER: Record<MoveCategory, number> = {
  general: 0,
  fuseCollection: 1,
  fuseTransfer: 2,
  boardCollection: 3,
  boardTransfer: 4,
};

const DEFAULT_RECORDING_TYPES: BetaRecordingType[] = [
  "keyboardControl",
  "leaderArm",
  "followerHandGuide",
  "keyboardFromActiveHold",
];

interface StarterMoveSeed {
  id: string;
  label: string;
  category: MoveCategory;
  iconKind: MoveIconKind;
  colorToken: string;
  description: string;
}

const STARTER_MOVE_SEEDS: StarterMoveSeed[] = [
  {
    id: "general-open-left-beam-door",
    label: "Open left beam door",
    category: "general",
    iconKind: "doorLeft",
    colorToken: "general-door-left",
    description: "Open the left beam door mechanism.",
  },
  {
    id: "general-open-right-beam-door",
    label: "Open right beam door",
    category: "general",
    iconKind: "doorRight",
    colorToken: "general-door-right",
    description: "Open the right beam door mechanism.",
  },
  {
    id: "fuse-collection-open-claw-blue",
    label: "Open claw position for driver to orient bot - blue",
    category: "fuseCollection",
    iconKind: "circleBlue",
    colorToken: "fuse-blue",
    description: "Put the claw in the blue fuse approach pose so the driver can line up the base.",
  },
  {
    id: "fuse-collection-close-blue-slot",
    label: "Close open claw -> blue slot",
    category: "fuseCollection",
    iconKind: "circleBlue",
    colorToken: "fuse-blue",
    description: "Close from the blue approach pose and place into the blue dowel/fuse storage slot.",
  },
  {
    id: "fuse-collection-open-claw-yellow",
    label: "Open claw position for driver to orient bot - yellow",
    category: "fuseCollection",
    iconKind: "circleYellow",
    colorToken: "fuse-yellow",
    description: "Put the claw in the yellow fuse approach pose so the driver can line up the base.",
  },
  {
    id: "fuse-collection-close-yellow-slot",
    label: "Close open claw -> yellow slot",
    category: "fuseCollection",
    iconKind: "circleYellow",
    colorToken: "fuse-yellow",
    description: "Close from the yellow approach pose and place into the yellow dowel/fuse storage slot.",
  },
  {
    id: "fuse-collection-open-claw-green",
    label: "Open claw position for driver to orient bot - green",
    category: "fuseCollection",
    iconKind: "circleGreen",
    colorToken: "fuse-green",
    description: "Put the claw in the green fuse approach pose so the driver can line up the base.",
  },
  {
    id: "fuse-collection-close-green-slot",
    label: "Close open claw -> green slot",
    category: "fuseCollection",
    iconKind: "circleGreen",
    colorToken: "fuse-green",
    description: "Close from the green approach pose and place into the green dowel/fuse storage slot.",
  },
  {
    id: "fuse-transfer-low-height",
    label: "Low fuse height",
    category: "fuseTransfer",
    iconKind: "heightLow",
    colorToken: "height-low",
    description: "Move the arm to the low fuse insertion/removal height.",
  },
  {
    id: "fuse-transfer-high-height",
    label: "High fuse height",
    category: "fuseTransfer",
    iconKind: "heightHigh",
    colorToken: "height-high",
    description: "Move the arm to the high fuse insertion/removal height.",
  },
  {
    id: "fuse-transfer-low-height-repeat",
    label: "Low fuse height repeat",
    category: "fuseTransfer",
    iconKind: "heightLow",
    colorToken: "height-low",
    description: "Second low-height utility move if the gameplay flow needs a distinct low-height version.",
  },
  {
    id: "fuse-transfer-green-slot-low-height",
    label: "Green slot -> low fuse height",
    category: "fuseTransfer",
    iconKind: "circleGreen",
    colorToken: "fuse-green",
    description: "Move a green stored fuse from its slot to the low fuse height.",
  },
  {
    id: "fuse-transfer-yellow-slot-high-height",
    label: "Yellow slot -> high fuse height",
    category: "fuseTransfer",
    iconKind: "circleYellow",
    colorToken: "fuse-yellow",
    description: "Move a yellow stored fuse from its slot to the high fuse height.",
  },
  {
    id: "fuse-transfer-blue-slot-low-height",
    label: "Blue slot -> low fuse height",
    category: "fuseTransfer",
    iconKind: "circleBlue",
    colorToken: "fuse-blue",
    description: "Move a blue stored fuse from its slot to the low fuse height.",
  },
  {
    id: "board-collection-open-claw-8-inch",
    label: "Open claw 8 inch board height",
    category: "boardCollection",
    iconKind: "board8",
    colorToken: "board-8",
    description: "Put the claw at the 8 inch board pickup height.",
  },
  {
    id: "board-collection-close-8-inch-slot",
    label: "Close open claw -> 8 inch board slot",
    category: "boardCollection",
    iconKind: "board8",
    colorToken: "board-8",
    description: "Close from the 8 inch board approach pose and place in the 8 inch board slot.",
  },
  {
    id: "board-collection-open-claw-4-inch",
    label: "Open claw 4 inch board height",
    category: "boardCollection",
    iconKind: "board4",
    colorToken: "board-4",
    description: "Put the claw at the 4 inch board pickup height.",
  },
  {
    id: "board-collection-close-4-inch-slot",
    label: "Close open claw -> 4 inch board slot",
    category: "boardCollection",
    iconKind: "board4",
    colorToken: "board-4",
    description: "Close from the 4 inch board approach pose and place in the 4 inch board slot.",
  },
  {
    id: "board-collection-open-claw-6-inch",
    label: "Open claw 6 inch board height",
    category: "boardCollection",
    iconKind: "board6",
    colorToken: "board-6",
    description: "Put the claw at the 6 inch board pickup height.",
  },
  {
    id: "board-collection-close-6-inch-slot",
    label: "Close open claw -> 6 inch board slot",
    category: "boardCollection",
    iconKind: "board6",
    colorToken: "board-6",
    description: "Close from the 6 inch board approach pose and place in the 6 inch board slot.",
  },
  {
    id: "board-transfer-6-inch-height",
    label: "6 inch board height",
    category: "boardTransfer",
    iconKind: "board6",
    colorToken: "board-6",
    description: "Move the arm to the 6 inch board insertion/removal height.",
  },
  {
    id: "board-transfer-8-inch-height",
    label: "8 inch board height",
    category: "boardTransfer",
    iconKind: "board8",
    colorToken: "board-8",
    description: "Move the arm to the 8 inch board insertion/removal height.",
  },
  {
    id: "board-transfer-4-inch-height",
    label: "4 inch board height",
    category: "boardTransfer",
    iconKind: "board4",
    colorToken: "board-4",
    description: "Move the arm to the 4 inch board insertion/removal height.",
  },
  {
    id: "board-transfer-6-inch-slot-height",
    label: "6 inch slot -> 6 inch board height",
    category: "boardTransfer",
    iconKind: "board6",
    colorToken: "board-6",
    description: "Move the 6 inch stored board from its slot to the 6 inch target height.",
  },
  {
    id: "board-transfer-8-inch-slot-height",
    label: "8 inch slot -> 8 inch board height",
    category: "boardTransfer",
    iconKind: "board8",
    colorToken: "board-8",
    description: "Move the 8 inch stored board from its slot to the 8 inch target height.",
  },
  {
    id: "board-transfer-4-inch-slot-height",
    label: "4 inch slot -> 4 inch board height",
    category: "boardTransfer",
    iconKind: "board4",
    colorToken: "board-4",
    description: "Move the 4 inch stored board from its slot to the 4 inch target height.",
  },
];

function defaultMoveDefinition(seed: StarterMoveSeed, sortOrder: number): MoveDefinition {
  return {
    ...seed,
    sortOrder,
    allowedRecordingTypes: [...DEFAULT_RECORDING_TYPES],
    defaultRecordingType: "keyboardControl",
    defaultVexReplaySetting: {
      includeBaseReplay: false,
      replayMode: "ecu",
    },
    defaultVexPositioningEnabled: false,
    defaultSensorRecordingSettings: {
      distanceSensors: true,
      inertialSensor: true,
      includeVexBaseSamples: false,
    },
    favoriteVersionId: null,
    archived: false,
  };
}

export function createDefaultMoveDefinitions(): MoveDefinition[] {
  const perCategoryIndex = new Map<MoveCategory, number>();
  return STARTER_MOVE_SEEDS.map((seed) => {
    const current = perCategoryIndex.get(seed.category) ?? 0;
    perCategoryIndex.set(seed.category, current + 1);
    return defaultMoveDefinition(seed, CATEGORY_SORT_ORDER[seed.category] * 100 + current * 10);
  });
}

function normalizeMoveCategory(value: unknown, fallback: MoveCategory): MoveCategory {
  return value === "general" ||
    value === "fuseCollection" ||
    value === "fuseTransfer" ||
    value === "boardCollection" ||
    value === "boardTransfer"
    ? value
    : fallback;
}

function normalizeMoveIconKind(value: unknown, fallback: MoveIconKind): MoveIconKind {
  return value === "doorLeft" ||
    value === "doorRight" ||
    value === "circleBlue" ||
    value === "circleYellow" ||
    value === "circleGreen" ||
    value === "heightLow" ||
    value === "heightHigh" ||
    value === "board4" ||
    value === "board6" ||
    value === "board8"
    ? value
    : fallback;
}

function normalizeRecordingType(value: unknown, fallback: BetaRecordingType): BetaRecordingType {
  return value === "keyboardControl" ||
    value === "leaderArm" ||
    value === "followerHandGuide" ||
    value === "keyboardFromActiveHold" ||
    value === "leaderFromSyncedHold"
    ? value
    : fallback;
}

function normalizeReplayMode(value: unknown, fallback: VexReplayMode): VexReplayMode {
  return value === "drive" ? "drive" : fallback;
}

function normalizePositiveNumber(value: unknown, fallback: number): number {
  const numeric = Number(value);
  return Number.isFinite(numeric) && numeric > 0 ? numeric : fallback;
}

function normalizeIsoTimestamp(value: unknown): string {
  if (typeof value === "string") {
    const parsed = Date.parse(value);
    if (Number.isFinite(parsed)) {
      return new Date(parsed).toISOString();
    }
  }
  return new Date(0).toISOString();
}

function normalizeSafetyStatus(value: unknown): MoveRecordingSafetyStatus {
  return value === "clear" || value === "latched" ? value : "unknown";
}

function normalizeTransitionPolicy(value: unknown): GamePlanStepTransitionPolicy {
  return value === "requireStartPoseTolerance" ||
    value === "runNamedTransitionMoveFirst" ||
    value === "askForManualConfirmation"
    ? value
    : "returnToActiveHoldFirst";
}

export function sortMoveDefinitions(moves: MoveDefinition[]): MoveDefinition[] {
  return [...moves].sort(
    (left, right) =>
      CATEGORY_SORT_ORDER[left.category] - CATEGORY_SORT_ORDER[right.category] ||
      left.sortOrder - right.sortOrder ||
      left.label.localeCompare(right.label),
  );
}

export function normalizeMoveDefinitions(raw: unknown): MoveDefinition[] {
  const defaults = createDefaultMoveDefinitions();
  const defaultsById = new Map(defaults.map((move) => [move.id, move]));
  const rawItems = Array.isArray(raw) ? raw : [];
  const rawById = new Map(
    rawItems
      .filter((item): item is Partial<MoveDefinition> => Boolean(item && typeof item === "object"))
      .map((item) => [typeof item.id === "string" && item.id.trim() ? item.id.trim() : crypto.randomUUID(), item]),
  );

  const mergedDefaults = defaults.map((base) => normalizeMoveDefinition(rawById.get(base.id), base));
  const customMoves = [...rawById.entries()]
    .filter(([id]) => !defaultsById.has(id))
    .map(([id, item], index) =>
      normalizeMoveDefinition(
        {
          ...item,
          id,
        },
        {
          ...defaultMoveDefinition(
            {
              id,
              label: typeof item.label === "string" && item.label.trim() ? item.label.trim() : "Custom move",
              category: "general",
              iconKind: "doorLeft",
              colorToken: "custom",
              description: "",
            },
            9000 + index * 10,
          ),
        },
      ),
    );

  return sortMoveDefinitions([...mergedDefaults, ...customMoves]);
}

export function normalizeMoveDefinition(
  raw: Partial<MoveDefinition> | undefined,
  fallback: MoveDefinition,
): MoveDefinition {
  const allowedRecordingTypes = Array.isArray(raw?.allowedRecordingTypes)
    ? raw.allowedRecordingTypes
        .map((type) => normalizeRecordingType(type, fallback.defaultRecordingType))
        .filter((type, index, values) => values.indexOf(type) === index)
    : fallback.allowedRecordingTypes;
  const defaultRecordingType = normalizeRecordingType(
    raw?.defaultRecordingType,
    fallback.defaultRecordingType,
  );
  const safeAllowedTypes = allowedRecordingTypes.length ? allowedRecordingTypes : [...fallback.allowedRecordingTypes];
  return {
    id: typeof raw?.id === "string" && raw.id.trim() ? raw.id.trim() : fallback.id,
    label: typeof raw?.label === "string" && raw.label.trim() ? raw.label.trim() : fallback.label,
    category: normalizeMoveCategory(raw?.category, fallback.category),
    iconKind: normalizeMoveIconKind(raw?.iconKind, fallback.iconKind),
    colorToken:
      typeof raw?.colorToken === "string" && raw.colorToken.trim()
        ? raw.colorToken.trim()
        : fallback.colorToken,
    sortOrder: Number.isFinite(Number(raw?.sortOrder)) ? Number(raw?.sortOrder) : fallback.sortOrder,
    description:
      typeof raw?.description === "string" ? raw.description : fallback.description,
    allowedRecordingTypes: safeAllowedTypes,
    defaultRecordingType: safeAllowedTypes.includes(defaultRecordingType)
      ? defaultRecordingType
      : safeAllowedTypes[0],
    defaultVexReplaySetting: {
      includeBaseReplay: raw?.defaultVexReplaySetting?.includeBaseReplay === true,
      replayMode: normalizeReplayMode(
        raw?.defaultVexReplaySetting?.replayMode,
        fallback.defaultVexReplaySetting.replayMode,
      ),
    },
    defaultVexPositioningEnabled: raw?.defaultVexPositioningEnabled === true,
    defaultSensorRecordingSettings: {
      distanceSensors:
        raw?.defaultSensorRecordingSettings?.distanceSensors ??
        fallback.defaultSensorRecordingSettings.distanceSensors,
      inertialSensor:
        raw?.defaultSensorRecordingSettings?.inertialSensor ??
        fallback.defaultSensorRecordingSettings.inertialSensor,
      includeVexBaseSamples: raw?.defaultSensorRecordingSettings?.includeVexBaseSamples === true,
    },
    favoriteVersionId:
      typeof raw?.favoriteVersionId === "string" && raw.favoriteVersionId.trim()
        ? raw.favoriteVersionId.trim()
        : null,
    archived: raw?.archived === true,
  };
}

export function nextMoveVersionNumber(
  versions: MoveRecordingVersion[],
  moveId: string,
): number {
  return (
    Math.max(
      0,
      ...versions
        .filter((version) => version.moveId === moveId)
        .map((version) => version.versionNumber),
    ) + 1
  );
}

export function normalizeMoveRecordingVersions(
  raw: unknown,
  moves: MoveDefinition[],
): MoveRecordingVersion[] {
  const moveIds = new Set(moves.map((move) => move.id));
  const versions = Array.isArray(raw)
    ? raw
        .map((item) => normalizeMoveRecordingVersion(item, moveIds))
        .filter((item): item is MoveRecordingVersion => item !== null)
    : [];
  return versions.sort(
    (left, right) =>
      left.moveId.localeCompare(right.moveId) ||
      left.versionNumber - right.versionNumber ||
      left.recordedAtIso.localeCompare(right.recordedAtIso),
  );
}

export function normalizeMoveRecordingVersion(
  raw: unknown,
  moveIds: Set<string>,
): MoveRecordingVersion | null {
  if (!raw || typeof raw !== "object") {
    return null;
  }
  const item = raw as Partial<MoveRecordingVersion>;
  const moveId = typeof item.moveId === "string" ? item.moveId.trim() : "";
  if (!moveIds.has(moveId)) {
    return null;
  }
  return {
    id: typeof item.id === "string" && item.id.trim() ? item.id.trim() : crypto.randomUUID(),
    moveId,
    versionNumber: Math.max(1, Math.floor(Number(item.versionNumber) || 1)),
    trajectoryPath:
      typeof item.trajectoryPath === "string" && item.trajectoryPath.trim()
        ? item.trajectoryPath.trim()
        : null,
    displayName:
      typeof item.displayName === "string" && item.displayName.trim()
        ? item.displayName.trim()
        : `v${Math.max(1, Math.floor(Number(item.versionNumber) || 1))}`,
    recordedAtIso: normalizeIsoTimestamp(item.recordedAtIso),
    recordingType: normalizeRecordingType(item.recordingType, "keyboardControl"),
    playbackSpeed: normalizePositiveNumber(item.playbackSpeed, 1),
    vexBaseSamplesPresent: item.vexBaseSamplesPresent === true,
    distanceSensorSamplesPresent: item.distanceSensorSamplesPresent === true,
    inertialSamplesPresent: item.inertialSamplesPresent === true,
    autoVexPositioningEnabled: item.autoVexPositioningEnabled === true,
    isFavorite: item.isFavorite === true,
    notes: typeof item.notes === "string" ? item.notes : "",
    safetyStatus: normalizeSafetyStatus(item.safetyStatus),
  };
}

export function withFavoriteVersion(
  moves: MoveDefinition[],
  versions: MoveRecordingVersion[],
  moveId: string,
  versionId: string,
): { moves: MoveDefinition[]; versions: MoveRecordingVersion[] } {
  const selected = versions.find((version) => version.id === versionId && version.moveId === moveId);
  if (!selected) {
    throw new Error("Favorite version must belong to the selected move.");
  }
  return {
    moves: moves.map((move) =>
      move.id === moveId ? { ...move, favoriteVersionId: versionId } : move,
    ),
    versions: versions.map((version) => ({
      ...version,
      isFavorite: version.moveId === moveId && version.id === versionId,
    })),
  };
}

export function favoriteVersionForMove(
  move: MoveDefinition,
  versions: MoveRecordingVersion[],
): MoveRecordingVersion | null {
  if (!move.favoriteVersionId) {
    return null;
  }
  return versions.find((version) => version.id === move.favoriteVersionId && version.moveId === move.id) ?? null;
}

export function favoriteTrajectoryIsBroken(
  move: MoveDefinition,
  versions: MoveRecordingVersion[],
  trajectoryExists: (trajectoryPath: string) => boolean,
): boolean {
  const favorite = favoriteVersionForMove(move, versions);
  return Boolean(!favorite?.trajectoryPath || !trajectoryExists(favorite.trajectoryPath));
}

export function normalizeGamePlans(
  raw: unknown,
  moves: MoveDefinition[],
  versions: MoveRecordingVersion[],
): GamePlan[] {
  const moveIds = new Set(moves.map((move) => move.id));
  const versionIds = new Set(versions.map((version) => version.id));
  return Array.isArray(raw)
    ? raw
        .map((item) => normalizeGamePlan(item, moveIds, versionIds))
        .filter((item): item is GamePlan => item !== null)
    : [];
}

function normalizeGamePlan(
  raw: unknown,
  moveIds: Set<string>,
  versionIds: Set<string>,
): GamePlan | null {
  if (!raw || typeof raw !== "object") {
    return null;
  }
  const item = raw as Partial<GamePlan>;
  const steps = Array.isArray(item.steps)
    ? item.steps
        .map((step) => normalizeGamePlanStep(step, moveIds, versionIds))
        .filter((step): step is GamePlanStep => step !== null)
    : [];
  return {
    id: typeof item.id === "string" && item.id.trim() ? item.id.trim() : crypto.randomUUID(),
    name: typeof item.name === "string" && item.name.trim() ? item.name.trim() : "Game plan",
    steps,
    createdAtIso: normalizeIsoTimestamp(item.createdAtIso),
    updatedAtIso: normalizeIsoTimestamp(item.updatedAtIso),
  };
}

function normalizeGamePlanStep(
  raw: unknown,
  moveIds: Set<string>,
  versionIds: Set<string>,
): GamePlanStep | null {
  if (!raw || typeof raw !== "object") {
    return null;
  }
  const item = raw as Partial<GamePlanStep>;
  const moveId = typeof item.moveId === "string" ? item.moveId.trim() : "";
  const favoriteVersionId =
    typeof item.favoriteVersionId === "string" ? item.favoriteVersionId.trim() : "";
  if (!moveIds.has(moveId) || !versionIds.has(favoriteVersionId)) {
    return null;
  }
  return {
    id: typeof item.id === "string" && item.id.trim() ? item.id.trim() : crypto.randomUUID(),
    moveId,
    favoriteVersionId,
    labelOverride:
      typeof item.labelOverride === "string" && item.labelOverride.trim()
        ? item.labelOverride.trim()
        : null,
    hotkey: typeof item.hotkey === "string" && item.hotkey.trim() ? item.hotkey.trim() : null,
    playbackSpeedOverride:
      Number.isFinite(Number(item.playbackSpeedOverride)) && Number(item.playbackSpeedOverride) > 0
        ? Number(item.playbackSpeedOverride)
        : null,
    includeVexBaseReplay: item.includeVexBaseReplay === true,
    autoVexPositioning: item.autoVexPositioning === true,
    returnToActiveHold: item.returnToActiveHold === false ? false : true,
    requireConfirmationAfter: item.requireConfirmationAfter === true,
    pauseAfter: item.pauseAfter === true,
    transitionPolicy: normalizeTransitionPolicy(item.transitionPolicy),
    transitionMoveId:
      typeof item.transitionMoveId === "string" && moveIds.has(item.transitionMoveId)
        ? item.transitionMoveId
        : null,
  };
}
