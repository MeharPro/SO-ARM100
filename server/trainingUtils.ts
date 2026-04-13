import fs from "node:fs";
import path from "node:path";

import type {
  PolicyBenchmarkResult,
  TrainingArtifact,
  TrainingConfig,
  TrainingProfile,
  TrainingSettings,
} from "./types.js";

export const DEFAULT_TRAINING_SETTINGS: TrainingSettings = {
  defaultPolicyType: "act",
  localDevice: "mps",
  benchmarkIterations: 20,
  deployBenchmarkMargin: 1.1,
};

export function createEmptyTrainingArtifact(): TrainingArtifact {
  return {
    datasetEpisodeCount: null,
    datasetCameraKeys: [],
    lastDatasetEpisodeIndex: null,
    lastDatasetSyncAt: null,
    lastTrainingStartedAt: null,
    lastTrainingCompletedAt: null,
    lastCheckpointPath: null,
    availableCheckpointPaths: [],
    deployedCheckpointPath: null,
    deployedAt: null,
    latestEvalDatasetPath: null,
    latestEvalAt: null,
    benchmark: null,
  };
}

export function createDefaultTrainingProfile(rootDir: string, name = "Wood Pick"): TrainingProfile {
  const id = slugify(name) || "training-task";
  return {
    id,
    name,
    task: "Pick up the wooden piece from the table, close the gripper around it, and lift it.",
    captureMode: "leader",
    numEpisodes: 50,
    episodeTimeS: 20,
    resetTimeS: 10,
    fps: 30,
    camerasMode: "default",
    policyType: "act",
    piDatasetPath: `/home/pi/lerobot-datasets/${id}`,
    macDatasetPath: path.join(rootDir, "output", "training-datasets", id),
    macTrainOutputDir: path.join(rootDir, "output", "training-runs", id),
    selectedCheckpointPath: "",
    piDeployPath: `/home/pi/lerobot-training/${id}/deployed-policy`,
    piEvalDatasetPath: `/home/pi/lerobot-eval/${id}`,
    artifacts: createEmptyTrainingArtifact(),
  };
}

export function createDefaultTrainingConfig(rootDir: string): TrainingConfig {
  const defaultProfile = createDefaultTrainingProfile(rootDir);
  return {
    settings: { ...DEFAULT_TRAINING_SETTINGS },
    profiles: [defaultProfile],
    selectedProfileId: defaultProfile.id,
  };
}

export function slugify(value: string): string {
  return value
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "")
    .slice(0, 64);
}

export function deriveDatasetRepoId(profile: TrainingProfile): string {
  return `local/${profile.id}`;
}

export function deriveEvalDatasetRepoId(profile: TrainingProfile): string {
  return `local/${profile.id}-eval`;
}

export function benchmarkPasses(
  benchmark: Pick<PolicyBenchmarkResult, "effectiveFps" | "targetFps">,
  margin: number,
): boolean {
  return benchmark.effectiveFps >= benchmark.targetFps * margin;
}

export function normalizeTrainingConfig(
  raw: Partial<TrainingConfig> | undefined,
  rootDir: string,
): TrainingConfig {
  const fallback = createDefaultTrainingConfig(rootDir);
  const profiles = Array.isArray(raw?.profiles) && raw?.profiles.length
    ? raw.profiles.map((profile) => normalizeTrainingProfile(profile, rootDir))
    : fallback.profiles;
  const selectedProfileId =
    typeof raw?.selectedProfileId === "string" &&
    profiles.some((profile) => profile.id === raw.selectedProfileId)
      ? raw.selectedProfileId
      : profiles[0]?.id ?? null;

  return {
    settings: {
      ...DEFAULT_TRAINING_SETTINGS,
      ...(raw?.settings ?? {}),
    },
    profiles,
    selectedProfileId,
  };
}

export function normalizeTrainingProfile(
  raw: Partial<TrainingProfile> | undefined,
  rootDir: string,
): TrainingProfile {
  const fallback = createDefaultTrainingProfile(rootDir, raw?.name ?? "Training Task");
  const normalizedName = typeof raw?.name === "string" && raw.name.trim() ? raw.name.trim() : fallback.name;
  const normalizedId = slugify(typeof raw?.id === "string" && raw.id.trim() ? raw.id : normalizedName) || fallback.id;
  const fallbackForName = createDefaultTrainingProfile(rootDir, normalizedName);

  return {
    ...fallbackForName,
    id: normalizedId,
    name: normalizedName,
    task: typeof raw?.task === "string" && raw.task.trim() ? raw.task.trim() : fallbackForName.task,
    captureMode:
      raw?.captureMode === "free-teach" || raw?.captureMode === "leader-as-follower"
        ? raw.captureMode
        : "leader",
    numEpisodes: normalizePositiveInteger(raw?.numEpisodes, fallbackForName.numEpisodes),
    episodeTimeS: normalizePositiveInteger(raw?.episodeTimeS, fallbackForName.episodeTimeS),
    resetTimeS: normalizeNonNegativeInteger(raw?.resetTimeS, fallbackForName.resetTimeS),
    fps: normalizePositiveInteger(raw?.fps, fallbackForName.fps),
    camerasMode:
      typeof raw?.camerasMode === "string" && raw.camerasMode.trim()
        ? raw.camerasMode.trim()
        : fallbackForName.camerasMode,
    policyType: "act",
    piDatasetPath: normalizePathString(raw?.piDatasetPath, fallbackForName.piDatasetPath),
    macDatasetPath: normalizePathString(raw?.macDatasetPath, fallbackForName.macDatasetPath),
    macTrainOutputDir: normalizePathString(raw?.macTrainOutputDir, fallbackForName.macTrainOutputDir),
    selectedCheckpointPath:
      typeof raw?.selectedCheckpointPath === "string" ? raw.selectedCheckpointPath.trim() : "",
    piDeployPath: normalizePathString(raw?.piDeployPath, fallbackForName.piDeployPath),
    piEvalDatasetPath: normalizePathString(raw?.piEvalDatasetPath, fallbackForName.piEvalDatasetPath),
    artifacts: normalizeTrainingArtifact(raw?.artifacts),
  };
}

export function validateTrainingProfile(profile: TrainingProfile): void {
  if (!profile.name.trim()) {
    throw new Error("Training profile name cannot be empty.");
  }
  if (!profile.task.trim()) {
    throw new Error("Training task cannot be empty.");
  }
  if (profile.numEpisodes < 1) {
    throw new Error("Training episode count must be at least 1.");
  }
  if (profile.episodeTimeS < 1) {
    throw new Error("Episode time must be at least 1 second.");
  }
  if (profile.resetTimeS < 0) {
    throw new Error("Reset time cannot be negative.");
  }
  if (profile.fps < 1) {
    throw new Error("Capture FPS must be at least 1.");
  }
  if (!profile.piDatasetPath.trim()) {
    throw new Error("Pi dataset path cannot be empty.");
  }
  if (!profile.macDatasetPath.trim()) {
    throw new Error("Mac dataset path cannot be empty.");
  }
  if (!profile.macTrainOutputDir.trim()) {
    throw new Error("Mac training output directory cannot be empty.");
  }
  if (!profile.piDeployPath.trim()) {
    throw new Error("Pi deploy path cannot be empty.");
  }
  if (!profile.piEvalDatasetPath.trim()) {
    throw new Error("Pi eval dataset path cannot be empty.");
  }
  if (profile.camerasMode.trim() !== "default") {
    throw new Error("Training profiles currently require cameras mode to be 'default'.");
  }
}

export function listCheckpointCandidates(profile: TrainingProfile): string[] {
  const outputDir = profile.macTrainOutputDir.trim();
  if (!outputDir || !fs.existsSync(outputDir)) {
    return [];
  }

  const checkpointsDir = path.join(outputDir, "checkpoints");
  if (!fs.existsSync(checkpointsDir)) {
    return [];
  }

  const entries = fs.readdirSync(checkpointsDir, { withFileTypes: true });
  return entries
    .filter((entry) => entry.isDirectory())
    .map((entry) => path.join(checkpointsDir, entry.name, "pretrained_model"))
    .filter((candidate) => fs.existsSync(path.join(candidate, "config.json")))
    .sort();
}

function normalizeTrainingArtifact(raw: Partial<TrainingArtifact> | undefined): TrainingArtifact {
  return {
    ...createEmptyTrainingArtifact(),
    ...(raw ?? {}),
    datasetCameraKeys: Array.isArray(raw?.datasetCameraKeys) ? raw!.datasetCameraKeys.filter(Boolean) : [],
    availableCheckpointPaths: Array.isArray(raw?.availableCheckpointPaths)
      ? raw!.availableCheckpointPaths.filter(Boolean)
      : [],
  };
}

function normalizePositiveInteger(value: unknown, fallback: number): number {
  const numeric = Math.round(Number(value));
  return Number.isFinite(numeric) && numeric > 0 ? numeric : fallback;
}

function normalizeNonNegativeInteger(value: unknown, fallback: number): number {
  const numeric = Math.round(Number(value));
  return Number.isFinite(numeric) && numeric >= 0 ? numeric : fallback;
}

function normalizePathString(value: unknown, fallback: string): string {
  return typeof value === "string" && value.trim() ? value.trim() : fallback;
}
