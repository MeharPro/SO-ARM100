import { shellQuote } from "./system.js";
import {
  deriveDatasetRepoId,
  deriveEvalDatasetRepoId,
} from "./trainingUtils.js";
import type { AppSettings, PolicyBenchmarkResult, TrainingProfile } from "./types.js";

export function buildMacTrainingCommand(settings: AppSettings, profile: TrainingProfile): string {
  const jobName = `act_${profile.id}`;
  return [
    `source ${shellQuote(settings.mac.condaScript)}`,
    "conda activate lerobot",
    `cd ${shellQuote(settings.mac.projectDir)}`,
    `mkdir -p ${shellQuote(profile.macTrainOutputDir)}`,
    "lerobot-train \\",
    `  --dataset.repo_id=${shellQuote(deriveDatasetRepoId(profile))} \\`,
    `  --dataset.root=${shellQuote(profile.macDatasetPath)} \\`,
    "  --policy.type=act \\",
    `  --policy.device=${shellQuote("mps")} \\`,
    `  --output_dir=${shellQuote(profile.macTrainOutputDir)} \\`,
    `  --job_name=${shellQuote(jobName)} \\`,
    "  --policy.push_to_hub=false",
  ].join("\n");
}

export function buildPiDatasetCaptureCommand(
  settings: AppSettings,
  profile: TrainingProfile,
  torquePath: string,
): string {
  const saferServoModeFlag = settings.host.saferServoMode ? " \\\n  --safer-servo-mode" : "";
  return [
    `source ${shellQuote(settings.pi.condaScript)}`,
    "conda activate lerobot",
    `cd ${shellQuote(settings.pi.projectDir)}`,
    `python ${shellQuote(`${settings.pi.remoteHelperDir}/scripts/lekiwi_record_dataset_host.py`)} \\`,
    `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
    `  --robot-port ${shellQuote(settings.pi.robotPort)} \\`,
    `  --dataset-repo-id ${shellQuote(deriveDatasetRepoId(profile))} \\`,
    `  --dataset-root ${shellQuote(profile.piDatasetPath)} \\`,
    `  --task ${shellQuote(profile.task)} \\`,
    `  --capture-mode ${shellQuote(profile.captureMode)} \\`,
    `  --fps ${profile.fps} \\`,
    `  --num-episodes ${profile.numEpisodes} \\`,
    `  --episode-time-s ${profile.episodeTimeS} \\`,
    `  --reset-time-s ${profile.resetTimeS} \\`,
    `  --robot-cameras-json ${shellQuote("default")} \\`,
    "  --dataset-vcodec h264 \\",
    "  --dataset-streaming-encoding false \\",
    "  --use-degrees true \\",
    `  --torque-limits-json ${shellQuote(JSON.stringify(settings.host.armTorqueLimits))} \\`,
    `  --torque-limits-path ${shellQuote(torquePath)} \\`,
    `  --base-max-raw-velocity ${settings.host.baseMaxRawVelocity} \\`,
    `  --base-wheel-torque-limit ${settings.host.baseWheelTorqueLimit} \\`,
    `  --enable-base false${saferServoModeFlag}`,
  ].join("\n");
}

export function buildPiPolicyEvalCommand(
  settings: AppSettings,
  profile: TrainingProfile,
  benchmark: PolicyBenchmarkResult | null,
  evalDatasetPath: string,
): string {
  const deployPath = profile.piDeployPath;
  return [
    `source ${shellQuote(settings.pi.condaScript)}`,
    "conda activate lerobot",
    `cd ${shellQuote(settings.pi.projectDir)}`,
    `python ${shellQuote(`${settings.pi.remoteHelperDir}/scripts/lekiwi_run_policy_eval.py`)} \\`,
    `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
    `  --robot-port ${shellQuote(settings.pi.robotPort)} \\`,
    `  --policy-path ${shellQuote(deployPath)} \\`,
    `  --dataset-repo-id ${shellQuote(deriveEvalDatasetRepoId(profile))} \\`,
    `  --dataset-root ${shellQuote(evalDatasetPath)} \\`,
    `  --task ${shellQuote(profile.task)} \\`,
    `  --fps ${profile.fps} \\`,
    `  --num-episodes ${profile.numEpisodes} \\`,
    `  --episode-time-s ${profile.episodeTimeS} \\`,
    `  --reset-time-s ${profile.resetTimeS} \\`,
    `  --robot-cameras-json ${shellQuote("default")} \\`,
    benchmark ? `  --target-effective-fps ${benchmark.effectiveFps.toFixed(3)} \\` : "  --target-effective-fps 0 \\",
    "  --dataset-vcodec h264 \\",
    "  --dataset-streaming-encoding false \\",
    "  --use-degrees true \\",
    "  --display-data false \\",
    "  --enable-base false",
  ].join("\n");
}

export function buildPiPolicyBenchmarkCommand(
  settings: AppSettings,
  profile: TrainingProfile,
  benchmarkIterations: number,
): string {
  return [
    `source ${shellQuote(settings.pi.condaScript)}`,
    "conda activate lerobot",
    `cd ${shellQuote(settings.pi.projectDir)}`,
    `python ${shellQuote(`${settings.pi.remoteHelperDir}/scripts/lekiwi_benchmark_policy.py`)} \\`,
    `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
    `  --robot-port ${shellQuote(settings.pi.robotPort)} \\`,
    `  --policy-path ${shellQuote(profile.piDeployPath)} \\`,
    `  --fps ${profile.fps} \\`,
    `  --iterations ${benchmarkIterations} \\`,
    `  --robot-cameras-json ${shellQuote("default")} \\`,
    "  --use-degrees true \\",
    "  --enable-base false",
  ].join("\n");
}
