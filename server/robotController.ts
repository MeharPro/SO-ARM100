import fs from "node:fs";
import path from "node:path";
import { spawn } from "node:child_process";

import { Client, type ConnectConfig, type SFTPWrapper } from "ssh2";

import { ConfigStore } from "./configStore.js";
import { defaultConfig } from "./defaultConfig.js";
import {
  LocalProcessRunner,
  RemoteProcessRunner,
  TaskRunner,
} from "./processRunners.js";
import {
  getWifiStatus,
  isTcpReachable,
  runZshScript,
  shellQuote,
  sleep,
} from "./system.js";
import {
  buildMacTrainingCommand,
  buildPiDatasetCaptureCommand,
  buildPiPolicyBenchmarkCommand,
  buildPiPolicyEvalCommand,
} from "./trainingCommands.js";
import {
  benchmarkPasses,
  createDefaultTrainingProfile,
  deriveDatasetRepoId,
  listCheckpointCandidates,
  normalizeTrainingConfig,
  normalizeTrainingProfile,
  validateTrainingProfile,
} from "./trainingUtils.js";
import type {
  AppSettings,
  BenchmarkPolicyRequest,
  CreatePinnedMoveRequest,
  DashboardState,
  DeployTrainingCheckpointRequest,
  DeleteTrainingProfileRequest,
  LeaderStatus,
  PinnedMove,
  RecordingDetail,
  RecordingDetailRequest,
  RenameRecordingRequest,
  RecordingEntry,
  ReplayRequest,
  SelectTrainingProfileRequest,
  ServiceSnapshot,
  ServoCalibrationRequest,
  StartPolicyEvalRequest,
  StartTrainingCaptureRequest,
  StartTrainingRunRequest,
  StartTrainingSyncRequest,
  TrainingArtifact,
  TrainingConfig,
  TrainingProfile,
  TrimRecordingRequest,
  TorqueLimitsRequest,
  VexBrainStatus,
} from "./types.js";

const ROOT_DIR = process.cwd();
const HOST_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_host.py");
const POWER_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_power.py");
const RUNTIME_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_runtime.py");
const VEX_BASE_BRIDGE_SCRIPT = path.join(ROOT_DIR, "scripts", "vex_base_bridge.py");
const PI_CALIBRATION_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_calibrate.py");
const RECORD_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_record_trajectory.py");
const REPLAY_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_replay_trajectory.py");
const UI_TELEOP_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_ui_teleop.py");
const DATASET_CAPTURE_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_record_dataset_host.py");
const LOCAL_LEADER_CAPTURE_SCRIPT = path.join(
  ROOT_DIR,
  "scripts",
  "lekiwi_local_leader_dataset_capture.py",
);
const LOCAL_LEADER_REPLAY_SCRIPT = path.join(
  ROOT_DIR,
  "scripts",
  "lekiwi_local_leader_replay.py",
);
const POLICY_BENCHMARK_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_benchmark_policy.py");
const POLICY_EVAL_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_run_policy_eval.py");
const LOCAL_LEADER_REPLAY_DIR = path.join(ROOT_DIR, ".tmp", "leader-replay");
const LOCAL_POWER_LOG_DIR = path.join(ROOT_DIR, ".lekiwi-ui", "power-logs");
const LOCAL_LEADER_ROBOT_ID = "leader";
const MAX_ACTIVITY_LINES = 220;
const HOST_READY_TIMEOUT_MS = 15000;
const HOST_READY_PORTS = [5555, 5556];
const TRAINING_METADATA_REFRESH_MS = 15000;
const POWER_LOG_SYNC_INTERVAL_MS = 15000;
const VEX_STATUS_REFRESH_MS = 5000;
const ARM_MOTORS = [
  "arm_shoulder_pan",
  "arm_shoulder_lift",
  "arm_elbow_flex",
  "arm_wrist_flex",
  "arm_wrist_roll",
  "arm_gripper",
] as const;
const TORQUE_LIMIT_MIN = 0;
const TORQUE_LIMIT_MAX = 1000;
const LEKIWI_MOTOR_NAMES_BY_ID: Record<string, string> = {
  "1": "arm_shoulder_pan",
  "2": "arm_shoulder_lift",
  "3": "arm_elbow_flex",
  "4": "arm_wrist_flex",
  "5": "arm_wrist_roll",
  "6": "arm_gripper",
  "7": "base_left_wheel",
  "8": "base_back_wheel",
  "9": "base_right_wheel",
};
const LEADER_PORT_PREFIXES = [
  "tty.usbmodem",
  "tty.usbserial",
  "cu.usbmodem",
  "cu.usbserial",
];

function assertHelperExists(localPath: string): void {
  if (!fs.existsSync(localPath)) {
    throw new Error(`Helper script is missing: ${localPath}`);
  }
}

function formatFileTimestamp(date = new Date()): string {
  const pad = (value: number) => String(value).padStart(2, "0");
  return [
    date.getFullYear(),
    pad(date.getMonth() + 1),
    pad(date.getDate()),
    "-",
    pad(date.getHours()),
    pad(date.getMinutes()),
    pad(date.getSeconds()),
  ].join("");
}

function normalizeReplayRequest(
  payload: ReplayRequest,
  defaults: AppSettings["trajectories"],
): ReplayRequest {
  return {
    trajectoryPath: payload.trajectoryPath.trim(),
    target: payload.target === "leader" ? "leader" : "pi",
    speed: payload.speed > 0 ? payload.speed : defaults.defaultReplaySpeed,
    includeBase: payload.target === "leader" ? false : Boolean(payload.includeBase),
    holdFinalS:
      payload.holdFinalS >= 0 ? payload.holdFinalS : defaults.defaultHoldFinalS,
  };
}

export class RobotController {
  private readonly configStore = new ConfigStore(defaultConfig, ROOT_DIR);
  private readonly hostRunner = new RemoteProcessRunner("Pi host");
  private readonly teleopRunner = new LocalProcessRunner("Mac teleop");
  private readonly replayRunner = new RemoteProcessRunner("Replay");
  private readonly localReplayRunner = new LocalProcessRunner("Leader replay");
  private readonly piCalibrationRunner = new RemoteProcessRunner("Pi calibration");
  private readonly macCalibrationRunner = new LocalProcessRunner("Mac calibration");
  private readonly datasetCaptureRunner = new RemoteProcessRunner("Dataset capture");
  private readonly localDatasetCaptureRunner = new LocalProcessRunner("Mac dataset capture");
  private readonly datasetSyncRunner = new TaskRunner("Dataset sync");
  private readonly trainingRunner = new LocalProcessRunner("Training");
  private readonly deploymentRunner = new TaskRunner("Deployment");
  private readonly policyBenchmarkRunner = new TaskRunner("Policy benchmark");
  private readonly policyEvalRunner = new RemoteProcessRunner("Policy eval");
  private lastError: string | null = null;
  private cachedRecordings: RecordingEntry[] = [];
  private lastRecordingRefresh = 0;
  private lastTrainingMetadataRefresh = 0;
  private lastPowerLogSync = 0;
  private lastVexBrainStatusRefresh = 0;
  private lastResolvedHost: string | null = null;
  private activityLog: string[] = [];
  private queue: Promise<void> = Promise.resolve();
  private readonly remoteHelperSyncCache = new Map<string, string>();
  private readonly remoteTorqueCache = new Map<string, string>();
  private readonly remotePowerLogCache = new Map<string, string>();
  private coupledTeleopCleanup: Promise<void> | null = null;
  private cachedVexBrainStatus: VexBrainStatus = {
    connected: false,
    telemetryActive: false,
    consolePort: null,
    commPort: null,
    source: null,
    message: "VEX Brain is not connected to the Pi.",
  };

  async getState(): Promise<DashboardState> {
    const { settings, pinnedMoves, training } = this.configStore.getConfig();
    const wifi = await getWifiStatus();
    const resolvedPiHost = await this.findReachablePiHost(settings);
    const piReachable = Boolean(resolvedPiHost);
    const leader = await this.getLeaderStatus(settings);
    const hostSnapshot = this.hostRunner.getSnapshot();
    const teleopSnapshot = this.teleopRunner.getSnapshot();
    const replaySnapshot = this.replayRunner.getSnapshot();
    const datasetCaptureSnapshot = this.datasetCaptureRunner.getSnapshot();
    this.lastResolvedHost = resolvedPiHost;

    if (resolvedPiHost && Date.now() - this.lastRecordingRefresh > 15000) {
      try {
        this.cachedRecordings = await this.fetchRecordings(settings, resolvedPiHost, true);
        this.lastRecordingRefresh = Date.now();
      } catch (error) {
        this.noteError(error);
      }
    }

    if (resolvedPiHost && Date.now() - this.lastPowerLogSync > POWER_LOG_SYNC_INTERVAL_MS) {
      try {
        await this.syncRemotePowerLogs(settings, resolvedPiHost, true);
      } catch (error) {
        this.noteError(error);
      } finally {
        this.lastPowerLogSync = Date.now();
      }
    }

    const vexBrain = await this.getVexBrainStatus(
      settings,
      resolvedPiHost,
      hostSnapshot,
      replaySnapshot,
      datasetCaptureSnapshot,
    );

    let trainingConfig = training;
    if (Date.now() - this.lastTrainingMetadataRefresh > TRAINING_METADATA_REFRESH_MS) {
      try {
        trainingConfig = await this.refreshTrainingArtifacts(trainingConfig, settings, resolvedPiHost);
        this.lastTrainingMetadataRefresh = Date.now();
      } catch (error) {
        this.noteError(error);
      }
    }

    const selectedProfile =
      trainingConfig.profiles.find((profile) => profile.id === trainingConfig.selectedProfileId) ?? null;
    this.maybeCleanupCoupledTeleopFailure(hostSnapshot, teleopSnapshot);
    const warmHostReplaySnapshot = this.deriveWarmHostReplaySnapshot(hostSnapshot);

    return {
      settings,
      pinnedMoves,
      training: {
        ...trainingConfig,
        selectedProfile,
      },
      wifi,
      piReachable,
      resolvedPiHost,
      leader,
      vexBrain,
      recordings: this.cachedRecordings,
      lastError: this.lastError,
      activityLog: [...this.activityLog],
      services: {
        host: hostSnapshot,
        teleop: teleopSnapshot,
        replay: this.pickServiceSnapshot(
          replaySnapshot,
          this.localReplayRunner.getSnapshot(),
          warmHostReplaySnapshot,
        ),
        piCalibration: this.piCalibrationRunner.getSnapshot(),
        macCalibration: this.macCalibrationRunner.getSnapshot(),
        datasetCapture: this.pickServiceSnapshot(
          datasetCaptureSnapshot,
          this.localDatasetCaptureRunner.getSnapshot(),
        ),
        datasetSync: this.datasetSyncRunner.getSnapshot(),
        training: this.trainingRunner.getSnapshot(),
        deployment: this.deploymentRunner.getSnapshot(),
        policyBenchmark: this.policyBenchmarkRunner.getSnapshot(),
        policyEval: this.policyEvalRunner.getSnapshot(),
      },
    };
  }

  async saveSettings(settings: AppSettings): Promise<DashboardState> {
    return this.runExclusive(async () => {
      this.logActivity("Saving updated dashboard settings.");
      this.configStore.saveSettings(settings);
      const nextSettings = this.configStore.getConfig().settings;
      const host = await this.findReachablePiHost(nextSettings);
      if (host && nextSettings.vex.autoRunTelemetry) {
        await this.syncVexTelemetryProgramOnPi(
          nextSettings,
          host,
          "apply updated VEX control settings",
          false,
        );
      }
      this.lastError = null;
      return this.getState();
    });
  }

  async syncVexTelemetryProgram(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Sync VEX telemetry requested.");
      const host = await this.preparePi(settings, "sync VEX telemetry");
      await this.syncVexTelemetryProgramOnPi(settings, host, "manual VEX telemetry sync", true);
      this.lastError = null;
      return this.getState();
    });
  }

  async saveTrainingProfile(payload: TrainingProfile): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const normalized = normalizeTrainingProfile(payload, ROOT_DIR);
      validateTrainingProfile(normalized);

      const existing = config.training.profiles.find((profile) => profile.id === normalized.id);
      const nextProfile: TrainingProfile = {
        ...normalized,
        artifacts: existing?.artifacts ?? normalized.artifacts,
      };

      const profiles = existing
        ? config.training.profiles.map((profile) =>
            profile.id === nextProfile.id ? nextProfile : profile,
          )
        : [...config.training.profiles, nextProfile];

      this.logActivity(`Saved training profile ${nextProfile.name}.`);
      this.saveTrainingConfig({
        ...config.training,
        profiles,
        selectedProfileId: nextProfile.id,
      });
      this.lastError = null;
      return this.getState();
    });
  }

  async selectTrainingProfile(payload: SelectTrainingProfileRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const profile = this.requireTrainingProfile(config.training, payload.id);
      this.logActivity(`Selected training profile ${profile.name}.`);
      this.saveTrainingConfig({
        ...config.training,
        selectedProfileId: profile.id,
      });
      this.lastError = null;
      return this.getState();
    });
  }

  async deleteTrainingProfile(payload: DeleteTrainingProfileRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const profiles = config.training.profiles.filter((profile) => profile.id !== payload.id);
      const nextProfiles =
        profiles.length > 0 ? profiles : [createDefaultTrainingProfile(ROOT_DIR, "Training Task")];
      const nextSelectedId =
        nextProfiles.find((profile) => profile.id === config.training.selectedProfileId)?.id ??
        nextProfiles[0]?.id ??
        null;

      this.logActivity("Deleted a training profile.");
      this.saveTrainingConfig({
        ...config.training,
        profiles: nextProfiles,
        selectedProfileId: nextSelectedId,
      });
      this.lastError = null;
      return this.getState();
    });
  }

  async startTrainingCapture(payload: StartTrainingCaptureRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, training } = this.configStore.getConfig();
      const profile = this.requireTrainingProfile(training, payload.profileId);
      validateTrainingProfile(profile);

      this.logActivity(`Training dataset capture requested for ${profile.name}.`);
      await this.stopRobotExclusiveProcesses("Training dataset capture needs the robot exclusively.");
      await this.datasetCaptureRunner.stop("Restarting training dataset capture.");
      await this.localDatasetCaptureRunner.stop("Restarting training dataset capture.");

      if (profile.captureMode === "leader-as-follower") {
        const leader = await this.ensureLeaderConnected(settings);
        assertHelperExists(LOCAL_LEADER_CAPTURE_SCRIPT);

        await this.localDatasetCaptureRunner.start(
          this.buildLocalLeaderDatasetCaptureScript(settings, profile, leader),
          "training-capture",
          {
            profileId: profile.id,
            captureMode: profile.captureMode,
            leaderPort: leader.expectedPort,
            macDatasetPath: profile.macDatasetPath,
          },
        );
      } else {
        const host = await this.preparePi(settings, "start training capture");
        const leader =
          profile.captureMode === "leader" ? await this.ensureLeaderConnected(settings) : null;

        assertHelperExists(DATASET_CAPTURE_SCRIPT);
        assertHelperExists(POWER_SCRIPT);
        assertHelperExists(RUNTIME_SCRIPT);
        await this.ensureRemoteHelpers(settings, host);
        if (settings.vex.autoRunTelemetry) {
          await this.syncVexTelemetryProgramOnPi(settings, host, "start training capture", false);
        }
        await this.writeRemoteTorqueLimits(settings, host);

        await this.datasetCaptureRunner.start(
          buildPiDatasetCaptureCommand(
            settings,
            profile,
            this.getRemoteTorqueLimitsPath(settings),
          ),
          this.toConnectConfig(settings, host),
          "training-capture",
          {
            host,
            profileId: profile.id,
            captureMode: profile.captureMode,
            piDatasetPath: profile.piDatasetPath,
          },
        );

        try {
          await this.waitForHostReady(host, "Dataset capture host", this.datasetCaptureRunner);
          if (leader) {
            await this.teleopRunner.start(
              this.buildTeleopScript(settings, host, leader),
              "training-capture",
              {
                profileId: profile.id,
                leaderPort: leader.expectedPort,
              },
            );
            await this.waitForLocalProcessReady("Mac teleop", this.teleopRunner);
          }
        } catch (error) {
          await this.datasetCaptureRunner.stop("Training dataset capture failed to launch.");
          throw error;
        }
      }

      this.lastError = null;
      return this.getState();
    });
  }

  async stopTrainingCapture(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, training } = this.configStore.getConfig();
      const remoteCapture = this.datasetCaptureRunner.getSnapshot();
      const localCapture = this.localDatasetCaptureRunner.getSnapshot();
      const activeProfileId =
        typeof localCapture.meta.profileId === "string"
          ? String(localCapture.meta.profileId)
          : typeof remoteCapture.meta.profileId === "string"
            ? String(remoteCapture.meta.profileId)
            : training.selectedProfileId;

      if (this.teleopRunner.getSnapshot().mode === "training-capture") {
        await this.teleopRunner.stop("Stopping training teleop relay.");
      }
      await this.datasetCaptureRunner.stop("Stopping training dataset capture.");
      await this.localDatasetCaptureRunner.stop("Stopping training dataset capture.");

      const profile = activeProfileId
        ? training.profiles.find((item) => item.id === activeProfileId)
        : null;
      if (profile) {
        const summary =
          profile.captureMode === "leader-as-follower"
            ? await this.inspectLocalDataset(profile, settings)
            : await (async () => {
                const host = await this.findReachablePiHost(settings);
                if (!host) {
                  return null;
                }
                return this.fetchRemoteDatasetSummary(settings, host, profile);
              })();

        if (summary) {
          this.updateTrainingProfile(profile.id, (current) => ({
            ...current,
            artifacts: {
              ...current.artifacts,
              datasetEpisodeCount: summary.episodeCount,
              datasetCameraKeys: summary.cameraKeys,
              lastDatasetEpisodeIndex: summary.lastEpisodeIndex,
            },
          }));
        }
      }

      this.lastError = null;
      return this.getState();
    });
  }

  async startTrainingSync(payload: StartTrainingSyncRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, training } = this.configStore.getConfig();
      const profile = this.requireTrainingProfile(training, payload.profileId);

      await this.datasetSyncRunner.run("dataset-sync", { profileId: profile.id }, async (log) => {
        if (profile.captureMode === "leader-as-follower") {
          log("system", `Dataset already lives on the Mac at ${profile.macDatasetPath}. Skipping Pi download.`);
          return;
        }

        const host = await this.preparePi(settings, "sync training dataset");
        log("system", `Syncing ${profile.piDatasetPath} to ${profile.macDatasetPath}.`);
        await this.syncRemoteDirectoryToLocal(settings, host, profile.piDatasetPath, profile.macDatasetPath, log);
      });

      const now = new Date().toISOString();
      const localSummary = await this.inspectLocalDataset(profile, settings);
      this.updateTrainingProfile(profile.id, (current) => ({
        ...current,
        artifacts: {
          ...current.artifacts,
          datasetEpisodeCount: localSummary.episodeCount,
          datasetCameraKeys: localSummary.cameraKeys,
          lastDatasetEpisodeIndex: localSummary.lastEpisodeIndex,
          lastDatasetSyncAt: now,
        },
      }));

      this.lastError = null;
      return this.getState();
    });
  }

  async startTrainingRun(payload: StartTrainingRunRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, training } = this.configStore.getConfig();
      const profile = this.requireTrainingProfile(training, payload.profileId);
      validateTrainingProfile(profile);
      if (!fs.existsSync(profile.macDatasetPath)) {
        throw new Error(`Training dataset does not exist on the Mac: ${profile.macDatasetPath}`);
      }

      fs.mkdirSync(profile.macTrainOutputDir, { recursive: true });
      const now = new Date().toISOString();
      this.logActivity(`Training run requested for ${profile.name}.`);
      this.updateTrainingProfile(profile.id, (current) => ({
        ...current,
        artifacts: {
          ...current.artifacts,
          lastTrainingStartedAt: now,
        },
      }));

      await this.trainingRunner.start(
        buildMacTrainingCommand(settings, profile),
        "training",
        { profileId: profile.id },
      );

      this.lastError = null;
      return this.getState();
    });
  }

  async stopTrainingRun(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      await this.trainingRunner.stop("Stopping the training run.");
      this.lastError = null;
      return this.getState();
    });
  }

  async deployTrainingCheckpoint(
    payload: DeployTrainingCheckpointRequest,
  ): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, training } = this.configStore.getConfig();
      const profile = this.requireTrainingProfile(training, payload.profileId);
      const checkpointPath =
        profile.selectedCheckpointPath.trim() ||
        profile.artifacts.lastCheckpointPath ||
        profile.artifacts.availableCheckpointPaths.at(-1) ||
        "";
      const configPath = path.join(checkpointPath, "config.json");
      if (!checkpointPath || !fs.existsSync(configPath)) {
        throw new Error("Choose a valid trained checkpoint before deploying to the Pi.");
      }

      const host = await this.preparePi(settings, "deploy policy checkpoint");
      await this.deploymentRunner.run("deploy", { profileId: profile.id }, async (log) => {
        log("system", `Uploading ${checkpointPath} to ${profile.piDeployPath}.`);
        await this.syncLocalDirectoryToRemote(
          settings,
          host,
          checkpointPath,
          profile.piDeployPath,
          log,
        );
        await this.writeRemoteDeployManifest(settings, host, profile, checkpointPath);
      });

      const now = new Date().toISOString();
      this.updateTrainingProfile(profile.id, (current) => ({
        ...current,
        selectedCheckpointPath: checkpointPath,
        artifacts: {
          ...current.artifacts,
          deployedCheckpointPath: current.piDeployPath,
          deployedAt: now,
          benchmark: null,
        },
      }));

      this.lastError = null;
      return this.getState();
    });
  }

  async runPolicyBenchmark(payload: BenchmarkPolicyRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, training } = this.configStore.getConfig();
      const profile = this.requireTrainingProfile(training, payload.profileId);
      if (!profile.artifacts.deployedCheckpointPath) {
        throw new Error("Deploy a checkpoint to the Pi before running the benchmark.");
      }

      const host = await this.preparePi(settings, "benchmark deployed policy");
      await this.ensureRemoteHelpers(settings, host);
      await this.stopRobotExclusiveProcesses("Policy benchmark needs the robot exclusively.");

      let benchmarkResult: TrainingArtifact["benchmark"] = null;
      await this.policyBenchmarkRunner.run(
        "policy-benchmark",
        { profileId: profile.id },
        async (log) => {
          const stdout = await this.execRemoteScript(
            buildPiPolicyBenchmarkCommand(
              settings,
              profile,
              training.settings.benchmarkIterations,
            ),
            host,
            settings,
          );
          benchmarkResult = JSON.parse(stdout.stdout) as TrainingArtifact["benchmark"];
          if (!benchmarkResult) {
            throw new Error("The policy benchmark returned no result.");
          }
          benchmarkResult.passed = benchmarkPasses(
            benchmarkResult,
            training.settings.deployBenchmarkMargin,
          );
          log(
            "system",
            `Benchmark complete: ${benchmarkResult.effectiveFps.toFixed(2)} fps effective, ` +
              `avg ${benchmarkResult.averageLatencyMs.toFixed(2)} ms, ` +
              `${benchmarkResult.passed ? "PASS" : "FAIL"}.`,
          );
        },
      );

      if (!benchmarkResult) {
        throw new Error("The policy benchmark did not produce a result.");
      }

      this.updateTrainingProfile(profile.id, (current) => ({
        ...current,
        artifacts: {
          ...current.artifacts,
          benchmark: benchmarkResult,
        },
      }));

      this.lastError = null;
      return this.getState();
    });
  }

  async startPolicyEval(payload: StartPolicyEvalRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, training } = this.configStore.getConfig();
      const profile = this.requireTrainingProfile(training, payload.profileId);
      const benchmark = profile.artifacts.benchmark;
      if (!profile.artifacts.deployedCheckpointPath) {
        throw new Error("Deploy a checkpoint to the Pi before starting policy evaluation.");
      }
      if (!benchmark?.passed) {
        throw new Error("Run and pass the Pi policy benchmark before starting evaluation.");
      }

      const host = await this.preparePi(settings, "start policy evaluation");
      await this.ensureRemoteHelpers(settings, host);
      await this.stopRobotExclusiveProcesses("Policy evaluation needs the robot exclusively.");
      await this.policyEvalRunner.stop("Restarting policy evaluation.");

      const evalDatasetPath = `${profile.piEvalDatasetPath.replace(/\/$/, "")}/run-${formatFileTimestamp()}`;
      await this.policyEvalRunner.start(
        buildPiPolicyEvalCommand(settings, profile, benchmark, evalDatasetPath),
        this.toConnectConfig(settings, host),
        "policy-eval",
        {
          host,
          profileId: profile.id,
          evalDatasetPath,
        },
      );

      this.updateTrainingProfile(profile.id, (current) => ({
        ...current,
        artifacts: {
          ...current.artifacts,
          latestEvalDatasetPath: evalDatasetPath,
          latestEvalAt: new Date().toISOString(),
        },
      }));

      this.lastError = null;
      return this.getState();
    });
  }

  async stopPolicyEval(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      await this.policyEvalRunner.stop("Stopping policy evaluation.");
      this.lastError = null;
      return this.getState();
    });
  }

  async startControl(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Start control requested.");
      const leader = await this.ensureLeaderConnected(settings);
      const host = await this.preparePi(settings, "start control");

      assertHelperExists(HOST_SCRIPT);
      assertHelperExists(POWER_SCRIPT);
      assertHelperExists(RUNTIME_SCRIPT);
      await this.ensureRemoteHelpers(settings, host);
      if (settings.vex.autoRunTelemetry) {
        await this.syncVexTelemetryProgramOnPi(settings, host, "start control", false);
      }
      await this.writeRemoteTorqueLimits(settings, host);

      await this.replayRunner.stop("Stopping replay before live control.");
      await this.localReplayRunner.stop("Stopping replay before live control.");
      await this.datasetCaptureRunner.stop("Live control needs the robot exclusively.");
      await this.localDatasetCaptureRunner.stop("Live control needs the robot exclusively.");
      await this.policyEvalRunner.stop("Live control needs the robot exclusively.");
      await this.teleopRunner.stop("Restarting teleop.");
      await this.hostRunner.stop("Restarting Pi host.");
      await this.piCalibrationRunner.stop("Live control needs the robot exclusively.");
      await this.macCalibrationRunner.stop("Live control needs the leader arm exclusively.");
      await this.clearRemoteHostCommand(settings, host);

      await this.hostRunner.start(
        this.buildHostScript(settings, true),
        this.toConnectConfig(settings, host),
        "control",
        { host },
      );

      try {
        await this.waitForHostReady(host, "Pi host");
        await this.teleopRunner.start(this.buildTeleopScript(settings, host, leader), "control");
        await this.waitForLocalProcessReady("Mac teleop", this.teleopRunner);
      } catch (error) {
        await this.hostRunner.stop("Teleop failed to launch, stopping the Pi host.");
        throw error;
      }
      this.lastError = null;
      return this.getState();
    });
  }

  async startKeyboardControl(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Keyboard hotkey host requested.");
      const host = await this.preparePi(settings, "arm keyboard hotkeys");

      assertHelperExists(HOST_SCRIPT);
      assertHelperExists(POWER_SCRIPT);
      assertHelperExists(RUNTIME_SCRIPT);
      await this.ensureRemoteHelpers(settings, host);
      if (settings.vex.autoRunTelemetry) {
        await this.syncVexTelemetryProgramOnPi(settings, host, "arm keyboard hotkeys", false);
      }
      await this.writeRemoteTorqueLimits(settings, host);

      await this.stopRobotExclusiveProcesses("Preparing keyboard-ready hotkey host.");
      await this.clearRemoteHostCommand(settings, host);

      await this.hostRunner.start(
        this.buildHostScript(settings, true),
        this.toConnectConfig(settings, host),
        "keyboard-control",
        { host, hotkeysArmed: true },
      );

      try {
        await this.waitForHostReady(host, "Keyboard hotkey host");
      } catch (error) {
        await this.hostRunner.stop("Keyboard hotkey host failed to launch.");
        throw error;
      }

      this.lastError = null;
      return this.getState();
    });
  }

  async stopControl(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      this.logActivity("Stop control requested.");
      await this.teleopRunner.stop("Stopping teleop.");
      await this.hostRunner.stop("Stopping the Pi host.");
      this.lastError = null;
      return this.getState();
    });
  }

  async emergencyStop(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Emergency stop requested.");
      const host = await this.preparePi(settings, "emergency stop");

      await this.teleopRunner.stop("Emergency stop requested.");
      await this.replayRunner.stop("Emergency stop requested.");
      await this.localReplayRunner.stop("Emergency stop requested.");
      await this.hostRunner.stop("Emergency stop requested.");
      await this.piCalibrationRunner.stop("Emergency stop requested.");
      await this.macCalibrationRunner.stop("Emergency stop requested.");
      await this.datasetCaptureRunner.stop("Emergency stop requested.");
      await this.localDatasetCaptureRunner.stop("Emergency stop requested.");
      await this.policyEvalRunner.stop("Emergency stop requested.");
      await this.execRemoteScript(this.buildStopAllScript(settings), host, settings);

      this.lastError = null;
      return this.getState();
    });
  }

  async setArmTorqueLimits(payload: TorqueLimitsRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const nextSettings = {
        ...config.settings,
        host: {
          ...config.settings.host,
          armTorqueLimits: this.normalizeArmTorqueLimits(
            payload?.limits ?? {},
            config.settings.host.armTorqueLimits,
          ),
        },
      };

      this.configStore.saveSettings(nextSettings);
      const host = await this.findReachablePiHost(nextSettings);
      if (host) {
        await this.writeRemoteTorqueLimits(nextSettings, host);
      }

      this.lastError = null;
      return this.getState();
    });
  }

  async startPiCalibration(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Pi calibration requested.");
      const host = await this.preparePi(settings, "start Pi calibration");
      assertHelperExists(PI_CALIBRATION_SCRIPT);
      await this.ensureRemoteHelpers(settings, host);

      await this.teleopRunner.stop("Pi calibration needs the robot exclusively.");
      await this.replayRunner.stop("Pi calibration needs the robot exclusively.");
      await this.hostRunner.stop("Pi calibration needs the robot exclusively.");
      await this.datasetCaptureRunner.stop("Pi calibration needs the robot exclusively.");
      await this.localDatasetCaptureRunner.stop("Pi calibration needs the robot exclusively.");
      await this.localReplayRunner.stop("Pi calibration needs the robot exclusively.");
      await this.policyEvalRunner.stop("Pi calibration needs the robot exclusively.");
      await this.piCalibrationRunner.stop("Restarting Pi calibration.");

      await this.piCalibrationRunner.start(
        this.buildPiCalibrationScript(settings),
        this.toConnectConfig(settings, host),
        "pi-calibration",
        { host },
      );

      this.lastError = null;
      return this.getState();
    });
  }

  async startPiServoCalibration(payload: ServoCalibrationRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const servoId =
        typeof payload?.servoId === "string" &&
        ARM_MOTORS.includes(payload.servoId as (typeof ARM_MOTORS)[number])
          ? payload.servoId
          : null;
      if (!servoId) {
        throw new Error("A valid arm servo id is required for calibration.");
      }

      const { settings } = this.configStore.getConfig();
      this.logActivity(`Pi single-servo calibration requested for ${servoId}.`);
      const host = await this.preparePi(settings, `start ${servoId} calibration`);
      assertHelperExists(PI_CALIBRATION_SCRIPT);
      await this.ensureRemoteHelpers(settings, host);

      await this.teleopRunner.stop("Pi calibration needs the robot exclusively.");
      await this.replayRunner.stop("Pi calibration needs the robot exclusively.");
      await this.hostRunner.stop("Pi calibration needs the robot exclusively.");
      await this.datasetCaptureRunner.stop("Pi calibration needs the robot exclusively.");
      await this.localDatasetCaptureRunner.stop("Pi calibration needs the robot exclusively.");
      await this.localReplayRunner.stop("Pi calibration needs the robot exclusively.");
      await this.policyEvalRunner.stop("Pi calibration needs the robot exclusively.");
      await this.piCalibrationRunner.stop("Restarting Pi calibration.");

      await this.piCalibrationRunner.start(
        this.buildPiServoCalibrationScript(settings, servoId),
        this.toConnectConfig(settings, host),
        "pi-servo-calibration",
        { host, servoId },
      );

      this.lastError = null;
      return this.getState();
    });
  }

  async startMacCalibration(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Mac leader calibration requested.");
      const leader = await this.ensureLeaderConnected(settings);

      await this.teleopRunner.stop("Mac calibration needs the leader arm exclusively.");
      await this.replayRunner.stop("Mac calibration requested.");
      await this.hostRunner.stop("Mac calibration requested.");
      await this.datasetCaptureRunner.stop("Mac calibration requested.");
      await this.localDatasetCaptureRunner.stop("Mac calibration requested.");
      await this.localReplayRunner.stop("Mac calibration requested.");
      await this.policyEvalRunner.stop("Mac calibration requested.");
      await this.macCalibrationRunner.stop("Restarting Mac calibration.");

      await this.macCalibrationRunner.start(
        this.buildMacCalibrationScript(settings, leader),
        "mac-calibration",
        { leaderPort: leader.expectedPort },
      );

      this.lastError = null;
      return this.getState();
    });
  }

  async sendPiCalibrationInput(input: "enter" | "c"): Promise<DashboardState> {
    return this.runExclusive(async () => {
      if (!this.piCalibrationRunner.sendInput(this.formatCalibrationInput(input), input === "c" ? "c + Enter" : "Enter")) {
        throw new Error("Pi calibration is not running.");
      }
      return this.getState();
    });
  }

  async sendMacCalibrationInput(input: "enter" | "c"): Promise<DashboardState> {
    return this.runExclusive(async () => {
      if (!this.macCalibrationRunner.sendInput(this.formatCalibrationInput(input), input === "c" ? "c + Enter" : "Enter")) {
        throw new Error("Mac calibration is not running.");
      }
      return this.getState();
    });
  }

  async stopPiCalibration(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      this.logActivity("Stop Pi calibration requested.");
      await this.piCalibrationRunner.stop("Stopping Pi calibration.");
      return this.getState();
    });
  }

  async stopMacCalibration(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      this.logActivity("Stop Mac calibration requested.");
      await this.macCalibrationRunner.stop("Stopping Mac calibration.");
      return this.getState();
    });
  }

  async startRecording(label: string): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity(`Start recording requested${label.trim() ? ` (${label.trim()})` : ""}.`);
      const leader = await this.ensureLeaderConnected(settings);
      const host = await this.preparePi(settings, "start recording");

      assertHelperExists(RECORD_SCRIPT);
      assertHelperExists(RUNTIME_SCRIPT);
      await this.ensureRemoteHelpers(settings, host);
      if (settings.vex.autoRunTelemetry) {
        await this.syncVexTelemetryProgramOnPi(settings, host, "start recording", false);
      }
      await this.writeRemoteTorqueLimits(settings, host);

      await this.replayRunner.stop("Recording takes over the robot.");
      await this.localReplayRunner.stop("Recording takes over the robot.");
      await this.teleopRunner.stop("Restarting teleop for recording.");
      await this.hostRunner.stop("Replacing the live host with the recorder.");
      await this.piCalibrationRunner.stop("Recording needs the robot exclusively.");
      await this.macCalibrationRunner.stop("Recording needs the leader arm exclusively.");
      await this.datasetCaptureRunner.stop("Recording needs the robot exclusively.");
      await this.localDatasetCaptureRunner.stop("Recording needs the robot exclusively.");
      await this.policyEvalRunner.stop("Recording needs the robot exclusively.");

      const timestamp = formatFileTimestamp();
      const trajectoryPath = `${settings.trajectories.remoteDir}/trajectory-${timestamp}.json`;

      await this.hostRunner.start(
        this.buildRecordScript(settings, trajectoryPath, label.trim()),
        this.toConnectConfig(settings, host),
        "recording",
        {
          host,
          trajectoryPath,
          label: label.trim(),
        },
      );

      try {
        await this.waitForHostReady(host, "Recorder");
        await this.teleopRunner.start(this.buildTeleopScript(settings, host, leader), "recording");
        await this.waitForLocalProcessReady("Mac teleop", this.teleopRunner);
      } catch (error) {
        await this.hostRunner.stop("Teleop failed to launch, stopping the recorder.");
        throw error;
      }

      this.cachedRecordings = await this.fetchRecordings(settings, host);
      this.lastRecordingRefresh = Date.now();
      this.lastError = null;
      return this.getState();
    });
  }

  async stopRecording(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const state = this.hostRunner.getSnapshot();
      const { settings } = this.configStore.getConfig();
      this.logActivity("Stop recording requested.");

      await this.teleopRunner.stop("Stopping teleop for recording save.");
      if (state.mode === "recording") {
        await this.hostRunner.stop("Stopping the recorder and saving the trajectory.");
      }

      const host = await this.findReachablePiHost(settings);
      if (host) {
        this.cachedRecordings = await this.fetchRecordings(settings, host);
        this.lastRecordingRefresh = Date.now();
      }

      this.lastError = null;
      return this.getState();
    });
  }

  async refreshRecordings(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Refreshing recording list from the Pi.");
      const host = await this.preparePi(settings, "refresh recordings");
      this.cachedRecordings = await this.fetchRecordings(settings, host);
      this.lastRecordingRefresh = Date.now();
      this.lastError = null;
      return this.getState();
    });
  }

  async getRecordingDetail(payload: RecordingDetailRequest): Promise<RecordingDetail> {
    const { settings } = this.configStore.getConfig();
    const recordingPath = payload.path.trim();
    if (!recordingPath) {
      throw new Error("Choose a recording first.");
    }

    this.logActivity(`Loading recording detail for ${recordingPath}.`);
    const host = await this.preparePi(settings, "inspect recording");
    return this.readRecordingDetail(settings, host, recordingPath);
  }

  async renameRecording(payload: RenameRecordingRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const recordingPath = payload.path.trim();
      const label = payload.label.trim();

      if (!recordingPath) {
        throw new Error("Choose a recording before renaming it.");
      }
      if (!label) {
        throw new Error("Recording name cannot be empty.");
      }

      this.logActivity(`Renaming recording ${recordingPath} to ${label}.`);
      const host = await this.preparePi(settings, "rename recording");
      await this.writeRecordingLabel(settings, host, recordingPath, label);
      this.cachedRecordings = await this.fetchRecordings(settings, host);
      this.lastRecordingRefresh = Date.now();
      this.lastError = null;
      return this.getState();
    });
  }

  async trimRecording(payload: TrimRecordingRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const recordingPath = payload.path.trim();
      const trimStartS = Number(payload.trimStartS);
      const trimEndS = Number(payload.trimEndS);

      if (!recordingPath) {
        throw new Error("Choose a recording before trimming it.");
      }
      if (!Number.isFinite(trimStartS) || !Number.isFinite(trimEndS)) {
        throw new Error("Trim start and end must be valid numbers.");
      }
      if (trimStartS < 0 || trimEndS <= trimStartS) {
        throw new Error("Trim end must be greater than trim start.");
      }

      this.logActivity(
        `Trimming recording ${recordingPath} to ${trimStartS.toFixed(2)}s-${trimEndS.toFixed(2)}s.`,
      );
      const host = await this.preparePi(settings, "trim recording");
      await this.trimRecordingRange(settings, host, recordingPath, trimStartS, trimEndS);
      this.cachedRecordings = await this.fetchRecordings(settings, host);
      this.lastRecordingRefresh = Date.now();
      this.lastError = null;
      return this.getState();
    });
  }

  async startReplay(payload: ReplayRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const replay = normalizeReplayRequest(payload, settings.trajectories);
      if (!replay.trajectoryPath) {
        throw new Error("Choose a saved trajectory before starting replay.");
      }
      this.logActivity(
        `Starting ${replay.target === "leader" ? "leader" : "Pi"} replay for ${replay.trajectoryPath}${
          replay.target === "pi" ? ` (${replay.includeBase ? "VEX base + arm" : "arm only"})` : ""
        }.`,
      );

      await this.replayRunner.stop("Restarting replay.");
      await this.localReplayRunner.stop("Restarting replay.");
      await this.stopWarmHostReplay(settings, "Restarting warm-host replay.");

      if (this.canUseWarmHostReplay(replay)) {
        const host = this.getActiveHostAddress();
        if (!host) {
          throw new Error("The running Pi host is active, but its address is unavailable.");
        }

        await this.teleopRunner.stop("Switching from leader control to Pi replay.");
        await this.piCalibrationRunner.stop("Replay needs the robot exclusively.");
        await this.macCalibrationRunner.stop("Replay requested.");
        await this.datasetCaptureRunner.stop("Replay needs the robot exclusively.");
        await this.localDatasetCaptureRunner.stop("Replay needs the robot exclusively.");
        await this.policyEvalRunner.stop("Replay needs the robot exclusively.");
        await this.sendWarmHostReplay(settings, host, replay);

        this.lastError = null;
        return this.getState();
      }

      await this.teleopRunner.stop("Replay takes control away from teleop.");
      await this.hostRunner.stop("Replay needs exclusive access.");
      await this.piCalibrationRunner.stop("Replay needs the robot exclusively.");
      await this.macCalibrationRunner.stop("Replay requested.");
      await this.datasetCaptureRunner.stop("Replay needs the robot exclusively.");
      await this.localDatasetCaptureRunner.stop("Replay needs the robot exclusively.");
      await this.policyEvalRunner.stop("Replay needs the robot exclusively.");

      if (replay.target === "leader") {
        const leader = await this.ensureLeaderConnected(settings);
        const host = await this.preparePi(settings, "download replay recording");
        assertHelperExists(LOCAL_LEADER_REPLAY_SCRIPT);
        const localTrajectoryPath = await this.downloadRemoteRecordingToLocal(
          settings,
          host,
          replay.trajectoryPath,
        );

        await this.localReplayRunner.start(
          this.buildLocalLeaderReplayScript(settings, replay, leader, localTrajectoryPath),
          "replay",
          {
            ...replay,
            leaderPort: leader.expectedPort,
            localTrajectoryPath,
          },
        );
      } else {
        const host = await this.preparePi(settings, "start replay");

        assertHelperExists(REPLAY_SCRIPT);
        assertHelperExists(RUNTIME_SCRIPT);
        await this.ensureRemoteHelpers(settings, host);
        await this.writeRemoteTorqueLimits(settings, host);

        await this.replayRunner.start(
          this.buildReplayScript(settings, replay),
          this.toConnectConfig(settings, host),
          "replay",
          { ...replay },
        );
      }

      this.lastError = null;
      return this.getState();
    });
  }

  async stopReplay(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      this.logActivity("Stop replay requested.");
      const { settings } = this.configStore.getConfig();
      await this.replayRunner.stop("Stopping replay.");
      await this.localReplayRunner.stop("Stopping replay.");
      await this.stopWarmHostReplay(settings, "Stopping warm-host replay.");
      const host = await this.findReachablePiHost(settings);
      if (host && settings.vex.autoRunTelemetry) {
        await this.syncVexTelemetryProgramOnPi(settings, host, "restore controller telemetry after replay", false);
      }
      this.lastError = null;
      return this.getState();
    });
  }

  async createPinnedMove(payload: CreatePinnedMoveRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const replay = normalizeReplayRequest(payload, config.settings.trajectories);
      this.logActivity(`Saving pinned move ${payload.name.trim() || replay.trajectoryPath}.`);
      const pinnedMoves = [
        ...config.pinnedMoves,
        {
          id: crypto.randomUUID(),
          name: payload.name.trim(),
          keyBinding: payload.keyBinding.trim(),
          ...replay,
        },
      ];

      this.configStore.savePinnedMoves(pinnedMoves);
      this.lastError = null;
      return this.getState();
    });
  }

  async deletePinnedMove(id: string): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      this.logActivity("Removing pinned move.");
      this.configStore.savePinnedMoves(
        config.pinnedMoves.filter((move) => move.id !== id),
      );
      this.lastError = null;
      return this.getState();
    });
  }

  async triggerPinnedMove(id: string): Promise<DashboardState> {
    const config = this.configStore.getConfig();
    const pinnedMove = config.pinnedMoves.find((move) => move.id === id);
    if (!pinnedMove) {
      throw new Error("Pinned move not found.");
    }

    this.logActivity(`Pinned move triggered: ${pinnedMove.name}.`);
    return this.startReplay(pinnedMove);
  }

  private async runExclusive<T>(operation: () => Promise<T>): Promise<T> {
    const previous = this.queue;
    let release: () => void = () => undefined;
    this.queue = new Promise<void>((resolve) => {
      release = resolve;
    });

    await previous;

    try {
      return await operation();
    } catch (error) {
      this.noteError(error);
      throw error;
    } finally {
      release();
    }
  }

  private noteError(error: unknown): void {
    this.lastError = error instanceof Error ? error.message : String(error);
    this.logActivity(`Error: ${this.lastError}`);
  }

  private async preparePi(settings: AppSettings, actionLabel: string): Promise<string> {
    const maxAttempts = 5;
    for (let attempt = 1; attempt <= maxAttempts; attempt += 1) {
      this.logActivity(
        `Pi connection attempt ${attempt}/${maxAttempts} for ${actionLabel}.`,
      );
      const host = await this.findReachablePiHost(settings);
      if (host) {
        this.logActivity(`Pi reachable on ${host}.`);
        this.lastResolvedHost = host;
        return host;
      }
      if (attempt < maxAttempts) {
        await sleep(1000);
      }
    }

    const wifi = await getWifiStatus();
    const hotspotHint =
      wifi.ssid === settings.hotspot.ssid
        ? ` You are already on ${settings.hotspot.ssid}, so the Pi may still be booting or unreachable.`
        : ` You are probably not on ${settings.hotspot.ssid}. Current Wi-Fi: ${wifi.ssid ?? "none"}.`;
    throw new Error(
      `Could not ${actionLabel}: the Pi is not reachable on ${settings.pi.host} or ${settings.pi.fallbackHost} after ${maxAttempts} attempts.${hotspotHint}`,
    );
  }

  private async findReachablePiHost(settings: AppSettings): Promise<string | null> {
    const candidates = [settings.pi.host, settings.pi.fallbackHost].filter(Boolean);
    for (const candidate of candidates) {
      if (await isTcpReachable(candidate, 22)) {
        return candidate;
      }
    }
    return null;
  }

  private async waitForHostReady(
    host: string,
    label: string,
    runner: RemoteProcessRunner = this.hostRunner,
  ): Promise<void> {
    this.logActivity(`Waiting for ${label} ports on ${host}.`);
    const startedAt = Date.now();

    while (Date.now() - startedAt < HOST_READY_TIMEOUT_MS) {
      const portChecks = await Promise.all(
        HOST_READY_PORTS.map((port) => isTcpReachable(host, port, 300)),
      );
      if (portChecks.every(Boolean)) {
        this.logActivity(`${label} is listening on ports ${HOST_READY_PORTS.join(", ")}.`);
        return;
      }

      const exited = await runner.waitForExit(250);
      if (exited) {
        throw new Error(this.formatStartupExit(label, exited));
      }
    }

    const exited = await runner.waitForExit(0);
    if (exited) {
      throw new Error(this.formatStartupExit(label, exited));
    }
    throw new Error(
      `${label} did not open ports ${HOST_READY_PORTS.join(", ")} on ${host} within ${Math.round(
        HOST_READY_TIMEOUT_MS / 1000,
      )} seconds.`,
    );
  }

  private async waitForLocalProcessReady(
    label: string,
    runner: LocalProcessRunner,
    timeoutMs = 1500,
  ): Promise<void> {
    const exited = await runner.waitForExit(timeoutMs);
    if (exited) {
      throw new Error(this.formatStartupExit(label, exited));
    }
  }

  private maybeCleanupCoupledTeleopFailure(
    hostSnapshot: ServiceSnapshot,
    teleopSnapshot: ServiceSnapshot,
  ): void {
    if (this.coupledTeleopCleanup) {
      return;
    }

    if (teleopSnapshot.state !== "error") {
      return;
    }

    if (hostSnapshot.state !== "running") {
      return;
    }

    if (hostSnapshot.mode !== teleopSnapshot.mode) {
      return;
    }

    if (hostSnapshot.mode !== "control" && hostSnapshot.mode !== "recording") {
      return;
    }

    const reason =
      hostSnapshot.mode === "recording"
        ? "Mac teleop failed; stopping the recorder."
        : "Mac teleop failed; stopping the Pi host.";
    this.logActivity(`${reason} ${teleopSnapshot.detail}`);

    this.coupledTeleopCleanup = this.runExclusive(async () => {
      await this.hostRunner.stop(reason);
    })
      .catch((error) => {
        this.noteError(error);
      })
      .finally(() => {
        this.coupledTeleopCleanup = null;
      });
  }

  private formatStartupExit(label: string, snapshot: ServiceSnapshot): string {
    const exit =
      snapshot.exitCode === null ? "" : ` with code ${snapshot.exitCode}`;
    const diagnostic = this.extractFailureLine(snapshot);
    return `${label} exited during startup${exit}.${diagnostic ? ` ${diagnostic}` : ""}`;
  }

  private extractFailureLine(snapshot: ServiceSnapshot): string | null {
    const logLine = [...snapshot.logs].reverse().find((line) => {
      return /\[stderr\].*(Error|Exception|Traceback|Failed)/.test(line);
    });

    if (!logLine) {
      return snapshot.detail && snapshot.detail !== "Stopped." ? snapshot.detail : null;
    }

    const cleaned = logLine.replace(/^\d{1,2}:\d{2}:\d{2}\s[AP]M\s\[[^\]]+\]\s*/, "");
    const motorId = cleaned.match(/id_=(\d+)/)?.[1];
    const motorName = motorId ? LEKIWI_MOTOR_NAMES_BY_ID[motorId] : null;
    return motorName ? `${cleaned} Motor id ${motorId} is ${motorName}.` : cleaned;
  }

  private deriveWarmHostReplaySnapshot(hostSnapshot: ServiceSnapshot): ServiceSnapshot {
    const replayLogs = hostSnapshot.logs.filter((line) => line.includes("[replay]"));
    const inlineReplayReady =
      hostSnapshot.state === "running" && this.hostSupportsInlineReplay(hostSnapshot.mode);
    const baseDetail =
      inlineReplayReady
        ? hostSnapshot.mode === "control"
          ? "Live control host ready for torque-preserving Pi replay handoff."
          : "Hotkey host armed for instant Pi replays."
        : "No warm-host replay activity yet.";
    const lastStartIndex = this.findLastLogIndex(hostSnapshot.logs, /\[replay\] start\b/);
    const lastEndIndex = this.findLastLogIndex(
      hostSnapshot.logs,
      /\[replay\] (complete|stopped|interrupted|error)\b/,
    );
    const active = lastStartIndex !== -1 && lastStartIndex > lastEndIndex;
    const lastReplayLine =
      hostSnapshot.logs[Math.max(lastStartIndex, lastEndIndex)] ?? replayLogs.at(-1) ?? null;
    const state: ServiceSnapshot["state"] =
      active ? "running" : lastReplayLine?.includes("[replay] error") ? "error" : "idle";

    return {
      label: "Replay",
      state,
      detail: lastReplayLine ? this.cleanServiceLogLine(lastReplayLine) : baseDetail,
      pid: hostSnapshot.pid,
      exitCode: null,
      startedAt:
        lastStartIndex === -1 ? null : this.parseServiceLogTimestamp(hostSnapshot.logs[lastStartIndex]),
      stoppedAt:
        active || lastEndIndex === -1 ? null : this.parseServiceLogTimestamp(hostSnapshot.logs[lastEndIndex]),
      mode: active ? "replay" : inlineReplayReady ? hostSnapshot.mode : null,
      logs: replayLogs.slice(-60),
      meta: {
        transport: "warm-host",
        host: this.getActiveHostAddress(),
      },
    };
  }

  private findLastLogIndex(lines: string[], pattern: RegExp): number {
    for (let index = lines.length - 1; index >= 0; index -= 1) {
      if (pattern.test(lines[index])) {
        return index;
      }
    }
    return -1;
  }

  private cleanServiceLogLine(line: string): string {
    return line.replace(/^\d{1,2}:\d{2}:\d{2}\s[AP]M\s\[[^\]]+\]\s*/, "");
  }

  private parseServiceLogTimestamp(line: string): string | null {
    const match = line.match(/^(\d{1,2}):(\d{2}):(\d{2})\s(AM|PM)\b/);
    if (!match) {
      return null;
    }

    let hours = Number(match[1]) % 12;
    if (match[4] === "PM") {
      hours += 12;
    }

    const stamp = new Date();
    stamp.setHours(hours, Number(match[2]), Number(match[3]), 0);
    return stamp.toISOString();
  }

  private pickServiceSnapshot(...snapshots: ServiceSnapshot[]): ServiceSnapshot {
    return snapshots
      .slice()
      .sort((left, right) => {
        const leftRunning = left.state !== "idle" ? 1 : 0;
        const rightRunning = right.state !== "idle" ? 1 : 0;
        if (leftRunning !== rightRunning) {
          return leftRunning - rightRunning;
        }
        return this.snapshotActivityAt(left) - this.snapshotActivityAt(right);
      })
      .at(-1) ?? snapshots[0];
  }

  private snapshotActivityAt(snapshot: ServiceSnapshot): number {
    const startedAt = snapshot.startedAt ? Date.parse(snapshot.startedAt) : Number.NEGATIVE_INFINITY;
    const stoppedAt = snapshot.stoppedAt ? Date.parse(snapshot.stoppedAt) : Number.NEGATIVE_INFINITY;
    const activityAt = Math.max(startedAt, stoppedAt);
    return Number.isFinite(activityAt) ? activityAt : 0;
  }

  private toConnectConfig(settings: AppSettings, host: string): ConnectConfig {
    return {
      host,
      port: 22,
      username: settings.pi.username,
      password: settings.pi.password,
      readyTimeout: 10000,
      hostVerifier: () => true,
    };
  }

  private buildMacActivation(settings: AppSettings): string {
    return [
      `source ${shellQuote(settings.mac.condaScript)}`,
      "conda activate lerobot",
      `cd ${shellQuote(settings.mac.projectDir)}`,
    ].join("\n");
  }

  private buildPiActivation(settings: AppSettings): string {
    return [
      `source ${shellQuote(settings.pi.condaScript)}`,
      "conda activate lerobot",
      `cd ${shellQuote(settings.pi.projectDir)}`,
    ].join("\n");
  }

  private buildTeleopScript(
    settings: AppSettings,
    host: string,
    leader: LeaderStatus,
  ): string {
    assertHelperExists(UI_TELEOP_SCRIPT);
    return [
      this.buildMacActivation(settings),
      `python ${shellQuote(UI_TELEOP_SCRIPT)} \\`,
      `  --remote-host ${shellQuote(host)} \\`,
      `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
      `  --leader-port ${shellQuote(leader.expectedPort ?? "auto")}`,
    ].join("\n");
  }

  private buildPiCalibrationScript(settings: AppSettings): string {
    return [
      this.buildPiActivation(settings),
      `python ${shellQuote(`${settings.pi.remoteHelperDir}/scripts/lekiwi_calibrate.py`)} \\`,
      "  --mode full-arm \\",
      `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
      `  --robot-port ${shellQuote(settings.pi.robotPort)}`,
    ].join("\n");
  }

  private buildPiServoCalibrationScript(settings: AppSettings, servoId: string): string {
    return [
      this.buildPiActivation(settings),
      `python ${shellQuote(`${settings.pi.remoteHelperDir}/scripts/lekiwi_calibrate.py`)} \\`,
      "  --mode single-servo \\",
      `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
      `  --robot-port ${shellQuote(settings.pi.robotPort)} \\`,
      `  --servo ${shellQuote(servoId)}`,
    ].join("\n");
  }

  private buildMacCalibrationScript(
    settings: AppSettings,
    leader: LeaderStatus,
  ): string {
    return [
      this.buildMacActivation(settings),
      "lerobot-calibrate \\",
      "  --teleop.type=so100_leader \\",
      `  --teleop.port=${shellQuote(leader.expectedPort ?? "auto")} \\`,
      "  --teleop.id=leader",
    ].join("\n");
  }

  private formatCalibrationInput(input: "enter" | "c"): string {
    return input === "c" ? "c\n" : "\n";
  }

  private getRemoteTorqueLimitsPath(settings: AppSettings): string {
    return `${settings.pi.remoteHelperDir}/torque_limits.json`;
  }

  private getRemoteHostCommandPath(settings: AppSettings): string {
    return `${settings.pi.remoteHelperDir}/host_command.json`;
  }

  private getRemotePowerLogDir(): string {
    return "~/lekiwi-power-logs";
  }

  private buildSaferServoModeFlag(settings: AppSettings): string {
    return settings.host.saferServoMode ? " \\\n  --safer-servo-mode" : "";
  }

  private buildVexControlConfig(settings: AppSettings): Record<string, unknown> {
    return {
      motors: structuredClone(settings.vex.motors),
      controls: structuredClone(settings.vex.controls),
      tuning: structuredClone(settings.vex.tuning),
    };
  }

  private buildVexBaseFlags(settings: AppSettings): string {
    return [
      `  --vex-telemetry-slot ${settings.vex.telemetrySlot} \\`,
      `  --vex-telemetry-program-name ${shellQuote(settings.vex.telemetryProgramName)} \\`,
      `  --vex-replay-slot ${settings.vex.replaySlot} \\`,
      `  --vex-control-config-json ${shellQuote(JSON.stringify(this.buildVexControlConfig(settings)))}`,
    ].join("\n");
  }

  private clampTorqueLimit(value: unknown): number {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) {
      return TORQUE_LIMIT_MAX;
    }
    return Math.min(TORQUE_LIMIT_MAX, Math.max(TORQUE_LIMIT_MIN, Math.round(numeric)));
  }

  private normalizeArmTorqueLimits(
    incoming: Record<string, unknown> | undefined,
    existing: Record<string, number> | undefined,
  ): Record<string, number> {
    const next = Object.fromEntries(
      ARM_MOTORS.map((motor) => [motor, this.clampTorqueLimit(existing?.[motor] ?? TORQUE_LIMIT_MAX)]),
    ) as Record<string, number>;

    for (const motor of ARM_MOTORS) {
      if (incoming && Object.prototype.hasOwnProperty.call(incoming, motor)) {
        next[motor] = this.clampTorqueLimit(incoming[motor]);
      }
    }

    return next;
  }

  private async writeRemoteTorqueLimits(settings: AppSettings, host: string): Promise<void> {
    const torqueLimits = this.normalizeArmTorqueLimits({}, settings.host.armTorqueLimits);
    const torquePath = this.getRemoteTorqueLimitsPath(settings);
    const serializedTorqueLimits = JSON.stringify(torqueLimits);
    const cacheKey = `${host}:${torquePath}`;
    if (this.remoteTorqueCache.get(cacheKey) === serializedTorqueLimits) {
      return;
    }
    const script = `
mkdir -p ${shellQuote(path.posix.dirname(torquePath))}
cat > ${shellQuote(torquePath)} <<'JSON'
${serializedTorqueLimits}
JSON
`.trim();

    await this.execRemoteScript(script, host, settings);
    this.remoteTorqueCache.set(cacheKey, serializedTorqueLimits);
  }

  private getTeleoperateScriptPath(settings: AppSettings): string {
    return path.join(settings.mac.projectDir, "examples", "lekiwi", "teleoperate.py");
  }

  private defaultVexBrainStatus(message: string): VexBrainStatus {
    return {
      connected: false,
      telemetryActive: false,
      consolePort: null,
      commPort: null,
      source: null,
      message,
    };
  }

  private shouldProbeVexBrainConsole(...snapshots: ServiceSnapshot[]): boolean {
    return snapshots.every((snapshot) => snapshot.state === "idle");
  }

  private describeActiveVexConsumer(...snapshots: ServiceSnapshot[]): string {
    return snapshots.find((snapshot) => snapshot.state !== "idle")?.label ?? "another Pi task";
  }

  private buildVexBrainStatusScript(
    settings: AppSettings,
    probeConsole: boolean,
    activeConsumer: string | null,
  ): string {
    const activeConsumerLiteral = activeConsumer === null ? "None" : JSON.stringify(activeConsumer);
    const probeConsoleLiteral = probeConsole ? "True" : "False";
    const telemetryMessage = JSON.stringify(
      `VEX Brain is connected, but no telemetry is coming out. Run slot ${settings.vex.telemetrySlot} ${settings.vex.telemetryProgramName} on the Brain.`,
    );
    return `
python - <<'PY'
import glob
import json
import time
from pathlib import Path

try:
    import serial
except Exception:
    serial = None

PORT_GLOB = "/dev/serial/by-id/*VEX_Robotics_V5_Brain*"
paths = sorted(glob.glob(PORT_GLOB))
probe_console = ${probeConsoleLiteral}
active_consumer = ${activeConsumerLiteral}


def pick(*suffixes):
    for suffix in suffixes:
        for candidate in paths:
            if suffix in Path(candidate).name:
                return candidate
    return None


console_port = pick("if02", "if00")
comm_port = pick("if00", "if01")
payload = {
    "connected": bool(paths),
    "telemetryActive": False,
    "consolePort": console_port,
    "commPort": comm_port,
    "source": None,
}

if not paths:
    payload["message"] = "VEX Brain is not connected to the Pi."
    print(json.dumps(payload))
    raise SystemExit(0)

if not probe_console:
    payload["message"] = (
        f"VEX Brain is connected on the Pi. Live telemetry probe is paused while {active_consumer} is active."
    )
    print(json.dumps(payload))
    raise SystemExit(0)

if serial is None:
    payload["message"] = "VEX Brain is connected, but pyserial is unavailable on the Pi."
    print(json.dumps(payload))
    raise SystemExit(0)

if console_port is None:
    payload["message"] = "VEX Brain is connected, but the Pi cannot see the Brain console port."
    print(json.dumps(payload))
    raise SystemExit(0)

try:
    handle = serial.Serial(console_port, baudrate=115200, timeout=0.05)
except Exception as exc:
    payload["message"] = f"VEX Brain is connected on {console_port}, but the console port could not be opened: {exc}"
    print(json.dumps(payload))
    raise SystemExit(0)

buffer = ""
deadline = time.time() + 0.8

try:
    while time.time() < deadline:
        available = int(getattr(handle, "in_waiting", 0) or 0)
        chunk = handle.read(available or 1)
        if chunk:
            buffer += chunk.decode("utf-8", errors="ignore")

        while "\\n" in buffer:
            line, buffer = buffer.split("\\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                candidate = json.loads(line)
            except Exception:
                continue
            if not isinstance(candidate, dict):
                continue
            if not all(key in candidate for key in ("x.vel", "y.vel", "theta.vel")):
                continue
            payload["telemetryActive"] = True
            source = candidate.get("source")
            payload["source"] = source if isinstance(source, str) else None
            break

        if payload["telemetryActive"]:
            break

        time.sleep(0.03)
finally:
    handle.close()

if payload["telemetryActive"]:
    source = payload["source"] or "telemetry"
    payload["message"] = f"VEX Brain telemetry is active on {console_port} ({source})."
else:
    payload["message"] = ${telemetryMessage}

print(json.dumps(payload))
PY
`.trim();
  }

  private buildVexTelemetrySyncScript(settings: AppSettings, runAfterInstall: boolean): string {
    return `
export PYTHONPATH=${shellQuote(`${settings.pi.remoteHelperDir}/scripts`)}:$PYTHONPATH
export LEKIWI_VEX_CONTROL_CONFIG=${shellQuote(JSON.stringify(this.buildVexControlConfig(settings)))}
export LEKIWI_VEX_TELEMETRY_SLOT=${shellQuote(String(settings.vex.telemetrySlot))}
export LEKIWI_VEX_TELEMETRY_NAME=${shellQuote(settings.vex.telemetryProgramName)}
export LEKIWI_VEX_RUN_AFTER_INSTALL=${shellQuote(runAfterInstall ? "1" : "0")}
python3 - <<'PY'
import json
import logging
import os

from vex_base_bridge import VexBaseTelemetryManager

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("vex_base_telemetry_sync")

control_config = json.loads(os.environ["LEKIWI_VEX_CONTROL_CONFIG"])
manager = VexBaseTelemetryManager(
    requested_vexcom_path="auto",
    telemetry_slot=int(os.environ["LEKIWI_VEX_TELEMETRY_SLOT"]),
    cache_dir="~/.lekiwi-vex/programs",
    logger=logger,
)
result = {
    "enabled": manager.enabled,
    "success": False,
    "status": manager.status_message,
}
if manager.enabled:
    result["success"] = manager.install_and_run(
        control_config,
        program_name=os.environ["LEKIWI_VEX_TELEMETRY_NAME"],
        run_after_install=os.environ["LEKIWI_VEX_RUN_AFTER_INSTALL"] == "1",
    )
print(json.dumps(result))
PY
`.trim();
  }

  private async syncVexTelemetryProgramOnPi(
    settings: AppSettings,
    host: string,
    reason: string,
    strict: boolean,
  ): Promise<boolean> {
    try {
      await this.ensureRemoteHelpers(settings, host);
      const { stdout } = await this.execRemoteScript(
        this.buildVexTelemetrySyncScript(settings, true),
        host,
        settings,
      );
      const parsed = stdout ? JSON.parse(stdout) : null;
      const enabled = Boolean(parsed?.enabled);
      const success = Boolean(parsed?.success);
      const status =
        typeof parsed?.status === "string" && parsed.status.trim()
          ? parsed.status.trim()
          : "VEX telemetry sync completed.";

      this.lastVexBrainStatusRefresh = 0;

      if (!enabled) {
        if (strict) {
          throw new Error(status);
        }
        this.logActivity(`Skipping ${reason}: ${status}`);
        return false;
      }

      if (!success) {
        if (strict) {
          throw new Error(status);
        }
        this.logActivity(`VEX telemetry sync failed during ${reason}: ${status}`);
        return false;
      }

      this.logActivity(`VEX telemetry ready for ${reason} on slot ${settings.vex.telemetrySlot}.`);
      return true;
    } catch (error) {
      this.lastVexBrainStatusRefresh = 0;
      if (strict) {
        throw error;
      }
      const message = error instanceof Error ? error.message : String(error);
      this.logActivity(`VEX telemetry sync skipped during ${reason}: ${message}`);
      return false;
    }
  }

  private normalizeVexBrainStatus(candidate: unknown, fallbackMessage: string): VexBrainStatus {
    if (!candidate || typeof candidate !== "object") {
      return this.defaultVexBrainStatus(fallbackMessage);
    }

    const payload = candidate as Partial<VexBrainStatus>;
    return {
      connected: Boolean(payload.connected),
      telemetryActive: Boolean(payload.telemetryActive),
      consolePort: typeof payload.consolePort === "string" ? payload.consolePort : null,
      commPort: typeof payload.commPort === "string" ? payload.commPort : null,
      source: typeof payload.source === "string" ? payload.source : null,
      message: typeof payload.message === "string" && payload.message.trim() ? payload.message : fallbackMessage,
    };
  }

  private async getVexBrainStatus(
    settings: AppSettings,
    resolvedPiHost: string | null,
    hostSnapshot: ServiceSnapshot,
    replaySnapshot: ServiceSnapshot,
    datasetCaptureSnapshot: ServiceSnapshot,
  ): Promise<VexBrainStatus> {
    if (!resolvedPiHost) {
      const offline = this.defaultVexBrainStatus("Pi is offline, so VEX Brain status is unavailable.");
      this.cachedVexBrainStatus = offline;
      this.lastVexBrainStatusRefresh = Date.now();
      return offline;
    }

    if (Date.now() - this.lastVexBrainStatusRefresh < VEX_STATUS_REFRESH_MS) {
      return this.cachedVexBrainStatus;
    }

    const probeConsole = this.shouldProbeVexBrainConsole(
      hostSnapshot,
      replaySnapshot,
      datasetCaptureSnapshot,
    );
    const activeConsumer = probeConsole
      ? null
      : this.describeActiveVexConsumer(hostSnapshot, replaySnapshot, datasetCaptureSnapshot);

    try {
      const { stdout } = await this.execRemoteScript(
        this.buildVexBrainStatusScript(settings, probeConsole, activeConsumer),
        resolvedPiHost,
        settings,
      );
      const parsed = stdout ? JSON.parse(stdout) : null;
      const fallbackMessage = probeConsole
        ? "VEX Brain status probe returned no data."
        : `VEX Brain is connected on the Pi. Live telemetry probe is paused while ${activeConsumer} is active.`;
      const normalized = this.normalizeVexBrainStatus(parsed, fallbackMessage);
      this.cachedVexBrainStatus = normalized;
      this.lastVexBrainStatusRefresh = Date.now();
      return normalized;
    } catch (error) {
      const message =
        error instanceof Error
          ? `Could not check the VEX Brain on the Pi: ${error.message}`
          : "Could not check the VEX Brain on the Pi.";
      const fallback = {
        ...this.cachedVexBrainStatus,
        message,
      };
      this.cachedVexBrainStatus = fallback;
      this.lastVexBrainStatusRefresh = Date.now();
      return fallback;
    }
  }

  private async getLeaderStatus(settings: AppSettings): Promise<LeaderStatus> {
    const teleoperateScriptPath = this.getTeleoperateScriptPath(settings);
    const availablePorts = await this.listLocalLeaderPorts();

    let expectedPort: string | null = null;
    try {
      const scriptText = await fs.promises.readFile(teleoperateScriptPath, "utf8");
      const match = scriptText.match(/SO100LeaderConfig\s*\(\s*port=(["'])(.*?)\1/);
      expectedPort = match?.[2] ?? null;
    } catch (error) {
      return {
        teleoperateScriptPath,
        expectedPort: null,
        connected: false,
        availablePorts,
        message:
          error instanceof Error
            ? `Could not read teleop script: ${error.message}`
            : "Could not read teleop script.",
      };
    }

    if (!expectedPort) {
      return {
        teleoperateScriptPath,
        expectedPort: null,
        connected: false,
        availablePorts,
        message: "The teleop script does not expose a leader-arm port to check.",
      };
    }

    if (availablePorts.includes(expectedPort)) {
      return {
        teleoperateScriptPath,
        expectedPort,
        connected: true,
        availablePorts,
        message: `Leader arm detected on ${expectedPort}.`,
      };
    }

    const availableSummary = availablePorts.length
      ? ` Available serial ports: ${availablePorts.join(", ")}.`
      : " No matching USB serial ports are visible on the Mac.";

    return {
      teleoperateScriptPath,
      expectedPort,
      connected: false,
      availablePorts,
      message: `Leader arm is not connected on ${expectedPort}.${availableSummary}`,
    };
  }

  private async ensureLeaderConnected(settings: AppSettings): Promise<LeaderStatus> {
    const leader = await this.getLeaderStatus(settings);
    if (!leader.connected) {
      throw new Error(leader.message);
    }
    this.logActivity(leader.message);
    return leader;
  }

  private async listLocalLeaderPorts(): Promise<string[]> {
    try {
      const entries = await fs.promises.readdir("/dev");
      return entries
        .filter((entry) => LEADER_PORT_PREFIXES.some((prefix) => entry.startsWith(prefix)))
        .sort()
        .map((entry) => path.join("/dev", entry));
    } catch {
      return [];
    }
  }

  private buildHostScript(settings: AppSettings, includeUiCommandPath = false): string {
    const uiCommandFlag = includeUiCommandPath
      ? ` \\\n  --ui-command-path ${shellQuote(this.getRemoteHostCommandPath(settings))}`
      : "";
    const saferServoModeFlag = this.buildSaferServoModeFlag(settings);
    const vexBaseFlags = this.buildVexBaseFlags(settings);
    return [
      this.buildPiActivation(settings),
      `python ${shellQuote(`${settings.pi.remoteHelperDir}/scripts/lekiwi_host.py`)} \\`,
      `  --connection-time-s ${settings.host.connectionTimeS} \\`,
      `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
      `  --robot-port ${shellQuote(settings.pi.robotPort)} \\`,
      `  --robot-cameras-json ${shellQuote(settings.host.camerasJson)} \\`,
      `  --use-degrees true \\`,
      `  --torque-limits-json ${shellQuote(JSON.stringify(this.normalizeArmTorqueLimits({}, settings.host.armTorqueLimits)))} \\`,
      `  --torque-limits-path ${shellQuote(this.getRemoteTorqueLimitsPath(settings))} \\`,
      `  --base-max-raw-velocity ${settings.host.baseMaxRawVelocity} \\`,
      `  --base-wheel-torque-limit ${settings.host.baseWheelTorqueLimit} \\`,
      `  --enable-base false \\`,
      `${vexBaseFlags}${uiCommandFlag}${saferServoModeFlag}`,
    ].join("\n");
  }

  private buildRecordScript(
    settings: AppSettings,
    trajectoryPath: string,
    label: string,
  ): string {
    const labelFlag = label ? ` \\\n  --label ${shellQuote(label)}` : "";
    const saferServoModeFlag = this.buildSaferServoModeFlag(settings);
    return [
      this.buildPiActivation(settings),
      `mkdir -p ${shellQuote(settings.trajectories.remoteDir)}`,
      `python ${shellQuote(`${settings.pi.remoteHelperDir}/scripts/lekiwi_record_trajectory.py`)} \\`,
      `  --connection-time-s ${settings.host.connectionTimeS} \\`,
      `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
      `  --robot-port ${shellQuote(settings.pi.robotPort)} \\`,
      `  --robot-cameras-json ${shellQuote(settings.host.camerasJson)} \\`,
      `  --use-degrees true \\`,
      `  --torque-limits-json ${shellQuote(JSON.stringify(this.normalizeArmTorqueLimits({}, settings.host.armTorqueLimits)))} \\`,
      `  --torque-limits-path ${shellQuote(this.getRemoteTorqueLimitsPath(settings))} \\`,
      `  --base-max-raw-velocity ${settings.host.baseMaxRawVelocity} \\`,
      `  --base-wheel-torque-limit ${settings.host.baseWheelTorqueLimit} \\`,
      `  --enable-base false \\`,
      `  --output ${shellQuote(trajectoryPath)}${labelFlag}${saferServoModeFlag}`,
    ].join("\n");
  }

  private buildReplayScript(settings: AppSettings, replay: ReplayRequest): string {
    const includeBaseFlag = replay.includeBase ? " \\\n  --include-base" : "";
    const saferServoModeFlag = this.buildSaferServoModeFlag(settings);
    const vexBaseFlags = this.buildVexBaseFlags(settings);
    return [
      this.buildPiActivation(settings),
      `python ${shellQuote(`${settings.pi.remoteHelperDir}/scripts/lekiwi_replay_trajectory.py`)} \\`,
      `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
      `  --robot-port ${shellQuote(settings.pi.robotPort)} \\`,
      `  --robot-cameras-json ${shellQuote(settings.host.camerasJson)} \\`,
      `  --use-degrees true \\`,
      `  --torque-limits-json ${shellQuote(JSON.stringify(this.normalizeArmTorqueLimits({}, settings.host.armTorqueLimits)))} \\`,
      `  --torque-limits-path ${shellQuote(this.getRemoteTorqueLimitsPath(settings))} \\`,
      `  --base-max-raw-velocity ${settings.host.baseMaxRawVelocity} \\`,
      `  --base-wheel-torque-limit ${settings.host.baseWheelTorqueLimit} \\`,
      `  --enable-base false \\`,
      `${vexBaseFlags} \\`,
      `  --input ${shellQuote(replay.trajectoryPath)} \\`,
      `  --speed ${replay.speed} \\`,
      `  --hold-final-s ${replay.holdFinalS}${includeBaseFlag}${saferServoModeFlag}`,
    ].join("\n");
  }

  private buildLocalLeaderDatasetCaptureScript(
    settings: AppSettings,
    profile: TrainingProfile,
    leader: LeaderStatus,
  ): string {
    return [
      this.buildMacActivation(settings),
      `mkdir -p ${shellQuote(profile.macDatasetPath)}`,
      `python ${shellQuote(LOCAL_LEADER_CAPTURE_SCRIPT)} \\`,
      `  --robot-id ${shellQuote(LOCAL_LEADER_ROBOT_ID)} \\`,
      `  --robot-port ${shellQuote(leader.expectedPort ?? "auto")} \\`,
      `  --dataset-repo-id ${shellQuote(deriveDatasetRepoId(profile))} \\`,
      `  --dataset-root ${shellQuote(profile.macDatasetPath)} \\`,
      `  --task ${shellQuote(profile.task)} \\`,
      `  --fps ${profile.fps} \\`,
      `  --num-episodes ${profile.numEpisodes} \\`,
      `  --episode-time-s ${profile.episodeTimeS} \\`,
      `  --reset-time-s ${profile.resetTimeS}`,
    ].join("\n");
  }

  private buildLocalLeaderReplayScript(
    settings: AppSettings,
    replay: ReplayRequest,
    leader: LeaderStatus,
    localTrajectoryPath: string,
  ): string {
    return [
      this.buildMacActivation(settings),
      `python ${shellQuote(LOCAL_LEADER_REPLAY_SCRIPT)} \\`,
      `  --robot-id ${shellQuote(LOCAL_LEADER_ROBOT_ID)} \\`,
      `  --robot-port ${shellQuote(leader.expectedPort ?? "auto")} \\`,
      `  --input ${shellQuote(localTrajectoryPath)} \\`,
      `  --speed ${replay.speed} \\`,
      `  --hold-final-s ${replay.holdFinalS}`,
    ].join("\n");
  }

  private buildStopAllScript(settings: AppSettings): string {
    return `
export LEKIWI_ROBOT_PORT=${shellQuote(settings.pi.robotPort)}

if command -v lsof >/dev/null 2>&1; then
  pids="$(lsof -tiTCP:5555 -sTCP:LISTEN || true) $(lsof -tiTCP:5556 -sTCP:LISTEN || true)"
  if [ -n "$pids" ]; then
    kill $pids >/dev/null 2>&1 || true
    sleep 1
    kill -9 $pids >/dev/null 2>&1 || true
  fi
fi

pkill -f 'python -m lerobot\\.robots\\.lekiwi\\.lekiwi_host' >/dev/null 2>&1 || true
pkill -f 'scripts/lekiwi_.*host\\.py' >/dev/null 2>&1 || true
pkill -f 'lekiwi_record_trajectory\\.py' >/dev/null 2>&1 || true
pkill -f 'lekiwi_replay_trajectory\\.py' >/dev/null 2>&1 || true
sleep 1

${this.buildPiActivation(settings)}

python - <<'PY'
import os
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

motors = {
    f"m{i}": Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
    for i in range(1, 10)
}

bus = FeetechMotorsBus(
    port=os.environ["LEKIWI_ROBOT_PORT"],
    motors=motors,
)

bus.connect(handshake=False)
disabled = []
failed = []

for motor_name in motors:
    try:
        bus.disable_torque(motor_name)
        disabled.append(motor_name)
    except Exception as exc:
        failed.append((motor_name, str(exc)))

bus.disconnect(disable_torque=False)

print(f"Disabled torque on: {', '.join(disabled) if disabled else 'none'}")
if failed:
    print("Failed to disable torque on:")
    for motor_name, error in failed:
        print(f"  - {motor_name}: {error}")
PY
`.trim();
  }

  private canUseWarmHostReplay(replay: ReplayRequest): boolean {
    const hostSnapshot = this.hostRunner.getSnapshot();
    return (
      replay.target === "pi" &&
      hostSnapshot.state === "running" &&
      this.hostSupportsInlineReplay(hostSnapshot.mode)
    );
  }

  private hostSupportsInlineReplay(mode: string | null): boolean {
    return mode === "control" || mode === "keyboard-control";
  }

  private getActiveHostAddress(): string | null {
    const hostSnapshot = this.hostRunner.getSnapshot();
    const metaHost =
      typeof hostSnapshot.meta.host === "string" && hostSnapshot.meta.host.trim()
        ? hostSnapshot.meta.host.trim()
        : null;
    return metaHost ?? this.lastResolvedHost;
  }

  private async sendWarmHostReplay(
    settings: AppSettings,
    host: string,
    replay: ReplayRequest,
  ): Promise<void> {
    await this.writeRemoteHostCommand(settings, host, {
      command: "replay",
      trajectoryPath: replay.trajectoryPath,
      speed: replay.speed,
      holdFinalS: replay.holdFinalS,
      includeBase: replay.includeBase,
    });
    this.logActivity(`Queued replay command on the warm host at ${host}.`);
  }

  private async stopWarmHostReplay(settings: AppSettings, reason: string): Promise<void> {
    const hostSnapshot = this.hostRunner.getSnapshot();
    if (
      hostSnapshot.state !== "running" ||
      !this.hostSupportsInlineReplay(hostSnapshot.mode)
    ) {
      return;
    }

    const host = this.getActiveHostAddress();
    if (!host) {
      return;
    }

    await this.writeRemoteHostCommand(settings, host, {
      command: "stop-replay",
    }).catch(() => undefined);
    this.logActivity(reason);
  }

  private async writeRemoteHostCommand(
    settings: AppSettings,
    host: string,
    command:
      | {
          command: "replay";
          trajectoryPath: string;
          speed: number;
          holdFinalS: number;
          includeBase: boolean;
        }
      | { command: "stop-replay" },
  ): Promise<void> {
    const commandPath = this.getRemoteHostCommandPath(settings);
    const payload =
      command.command === "replay"
        ? {
            command: "replay",
            trajectory_path: command.trajectoryPath,
            speed: command.speed,
            hold_final_s: command.holdFinalS,
            include_base: command.includeBase,
            requested_at: new Date().toISOString(),
          }
        : {
            command: "stop-replay",
            requested_at: new Date().toISOString(),
          };
    const serialized = JSON.stringify(payload);
    const script = `
mkdir -p ${shellQuote(path.posix.dirname(commandPath))}
cat > ${shellQuote(commandPath)} <<'JSON'
${serialized}
JSON
`.trim();
    await this.execRemoteScript(script, host, settings);
  }

  private async clearRemoteHostCommand(settings: AppSettings, host: string): Promise<void> {
    await this.execRemoteScript(
      `rm -f ${shellQuote(this.getRemoteHostCommandPath(settings))}`,
      host,
      settings,
    );
  }

  private async ensureRemoteHelpers(settings: AppSettings, host: string): Promise<void> {
    const helperDir = `${settings.pi.remoteHelperDir}/scripts`;
    const fingerprintPath = `${settings.pi.remoteHelperDir}/.ui-helper-sync`;
    const fingerprint = await this.computeRemoteHelperFingerprint(settings);
    const cacheKey = `${host}:${settings.pi.remoteHelperDir}`;

    if (this.remoteHelperSyncCache.get(cacheKey) === fingerprint) {
      return;
    }

    const probeScript = `
if [ -f ${shellQuote(fingerprintPath)} ]; then
  cat ${shellQuote(fingerprintPath)}
fi
`.trim();
    const probeResult = await this.execRemoteScript(probeScript, host, settings);
    if (probeResult.stdout.trim() === fingerprint) {
      this.remoteHelperSyncCache.set(cacheKey, fingerprint);
      return;
    }

    this.logActivity(`Uploading helper scripts to ${host}.`);
    await this.execRemoteScript(
      `mkdir -p ${shellQuote(helperDir)} ${shellQuote(settings.trajectories.remoteDir)}`,
      host,
      settings,
    );

    await this.withSftp(settings, host, async (sftp) => {
      await Promise.all([
        this.fastPut(sftp, HOST_SCRIPT, `${helperDir}/lekiwi_host.py`),
        this.fastPut(sftp, POWER_SCRIPT, `${helperDir}/lekiwi_power.py`),
        this.fastPut(sftp, RUNTIME_SCRIPT, `${helperDir}/lekiwi_runtime.py`),
        this.fastPut(sftp, VEX_BASE_BRIDGE_SCRIPT, `${helperDir}/vex_base_bridge.py`),
        this.fastPut(sftp, PI_CALIBRATION_SCRIPT, `${helperDir}/lekiwi_calibrate.py`),
        this.fastPut(sftp, RECORD_SCRIPT, `${helperDir}/lekiwi_record_trajectory.py`),
        this.fastPut(sftp, REPLAY_SCRIPT, `${helperDir}/lekiwi_replay_trajectory.py`),
        this.fastPut(sftp, DATASET_CAPTURE_SCRIPT, `${helperDir}/lekiwi_record_dataset_host.py`),
        this.fastPut(sftp, POLICY_BENCHMARK_SCRIPT, `${helperDir}/lekiwi_benchmark_policy.py`),
        this.fastPut(sftp, POLICY_EVAL_SCRIPT, `${helperDir}/lekiwi_run_policy_eval.py`),
      ]);
    });

    await this.execRemoteScript(
      `
cat > ${shellQuote(fingerprintPath)} <<'EOF'
${fingerprint}
EOF
`.trim(),
      host,
      settings,
    );
    this.remoteHelperSyncCache.set(cacheKey, fingerprint);
  }

  private async fetchRecordings(
    settings: AppSettings,
    host: string,
    silent = false,
  ): Promise<RecordingEntry[]> {
    if (!silent) {
      this.logActivity(`Reading recording list from ${host}.`);
    }
    const script = `
export LEKIWI_TRAJECTORY_DIR=${shellQuote(settings.trajectories.remoteDir)}
python - <<'PY'
from datetime import datetime
import json
import os
from pathlib import Path

root = Path(os.environ["LEKIWI_TRAJECTORY_DIR"]).expanduser()
records = []
if root.exists():
    for path in sorted(root.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True):
        stat = path.stat()
        label = None
        duration_s = None
        try:
            payload = json.loads(path.read_text())
            if isinstance(payload, dict):
                raw_label = payload.get("label")
                if isinstance(raw_label, str) and raw_label.strip():
                    label = raw_label.strip()
                raw_duration = payload.get("duration_s")
                if isinstance(raw_duration, (int, float)):
                    duration_s = round(float(raw_duration), 3)
                elif isinstance(payload.get("samples"), list) and payload["samples"]:
                    last_sample = payload["samples"][-1]
                    if isinstance(last_sample, dict):
                        sample_duration = last_sample.get("t_s")
                        if isinstance(sample_duration, (int, float)):
                            duration_s = round(float(sample_duration), 3)
        except Exception:
            pass
        records.append(
            {
                "path": str(path),
                "name": label or path.stem,
                "fileName": path.name,
                "label": label,
                "size": stat.st_size,
                "modifiedAt": datetime.fromtimestamp(stat.st_mtime).astimezone().isoformat(timespec="seconds"),
                "durationS": duration_s,
            }
        )
print(json.dumps(records))
PY
`.trim();

    const { stdout } = await this.execRemoteScript(script, host, settings);
    return JSON.parse(stdout) as RecordingEntry[];
  }

  private async listRemotePowerLogs(
    settings: AppSettings,
    host: string,
  ): Promise<Array<{ path: string; name: string; size: number; modifiedAtNs: number }>> {
    const script = `
export LEKIWI_POWER_LOG_DIR=${shellQuote(this.getRemotePowerLogDir())}
python - <<'PY'
from pathlib import Path
import json
import os

root = Path(os.path.expanduser(os.environ["LEKIWI_POWER_LOG_DIR"]))
records = []
if root.exists():
    for path in sorted(root.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True):
        stat = path.stat()
        records.append(
            {
                "path": str(path),
                "name": path.name,
                "size": stat.st_size,
                "modifiedAtNs": stat.st_mtime_ns,
            }
        )

print(json.dumps(records))
PY
`.trim();

    const { stdout } = await this.execRemoteScript(script, host, settings);
    if (!stdout) {
      return [];
    }
    return JSON.parse(stdout) as Array<{
      path: string;
      name: string;
      size: number;
      modifiedAtNs: number;
    }>;
  }

  private async syncRemotePowerLogs(
    settings: AppSettings,
    host: string,
    silent = false,
  ): Promise<void> {
    const remoteLogs = await this.listRemotePowerLogs(settings, host);
    if (!remoteLogs.length) {
      return;
    }

    await fs.promises.mkdir(LOCAL_POWER_LOG_DIR, { recursive: true });

    let syncedCount = 0;
    await this.withSftp(settings, host, async (sftp) => {
      for (const entry of remoteLogs) {
        const cacheKey = entry.path;
        const fingerprint = `${entry.size}:${entry.modifiedAtNs}`;
        const localPath = path.join(LOCAL_POWER_LOG_DIR, entry.name);

        if (
          this.remotePowerLogCache.get(cacheKey) === fingerprint &&
          fs.existsSync(localPath)
        ) {
          continue;
        }

        await this.fastGet(sftp, entry.path, localPath);
        this.remotePowerLogCache.set(cacheKey, fingerprint);
        syncedCount += 1;
      }
    });

    if (!syncedCount || silent) {
      return;
    }

    this.logActivity(
      `Downloaded ${syncedCount} power log ${syncedCount === 1 ? "file" : "files"} from ${host} to ${LOCAL_POWER_LOG_DIR}.`,
    );
  }

  private async readRecordingDetail(
    settings: AppSettings,
    host: string,
    recordingPath: string,
  ): Promise<RecordingDetail> {
    const script = `
export LEKIWI_TRAJECTORY_DIR=${shellQuote(settings.trajectories.remoteDir)}
export LEKIWI_RECORDING_PATH=${shellQuote(recordingPath)}
python - <<'PY'
import json
import os
from pathlib import Path

root = Path(os.environ["LEKIWI_TRAJECTORY_DIR"]).expanduser().resolve()
path = Path(os.environ["LEKIWI_RECORDING_PATH"]).expanduser().resolve()

if root not in path.parents:
    raise SystemExit("Recording path is outside the trajectory directory.")
if not path.is_file():
    raise SystemExit(f"Recording does not exist: {path}")

payload = json.loads(path.read_text())
if not isinstance(payload, dict):
    raise SystemExit("Recording payload is invalid.")

arm_keys = [key for key in payload.get("arm_state_keys", []) if isinstance(key, str)]
base_keys = [key for key in payload.get("base_state_keys", []) if isinstance(key, str)]
keys = arm_keys + base_keys

def normalize_points(items, value_key):
    points = []
    if not isinstance(items, list):
        return points
    for item in items:
        if not isinstance(item, dict):
            continue
        raw_t_s = item.get("t_s")
        raw_values = item.get(value_key)
        if not isinstance(raw_t_s, (int, float)) or not isinstance(raw_values, dict):
            continue
        point = {}
        for key in keys:
            value = raw_values.get(key)
            point[key] = float(value) if isinstance(value, (int, float)) else 0.0
        points.append({"tS": round(float(raw_t_s), 6), "values": point})
    return points

samples = normalize_points(payload.get("samples"), "state")
command_samples = normalize_points(payload.get("command_samples"), "action")

raw_duration = payload.get("duration_s")
if isinstance(raw_duration, (int, float)):
    duration_s = float(raw_duration)
elif samples:
    duration_s = float(samples[-1]["tS"])
elif command_samples:
    duration_s = float(command_samples[-1]["tS"])
else:
    duration_s = 0.0

print(
    json.dumps(
        {
            "path": str(path),
            "durationS": round(duration_s, 6),
            "sampleCount": len(samples),
            "commandSampleCount": len(command_samples),
            "armKeys": arm_keys,
            "baseKeys": base_keys,
            "timelineSource": "commands" if command_samples else "state",
            "samples": samples,
            "commandSamples": command_samples,
        }
    )
)
PY
`.trim();

    const { stdout } = await this.execRemoteScript(script, host, settings);
    return JSON.parse(stdout) as RecordingDetail;
  }

  private async downloadRemoteRecordingToLocal(
    settings: AppSettings,
    host: string,
    remotePath: string,
  ): Promise<string> {
    await fs.promises.mkdir(LOCAL_LEADER_REPLAY_DIR, { recursive: true });
    const localPath = path.join(
      LOCAL_LEADER_REPLAY_DIR,
      `${path.basename(remotePath, ".json")}-${formatFileTimestamp()}.json`,
    );
    await this.withSftp(settings, host, async (sftp) => {
      await this.fastGet(sftp, remotePath, localPath);
    });
    this.logActivity(`Downloaded ${remotePath} to ${localPath} for leader replay.`);
    return localPath;
  }

  private async execRemoteScript(
    script: string,
    host: string,
    settings: AppSettings,
  ): Promise<{ stdout: string; stderr: string }> {
    const connection = this.toConnectConfig(settings, host);

    return new Promise((resolve, reject) => {
      const client = new Client();
      let stdout = "";
      let stderr = "";

      client.once("ready", () => {
        client.exec(`/bin/bash -lc ${shellQuote(script)}`, (error, channel) => {
          if (error) {
            client.end();
            reject(error);
            return;
          }

          channel.on("data", (chunk: Buffer) => {
            stdout += chunk.toString("utf8");
          });

          channel.stderr.on("data", (chunk: Buffer) => {
            stderr += chunk.toString("utf8");
          });

          channel.once("close", (code: number | undefined | null) => {
            client.end();
            if (code && code !== 0) {
              reject(new Error(stderr.trim() || stdout.trim() || `Remote command failed with code ${code}.`));
              return;
            }
            resolve({ stdout: stdout.trim(), stderr: stderr.trim() });
          });
        });
      });

      client.once("error", reject);
      client.connect(connection);
    });
  }

  private async withSftp<T>(
    settings: AppSettings,
    host: string,
    callback: (sftp: SFTPWrapper) => Promise<T>,
  ): Promise<T> {
    const connection = this.toConnectConfig(settings, host);
    const client = new Client();

    return new Promise<T>((resolve, reject) => {
      client.once("ready", () => {
        client.sftp(async (error, sftp) => {
          if (error || !sftp) {
            client.end();
            reject(error ?? new Error("SFTP could not be opened."));
            return;
          }

          try {
            const result = await callback(sftp);
            client.end();
            resolve(result);
          } catch (callbackError) {
            client.end();
            reject(callbackError);
          }
        });
      });

      client.once("error", reject);
      client.connect(connection);
    });
  }

  private async fastPut(
    sftp: SFTPWrapper,
    localPath: string,
    remotePath: string,
  ): Promise<void> {
    return new Promise((resolve, reject) => {
      sftp.fastPut(localPath, remotePath, (error?: Error | null) => {
        if (error) {
          reject(error);
          return;
        }
        resolve();
      });
    });
  }

  private async fastGet(
    sftp: SFTPWrapper,
    remotePath: string,
    localPath: string,
  ): Promise<void> {
    return new Promise((resolve, reject) => {
      sftp.fastGet(remotePath, localPath, (error?: Error | null) => {
        if (error) {
          reject(error);
          return;
        }
        resolve();
      });
    });
  }

  private async computeRemoteHelperFingerprint(settings: AppSettings): Promise<string> {
    const localScripts = [
      HOST_SCRIPT,
      POWER_SCRIPT,
      RUNTIME_SCRIPT,
      VEX_BASE_BRIDGE_SCRIPT,
      PI_CALIBRATION_SCRIPT,
      RECORD_SCRIPT,
      REPLAY_SCRIPT,
      DATASET_CAPTURE_SCRIPT,
      POLICY_BENCHMARK_SCRIPT,
      POLICY_EVAL_SCRIPT,
    ];
    const parts = await Promise.all(
      localScripts.map(async (localScript) => {
        const stat = await fs.promises.stat(localScript);
        return `${path.basename(localScript)}:${stat.size}:${Math.trunc(stat.mtimeMs)}`;
      }),
    );
    return `${settings.pi.remoteHelperDir}|${parts.join("|")}`;
  }

  private async writeRecordingLabel(
    settings: AppSettings,
    host: string,
    recordingPath: string,
    label: string,
  ): Promise<void> {
    const script = `
export LEKIWI_TRAJECTORY_DIR=${shellQuote(settings.trajectories.remoteDir)}
export LEKIWI_RECORDING_PATH=${shellQuote(recordingPath)}
export LEKIWI_RECORDING_LABEL=${shellQuote(label)}
python - <<'PY'
import json
import os
from pathlib import Path

root = Path(os.environ["LEKIWI_TRAJECTORY_DIR"]).expanduser().resolve()
path = Path(os.environ["LEKIWI_RECORDING_PATH"]).expanduser().resolve()
label = os.environ["LEKIWI_RECORDING_LABEL"].strip()

if not label:
    raise SystemExit("Recording name cannot be empty.")
if root not in path.parents:
    raise SystemExit("Recording path is outside the trajectory directory.")
if not path.is_file():
    raise SystemExit(f"Recording does not exist: {path}")

payload = json.loads(path.read_text())
if not isinstance(payload, dict):
    raise SystemExit("Recording payload is invalid.")

payload["label"] = label
path.write_text(json.dumps(payload, indent=2) + "\\n")
PY
`.trim();

    await this.execRemoteScript(script, host, settings);
  }

  private async trimRecordingRange(
    settings: AppSettings,
    host: string,
    recordingPath: string,
    trimStartS: number,
    trimEndS: number,
  ): Promise<void> {
    const script = `
export LEKIWI_TRAJECTORY_DIR=${shellQuote(settings.trajectories.remoteDir)}
export LEKIWI_RECORDING_PATH=${shellQuote(recordingPath)}
export LEKIWI_TRIM_START_S=${shellQuote(String(trimStartS))}
export LEKIWI_TRIM_END_S=${shellQuote(String(trimEndS))}
python - <<'PY'
import json
import os
from pathlib import Path

root = Path(os.environ["LEKIWI_TRAJECTORY_DIR"]).expanduser().resolve()
path = Path(os.environ["LEKIWI_RECORDING_PATH"]).expanduser().resolve()
trim_start_s = float(os.environ["LEKIWI_TRIM_START_S"])
trim_end_s = float(os.environ["LEKIWI_TRIM_END_S"])

if root not in path.parents:
    raise SystemExit("Recording path is outside the trajectory directory.")
if not path.is_file():
    raise SystemExit(f"Recording does not exist: {path}")
if trim_start_s < 0 or trim_end_s <= trim_start_s:
    raise SystemExit("Trim end must be greater than trim start.")

payload = json.loads(path.read_text())
samples = payload.get("samples")
if not isinstance(samples, list) or not samples:
    raise SystemExit("Recording does not contain any samples.")

def trim_points(items):
    trimmed_items = []
    if not isinstance(items, list):
        return trimmed_items
    for item in items:
        if not isinstance(item, dict):
            continue
        item_t_s = item.get("t_s")
        if not isinstance(item_t_s, (int, float)):
            continue
        item_t_s = float(item_t_s)
        if trim_start_s <= item_t_s <= trim_end_s:
            trimmed_items.append(
                {
                    **item,
                    "t_s": round(item_t_s - trim_start_s, 6),
                }
            )
    return trimmed_items

trimmed = trim_points(samples)

if not trimmed:
    raise SystemExit("Trim range does not include any recorded samples.")

payload["samples"] = trimmed
if "command_samples" in payload:
    payload["command_samples"] = trim_points(payload.get("command_samples"))
payload["duration_s"] = round(float(trimmed[-1]["t_s"]), 6)
path.write_text(json.dumps(payload, indent=2) + "\\n")
PY
`.trim();

    await this.execRemoteScript(script, host, settings);
  }

  private requireTrainingProfile(training: TrainingConfig, id: string | null): TrainingProfile {
    const profile = training.profiles.find((item) => item.id === id);
    if (!profile) {
      throw new Error("Training profile not found.");
    }
    return profile;
  }

  private saveTrainingConfig(training: TrainingConfig): TrainingConfig {
    const normalized = normalizeTrainingConfig(training, ROOT_DIR);
    this.configStore.saveTraining(normalized);
    return normalized;
  }

  private updateTrainingProfile(
    profileId: string,
    updater: (profile: TrainingProfile) => TrainingProfile,
  ): TrainingConfig {
    const config = this.configStore.getConfig();
    const nextProfiles = config.training.profiles.map((profile) =>
      profile.id === profileId ? normalizeTrainingProfile(updater(profile), ROOT_DIR) : profile,
    );
    return this.saveTrainingConfig({
      ...config.training,
      profiles: nextProfiles,
      selectedProfileId:
        config.training.selectedProfileId && nextProfiles.some((profile) => profile.id === config.training.selectedProfileId)
          ? config.training.selectedProfileId
          : nextProfiles[0]?.id ?? null,
    });
  }

  private async refreshTrainingArtifacts(
    training: TrainingConfig,
    settings: AppSettings,
    resolvedPiHost: string | null,
  ): Promise<TrainingConfig> {
    let next = normalizeTrainingConfig(training, ROOT_DIR);
    let changed = false;

    const trainingSnapshot = this.trainingRunner.getSnapshot();
    next = {
      ...next,
      profiles: next.profiles.map((profile) => {
        const checkpoints = listCheckpointCandidates(profile);
        const latestCheckpoint = checkpoints.at(-1) ?? null;
        const nextProfile: TrainingProfile = {
          ...profile,
          selectedCheckpointPath:
            profile.selectedCheckpointPath || latestCheckpoint || "",
          artifacts: {
            ...profile.artifacts,
            availableCheckpointPaths: checkpoints,
            lastCheckpointPath: latestCheckpoint,
            lastTrainingCompletedAt:
              trainingSnapshot.meta.profileId === profile.id && trainingSnapshot.stoppedAt
                ? trainingSnapshot.stoppedAt
                : profile.artifacts.lastTrainingCompletedAt,
          },
        };

        if (JSON.stringify(nextProfile) !== JSON.stringify(profile)) {
          changed = true;
        }
        return nextProfile;
      }),
    };

    if (next.selectedProfileId) {
      const selectedProfile = this.requireTrainingProfile(next, next.selectedProfileId);
      let summary:
        | {
            episodeCount: number | null;
            cameraKeys: string[];
            lastEpisodeIndex: number | null;
          }
        | null = null;

      if (selectedProfile.captureMode === "leader-as-follower" && fs.existsSync(selectedProfile.macDatasetPath)) {
        summary = await this.inspectLocalDataset(selectedProfile, settings).catch(() => null);
      } else if (resolvedPiHost) {
        summary = await this.fetchRemoteDatasetSummary(settings, resolvedPiHost, selectedProfile).catch(
          () => null,
        );
      } else if (fs.existsSync(selectedProfile.macDatasetPath)) {
        summary = await this.inspectLocalDataset(selectedProfile, settings).catch(() => null);
      }

      if (summary) {
        next = {
          ...next,
          profiles: next.profiles.map((profile) =>
            profile.id === selectedProfile.id
              ? {
                  ...profile,
                  artifacts: {
                    ...profile.artifacts,
                    datasetEpisodeCount: summary.episodeCount,
                    datasetCameraKeys: summary.cameraKeys,
                    lastDatasetEpisodeIndex: summary.lastEpisodeIndex,
                  },
                }
              : profile,
          ),
        };
        changed = true;
      }
    }

    if (changed) {
      next = this.saveTrainingConfig(next);
    }
    return next;
  }

  private async stopRobotExclusiveProcesses(reason: string): Promise<void> {
    await this.teleopRunner.stop(reason);
    await this.replayRunner.stop(reason);
    await this.localReplayRunner.stop(reason);
    await this.hostRunner.stop(reason);
    await this.piCalibrationRunner.stop(reason);
    await this.macCalibrationRunner.stop(reason);
    await this.datasetCaptureRunner.stop(reason);
    await this.localDatasetCaptureRunner.stop(reason);
    await this.policyEvalRunner.stop(reason);
  }

  private async inspectLocalDataset(
    profile: TrainingProfile,
    settings: AppSettings,
  ): Promise<{
    episodeCount: number | null;
    cameraKeys: string[];
    lastEpisodeIndex: number | null;
  }> {
    if (!fs.existsSync(profile.macDatasetPath)) {
      return {
        episodeCount: null,
        cameraKeys: [],
        lastEpisodeIndex: null,
      };
    }

    const script = [
      `export LEKIWI_DATASET_ROOT=${shellQuote(profile.macDatasetPath)}`,
      `export LEKIWI_DATASET_REPO_ID=${shellQuote(deriveDatasetRepoId(profile))}`,
      `source ${shellQuote(settings.mac.condaScript)}`,
      "conda activate lerobot",
      "python - <<'PY'",
      "import json",
      "import os",
      "from pathlib import Path",
      "from lerobot.datasets.lerobot_dataset import LeRobotDataset",
      "root = Path(os.environ['LEKIWI_DATASET_ROOT']).expanduser()",
      "repo_id = os.environ['LEKIWI_DATASET_REPO_ID']",
      "summary = {'episodeCount': None, 'cameraKeys': [], 'lastEpisodeIndex': None}",
      "if root.exists():",
      "    dataset = LeRobotDataset(repo_id, root=root)",
      "    summary['episodeCount'] = int(dataset.num_episodes)",
      "    summary['cameraKeys'] = list(getattr(dataset.meta, 'camera_keys', []))",
      "    summary['lastEpisodeIndex'] = dataset.num_episodes - 1 if dataset.num_episodes > 0 else None",
      "print(json.dumps(summary))",
      "PY",
    ].join("\n");

    const { stdout } = await runZshScript(script);
    return JSON.parse(stdout.trim()) as {
      episodeCount: number | null;
      cameraKeys: string[];
      lastEpisodeIndex: number | null;
    };
  }

  private async fetchRemoteDatasetSummary(
    settings: AppSettings,
    host: string,
    profile: TrainingProfile,
  ): Promise<{
    episodeCount: number | null;
    cameraKeys: string[];
    lastEpisodeIndex: number | null;
  }> {
    const script = [
      `export LEKIWI_DATASET_ROOT=${shellQuote(profile.piDatasetPath)}`,
      `export LEKIWI_DATASET_REPO_ID=${shellQuote(deriveDatasetRepoId(profile))}`,
      `source ${shellQuote(settings.pi.condaScript)}`,
      "conda activate lerobot",
      "python - <<'PY'",
      "import json",
      "import os",
      "from pathlib import Path",
      "from lerobot.datasets.lerobot_dataset import LeRobotDataset",
      "root = Path(os.environ['LEKIWI_DATASET_ROOT']).expanduser()",
      "repo_id = os.environ['LEKIWI_DATASET_REPO_ID']",
      "summary = {'episodeCount': None, 'cameraKeys': [], 'lastEpisodeIndex': None}",
      "if root.exists():",
      "    dataset = LeRobotDataset(repo_id, root=root)",
      "    summary['episodeCount'] = int(dataset.num_episodes)",
      "    summary['cameraKeys'] = list(getattr(dataset.meta, 'camera_keys', []))",
      "    summary['lastEpisodeIndex'] = dataset.num_episodes - 1 if dataset.num_episodes > 0 else None",
      "print(json.dumps(summary))",
      "PY",
    ].join("\n");

    const result = await this.execRemoteScript(script, host, settings);
    return JSON.parse(result.stdout) as {
      episodeCount: number | null;
      cameraKeys: string[];
      lastEpisodeIndex: number | null;
    };
  }

  private async syncRemoteDirectoryToLocal(
    settings: AppSettings,
    host: string,
    remoteDir: string,
    localDir: string,
    log: (stream: "stdout" | "stderr" | "system", line: string) => void,
  ): Promise<void> {
    await fs.promises.rm(localDir, { recursive: true, force: true });
    await fs.promises.mkdir(localDir, { recursive: true });

    const connection = this.toConnectConfig(settings, host);
    await new Promise<void>((resolve, reject) => {
      const client = new Client();
      client.once("ready", () => {
        const remoteCommand = [
          `if [ ! -d ${shellQuote(remoteDir)} ]; then`,
          `  echo "Missing directory: ${remoteDir}" >&2`,
          "  exit 1",
          "fi",
          `tar -C ${shellQuote(remoteDir)} -czf - .`,
        ].join("\n");

        client.exec(`/bin/bash -lc ${shellQuote(remoteCommand)}`, (error, channel) => {
          if (error) {
            client.end();
            reject(error);
            return;
          }

          const untar = spawn("/usr/bin/tar", ["-xzf", "-", "-C", localDir], {
            stdio: ["pipe", "ignore", "pipe"],
          });

          channel.stderr.on("data", (chunk: Buffer) => {
            log("stderr", chunk.toString("utf8").trim());
          });
          untar.stderr.on("data", (chunk: Buffer) => {
            log("stderr", chunk.toString("utf8").trim());
          });

          channel.pipe(untar.stdin);

          untar.once("close", (code) => {
            client.end();
            if (code && code !== 0) {
              reject(new Error(`Local tar extraction failed with code ${code}.`));
              return;
            }
            resolve();
          });

          channel.once("close", (code: number | undefined | null) => {
            if (code && code !== 0) {
              client.end();
              reject(new Error(`Remote dataset archive failed with code ${code}.`));
            }
          });
        });
      });

      client.once("error", reject);
      client.connect(connection);
    });
  }

  private async syncLocalDirectoryToRemote(
    settings: AppSettings,
    host: string,
    localDir: string,
    remoteDir: string,
    log: (stream: "stdout" | "stderr" | "system", line: string) => void,
  ): Promise<void> {
    if (!fs.existsSync(localDir)) {
      throw new Error(`Local directory does not exist: ${localDir}`);
    }

    const connection = this.toConnectConfig(settings, host);
    await new Promise<void>((resolve, reject) => {
      const client = new Client();
      client.once("ready", () => {
        const remoteCommand = [
          `rm -rf ${shellQuote(remoteDir)}`,
          `mkdir -p ${shellQuote(remoteDir)}`,
          `tar -xzf - -C ${shellQuote(remoteDir)}`,
        ].join("\n");

        client.exec(`/bin/bash -lc ${shellQuote(remoteCommand)}`, (error, channel) => {
          if (error) {
            client.end();
            reject(error);
            return;
          }

          const tarProcess = spawn("/usr/bin/tar", ["-czf", "-", "-C", localDir, "."], {
            stdio: ["ignore", "pipe", "pipe"],
          });

          tarProcess.stderr.on("data", (chunk: Buffer) => {
            log("stderr", chunk.toString("utf8").trim());
          });
          channel.stderr.on("data", (chunk: Buffer) => {
            log("stderr", chunk.toString("utf8").trim());
          });

          tarProcess.stdout.pipe(channel);

          tarProcess.once("close", (code) => {
            if (code && code !== 0) {
              client.end();
              reject(new Error(`Local tar archive failed with code ${code}.`));
            }
          });

          channel.once("close", (code: number | undefined | null) => {
            client.end();
            if (code && code !== 0) {
              reject(new Error(`Remote extract failed with code ${code}.`));
              return;
            }
            resolve();
          });
        });
      });

      client.once("error", reject);
      client.connect(connection);
    });
  }

  private async writeRemoteDeployManifest(
    settings: AppSettings,
    host: string,
    profile: TrainingProfile,
    sourceCheckpointPath: string,
  ): Promise<void> {
    const payload = {
      profileId: profile.id,
      name: profile.name,
      sourceCheckpointPath,
      deployedCheckpointPath: profile.piDeployPath,
      deployedAt: new Date().toISOString(),
      task: profile.task,
      policyType: profile.policyType,
    };
    const manifestPath = path.posix.join(profile.piDeployPath, "deploy-manifest.json");
    const script = `
mkdir -p ${shellQuote(profile.piDeployPath)}
cat > ${shellQuote(manifestPath)} <<'JSON'
${JSON.stringify(payload, null, 2)}
JSON
`.trim();

    await this.execRemoteScript(script, host, settings);
  }

  private logActivity(message: string): void {
    const stamped = `${new Date().toLocaleTimeString()} ${message}`;
    this.activityLog.push(stamped);
    if (this.activityLog.length > MAX_ACTIVITY_LINES) {
      this.activityLog.splice(0, this.activityLog.length - MAX_ACTIVITY_LINES);
    }
  }
}
