import fs from "node:fs";
import path from "node:path";
import crypto from "node:crypto";
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
import {
  nextMoveVersionNumber,
  withFavoriteVersion,
} from "./betaMoves.js";
import {
  errorMessage,
  isTransientRemoteTransportError,
} from "./transportErrors.js";
import type {
  AdjustRecordingServoRequest,
  ActiveArmHold,
  ArmHomeMode,
  ArmHomePosition,
  AppSettings,
  BenchmarkPolicyRequest,
  ChainLink,
  ControlAuthorityState,
  CreateMoveRecordingVersionRequest,
  CreatePinnedMoveRequest,
  DashboardState,
  DeployTrainingCheckpointRequest,
  DeleteTrainingProfileRequest,
  DeleteRecordingRequest,
  DuplicateRecordingRequest,
  GamePlan,
  LeaderStatus,
  MarkRecordingGyroZeroRequest,
  PinnedMove,
  RecordingDetail,
  RecordingDetailRequest,
  RenameRecordingRequest,
  RecordingEntry,
  RecordingReplayOptions,
  ResolveLeaderStaleRequest,
  ResetRecordingUltrasonicPositionRequest,
  ReplayRequest,
  RobotSensorState,
  RobotSensorStatus,
  RobotSensorsStatus,
  SaveChainLinkRequest,
  SelectTrainingProfileRequest,
  SetArmHomeFromRecordingRequest,
  SetMoveFavoriteVersionRequest,
  SetRecordingReplayOptionsRequest,
  ServiceSnapshot,
  ServoCalibrationRequest,
  StartControlRequest,
  StartPolicyEvalRequest,
  StartRecordingRequest,
  StartTrainingCaptureRequest,
  StartTrainingRunRequest,
  StartTrainingSyncRequest,
  TrainingArtifact,
  TrainingConfig,
  TrainingProfile,
  TrimRecordingRequest,
  TorqueLimitsRequest,
  UpdateMoveRecordingVersionRequest,
  UpdatePinnedMoveRequest,
  VexBrainStatus,
} from "./types.js";

type VexKeyboardControlMode = "drive" | "ecu";

const ROOT_DIR = process.cwd();
const HOST_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_host.py");
const POWER_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_power.py");
const RUNTIME_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_runtime.py");
const VEX_BASE_BRIDGE_SCRIPT = path.join(ROOT_DIR, "scripts", "vex_base_bridge.py");
const SENSOR_REPLAY_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_sensor_replay.py");
const PI_CALIBRATION_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_calibrate.py");
const RECORD_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_record_trajectory.py");
const REPLAY_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_replay_trajectory.py");
const UI_TELEOP_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_ui_teleop.py");
const KEYBOARD_TELEOP_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_keyboard_teleop.py");
const CONTROL_LOOP_HZ = 90;
const KEYBOARD_TELEOP_FPS = 90;
const KEYBOARD_TELEOP_PRINT_EVERY = 30;
const KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS = 0.06;
const KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS = 18;
const KEYBOARD_TELEOP_READY_TIMEOUT_MS = 15000;
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
const LOCAL_LEADER_MIMIC_POSE_SCRIPT = path.join(
  ROOT_DIR,
  "scripts",
  "lekiwi_leader_mimic_pose.py",
);
const POLICY_BENCHMARK_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_benchmark_policy.py");
const POLICY_EVAL_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_run_policy_eval.py");
const LOCAL_LEADER_REPLAY_DIR = path.join(ROOT_DIR, ".tmp", "leader-replay");
const LOCAL_RECORDING_DETAIL_DIR = path.join(ROOT_DIR, ".tmp", "recording-detail");
const LOCAL_POWER_LOG_DIR = path.join(ROOT_DIR, ".lekiwi-ui", "power-logs");
const LOCAL_LEADER_ROBOT_ID = "leader";
const MAX_ACTIVITY_LINES = 220;
const HOST_READY_TIMEOUT_MS = 15000;
const HOST_READY_PORTS = [5555, 5556];
const TRAINING_METADATA_REFRESH_MS = 15000;
const POWER_LOG_SYNC_INTERVAL_MS = 15000;
const VEX_STATUS_REFRESH_MS = 5000;
const LIVE_SENSOR_STATUS_STALE_MS = 6000;
const SENSOR_STATUS_LOG_PREFIX = "[sensor-status]";
const HOME_COMMAND_LOG_PREFIX = "[home-command]";
const REMOTE_HANDSHAKE_RETRY_DELAYS_MS = [500, 1200, 2500];
const DEFAULT_VEX_POSITIONING_TIMEOUT_S = 8;
const MIN_VEX_POSITIONING_TIMEOUT_S = 0.5;
const MAX_VEX_POSITIONING_TIMEOUT_S = 60;
const DEFAULT_VEX_POSITIONING_SPEED = 1;
const MIN_VEX_POSITIONING_SPEED = 0.1;
const MAX_VEX_POSITIONING_SPEED = 3;
const DEFAULT_VEX_POSITIONING_XY_TOLERANCE_M = 0.02;
const DEFAULT_VEX_POSITIONING_HEADING_TOLERANCE_DEG = 1.5;
const DEFAULT_VEX_POSITIONING_XY_TRIM_TOLERANCE_M = 0.05;
const DEFAULT_VEX_POSITIONING_HEADING_TRIM_TOLERANCE_DEG = 8.5;
const MIN_VEX_POSITIONING_XY_TOLERANCE_M = 0.001;
const MAX_VEX_POSITIONING_XY_TOLERANCE_M = 2;
const MIN_VEX_POSITIONING_HEADING_TOLERANCE_DEG = 0.1;
const MAX_VEX_POSITIONING_HEADING_TOLERANCE_DEG = 90;
const ARM_MOTORS = [
  "arm_shoulder_pan",
  "arm_shoulder_lift",
  "arm_elbow_flex",
  "arm_wrist_flex",
  "arm_wrist_roll",
  "arm_gripper",
] as const;
const ARM_HOME_JOINT_KEYS = ARM_MOTORS.map((motor) => `${motor}.pos`);
const DUMMY_RECORDING_PREFIX = "dummy://recordings/";
const DUMMY_RECORDING_DEFINITIONS = [
  {
    slug: "offline-pick-place",
    name: "Dummy: Offline pick and place",
    fileName: "dummy-offline-pick-place.json",
    durationS: 6.4,
    modifiedAt: "2026-01-01T12:00:00.000Z",
  },
  {
    slug: "offline-ecu-align",
    name: "Dummy: ECU alignment check",
    fileName: "dummy-ecu-alignment.json",
    durationS: 4.8,
    modifiedAt: "2026-01-01T12:05:00.000Z",
  },
  {
    slug: "offline-servo-tune",
    name: "Dummy: Servo tuning pass",
    fileName: "dummy-servo-tune.json",
    durationS: 8.2,
    modifiedAt: "2026-01-01T12:10:00.000Z",
  },
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
  const target = payload.target === "leader" ? "leader" : "pi";
  const startTimeS = normalizeReplayStartTime(payload.startTimeS);
  const startsMidRecording = startTimeS > 0;
  return {
    trajectoryPath: payload.trajectoryPath.trim(),
    target,
    vexReplayMode: payload.vexReplayMode === "drive" ? "drive" : "ecu",
    homeMode:
      target === "pi"
        ? startsMidRecording
          ? normalizeMidReplayHomeMode(payload.homeMode)
          : normalizeHomeMode(payload.homeMode)
        : "none",
    speed: normalizeReplaySpeed(payload.speed, defaults),
    autoVexPositioning:
      target === "pi" && !startsMidRecording ? payload.autoVexPositioning === true : false,
    vexPositioningSpeed:
      target === "pi"
        ? normalizeVexPositioningSpeed(payload.vexPositioningSpeed)
        : DEFAULT_VEX_POSITIONING_SPEED,
    vexPositioningTimeoutS:
      target === "pi"
        ? normalizeVexPositioningTimeout(payload.vexPositioningTimeoutS)
        : DEFAULT_VEX_POSITIONING_TIMEOUT_S,
    vexPositioningXyToleranceM:
      target === "pi"
        ? normalizeVexPositioningXyTolerance(payload.vexPositioningXyToleranceM)
        : DEFAULT_VEX_POSITIONING_XY_TOLERANCE_M,
    vexPositioningHeadingToleranceDeg:
      target === "pi"
        ? normalizeVexPositioningHeadingTolerance(payload.vexPositioningHeadingToleranceDeg)
        : DEFAULT_VEX_POSITIONING_HEADING_TOLERANCE_DEG,
    vexPositioningXyTrimToleranceM:
      target === "pi"
        ? normalizeVexPositioningXyTrimTolerance(payload.vexPositioningXyTrimToleranceM)
        : DEFAULT_VEX_POSITIONING_XY_TRIM_TOLERANCE_M,
    vexPositioningHeadingTrimToleranceDeg:
      target === "pi"
        ? normalizeVexPositioningHeadingTrimTolerance(payload.vexPositioningHeadingTrimToleranceDeg)
        : DEFAULT_VEX_POSITIONING_HEADING_TRIM_TOLERANCE_DEG,
    includeBase: target === "leader" ? false : Boolean(payload.includeBase),
    holdFinalS:
      payload.holdFinalS >= 0 ? payload.holdFinalS : defaults.defaultHoldFinalS,
    startTimeS,
  };
}

function normalizeReplayStartTime(value: unknown): number {
  const numeric = Number(value);
  return Number.isFinite(numeric) && numeric > 0 ? numeric : 0;
}

function normalizeMidReplayHomeMode(value: unknown): ArmHomeMode {
  const homeMode = normalizeHomeMode(value);
  if (homeMode === "both") {
    return "end";
  }
  if (homeMode === "start") {
    return "none";
  }
  return homeMode;
}

function normalizeReplaySpeed(
  value: unknown,
  defaults: AppSettings["trajectories"],
): number {
  const numeric = Number(value);
  return Number.isFinite(numeric) && numeric > 0 ? numeric : defaults.defaultReplaySpeed;
}

function normalizeVexPositioningSpeed(value: unknown): number {
  const numeric = Number(value);
  if (!Number.isFinite(numeric) || numeric <= 0) {
    return DEFAULT_VEX_POSITIONING_SPEED;
  }
  return Math.min(MAX_VEX_POSITIONING_SPEED, Math.max(MIN_VEX_POSITIONING_SPEED, numeric));
}

function normalizeRecordingReplayOptions(
  value: Partial<RecordingReplayOptions> | undefined,
  defaults: AppSettings["trajectories"],
): RecordingReplayOptions {
  return {
    homeMode: normalizeHomeMode(value?.homeMode),
    speed: normalizeReplaySpeed(value?.speed, defaults),
    autoVexPositioning: value?.autoVexPositioning === true,
    vexPositioningSpeed: normalizeVexPositioningSpeed(value?.vexPositioningSpeed),
    vexPositioningTimeoutS: normalizeVexPositioningTimeout(value?.vexPositioningTimeoutS),
    vexPositioningXyToleranceM: normalizeVexPositioningXyTolerance(
      value?.vexPositioningXyToleranceM,
    ),
    vexPositioningHeadingToleranceDeg: normalizeVexPositioningHeadingTolerance(
      value?.vexPositioningHeadingToleranceDeg,
    ),
    vexPositioningXyTrimToleranceM: normalizeVexPositioningXyTrimTolerance(
      value?.vexPositioningXyTrimToleranceM,
    ),
    vexPositioningHeadingTrimToleranceDeg: normalizeVexPositioningHeadingTrimTolerance(
      value?.vexPositioningHeadingTrimToleranceDeg,
    ),
  };
}

function recordingReplayOptionsAreDefault(
  value: RecordingReplayOptions,
  defaults: AppSettings["trajectories"],
): boolean {
  return (
    value.homeMode === "none" &&
    Math.abs(value.speed - defaults.defaultReplaySpeed) < 0.000001 &&
    !value.autoVexPositioning &&
    Math.abs(value.vexPositioningSpeed - DEFAULT_VEX_POSITIONING_SPEED) < 0.000001 &&
    Math.abs(value.vexPositioningTimeoutS - DEFAULT_VEX_POSITIONING_TIMEOUT_S) < 0.000001 &&
    Math.abs(value.vexPositioningXyToleranceM - DEFAULT_VEX_POSITIONING_XY_TOLERANCE_M) < 0.000001 &&
    Math.abs(value.vexPositioningHeadingToleranceDeg - DEFAULT_VEX_POSITIONING_HEADING_TOLERANCE_DEG) < 0.000001 &&
    Math.abs(value.vexPositioningXyTrimToleranceM - DEFAULT_VEX_POSITIONING_XY_TRIM_TOLERANCE_M) < 0.000001 &&
    Math.abs(value.vexPositioningHeadingTrimToleranceDeg - DEFAULT_VEX_POSITIONING_HEADING_TRIM_TOLERANCE_DEG) < 0.000001
  );
}

function normalizeHomeMode(value: unknown): ArmHomeMode {
  return value === "start" || value === "end" || value === "both" ? value : "none";
}

function normalizeVexPositioningTimeout(value: unknown): number {
  return normalizeBoundedNumber(
    value,
    DEFAULT_VEX_POSITIONING_TIMEOUT_S,
    MIN_VEX_POSITIONING_TIMEOUT_S,
    MAX_VEX_POSITIONING_TIMEOUT_S,
  );
}

function normalizeVexPositioningXyTolerance(value: unknown): number {
  return normalizeBoundedNumber(
    value,
    DEFAULT_VEX_POSITIONING_XY_TOLERANCE_M,
    MIN_VEX_POSITIONING_XY_TOLERANCE_M,
    MAX_VEX_POSITIONING_XY_TOLERANCE_M,
  );
}

function normalizeVexPositioningHeadingTolerance(value: unknown): number {
  return normalizeBoundedNumber(
    value,
    DEFAULT_VEX_POSITIONING_HEADING_TOLERANCE_DEG,
    MIN_VEX_POSITIONING_HEADING_TOLERANCE_DEG,
    MAX_VEX_POSITIONING_HEADING_TOLERANCE_DEG,
  );
}

function normalizeVexPositioningXyTrimTolerance(value: unknown): number {
  return normalizeBoundedNumber(
    value,
    DEFAULT_VEX_POSITIONING_XY_TRIM_TOLERANCE_M,
    MIN_VEX_POSITIONING_XY_TOLERANCE_M,
    MAX_VEX_POSITIONING_XY_TOLERANCE_M,
  );
}

function normalizeVexPositioningHeadingTrimTolerance(value: unknown): number {
  return normalizeBoundedNumber(
    value,
    DEFAULT_VEX_POSITIONING_HEADING_TRIM_TOLERANCE_DEG,
    MIN_VEX_POSITIONING_HEADING_TOLERANCE_DEG,
    MAX_VEX_POSITIONING_HEADING_TOLERANCE_DEG,
  );
}

function normalizeBoundedNumber(value: unknown, fallback: number, min: number, max: number): number {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) {
    return fallback;
  }
  return Math.min(max, Math.max(min, numeric));
}

function isFiniteHomePosition(homePosition: ArmHomePosition | null): homePosition is ArmHomePosition {
  return Boolean(
    homePosition &&
      ARM_HOME_JOINT_KEYS.every((key) => Number.isFinite(homePosition.joints[key])),
  );
}

function isIncompleteJsonOutputError(error: unknown): boolean {
  const message = error instanceof Error ? error.message : "";
  return (
    (error instanceof SyntaxError || /JSON/i.test(message)) &&
    /Unexpected end of JSON input|Expected ',' or '}' after property value|Unterminated string in JSON/i.test(message)
  );
}

function isDummyRecordingPath(recordingPath: string): boolean {
  return recordingPath.startsWith(DUMMY_RECORDING_PREFIX);
}

function hasDummyRecordingPath<T extends { trajectoryPath: string }>(item: T): boolean {
  return isDummyRecordingPath(item.trajectoryPath);
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
  private remoteStateRefreshInFlight = false;
  private exclusiveOperationInFlight = false;
  private suppressRemoteTransportErrorsUntil = 0;
  private vexKeyboardControlMode: VexKeyboardControlMode = "drive";
  private activityLog: string[] = [];
  private queue: Promise<void> = Promise.resolve();
  private readonly pinnedMoveTriggersInFlight = new Set<string>();
  private readonly activeRemoteClients = new Set<Client>();
  private readonly remoteHelperSyncCache = new Map<string, string>();
  private readonly remoteTorqueCache = new Map<string, string>();
  private readonly remotePowerLogCache = new Map<string, string>();
  private coupledTeleopCleanup: Promise<void> | null = null;
  private replayControlRestore:
    | {
        requestedAtMs: number;
        target: ReplayRequest["target"];
        trajectoryPath: string;
        restoreArmSource: "leader" | "keyboard" | "none" | null;
        returnToActiveHold: boolean;
      }
    | null = null;
  private replayControlRestoreInFlight: Promise<void> | null = null;
  private activeArmHold: ActiveArmHold | null = null;
  private keyboardOverrideArmHold: ActiveArmHold | null = null;
  private leaderPoseStale = false;
  private leaderPoseStaleReason: string | null = null;
  private runtimeSafetyLatched = false;
  private runtimeSafetyLatchReason: string | null = null;
  private runtimeSafetyLatchClearedAtMs = 0;
  private cachedVexBrainStatus: VexBrainStatus = {
    connected: false,
    telemetryActive: false,
    consolePort: null,
    commPort: null,
    source: null,
    message: "VEX Brain is not connected to the Pi.",
  };

  async getState(): Promise<DashboardState> {
    const {
      settings,
      pinnedMoves,
      chainLinks,
      homePosition,
      recordingReplayOptions,
      moveDefinitions,
      moveRecordingVersions,
      gamePlans,
      training,
    } = this.configStore.getConfig();
    const wifi = await getWifiStatus();
    const hostSnapshot = this.hostRunner.getSnapshot();
    const teleopSnapshot = this.teleopRunner.getSnapshot();
    const replaySnapshot = this.replayRunner.getSnapshot();
    const datasetCaptureSnapshot = this.datasetCaptureRunner.getSnapshot();
    const warmHost =
      hostSnapshot.state === "running" && this.hostSupportsInlineReplay(hostSnapshot.mode)
        ? this.getActiveHostAddress()
        : null;
    const resolvedPiHost = warmHost ?? (await this.findReachablePiHost(settings));
    const piReachable = Boolean(resolvedPiHost);
    const leader = await this.getLeaderStatus(settings);
    if (resolvedPiHost) {
      this.lastResolvedHost = resolvedPiHost;
    }
    if (piReachable && this.isPiConnectivityError(this.lastError)) {
      this.lastError = null;
    }

    if (!resolvedPiHost) {
      const offline = this.defaultVexBrainStatus("Pi is offline, so VEX Brain status is unavailable.");
      this.cachedVexBrainStatus = offline;
      this.lastVexBrainStatusRefresh = Date.now();
    } else if (!this.exclusiveOperationInFlight && !this.remoteStateRefreshInFlight) {
      this.startRemoteStateRefresh(
        settings,
        resolvedPiHost,
        hostSnapshot,
        replaySnapshot,
        datasetCaptureSnapshot,
        training,
      );
    }

    const vexBrain = this.cachedVexBrainStatus;
    const robotSensors = this.getRobotSensorsStatus(
      piReachable,
      vexBrain,
      hostSnapshot,
      replaySnapshot,
      datasetCaptureSnapshot,
    );
    const visiblePinnedMoves = piReachable
      ? pinnedMoves.filter((move) => !hasDummyRecordingPath(move))
      : pinnedMoves;
    if (
      this.activeArmHold &&
      !pinnedMoves.some(
        (move) => move.id === this.activeArmHold?.pinnedMoveId && move.holdArmPose,
      )
    ) {
      this.activeArmHold = null;
    }
    if (
      this.keyboardOverrideArmHold &&
      !pinnedMoves.some(
        (move) => move.id === this.keyboardOverrideArmHold?.pinnedMoveId && move.holdArmPose,
      )
    ) {
      this.keyboardOverrideArmHold = null;
    }
    const visibleChainLinks = piReachable
      ? chainLinks
          .map((chain) => ({
            ...chain,
            items: chain.items.filter((item) => !hasDummyRecordingPath(item)),
          }))
          .filter((chain) => chain.items.length > 0)
      : chainLinks;

    const selectedProfile =
      training.profiles.find((profile) => profile.id === training.selectedProfileId) ?? null;
    this.maybeCleanupCoupledTeleopFailure(hostSnapshot, teleopSnapshot);
    const warmHostReplaySnapshot = this.deriveWarmHostReplaySnapshot(hostSnapshot);
    this.maybeRestoreControlAfterReplay(
      settings,
      replaySnapshot,
      this.localReplayRunner.getSnapshot(),
      warmHostReplaySnapshot,
    );
    const effectiveReplaySnapshot = this.pickServiceSnapshot(
      replaySnapshot,
      this.localReplayRunner.getSnapshot(),
      warmHostReplaySnapshot,
    );
    const effectiveDatasetCaptureSnapshot = this.pickServiceSnapshot(
      datasetCaptureSnapshot,
      this.localDatasetCaptureRunner.getSnapshot(),
    );
    this.updateRuntimeSafetyLatchFromSnapshots([
      hostSnapshot,
      teleopSnapshot,
      replaySnapshot,
      this.localReplayRunner.getSnapshot(),
      this.piCalibrationRunner.getSnapshot(),
      this.macCalibrationRunner.getSnapshot(),
      datasetCaptureSnapshot,
      this.localDatasetCaptureRunner.getSnapshot(),
      this.policyEvalRunner.getSnapshot(),
    ]);

    return {
      settings,
      pinnedMoves: visiblePinnedMoves,
      chainLinks: visibleChainLinks,
      homePosition,
      activeArmHold: this.activeArmHold,
      keyboardOverrideArmHold: this.keyboardOverrideArmHold,
      controlAuthority: this.buildControlAuthorityState(
        leader,
        vexBrain,
        hostSnapshot,
        teleopSnapshot,
        effectiveReplaySnapshot,
        this.piCalibrationRunner.getSnapshot(),
        this.macCalibrationRunner.getSnapshot(),
      ),
      recordingReplayOptions,
      moveDefinitions,
      moveRecordingVersions,
      gamePlans,
      training: {
        ...training,
        selectedProfile,
      },
      wifi,
      piReachable,
      resolvedPiHost,
      leader,
      vexBrain,
      robotSensors,
      recordings: piReachable ? this.cachedRecordings : this.buildDummyRecordingEntries(),
      lastError: this.lastError,
      activityLog: [...this.activityLog],
      services: {
        host: hostSnapshot,
        teleop: teleopSnapshot,
        replay: effectiveReplaySnapshot,
        piCalibration: this.piCalibrationRunner.getSnapshot(),
        macCalibration: this.macCalibrationRunner.getSnapshot(),
        datasetCapture: effectiveDatasetCaptureSnapshot,
        datasetSync: this.datasetSyncRunner.getSnapshot(),
        training: this.trainingRunner.getSnapshot(),
        deployment: this.deploymentRunner.getSnapshot(),
        policyBenchmark: this.policyBenchmarkRunner.getSnapshot(),
        policyEval: this.policyEvalRunner.getSnapshot(),
      },
    };
  }

  private buildControlAuthorityState(
    leader: LeaderStatus,
    vexBrain: VexBrainStatus,
    hostSnapshot: ServiceSnapshot,
    teleopSnapshot: ServiceSnapshot,
    replaySnapshot: ServiceSnapshot,
    piCalibrationSnapshot: ServiceSnapshot,
    macCalibrationSnapshot: ServiceSnapshot,
  ): ControlAuthorityState {
    this.rememberVexControlModeFromSnapshot(hostSnapshot);
    this.rememberVexControlModeFromSnapshot(teleopSnapshot);
    const hostMode = hostSnapshot.mode;
    const teleopMode = teleopSnapshot.mode;
    const hostActive =
      hostSnapshot.state === "running" || hostSnapshot.state === "starting" || hostSnapshot.state === "stopping";
    const replayRunning = replaySnapshot.state === "running" || replaySnapshot.state === "starting";
    const keyboardCaptureActive =
      teleopSnapshot.state === "running" || teleopSnapshot.state === "starting";
    const liveBaseSource =
      this.normalizeBaseSourceMeta(teleopSnapshot.meta.baseSource) ??
      this.normalizeBaseSourceMeta(hostSnapshot.meta.baseSource);
    const safetyLatched = this.runtimeSafetyLatched;
    let arm: ControlAuthorityState["arm"] = "none";
    let base: ControlAuthorityState["base"] = "none";
    const speedPreset = this.vexKeyboardControlMode;

    if (safetyLatched) {
      return {
        arm: "none",
        base: "none",
        activeHoldId: this.activeArmHold?.pinnedMoveId ?? null,
        activeHoldName: this.activeArmHold?.name ?? null,
        leaderConnected: leader.connected,
        keyboardCaptureActive,
        vexTelemetryActive: vexBrain.telemetryActive,
        safetyLatched,
        leaderPoseStale: this.leaderPoseStale,
        leaderPoseStaleReason: this.leaderPoseStaleReason,
        speedPreset,
      };
    }

    if (piCalibrationSnapshot.state === "running" || macCalibrationSnapshot.state === "running") {
      arm = "calibration";
    } else if (replayRunning) {
      arm = "replay";
    } else if (hostActive && hostMode === "recording") {
      arm = hostSnapshot.meta.recordingInputMode === "free-teach" ? "handGuide" : "recording";
    } else if (this.activeArmHold && teleopSnapshot.meta.armInput === "disabled") {
      arm = "holdPose";
    } else if ((hostActive && hostSnapshot.meta.armSource === "leader") || teleopSnapshot.meta.armSource === "leader") {
      arm = "leader";
    } else if ((hostActive && hostSnapshot.meta.armSource === "keyboard") || teleopSnapshot.meta.armSource === "keyboard") {
      arm = "keyboard";
    } else if (this.activeArmHold) {
      arm = "holdPose";
    }

    if (replayRunning && replaySnapshot.meta.autoVexPositioning === true) {
      base = "vexPreposition";
    } else if (replayRunning && replaySnapshot.meta.includeBase === true) {
      base = "vexReplay";
    } else if (keyboardCaptureActive && liveBaseSource !== "vexController") {
      base = "keyboard";
    } else if (
      liveBaseSource === "vexController" ||
      (hostActive && hostSnapshot.meta.vexLiveBaseControl === true) ||
      vexBrain.telemetryActive
    ) {
      base = "vexController";
    } else if (this.activeArmHold) {
      base = "hold";
    }

    return {
      arm,
      base,
      activeHoldId: this.activeArmHold?.pinnedMoveId ?? null,
      activeHoldName: this.activeArmHold?.name ?? null,
      leaderConnected: leader.connected,
      keyboardCaptureActive,
      vexTelemetryActive: vexBrain.telemetryActive,
      safetyLatched,
      leaderPoseStale: this.leaderPoseStale,
      leaderPoseStaleReason: this.leaderPoseStaleReason,
      speedPreset,
    };
  }

  private markLeaderPoseStale(reason: string): void {
    this.leaderPoseStale = true;
    this.leaderPoseStaleReason = reason;
    this.logActivity(`Leader pose marked stale: ${reason}`);
  }

  private normalizeVexControlMode(value: unknown): VexKeyboardControlMode | null {
    if (value === "drive" || value === "ecu") {
      return value;
    }
    if (typeof value === "string") {
      const normalized = value.trim().toLowerCase();
      if (normalized === "drive" || normalized === "ecu") {
        return normalized;
      }
    }
    return null;
  }

  private rememberVexControlModeFromSnapshot(snapshot: ServiceSnapshot): void {
    const metaMode = this.normalizeVexControlMode(snapshot.meta.vexControlMode);
    if (metaMode) {
      this.vexKeyboardControlMode = metaMode;
    }

    for (let index = snapshot.logs.length - 1; index >= 0; index -= 1) {
      const line = snapshot.logs[index] ?? "";
      const switched = line.match(/VEX base mode switched to\s+(DRIVE|ECU)\s+speed/i);
      const started = line.match(/VEX base mode:\s+(DRIVE|ECU)\b/i);
      const mode = this.normalizeVexControlMode(switched?.[1] ?? started?.[1]);
      if (mode) {
        this.vexKeyboardControlMode = mode;
        return;
      }
    }
  }

  private clearLeaderPoseStale(reason: string): void {
    if (this.leaderPoseStale) {
      this.logActivity(`Leader stale state cleared: ${reason}`);
    }
    this.leaderPoseStale = false;
    this.leaderPoseStaleReason = null;
  }

  private discardLeaderAuthorityForKeyboardMode(reason: string): void {
    this.leaderPoseStale = true;
    this.leaderPoseStaleReason =
      "leader authority discarded; selecting leader again will move the follower to the leader pose";
    this.logActivity(`Leader authority discarded for keyboard/follower mode: ${reason}`);
  }

  private resetRuntimeSafetyLatch(reason: string): void {
    if (this.runtimeSafetyLatched) {
      this.logActivity(`Runtime safety latch cleared from dashboard state: ${reason}`);
    }
    this.runtimeSafetyLatched = false;
    this.runtimeSafetyLatchReason = null;
    this.runtimeSafetyLatchClearedAtMs = Date.now();
  }

  private refreshRuntimeSafetyLatchFromCurrentSnapshots(): void {
    this.updateRuntimeSafetyLatchFromSnapshots([
      this.hostRunner.getSnapshot(),
      this.teleopRunner.getSnapshot(),
      this.replayRunner.getSnapshot(),
      this.localReplayRunner.getSnapshot(),
      this.piCalibrationRunner.getSnapshot(),
      this.macCalibrationRunner.getSnapshot(),
      this.datasetCaptureRunner.getSnapshot(),
      this.localDatasetCaptureRunner.getSnapshot(),
      this.policyEvalRunner.getSnapshot(),
    ]);
  }

  private assertRuntimeSafetyClear(actionLabel: string): void {
    this.refreshRuntimeSafetyLatchFromCurrentSnapshots();
    if (!this.runtimeSafetyLatched) {
      return;
    }

    const reason = this.runtimeSafetyLatchReason
      ? ` Reason: ${this.runtimeSafetyLatchReason}`
      : "";
    throw new Error(
      `${actionLabel} is blocked because the runtime arm safety latch is active.${reason} Inspect the arm, clear any mechanical bind, then use Reset Pi Connections before commanding motion again.`,
    );
  }

  private updateRuntimeSafetyLatchFromSnapshots(snapshots: ServiceSnapshot[]): void {
    const safetyLine = snapshots
      .filter((snapshot) => this.snapshotCanRelatchRuntimeSafety(snapshot))
      .flatMap((snapshot) => [snapshot.detail, ...snapshot.logs])
      .find((line) => this.isRuntimeSafetyLatchLine(line));
    if (!safetyLine) {
      return;
    }

    const reason = this.cleanServiceLogLine(safetyLine);
    if (!this.runtimeSafetyLatched || this.runtimeSafetyLatchReason !== reason) {
      this.runtimeSafetyLatched = true;
      this.runtimeSafetyLatchReason = reason;
      this.logActivity(`Runtime safety latch detected: ${reason}`);
    }
  }

  private isRuntimeSafetyLatchLine(line: string): boolean {
    if (!/\[safety\].*latched|telemetry\.safety_fault_active/i.test(line)) {
      return false;
    }
    return /temperature|hard limit/i.test(line);
  }

  private snapshotCanRelatchRuntimeSafety(snapshot: ServiceSnapshot): boolean {
    if (!this.runtimeSafetyLatchClearedAtMs) {
      return true;
    }
    if (snapshot.state === "starting" || snapshot.state === "running" || snapshot.state === "stopping") {
      return true;
    }

    const stoppedAtMs = snapshot.stoppedAt ? Date.parse(snapshot.stoppedAt) : Number.NaN;
    if (Number.isFinite(stoppedAtMs) && stoppedAtMs <= this.runtimeSafetyLatchClearedAtMs) {
      return false;
    }

    const startedAtMs = snapshot.startedAt ? Date.parse(snapshot.startedAt) : Number.NaN;
    if (Number.isFinite(startedAtMs)) {
      return startedAtMs > this.runtimeSafetyLatchClearedAtMs;
    }

    return true;
  }

  private normalizeArmSourceMeta(value: unknown): "leader" | "keyboard" | "none" | null {
    return value === "leader" || value === "keyboard" || value === "none" ? value : null;
  }

  private normalizeBaseSourceMeta(value: unknown): "keyboard" | "vexController" | null {
    return value === "keyboard" || value === "vexController" ? value : null;
  }

  private normalizeLiveBaseSource(requested: unknown): "keyboard" | "vexController" {
    return requested === "vexController" ? "vexController" : "keyboard";
  }

  private liveInputLabel(
    armSource: "leader" | "keyboard" | "none",
    baseSource: "keyboard" | "vexController",
    leaderConnected: boolean,
  ): string {
    const baseLabel = baseSource === "keyboard" ? "keyboard-base" : "vex-controller-base";
    if (armSource === "leader" && leaderConnected) {
      return `leader+${baseLabel}`;
    }
    if (armSource === "keyboard") {
      return baseSource === "keyboard" ? "keyboard" : "keyboard-arm+vex-controller-base";
    }
    return baseSource === "keyboard" ? "base-only" : "vex-controller-base";
  }

  private resolveRecordingRestoreArmSource(
    hostSnapshot: ServiceSnapshot,
    teleopSnapshot: ServiceSnapshot,
  ): "leader" | "keyboard" | null {
    if (hostSnapshot.mode !== "recording") {
      return null;
    }

    const metaArmSource =
      this.normalizeArmSourceMeta(hostSnapshot.meta.armSource) ??
      this.normalizeArmSourceMeta(teleopSnapshot.meta.armSource);
    if (metaArmSource === "leader" || metaArmSource === "keyboard") {
      return metaArmSource;
    }
    if (metaArmSource === "none") {
      return null;
    }

    const hostInput = typeof hostSnapshot.meta.input === "string" ? hostSnapshot.meta.input : "";
    const teleopInput = typeof teleopSnapshot.meta.input === "string" ? teleopSnapshot.meta.input : "";
    if (hostInput.includes("leader") || teleopInput.includes("leader")) {
      return "leader";
    }
    if (
      hostInput.includes("keyboard") ||
      teleopInput.includes("keyboard") ||
      teleopSnapshot.mode === "keyboard-control"
    ) {
      return "keyboard";
    }
    return null;
  }

  private resolveLiveControlRestoreArmSource(): "leader" | "keyboard" | "none" | null {
    const hostSnapshot = this.hostRunner.getSnapshot();
    const teleopSnapshot = this.teleopRunner.getSnapshot();
    const hostIsLive =
      hostSnapshot.state === "running" && this.hostSupportsInlineReplay(hostSnapshot.mode);
    const teleopIsLive =
      teleopSnapshot.state === "running" && this.hostSupportsInlineReplay(teleopSnapshot.mode);

    if (!hostIsLive && !teleopIsLive) {
      return null;
    }

    const metaArmSource =
      this.normalizeArmSourceMeta(teleopSnapshot.meta.armSource) ??
      this.normalizeArmSourceMeta(hostSnapshot.meta.armSource);
    if (metaArmSource) {
      return metaArmSource;
    }

    const hostInput = typeof hostSnapshot.meta.input === "string" ? hostSnapshot.meta.input : "";
    const teleopInput = typeof teleopSnapshot.meta.input === "string" ? teleopSnapshot.meta.input : "";
    if (hostInput.includes("leader") || teleopInput.includes("leader")) {
      return "leader";
    }
    if (hostInput.includes("keyboard") || teleopInput.includes("keyboard")) {
      return "keyboard";
    }
    return null;
  }

  private acceptLeaderPoseForFollower(action: string): void {
    if (this.leaderPoseStale) {
      this.clearLeaderPoseStale(`${action}; follower will move to the leader pose`);
    }
  }

  private normalizePinnedReplay(replay: ReplayRequest, holdArmPose: boolean): ReplayRequest {
    if (!holdArmPose) {
      return replay;
    }
    if (replay.target !== "pi") {
      throw new Error("Arm hold pins must target the Pi follower.");
    }
    return {
      ...replay,
      target: "pi",
      homeMode: "none",
      includeBase: false,
      autoVexPositioning: false,
      vexReplayMode: "ecu",
    };
  }

  private normalizeLiveArmSource(
    requested: unknown,
    leaderConnected: boolean,
  ): "leader" | "keyboard" | "none" {
    if (this.activeArmHold) {
      return "none";
    }
    if (requested === "leader") {
      if (!leaderConnected) {
        throw new Error("Leader arm source was requested, but the leader arm is not connected.");
      }
      this.acceptLeaderPoseForFollower("Leader arm control selected");
      return "leader";
    }
    if (requested === "keyboard") {
      return "keyboard";
    }
    if (requested === "none") {
      return "none";
    }
    if (leaderConnected) {
      this.acceptLeaderPoseForFollower("Leader arm control selected");
      return "leader";
    }
    return "keyboard";
  }

  private buildDummyRecordingEntries(): RecordingEntry[] {
    return DUMMY_RECORDING_DEFINITIONS.map((definition, index) => ({
      path: `${DUMMY_RECORDING_PREFIX}${definition.slug}.json`,
      name: definition.name,
      fileName: definition.fileName,
      label: definition.name.replace(/^Dummy:\s*/i, ""),
      size: 46000 + index * 8200,
      modifiedAt: definition.modifiedAt,
      durationS: definition.durationS,
      dummy: true,
    }));
  }

  private buildDummyRecordingDetail(recordingPath: string): RecordingDetail {
    const definition =
      DUMMY_RECORDING_DEFINITIONS.find(
        (item) => `${DUMMY_RECORDING_PREFIX}${item.slug}.json` === recordingPath,
      ) ?? DUMMY_RECORDING_DEFINITIONS[0];
    const path = `${DUMMY_RECORDING_PREFIX}${definition.slug}.json`;
    const baseKeys = [
      "base_left_wheel.vel",
      "base_back_wheel.vel",
      "base_right_wheel.vel",
      "base_x_velocity_mps",
      "base_y_velocity_mps",
      "base_turn_rate_radps",
    ];
    const sensorKeys = [
      "ultrasonic_sensor_1.distance_m",
      "ultrasonic_sensor_2.distance_m",
      "ultrasonic_sensor_1.position_m",
      "ultrasonic_sensor_2.position_m",
    ];
    const durationS = definition.durationS;
    const sampleCount = 73;
    const centers = [0, -48, 78, 28, 0, 35];
    const spans = [16, 20, 26, 18, 32, 10];
    const phaseOffset = definition.slug.includes("align")
      ? 0.8
      : definition.slug.includes("tune")
        ? 1.5
        : 0.15;

    const samples: RecordingDetail["samples"] = Array.from({ length: sampleCount }, (_, index) => {
      const progress = sampleCount <= 1 ? 0 : index / (sampleCount - 1);
      const tS = Number((progress * durationS).toFixed(6));
      const wave = progress * Math.PI * 2 + phaseOffset;
      const armValues: Record<string, number> = Object.fromEntries(
        ARM_HOME_JOINT_KEYS.map((key, servoIndex) => [
          key,
          Number(
            (
              centers[servoIndex] +
              Math.sin(wave + servoIndex * 0.55) * spans[servoIndex] +
              Math.cos(wave * 0.5 + servoIndex) * 2
            ).toFixed(3),
          ),
        ]),
      );
      const baseValues = {
        "base_left_wheel.vel": Number((Math.sin(wave) * 0.28).toFixed(4)),
        "base_back_wheel.vel": Number((Math.cos(wave * 0.9) * 0.24).toFixed(4)),
        "base_right_wheel.vel": Number((Math.sin(wave + 1.1) * 0.27).toFixed(4)),
        "base_x_velocity_mps": Number((Math.sin(wave * 0.7) * 0.08).toFixed(4)),
        "base_y_velocity_mps": Number((0.18 + Math.cos(wave * 0.6) * 0.05).toFixed(4)),
        "base_turn_rate_radps": Number((Math.sin(wave * 0.4) * 0.05).toFixed(4)),
      };
      const sensorValues = {
        "ultrasonic_sensor_1.distance_m": Number((0.32 + progress * 0.08).toFixed(4)),
        "ultrasonic_sensor_2.distance_m": Number((0.76 - progress * 0.12).toFixed(4)),
        "ultrasonic_sensor_1.position_m": Number((0.18 + progress * 0.14).toFixed(4)),
        "ultrasonic_sensor_2.position_m": Number((0.48 + Math.sin(wave * 0.35) * 0.04).toFixed(4)),
      };

      return {
        tS,
        values: {
          ...armValues,
          ...baseValues,
          ...sensorValues,
        },
      };
    });

    const commandSamples = samples.map((sample) => ({
      tS: sample.tS,
      values: Object.fromEntries(
        [...ARM_HOME_JOINT_KEYS, ...baseKeys].map((key) => [key, sample.values[key] ?? 0]),
      ),
    }));

    return {
      path,
      durationS,
      sampleCount: samples.length,
      commandSampleCount: commandSamples.length,
      armKeys: [...ARM_HOME_JOINT_KEYS],
      baseKeys,
      sensorKeys,
      timelineSource: "commands",
      samples,
      commandSamples,
    };
  }

  private stringArray(value: unknown): string[] {
    return Array.isArray(value) ? value.filter((item): item is string => typeof item === "string") : [];
  }

  private finiteNumber(value: unknown, fallback = 0): number {
    return typeof value === "number" && Number.isFinite(value) ? value : fallback;
  }

  private normalizeRecordingPoints(
    items: unknown,
    valueKey: "state" | "action",
    keys: string[],
  ): RecordingDetail["samples"] {
    if (!Array.isArray(items)) {
      return [];
    }

    const points: RecordingDetail["samples"] = [];
    for (const item of items) {
      if (!item || typeof item !== "object") {
        continue;
      }
      const rawItem = item as Record<string, unknown>;
      const rawValues = rawItem[valueKey];
      if (
        typeof rawItem.t_s !== "number" ||
        !Number.isFinite(rawItem.t_s) ||
        !rawValues ||
        typeof rawValues !== "object" ||
        Array.isArray(rawValues)
      ) {
        continue;
      }

      const sourceValues = rawValues as Record<string, unknown>;
      points.push({
        tS: Number(rawItem.t_s.toFixed(6)),
        values: Object.fromEntries(
          keys.map((key) => [key, this.finiteNumber(sourceValues[key])]),
        ),
      });
    }
    return points;
  }

  private buildRecordingDetailFromPayload(
    recordingPath: string,
    payload: unknown,
  ): RecordingDetail {
    if (!payload || typeof payload !== "object" || Array.isArray(payload)) {
      throw new Error("Recording payload is invalid.");
    }
    const rawPayload = payload as Record<string, unknown>;
    const armKeys = this.stringArray(rawPayload.arm_state_keys);
    const baseKeys = this.stringArray(rawPayload.base_state_keys);
    const sensorKeys = this.stringArray(rawPayload.sensor_state_keys);
    const sampleKeys = [...armKeys, ...baseKeys, ...sensorKeys];
    const commandKeys = [...armKeys, ...baseKeys];
    const samples = this.normalizeRecordingPoints(rawPayload.samples, "state", sampleKeys);
    const commandSamples = this.normalizeRecordingPoints(
      rawPayload.command_samples,
      "action",
      commandKeys,
    );
    const rawDurationS = this.finiteNumber(rawPayload.duration_s, NaN);
    const durationS = Number.isFinite(rawDurationS)
      ? rawDurationS
      : samples.at(-1)?.tS || commandSamples.at(-1)?.tS || 0;

    return {
      path: recordingPath,
      durationS: Number(durationS.toFixed(6)),
      sampleCount: samples.length,
      commandSampleCount: commandSamples.length,
      armKeys,
      baseKeys,
      sensorKeys,
      timelineSource: commandSamples.length ? "commands" : "state",
      samples,
      commandSamples,
    };
  }

  private startRemoteStateRefresh(
    settings: AppSettings,
    resolvedPiHost: string,
    hostSnapshot: ServiceSnapshot,
    replaySnapshot: ServiceSnapshot,
    datasetCaptureSnapshot: ServiceSnapshot,
    trainingConfig: TrainingConfig,
  ): void {
    this.remoteStateRefreshInFlight = true;
    const liveHostOwnsPi =
      hostSnapshot.state === "running" && this.hostSupportsInlineReplay(hostSnapshot.mode);

    void (async () => {
      try {
        if (!liveHostOwnsPi && Date.now() - this.lastRecordingRefresh > 15000) {
          try {
            this.lastRecordingRefresh = Date.now();
            this.cachedRecordings = await this.fetchRecordings(settings, resolvedPiHost, true);
          } catch (error) {
            this.noteBackgroundRemoteError(error);
          }
        }

        if (!liveHostOwnsPi && Date.now() - this.lastPowerLogSync > POWER_LOG_SYNC_INTERVAL_MS) {
          try {
            this.lastPowerLogSync = Date.now();
            await this.syncRemotePowerLogs(settings, resolvedPiHost, true);
          } catch (error) {
            this.noteBackgroundRemoteError(error);
          }
        }

        if (!liveHostOwnsPi) {
          await this.getVexBrainStatus(
            settings,
            resolvedPiHost,
            hostSnapshot,
            replaySnapshot,
            datasetCaptureSnapshot,
          );
        }

        if (
          !liveHostOwnsPi &&
          Date.now() - this.lastTrainingMetadataRefresh > TRAINING_METADATA_REFRESH_MS
        ) {
          try {
            this.lastTrainingMetadataRefresh = Date.now();
            await this.refreshTrainingArtifacts(trainingConfig, settings, resolvedPiHost);
          } catch (error) {
            this.noteBackgroundRemoteError(error);
          }
        }
      } catch (error) {
        this.noteBackgroundRemoteError(error);
      } finally {
        this.remoteStateRefreshInFlight = false;
      }
    })();
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

  async zeroVexGyro(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Zero VEX gyro requested.");
      const host = await this.preparePi(settings, "zero VEX gyro");
      const commandHost = this.canUseWarmHostVexCommand()
        ? this.getActiveHostAddress() ?? host
        : host;
      await this.ensureRemoteHelpers(settings, commandHost);

      if (this.canUseWarmHostVexCommand()) {
        await this.zeroVexGyroViaWarmHost(settings, commandHost);
        this.lastVexBrainStatusRefresh = 0;
        this.logActivity("VEX gyro origin zeroed through the running Pi host.");
        this.lastError = null;
        return this.getState();
      }

      const { stdout } = await this.execRemoteScript(
        this.buildVexGyroZeroScript(settings),
        host,
        settings,
      );
      const parsed = stdout ? JSON.parse(stdout) : null;
      if (!parsed?.success) {
        const status =
          typeof parsed?.status === "string" && parsed.status.trim()
            ? parsed.status.trim()
            : "VEX gyro zero failed.";
        throw new Error(status);
      }
      this.lastVexBrainStatusRefresh = 0;
      this.logActivity("VEX gyro origin zeroed over USB.");
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
      this.assertRuntimeSafetyClear("Training dataset capture");
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

      this.assertRuntimeSafetyClear("Policy evaluation");
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

  async startControl(payload: StartControlRequest = {}): Promise<DashboardState> {
    return this.runExclusive(async () => {
      return this.startControlLocked(payload);
    });
  }

  private async startControlLocked(
    payload: StartControlRequest = {},
    options: {
      localReadyTimeoutMs?: number;
      skipSafetyAssert?: boolean;
      skipPiConnectionReset?: boolean;
    } = {},
  ): Promise<DashboardState> {
    const { settings } = this.configStore.getConfig();
    this.logActivity("Live control requested.");
    if (!options.skipSafetyAssert) {
      this.assertRuntimeSafetyClear("Live control");
    }
    if (!options.skipPiConnectionReset) {
      await this.resetPiConnectionsLocked({
        stopLocalTeleop: false,
        clearActiveArmHold: false,
        clearReplayControlRestore: false,
        clearRuntimeSafetyLatch: false,
      });
    }
    const leader = await this.getLeaderStatus(settings);
    const armSource = this.normalizeLiveArmSource(payload?.armSource, leader.connected);
    const baseSource = this.normalizeLiveBaseSource(payload?.baseSource);
    const leaderArmActive = armSource === "leader";
    const keyboardArmActive = armSource === "keyboard";
    const inputLabel = this.liveInputLabel(armSource, baseSource, leader.connected);
    if (keyboardArmActive) {
      this.markLeaderPoseStale("keyboard arm authority selected for live control");
    }
    if (leader.connected) {
      this.logActivity(leader.message);
    } else {
      this.logActivity(`Leader unavailable; starting keyboard arm/base control. ${leader.message}`);
    }
    if (this.activeArmHold && armSource === "none") {
      this.logActivity(`Live control will keep arm input disabled while holding ${this.activeArmHold.name}.`);
    }
    const host = await this.preparePi(settings, "start control");

    assertHelperExists(HOST_SCRIPT);
    assertHelperExists(POWER_SCRIPT);
    assertHelperExists(RUNTIME_SCRIPT);
    assertHelperExists(KEYBOARD_TELEOP_SCRIPT);
    await this.ensureRemoteHelpers(settings, host);
    if (settings.vex.autoRunTelemetry) {
      await this.syncVexTelemetryProgramOnPi(settings, host, "start control", false);
    }
    await this.writeRemoteTorqueLimits(settings, host);
    const keyboardJointLimitsJson = await this.loadKeyboardJointLimitsJson(settings, host);

    await this.replayRunner.stop("Stopping replay before live control.");
    await this.localReplayRunner.stop("Stopping replay before live control.");
    await this.datasetCaptureRunner.stop("Live control needs the robot exclusively.");
    await this.localDatasetCaptureRunner.stop("Live control needs the robot exclusively.");
    await this.policyEvalRunner.stop("Live control needs the robot exclusively.");
    this.rememberVexControlModeFromSnapshot(this.teleopRunner.getSnapshot());
    await this.teleopRunner.stop("Restarting teleop.");
    await this.hostRunner.stop("Restarting Pi host.");
    await this.piCalibrationRunner.stop("Live control needs the robot exclusively.");
    await this.macCalibrationRunner.stop("Live control needs the leader arm exclusively.");
    await this.clearRemoteHostCommand(settings, host);

    await this.hostRunner.start(
      this.buildHostScript(settings, true, {
        vexLiveBaseControl: baseSource === "keyboard",
        requireVexLiveBaseControl: false,
        releaseVexControllerBase: baseSource === "vexController",
      }),
      this.toConnectConfig(settings, host),
      "control",
      {
        host,
        input: inputLabel,
        keyboardBackup: true,
        leaderPort: leaderArmActive ? leader.expectedPort : null,
        armSource,
        baseSource,
        keyboardBaseInput: baseSource === "keyboard" ? "enabled" : "disabled",
        armInput: armSource === "none" ? "disabled" : "enabled",
        vexLiveBaseControl: baseSource === "keyboard",
      },
    );

    try {
      await this.waitForHostReady(host, "Pi host");
      await this.teleopRunner.start(
        this.buildKeyboardTeleopScript(
          settings,
          host,
          keyboardJointLimitsJson,
          leaderArmActive ? leader : null,
          { armSource, disableBaseInput: baseSource === "vexController" },
        ),
        "control",
        {
          host,
          input: inputLabel,
          keyboardBackup: true,
          leaderPort: leaderArmActive ? leader.expectedPort : null,
          armSource,
          baseSource,
          keyboardBaseInput: baseSource === "keyboard" ? "enabled" : "disabled",
          armInput: armSource === "none" ? "disabled" : "enabled",
          vexControlMode: baseSource === "keyboard" ? this.vexKeyboardControlMode : undefined,
        },
      );
      await this.waitForLocalProcessReady(
        baseSource === "keyboard"
          ? leaderArmActive ? "Leader arm + keyboard base teleop" : "Keyboard/base teleop"
          : leaderArmActive ? "Leader arm + VEX controller base teleop" : "VEX controller base teleop",
        this.teleopRunner,
        options.localReadyTimeoutMs ?? KEYBOARD_TELEOP_READY_TIMEOUT_MS,
        / is live\./,
      );
    } catch (error) {
      await this.hostRunner.stop("Teleop failed to launch, stopping the Pi host.");
      throw error;
    }
    this.lastError = null;
    return this.getState();
  }

  async startHotkeyHost(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Keyboard hotkey host requested.");
      this.assertRuntimeSafetyClear("Keyboard hotkey host");
      await this.ensureCommandReadyHost(settings, "arm keyboard hotkeys");

      this.lastError = null;
      return this.getState();
    });
  }

  async startKeyboardControl(payload: StartControlRequest = {}): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Keyboard/base control requested.");
      this.assertRuntimeSafetyClear("Keyboard/base control");
      const leader = await this.getLeaderStatus(settings);
      const armSource = this.normalizeLiveArmSource(payload?.armSource ?? "keyboard", leader.connected);
      const baseSource = this.normalizeLiveBaseSource(payload?.baseSource);
      const leaderArmActive = armSource === "leader";
      const keyboardArmActive = armSource === "keyboard";
      const inputLabel = this.liveInputLabel(armSource, baseSource, leader.connected);
      if (keyboardArmActive) {
        this.markLeaderPoseStale("keyboard arm authority selected for keyboard/base control");
      }
      if (leader.connected) {
        this.logActivity(leader.message);
      }
      const host = await this.preparePi(settings, "start keyboard backup control");

      assertHelperExists(HOST_SCRIPT);
      assertHelperExists(POWER_SCRIPT);
      assertHelperExists(RUNTIME_SCRIPT);
      assertHelperExists(KEYBOARD_TELEOP_SCRIPT);
      await this.ensureRemoteHelpers(settings, host);
      if (settings.vex.autoRunTelemetry) {
        await this.syncVexTelemetryProgramOnPi(
          settings,
          host,
          "start keyboard backup control",
          false,
        );
      }
      await this.writeRemoteTorqueLimits(settings, host);
      const keyboardJointLimitsJson = await this.loadKeyboardJointLimitsJson(settings, host);

      await this.stopRobotExclusiveProcesses("Preparing keyboard backup control.");
      await this.clearRemoteHostCommand(settings, host);

      await this.hostRunner.start(
        this.buildHostScript(settings, true, {
          vexLiveBaseControl: baseSource === "keyboard",
          requireVexLiveBaseControl: false,
          releaseVexControllerBase: baseSource === "vexController",
        }),
        this.toConnectConfig(settings, host),
        "keyboard-control",
        {
          host,
          hotkeysArmed: true,
          keyboardBackup: true,
          input: inputLabel,
          leaderPort: leaderArmActive ? leader.expectedPort : null,
          armSource,
          baseSource,
          keyboardBaseInput: baseSource === "keyboard" ? "enabled" : "disabled",
          armInput: armSource === "none" ? "disabled" : "enabled",
          vexLiveBaseControl: baseSource === "keyboard",
        },
      );

      try {
        await this.waitForHostReady(host, "Keyboard backup host");
        await this.teleopRunner.start(
          this.buildKeyboardTeleopScript(
            settings,
            host,
            keyboardJointLimitsJson,
            leaderArmActive ? leader : null,
            { armSource, disableBaseInput: baseSource === "vexController" },
          ),
          "keyboard-control",
          {
            host,
            input: inputLabel,
            leaderPort: leaderArmActive ? leader.expectedPort : null,
            armSource,
            baseSource,
            keyboardBaseInput: baseSource === "keyboard" ? "enabled" : "disabled",
            armInput: armSource === "none" ? "disabled" : "enabled",
            vexControlMode: baseSource === "keyboard" ? this.vexKeyboardControlMode : undefined,
          },
        );
        await this.waitForLocalProcessReady(
          baseSource === "keyboard"
            ? leaderArmActive ? "Leader arm + keyboard base teleop" : "Keyboard/base teleop"
            : leaderArmActive ? "Leader arm + VEX controller base teleop" : "VEX controller base teleop",
          this.teleopRunner,
          KEYBOARD_TELEOP_READY_TIMEOUT_MS,
          / is live\./,
        );
      } catch (error) {
        await this.teleopRunner.stop("Keyboard backup control failed to launch.");
        await this.hostRunner.stop("Keyboard backup control failed to launch.");
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
      this.activeArmHold = null;
      this.keyboardOverrideArmHold = null;
      this.clearReplayControlRestore();
      this.lastError = null;
      return this.getState();
    });
  }

  async resetPiConnections(): Promise<DashboardState> {
    if (this.exclusiveOperationInFlight) {
      throw new Error("A robot command is still starting. Wait for it to finish, then reset Pi connections if needed.");
    }
    return this.runExclusive(async () => {
      await this.resetPiConnectionsLocked({ stopLocalTeleop: true });
      return this.getState();
    });
  }

  private async resetPiConnectionsLocked(
    options: {
      stopLocalTeleop?: boolean;
      clearActiveArmHold?: boolean;
      clearReplayControlRestore?: boolean;
      clearRuntimeSafetyLatch?: boolean;
    } = {},
  ): Promise<void> {
    this.logActivity("Reset Pi connections requested.");
    this.suppressRemoteTransportErrorsUntil = Date.now() + 10000;
    const closedRemoteClients = this.closeActiveRemoteClients("Resetting auxiliary Pi SSH/SFTP sessions.");

    this.hostRunner.resetConnection("Resetting the Pi host SSH connection.");
    this.replayRunner.resetConnection("Resetting the replay SSH connection.");
    this.piCalibrationRunner.resetConnection("Resetting the Pi calibration SSH connection.");
    this.datasetCaptureRunner.resetConnection("Resetting the dataset capture SSH connection.");
    this.policyEvalRunner.resetConnection("Resetting the policy eval SSH connection.");

    if (options.stopLocalTeleop !== false) {
      await this.teleopRunner.stop("Resetting Pi connections.");
    }

    const now = Date.now();
    this.lastRecordingRefresh = now;
    this.lastPowerLogSync = now;
    this.lastTrainingMetadataRefresh = now;
    this.lastVexBrainStatusRefresh = now;
    this.lastResolvedHost = null;
    if (options.clearActiveArmHold !== false) {
      this.activeArmHold = null;
      this.keyboardOverrideArmHold = null;
    }
    if (options.clearRuntimeSafetyLatch !== false) {
      this.resetRuntimeSafetyLatch("Pi connections were reset");
    }
    if (options.clearReplayControlRestore !== false) {
      this.clearReplayControlRestore();
    }
    this.remoteStateRefreshInFlight = false;
    this.lastError = null;
    this.logActivity(
      `Pi connection reset complete. Closed ${closedRemoteClients} auxiliary SSH/SFTP ${closedRemoteClients === 1 ? "session" : "sessions"}.`,
    );
  }

  async startAgain(payload: StartControlRequest = {}): Promise<DashboardState> {
    return this.runExclusive(async () => {
      this.logActivity("Quick manual restart requested.");
      await this.resetPiConnectionsLocked({ stopLocalTeleop: false });
      return this.startControlLocked(
        {
          ...payload,
          armSource: payload.armSource ?? "leader",
          baseSource: payload.baseSource ?? "keyboard",
        },
        {
          localReadyTimeoutMs: KEYBOARD_TELEOP_READY_TIMEOUT_MS,
          skipSafetyAssert: true,
          skipPiConnectionReset: true,
        },
      );
    });
  }

  async resolveLeaderStale(payload: ResolveLeaderStaleRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      if (payload.action === "discardLeaderAuthority") {
        this.discardLeaderAuthorityForKeyboardMode("operator chose to stay in keyboard/follower mode");
        this.lastError = null;
        return this.getState();
      }
      if (payload.action === "moveFollowerToLeaderPose") {
        this.acceptLeaderPoseForFollower("Operator chose follower-to-leader control");
        this.lastError = null;
        return this.getState();
      }
      throw new Error("Choose a valid stale-leader transition.");
    });
  }

  async mimicLeaderToActiveHold(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Leader mimic requested for active hold.");
      this.assertRuntimeSafetyClear("Leader mimic");

      const activeHold = this.activeArmHold;
      if (!activeHold) {
        throw new Error("Turn on an arm hold before using Leader Mimic.");
      }
      let holdHomePosition = activeHold.homePosition;
      if (!isFiniteHomePosition(holdHomePosition)) {
        const host = await this.ensureCommandReadyHost(
          settings,
          "capture active hold pose for leader mimic",
        );
        const requestId = `capture-hold-${Date.now()}-${Math.random().toString(36).slice(2)}`;
        await this.writeRemoteHostCommand(settings, host, {
          command: "capture-home",
          requestId,
        });
        const result = await this.waitForWarmHostHomeCommandResult(requestId, 6000);
        if (!result || !result.success || !isFiniteHomePosition(result.homePosition)) {
          throw new Error(result?.status || "The running Pi host could not capture the active hold pose.");
        }
        holdHomePosition = result.homePosition;
        this.activeArmHold = {
          ...activeHold,
          homePosition: holdHomePosition,
        };
      }

      const leader = await this.ensureLeaderConnected(settings);
      await this.macCalibrationRunner.stop("Leader mimic needs the leader arm exclusively.");
      await this.localReplayRunner.stop("Leader mimic needs the leader arm exclusively.");
      await this.runLocalLeaderMimicPoseScript(
        this.buildLocalLeaderMimicPoseScript(settings, leader, holdHomePosition),
      );
      this.clearLeaderPoseStale(`leader mimicked active hold ${activeHold.name}`);
      this.logActivity(`Leader mimic complete for active hold ${activeHold.name}.`);
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

      this.activeArmHold = null;
      this.keyboardOverrideArmHold = null;
      this.clearReplayControlRestore();
      this.lastError = null;
      return this.getState();
    });
  }

  async setArmHomePosition(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Set arm home position requested.");
      this.assertRuntimeSafetyClear("Set arm home");
      const host = await this.ensureCommandReadyHost(settings, "set arm home");
      const requestId = `capture-home-${Date.now()}-${Math.random().toString(36).slice(2)}`;
      await this.writeRemoteHostCommand(settings, host, {
        command: "capture-home",
        requestId,
      });
      const result = await this.waitForWarmHostHomeCommandResult(requestId, 6000);
      if (!result) {
        throw new Error("Timed out waiting for the running Pi host to capture the arm home position.");
      }
      if (!result.success || !isFiniteHomePosition(result.homePosition)) {
        throw new Error(result.status || "The running Pi host could not capture a valid arm home position.");
      }

      this.configStore.saveHomePosition(result.homePosition);
      this.logActivity("Saved current arm pose as the home position.");
      this.lastError = null;
      return this.getState();
    });
  }

  async setArmHomePositionFromRecording(
    payload: SetArmHomeFromRecordingRequest,
  ): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const recordingPath = payload.path.trim();
      if (!recordingPath) {
        throw new Error("Choose a recording before saving its start as home.");
      }

      this.logActivity(`Saving recording start pose as arm home: ${recordingPath}.`);
      const host = await this.preparePi(settings, "save recording start as arm home");
      const homePosition = await this.readRecordingStartHomePosition(settings, host, recordingPath);
      if (!isFiniteHomePosition(homePosition)) {
        throw new Error("The recording start does not contain a valid arm home pose.");
      }

      this.configStore.saveHomePosition(homePosition);
      this.logActivity("Saved selected recording start pose as the arm home position.");
      this.lastError = null;
      return this.getState();
    });
  }

  async goToArmHomePosition(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, homePosition } = this.configStore.getConfig();
      if (!isFiniteHomePosition(homePosition)) {
        throw new Error("Set an arm home position before using Go Home.");
      }

      this.logActivity("Go home requested.");
      this.assertRuntimeSafetyClear("Go home");
      this.markLeaderPoseStale("follower moved with Go Home command");
      const host = await this.ensureCommandReadyHost(settings, "go home");
      await this.teleopRunner.stop(
        "Stopping live command stream before Go Home so the saved home pose holds.",
      );
      const requestId = `go-home-${Date.now()}-${Math.random().toString(36).slice(2)}`;
      await this.writeRemoteHostCommand(settings, host, {
        command: "go-home",
        requestId,
        homePosition,
      });
      const result = await this.waitForWarmHostHomeCommandResult(requestId, 12000);
      if (!result) {
        throw new Error("Timed out waiting for the running Pi host to move the arm home.");
      }
      if (!result.success) {
        throw new Error(result.status || "The running Pi host failed to move the arm home.");
      }

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
      this.assertRuntimeSafetyClear("Pi calibration");
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
      this.assertRuntimeSafetyClear("Pi servo calibration");
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
      this.assertRuntimeSafetyClear("Mac leader calibration");
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

  async startRecording(payload: StartRecordingRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const label = typeof payload?.label === "string" ? payload.label.trim() : "";
      const requestedInputMode = this.normalizeRecordingInputMode(payload?.inputMode);
      const useFreeTeachInput = requestedInputMode === "free-teach";
      const useKeyboardOnlyInput = requestedInputMode === "keyboard";
      this.logActivity(`Start recording requested${label ? ` (${label})` : ""}.`);
      this.assertRuntimeSafetyClear("Start recording");
      const leader = useFreeTeachInput || useKeyboardOnlyInput ? null : await this.getLeaderStatus(settings);
      const combinedInput = !useFreeTeachInput && leader?.connected ? "leader+keyboard-base" : "keyboard";
      const recordingArmSource =
        useFreeTeachInput
          ? "none"
          : !useKeyboardOnlyInput && leader?.connected
            ? "leader"
            : "keyboard";
      if (recordingArmSource === "leader") {
        this.acceptLeaderPoseForFollower("Leader arm recording selected");
      } else if (useFreeTeachInput) {
        this.markLeaderPoseStale("follower hand-guide recording selected");
      } else {
        this.markLeaderPoseStale("keyboard arm recording selected");
      }
      this.logActivity(
        `Recording input source: ${
          useFreeTeachInput
            ? "hand-guided follower arm with torque disabled"
            : useKeyboardOnlyInput
              ? "keyboard controls only"
              : leader?.connected
              ? "leader arm with keyboard base controls"
              : "keyboard controls; leader arm unavailable"
        }.`,
      );
      if (!useFreeTeachInput && leader?.connected) {
        this.logActivity(leader.message);
      } else if (!useFreeTeachInput && leader) {
        this.logActivity(`Leader unavailable; recording with keyboard backup only. ${leader.message}`);
      }
      if (useKeyboardOnlyInput && this.activeArmHold) {
        this.logActivity(
          `Keyboard-only recording is starting from held arm pose ${this.activeArmHold.name}; arm keys remain enabled for this recording.`,
        );
      }
      const host = await this.preparePi(settings, "start recording");

      assertHelperExists(RECORD_SCRIPT);
      assertHelperExists(RUNTIME_SCRIPT);
      if (!useFreeTeachInput) {
        assertHelperExists(KEYBOARD_TELEOP_SCRIPT);
      }
      await this.ensureRemoteHelpers(settings, host);
      if (!useFreeTeachInput && settings.vex.autoRunTelemetry) {
        await this.syncVexTelemetryProgramOnPi(settings, host, "start recording", false);
      }
      await this.writeRemoteTorqueLimits(settings, host);
      const keyboardJointLimitsJson = !useFreeTeachInput
        ? await this.loadKeyboardJointLimitsJson(settings, host)
        : "{}";

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
      const recordingInput = useFreeTeachInput ? "free-teach" : "keyboard";

      await this.hostRunner.start(
        this.buildRecordScript(settings, trajectoryPath, label, recordingInput),
        this.toConnectConfig(settings, host),
        "recording",
          {
            host,
            trajectoryPath,
            label,
            input: useFreeTeachInput ? "free-teach" : combinedInput,
            recordingInputMode: recordingInput,
            armSource: recordingArmSource,
            leaderPort: leader?.connected ? leader.expectedPort : null,
          },
        );

      try {
        await this.waitForHostReady(host, "Recorder");
        if (useFreeTeachInput) {
          this.logActivity("Hand-guide recorder is live with follower arm torque disabled.");
        } else {
          await this.teleopRunner.start(
            this.buildKeyboardTeleopScript(
              settings,
              host,
              keyboardJointLimitsJson,
              !useKeyboardOnlyInput && leader?.connected ? leader : null,
              { armSource: recordingArmSource === "leader" ? "leader" : "keyboard" },
            ),
            "recording",
            {
              host,
              input: useKeyboardOnlyInput ? "keyboard" : combinedInput,
              leaderPort: !useKeyboardOnlyInput && leader?.connected ? leader.expectedPort : null,
              armSource: recordingArmSource,
              vexControlMode: this.vexKeyboardControlMode,
            },
          );
          await this.waitForLocalProcessReady(
            useKeyboardOnlyInput || !leader?.connected
              ? "Keyboard-only recording teleop"
              : "Leader arm + keyboard base recording teleop",
            this.teleopRunner,
            KEYBOARD_TELEOP_READY_TIMEOUT_MS,
            / is live\./,
          );
        }
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
      const teleopState = this.teleopRunner.getSnapshot();
      const { settings } = this.configStore.getConfig();
      const recordingArmSource = this.resolveRecordingRestoreArmSource(state, teleopState);
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

      if (this.activeArmHold) {
        const hostForHold = await this.returnToActiveArmHoldPose(
          settings,
          "return to held arm pose after recording",
          "after recording",
        );
        await this.restoreBaseOnlyKeyboardControlForActiveHold(
          settings,
          hostForHold,
          {
            restoredAfterRecording: true,
            recordingArmSource: "hold",
          },
          "after recording",
        );
      } else if (recordingArmSource) {
        const restoreArmSource = recordingArmSource;
        const keyboardHost =
          host ?? (await this.preparePi(settings, "restore keyboard control after recording"));
        this.logActivity(
          restoreArmSource === "leader"
            ? "Restoring leader arm + keyboard base control after leader recording."
            : "Restoring keyboard arm/base control after keyboard recording.",
        );
        await this.startKeyboardControlSession(
          settings,
          keyboardHost,
          {
            restoredAfterRecording: true,
            recordingArmSource: restoreArmSource,
          },
          false,
          {
            allowLeader: restoreArmSource === "leader",
            armSource: restoreArmSource === "keyboard" ? "keyboard" : undefined,
          },
        );
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

    if (isDummyRecordingPath(recordingPath)) {
      this.logActivity(`Loading offline dummy recording detail for ${recordingPath}.`);
      return this.buildDummyRecordingDetail(recordingPath);
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

  async duplicateRecording(payload: DuplicateRecordingRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, recordingReplayOptions } = this.configStore.getConfig();
      const recordingPath = payload.path.trim();

      if (!recordingPath) {
        throw new Error("Choose a recording before duplicating it.");
      }

      this.logActivity(`Duplicating recording ${recordingPath}.`);
      const host = await this.preparePi(settings, "duplicate recording");
      const duplicate = await this.duplicateRemoteRecordingFile(settings, host, recordingPath);

      const sourceReplayOptions = recordingReplayOptions[recordingPath];
      if (sourceReplayOptions) {
        this.configStore.saveRecordingReplayOptions({
          ...recordingReplayOptions,
          [duplicate.path]: { ...sourceReplayOptions },
        });
      }

      this.logActivity(`Created duplicate recording ${duplicate.path}.`);
      this.cachedRecordings = await this.fetchRecordings(settings, host);
      this.lastRecordingRefresh = Date.now();
      this.lastError = null;
      return this.getState();
    });
  }

  async deleteRecording(payload: DeleteRecordingRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings, pinnedMoves, chainLinks, recordingReplayOptions } = this.configStore.getConfig();
      const recordingPath = payload.path.trim();

      if (!recordingPath) {
        throw new Error("Choose a recording before deleting it.");
      }

      this.logActivity(`Deleting recording ${recordingPath}.`);
      const host = await this.preparePi(settings, "delete recording");
      await this.deleteRemoteRecordingFile(settings, host, recordingPath);

      const remainingPins = pinnedMoves.filter((move) => move.trajectoryPath !== recordingPath);
      if (remainingPins.length !== pinnedMoves.length) {
        if (this.activeArmHold?.trajectoryPath === recordingPath) {
          this.activeArmHold = null;
        }
        if (this.keyboardOverrideArmHold?.trajectoryPath === recordingPath) {
          this.keyboardOverrideArmHold = null;
        }
        this.configStore.savePinnedMoves(remainingPins);
        this.logActivity("Removed pinned moves that referenced the deleted recording.");
      }
      if (recordingReplayOptions[recordingPath]) {
        const nextOptions = { ...recordingReplayOptions };
        delete nextOptions[recordingPath];
        this.configStore.saveRecordingReplayOptions(nextOptions);
      }
      const nextChainLinks = chainLinks
        .map((chain) => ({
          ...chain,
          items: chain.items.filter((item) => item.trajectoryPath !== recordingPath),
        }))
        .filter((chain) => chain.items.length > 0);
      if (
        nextChainLinks.length !== chainLinks.length ||
        nextChainLinks.some((chain, index) => chain.items.length !== chainLinks[index]?.items.length)
      ) {
        this.configStore.saveChainLinks(nextChainLinks);
        this.logActivity("Removed chain-link blocks that referenced the deleted recording.");
      }

      this.cachedRecordings = await this.fetchRecordings(settings, host);
      this.lastRecordingRefresh = Date.now();
      this.lastError = null;
      return this.getState();
    });
  }

  async setRecordingReplayOptions(
    payload: SetRecordingReplayOptionsRequest,
  ): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const recordingPath = payload.path.trim();
      if (!recordingPath) {
        throw new Error("Choose a recording before changing replay options.");
      }

      const options = normalizeRecordingReplayOptions(
        payload.options,
        config.settings.trajectories,
      );
      const nextOptions = { ...config.recordingReplayOptions };
      if (recordingReplayOptionsAreDefault(options, config.settings.trajectories)) {
        delete nextOptions[recordingPath];
      } else {
        nextOptions[recordingPath] = options;
      }

      this.configStore.saveRecordingReplayOptions(nextOptions);
      this.lastError = null;
      return this.getState();
    });
  }

  async createMoveRecordingVersion(
    payload: CreateMoveRecordingVersionRequest,
  ): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const move = config.moveDefinitions.find((item) => item.id === payload.moveId);
      if (!move || move.archived) {
        throw new Error("Choose an active beta move before saving a recording version.");
      }

      const versionNumber = nextMoveVersionNumber(config.moveRecordingVersions, move.id);
      const trajectoryPath =
        typeof payload.trajectoryPath === "string" && payload.trajectoryPath.trim()
          ? payload.trajectoryPath.trim()
          : null;
      const version = {
        id: crypto.randomUUID(),
        moveId: move.id,
        versionNumber,
        trajectoryPath,
        displayName:
          typeof payload.displayName === "string" && payload.displayName.trim()
            ? payload.displayName.trim()
            : `${move.label} v${versionNumber}`,
        recordedAtIso: new Date().toISOString(),
        recordingType: this.normalizeBetaRecordingType(
          payload.recordingType,
          move.defaultRecordingType,
        ),
        playbackSpeed: normalizeBoundedNumber(payload.playbackSpeed, 1, 0.1, 3),
        vexBaseSamplesPresent: payload.vexBaseSamplesPresent === true,
        distanceSensorSamplesPresent: payload.distanceSensorSamplesPresent === true,
        inertialSamplesPresent: payload.inertialSamplesPresent === true,
        autoVexPositioningEnabled: payload.autoVexPositioningEnabled === true,
        isFavorite: false,
        notes: typeof payload.notes === "string" ? payload.notes : "",
        safetyStatus: this.normalizeMoveRecordingSafetyStatus(payload.safetyStatus),
      };
      let moveDefinitions = config.moveDefinitions;
      let moveRecordingVersions = [...config.moveRecordingVersions, version];
      if (payload.markFavorite === true) {
        const favorite = withFavoriteVersion(moveDefinitions, moveRecordingVersions, move.id, version.id);
        moveDefinitions = favorite.moves;
        moveRecordingVersions = favorite.versions;
      }

      this.configStore.saveMoveDefinitions(moveDefinitions);
      this.configStore.saveMoveRecordingVersions(moveRecordingVersions);
      this.logActivity(`Saved ${move.label} v${versionNumber} to the beta move library.`);
      this.lastError = null;
      return this.getState();
    });
  }

  async updateMoveRecordingVersion(
    versionId: string,
    payload: UpdateMoveRecordingVersionRequest,
  ): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const existing = config.moveRecordingVersions.find((version) => version.id === versionId);
      if (!existing) {
        throw new Error("Beta recording version not found.");
      }

      const updated = {
        ...existing,
        displayName:
          typeof payload.displayName === "string" && payload.displayName.trim()
            ? payload.displayName.trim()
            : existing.displayName,
        playbackSpeed:
          payload.playbackSpeed === undefined
            ? existing.playbackSpeed
            : normalizeBoundedNumber(payload.playbackSpeed, existing.playbackSpeed, 0.1, 3),
        vexBaseSamplesPresent:
          typeof payload.vexBaseSamplesPresent === "boolean"
            ? payload.vexBaseSamplesPresent
            : existing.vexBaseSamplesPresent,
        distanceSensorSamplesPresent:
          typeof payload.distanceSensorSamplesPresent === "boolean"
            ? payload.distanceSensorSamplesPresent
            : existing.distanceSensorSamplesPresent,
        inertialSamplesPresent:
          typeof payload.inertialSamplesPresent === "boolean"
            ? payload.inertialSamplesPresent
            : existing.inertialSamplesPresent,
        autoVexPositioningEnabled:
          typeof payload.autoVexPositioningEnabled === "boolean"
            ? payload.autoVexPositioningEnabled
            : existing.autoVexPositioningEnabled,
        notes: typeof payload.notes === "string" ? payload.notes : existing.notes,
        safetyStatus:
          payload.safetyStatus === undefined
            ? existing.safetyStatus
            : this.normalizeMoveRecordingSafetyStatus(payload.safetyStatus),
      };

      this.configStore.saveMoveRecordingVersions(
        config.moveRecordingVersions.map((version) => (version.id === versionId ? updated : version)),
      );
      this.logActivity(`Updated beta recording version ${updated.displayName}.`);
      this.lastError = null;
      return this.getState();
    });
  }

  async setMoveFavoriteVersion(
    payload: SetMoveFavoriteVersionRequest,
  ): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const favorite = withFavoriteVersion(
        config.moveDefinitions,
        config.moveRecordingVersions,
        payload.moveId,
        payload.versionId,
      );
      this.configStore.saveMoveDefinitions(favorite.moves);
      this.configStore.saveMoveRecordingVersions(favorite.versions);
      this.logActivity("Updated beta move favorite version.");
      this.lastError = null;
      return this.getState();
    });
  }

  async saveGamePlan(payload: GamePlan): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const id = typeof payload.id === "string" && payload.id.trim() ? payload.id.trim() : crypto.randomUUID();
      const now = new Date().toISOString();
      const existing = config.gamePlans.find((plan) => plan.id === id);
      const plan: GamePlan = {
        id,
        name: typeof payload.name === "string" && payload.name.trim() ? payload.name.trim() : "Game plan",
        steps: Array.isArray(payload.steps) ? payload.steps : [],
        createdAtIso: existing?.createdAtIso ?? payload.createdAtIso ?? now,
        updatedAtIso: now,
      };
      const nextPlans = existing
        ? config.gamePlans.map((item) => (item.id === id ? plan : item))
        : [...config.gamePlans, plan];
      this.configStore.saveGamePlans(nextPlans);
      this.logActivity(`Saved Game Builder plan ${plan.name}.`);
      this.lastError = null;
      return this.getState();
    });
  }

  async adjustRecordingServos(payload: AdjustRecordingServoRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const recordingPath = payload.path.trim();
      const timeS = Number(payload.timeS);
      const values = payload.values;

      if (!recordingPath) {
        throw new Error("Choose a recording before saving servo adjustments.");
      }
      if (!Number.isFinite(timeS) || timeS < 0) {
        throw new Error("Servo adjustment time must be valid.");
      }
      if (!values || typeof values !== "object") {
        throw new Error("Servo adjustment values are missing.");
      }

      const servoValues: Record<string, number> = {};
      for (const key of ARM_HOME_JOINT_KEYS) {
        const value = Number(values[key]);
        if (!Number.isFinite(value)) {
          throw new Error(`Servo adjustment is missing numeric ${key}.`);
        }
        servoValues[key] = value;
      }

      this.logActivity(
        `Saving servo adjustments for ${recordingPath} at ${timeS.toFixed(2)}s.`,
      );
      if (isDummyRecordingPath(recordingPath)) {
        this.logActivity("Dummy recording servo adjustment accepted for offline UI testing.");
        this.lastError = null;
        return this.getState();
      }

      const host = await this.preparePi(settings, "save servo recording adjustment");
      await this.writeRecordingServoAdjustment(settings, host, recordingPath, timeS, servoValues);
      this.cachedRecordings = await this.fetchRecordings(settings, host);
      this.lastRecordingRefresh = Date.now();
      this.lastError = null;
      return this.getState();
    });
  }

  async markRecordingGyroZero(payload: MarkRecordingGyroZeroRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const recordingPath = payload.path.trim();

      if (!recordingPath) {
        throw new Error("Choose a recording before marking its gyro zero.");
      }

      this.logActivity(`Marking recording ${recordingPath} start as VEX gyro zero.`);
      const host = await this.preparePi(settings, "mark recording gyro zero");
      await this.markRecordingStartGyroZero(settings, host, recordingPath);
      this.cachedRecordings = await this.fetchRecordings(settings, host);
      this.lastRecordingRefresh = Date.now();
      this.lastError = null;
      return this.getState();
    });
  }

  async resetRecordingUltrasonicPosition(
    payload: ResetRecordingUltrasonicPositionRequest,
  ): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const recordingPath = payload.path.trim();

      if (!recordingPath) {
        throw new Error("Choose a recording before resetting its ultrasonic position.");
      }

      this.logActivity(
        `Replaying recording ${recordingPath} to recapture and overwrite its X/Y ultrasonic stream.`,
      );
      await this.stopWarmHostReplay(settings, "Ultrasonic stream recapture needs a standalone replay.");
      await this.stopRobotExclusiveProcesses("Ultrasonic stream recapture needs the robot exclusively.");

      const host = await this.preparePi(settings, "recapture recording ultrasonic stream");
      assertHelperExists(REPLAY_SCRIPT);
      assertHelperExists(POWER_SCRIPT);
      assertHelperExists(RUNTIME_SCRIPT);
      assertHelperExists(VEX_BASE_BRIDGE_SCRIPT);
      assertHelperExists(SENSOR_REPLAY_SCRIPT);
      await this.ensureRemoteHelpers(settings, host);
      await this.writeRemoteTorqueLimits(settings, host);

      const replay: ReplayRequest = {
        trajectoryPath: recordingPath,
        target: "pi",
        vexReplayMode: "ecu",
        homeMode: "none",
        speed: 1,
        autoVexPositioning: false,
        vexPositioningSpeed: DEFAULT_VEX_POSITIONING_SPEED,
        vexPositioningTimeoutS: DEFAULT_VEX_POSITIONING_TIMEOUT_S,
        vexPositioningXyToleranceM: DEFAULT_VEX_POSITIONING_XY_TOLERANCE_M,
        vexPositioningHeadingToleranceDeg: DEFAULT_VEX_POSITIONING_HEADING_TOLERANCE_DEG,
        vexPositioningXyTrimToleranceM: DEFAULT_VEX_POSITIONING_XY_TRIM_TOLERANCE_M,
        vexPositioningHeadingTrimToleranceDeg: DEFAULT_VEX_POSITIONING_HEADING_TRIM_TOLERANCE_DEG,
        includeBase: true,
        holdFinalS: 0,
      };
      await this.replayRunner.start(
        this.buildReplayScript(settings, replay, null, {
          recaptureUltrasonicStream: true,
        }),
        this.toConnectConfig(settings, host),
        "ultrasonic-recapture",
        { ...replay, recaptureUltrasonicStream: true },
      );
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
    return this.runExclusive(() => this.startReplayLocked(payload));
  }

  private async startReplayLocked(
    payload: ReplayRequest,
    homePositionOverride?: ArmHomePosition | null,
    options: { restoreControlAfterReplay?: boolean; returnToActiveHold?: boolean } = {},
  ): Promise<DashboardState> {
    this.assertRuntimeSafetyClear("Replay");
    const restoreControlAfterReplay = options.restoreControlAfterReplay !== false;
    const { settings, homePosition } = this.configStore.getConfig();
    const normalizedReplay = normalizeReplayRequest(payload, settings.trajectories);
    const activeArmHold = this.activeArmHold;
    const liveRestoreArmSource = this.resolveLiveControlRestoreArmSource();
    const restoreArmSource: "leader" | "keyboard" | "none" | null =
      activeArmHold ? "none" : liveRestoreArmSource;
    const shouldRestoreControlAfterReplay =
      restoreControlAfterReplay && (Boolean(activeArmHold) || restoreArmSource !== null);
    const shouldReturnToActiveHoldAfterReplay = Boolean(
      options.returnToActiveHold !== false &&
      normalizedReplay.target === "pi" &&
      activeArmHold &&
      normalizedReplay.trajectoryPath !== activeArmHold.trajectoryPath,
    );
    const replayHomePosition = homePositionOverride ?? homePosition;
    const replay: ReplayRequest = normalizedReplay;
    if (!replay.trajectoryPath) {
      throw new Error("Choose a saved trajectory before starting replay.");
    }
    if (isDummyRecordingPath(replay.trajectoryPath)) {
      throw new Error("Dummy recordings are for offline UI testing only. Connect the Pi to replay real recordings.");
    }
    if (replay.homeMode !== "none" && !isFiniteHomePosition(replayHomePosition)) {
      throw new Error("Set an arm home position before replaying with Go Home enabled.");
    }
    if (replay.target === "pi") {
      this.markLeaderPoseStale("Pi follower replay/preposition selected");
    }
    this.logActivity(
      `Starting ${replay.target === "leader" ? "leader" : "Pi"} replay for ${replay.trajectoryPath}${
        replay.target === "pi" ? ` (${replay.includeBase ? "VEX base + arm" : "arm only"})` : ""
      }${replay.startTimeS ? ` from ${replay.startTimeS.toFixed(2)}s` : ""}${
        replay.homeMode !== "none" ? `, home ${replay.homeMode}` : ""
      }${shouldReturnToActiveHoldAfterReplay ? `, then replaying held arm pose ${activeArmHold?.name}` : ""}.`,
    );
    this.clearReplayControlRestore();

    await this.replayRunner.stop("Restarting replay.");
    await this.localReplayRunner.stop("Restarting replay.");
    await this.stopWarmHostReplay(settings, "Restarting warm-host replay.");
    this.rememberVexControlModeFromSnapshot(this.teleopRunner.getSnapshot());

    if (this.canUseWarmHostReplay(replay)) {
      const host = this.getActiveHostAddress();
      if (!host) {
        throw new Error("The running Pi host is active, but its address is unavailable.");
      }

      await this.piCalibrationRunner.stop("Replay needs the robot exclusively.");
      await this.macCalibrationRunner.stop("Replay requested.");
      await this.datasetCaptureRunner.stop("Replay needs the robot exclusively.");
      await this.localDatasetCaptureRunner.stop("Replay needs the robot exclusively.");
      await this.policyEvalRunner.stop("Replay needs the robot exclusively.");
      if (shouldRestoreControlAfterReplay) {
        const keepKeyboardBaseDuringReplay =
          Boolean(activeArmHold) &&
          !replay.includeBase &&
          this.warmHostHasVexLiveBaseControl();
        if (keepKeyboardBaseDuringReplay) {
          await this.ensureLocalBaseOnlyKeyboardControlForWarmHost(
            settings,
            host,
            {
              activeArmHold: activeArmHold?.name ?? null,
              replayOverride: true,
              restoredArmSource: "none",
            },
            "during replay",
          );
        } else {
          await this.teleopRunner.stop("Replay overrides live control until it completes.");
        }
      }
      await this.sendWarmHostReplay(settings, host, replay, replayHomePosition);
      if (shouldRestoreControlAfterReplay) {
        this.markReplayControlRestorePending(replay, restoreArmSource, Boolean(shouldReturnToActiveHoldAfterReplay));
      }

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
        this.buildReplayScript(settings, replay, replayHomePosition),
        this.toConnectConfig(settings, host),
        "replay",
        { ...replay },
      );
    }

    if (shouldRestoreControlAfterReplay) {
      this.markReplayControlRestorePending(replay, restoreArmSource, Boolean(shouldReturnToActiveHoldAfterReplay));
    }
    this.lastError = null;
    return this.getState();
  }

  async stopReplay(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      this.logActivity("Stop replay requested.");
      const { settings } = this.configStore.getConfig();
      const warmReplayActiveBeforeStop =
        this.deriveWarmHostReplaySnapshot(this.hostRunner.getSnapshot()).state === "running";
      const warmReplayStopRequestedAtMs = Date.now();
      await this.replayRunner.stop("Stopping replay.");
      await this.localReplayRunner.stop("Stopping replay.");
      await this.stopWarmHostReplay(settings, "Stopping warm-host replay.");
      this.clearReplayControlRestore();
      if (warmReplayActiveBeforeStop) {
        await this.waitForWarmHostReplayStopped(warmReplayStopRequestedAtMs, 9000);
      }
      if (this.activeArmHold) {
        const hostForHold = await this.returnToActiveArmHoldPose(
          settings,
          "return to held arm pose after stopping replay",
          "after stopping replay",
        );
        await this.restoreBaseOnlyKeyboardControlForActiveHold(
          settings,
          hostForHold,
          { restoredAfterReplayStop: true },
          "after stopping replay",
        );
      } else {
        const host = await this.findReachablePiHost(settings);
        if (host && settings.vex.autoRunTelemetry) {
          await this.syncVexTelemetryProgramOnPi(
            settings,
            host,
            "restore controller telemetry after replay",
            false,
          );
        }
      }
      this.lastError = null;
      return this.getState();
    });
  }

  async createPinnedMove(payload: CreatePinnedMoveRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const holdArmPose = payload.holdArmPose === true;
      const replay = this.normalizePinnedReplay(
        normalizeReplayRequest(payload, config.settings.trajectories),
        holdArmPose,
      );
      this.logActivity(`Saving pinned move ${payload.name.trim() || replay.trajectoryPath}.`);
      const pinnedMoves = [
        ...config.pinnedMoves,
        {
          id: crypto.randomUUID(),
          name: payload.name.trim(),
          keyBinding: payload.keyBinding.trim(),
          holdArmPose,
          ...replay,
        },
      ];

      this.configStore.savePinnedMoves(pinnedMoves);
      this.lastError = null;
      return this.getState();
    });
  }

  async updatePinnedMove(id: string, payload: UpdatePinnedMoveRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const existing = config.pinnedMoves.find((move) => move.id === id);
      if (!existing) {
        throw new Error("Pinned move not found.");
      }

      const merged = {
        ...existing,
        ...payload,
      };
      const holdArmPose =
        typeof payload.holdArmPose === "boolean"
          ? payload.holdArmPose
          : existing.holdArmPose;
      const replay = this.normalizePinnedReplay(
        normalizeReplayRequest(merged, config.settings.trajectories),
        holdArmPose,
      );
      const updated: PinnedMove = {
        id: existing.id,
        name:
          typeof payload.name === "string" && payload.name.trim()
            ? payload.name.trim()
            : existing.name,
        keyBinding:
          typeof payload.keyBinding === "string"
            ? payload.keyBinding.trim()
            : existing.keyBinding,
        holdArmPose,
        ...replay,
      };

      this.logActivity(`Updating pinned move ${updated.name}.`);
      this.configStore.savePinnedMoves(
        config.pinnedMoves.map((move) => (move.id === id ? updated : move)),
      );
      if (this.activeArmHold?.pinnedMoveId === id) {
        this.activeArmHold = updated.holdArmPose
          ? {
              ...this.activeArmHold,
              name: updated.name,
              trajectoryPath: updated.trajectoryPath,
            }
          : null;
      }
      if (this.keyboardOverrideArmHold?.pinnedMoveId === id) {
        this.keyboardOverrideArmHold = updated.holdArmPose
          ? {
              ...this.keyboardOverrideArmHold,
              name: updated.name,
              trajectoryPath: updated.trajectoryPath,
            }
          : null;
      }
      this.lastError = null;
      return this.getState();
    });
  }

  async deletePinnedMove(id: string): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      this.logActivity("Removing pinned move.");
      if (this.activeArmHold?.pinnedMoveId === id) {
        this.activeArmHold = null;
      }
      if (this.keyboardOverrideArmHold?.pinnedMoveId === id) {
        this.keyboardOverrideArmHold = null;
      }
      this.configStore.savePinnedMoves(
        config.pinnedMoves.filter((move) => move.id !== id),
      );
      this.lastError = null;
      return this.getState();
    });
  }

  async triggerPinnedMove(id: string): Promise<DashboardState> {
    if (this.pinnedMoveTriggersInFlight.has(id)) {
      return this.getState();
    }
    this.pinnedMoveTriggersInFlight.add(id);
    try {
      return await this.runExclusive(async () => {
        const config = this.configStore.getConfig();
        const pinnedMove = config.pinnedMoves.find((move) => move.id === id);
        if (!pinnedMove) {
          throw new Error("Pinned move not found.");
        }

        if (pinnedMove.holdArmPose) {
          if (this.keyboardOverrideArmHold?.pinnedMoveId === pinnedMove.id) {
            const releasedHold = this.keyboardOverrideArmHold;
            this.keyboardOverrideArmHold = null;
            this.clearReplayControlRestore();
            this.logActivity(`Arm hold keyboard override released to leader: ${pinnedMove.name}.`);
            await this.restoreControlAfterHoldRelease(config.settings, releasedHold.name, "leader");
            this.lastError = null;
            return this.getState();
          }

          if (this.activeArmHold?.pinnedMoveId === pinnedMove.id) {
            const releasedHold = this.activeArmHold;
            const releasedHoldName = releasedHold.name;
            const restoreArmSource =
              this.normalizeArmSourceMeta(releasedHold.restoreArmSource) ??
              (this.resolveLiveControlRestoreArmSource() === "keyboard" ? "keyboard" : null);
            this.activeArmHold = null;
            this.clearReplayControlRestore();
            this.logActivity(`Arm hold toggled off: ${pinnedMove.name}.`);
            const warmReplayActiveBeforeStop =
              this.deriveWarmHostReplaySnapshot(this.hostRunner.getSnapshot()).state === "running";
            const warmReplayStopRequestedAtMs = Date.now();
            await this.replayRunner.stop("Releasing arm hold.");
            await this.localReplayRunner.stop("Releasing arm hold.");
            if (warmReplayActiveBeforeStop) {
              await this.stopWarmHostReplay(config.settings, "Releasing arm hold.");
              await this.waitForWarmHostReplayStopped(warmReplayStopRequestedAtMs, 9000);
            }
            await this.restoreControlAfterHoldRelease(config.settings, releasedHoldName, restoreArmSource);
            this.keyboardOverrideArmHold = null;
            this.lastError = null;
            return this.getState();
          }

          if (pinnedMove.target !== "pi") {
            throw new Error("Arm hold pins must target the Pi follower.");
          }
          if (isDummyRecordingPath(pinnedMove.trajectoryPath)) {
            throw new Error("Dummy pinned moves are for offline UI testing only. Connect the Pi to hold a real arm pose.");
          }

          const previousHold = this.activeArmHold;
          const restoreArmSource = this.resolveLiveControlRestoreArmSource();
          this.keyboardOverrideArmHold = null;
          this.activeArmHold = {
            pinnedMoveId: pinnedMove.id,
            name: pinnedMove.name,
            trajectoryPath: pinnedMove.trajectoryPath,
            activatedAt: new Date().toISOString(),
            homePosition: previousHold?.trajectoryPath === pinnedMove.trajectoryPath
              ? previousHold.homePosition
              : null,
            restoreArmSource: restoreArmSource ?? "none",
          };

          const holdReplay: ReplayRequest = {
            ...pinnedMove,
            target: "pi",
            homeMode: "none",
            includeBase: false,
            autoVexPositioning: false,
            startTimeS: 0,
          };
          this.logActivity(`Arm hold toggled on: ${pinnedMove.name}.`);
          try {
            await this.ensureCommandReadyHost(config.settings, "arm hold replay");
            return await this.startReplayLocked(holdReplay, null, {
              returnToActiveHold: false,
            });
          } catch (error) {
            this.activeArmHold = previousHold;
            throw error;
          }
        }

        const activeArmHold = this.activeArmHold;
        const shouldReturnToHold =
          pinnedMove.target === "pi" && Boolean(activeArmHold);

        this.logActivity(
          shouldReturnToHold
            ? `Pinned move triggered: ${pinnedMove.name}; will replay held arm pose ${activeArmHold?.name} afterward.`
            : `Pinned move triggered: ${pinnedMove.name}.`,
        );
        if (shouldReturnToHold) {
          await this.ensureCommandReadyHost(config.settings, "held arm pinned replay");
        }
        return this.startReplayLocked(pinnedMove);
      });
    } finally {
      this.pinnedMoveTriggersInFlight.delete(id);
    }
  }

  async overrideActiveArmHoldWithKeyboard(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const activeHold = this.activeArmHold;
      if (!activeHold) {
        this.lastError = null;
        return this.getState();
      }

      this.keyboardOverrideArmHold = activeHold;
      this.activeArmHold = null;
      this.clearReplayControlRestore();
      this.logActivity(`Arm hold keyboard override: ${activeHold.name}.`);

      const warmReplayActiveBeforeStop =
        this.deriveWarmHostReplaySnapshot(this.hostRunner.getSnapshot()).state === "running";
      const warmReplayStopRequestedAtMs = Date.now();
      await this.replayRunner.stop("Keyboard arm control is overriding the hold.");
      await this.localReplayRunner.stop("Keyboard arm control is overriding the hold.");
      if (warmReplayActiveBeforeStop) {
        await this.stopWarmHostReplay(settings, "Keyboard arm control is overriding the hold.");
        await this.waitForWarmHostReplayStopped(warmReplayStopRequestedAtMs, 9000);
      }

      await this.restoreControlAfterHoldRelease(settings, activeHold.name, "keyboard");
      this.lastError = null;
      return this.getState();
    });
  }

  async returnToKeyboardOverrideArmHold(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const suspendedHold = this.keyboardOverrideArmHold;
      if (!suspendedHold || this.activeArmHold) {
        this.lastError = null;
        return this.getState();
      }

      const pinnedMove = config.pinnedMoves.find(
        (move) => move.id === suspendedHold.pinnedMoveId && move.holdArmPose,
      );
      if (!pinnedMove) {
        this.keyboardOverrideArmHold = null;
        throw new Error("The held arm pin is no longer available.");
      }
      if (pinnedMove.target !== "pi") {
        this.keyboardOverrideArmHold = null;
        throw new Error("Arm hold pins must target the Pi follower.");
      }
      if (isDummyRecordingPath(pinnedMove.trajectoryPath)) {
        throw new Error("Dummy pinned moves are for offline UI testing only. Connect the Pi to hold a real arm pose.");
      }

      const previousHold = this.activeArmHold;
      this.activeArmHold = {
        pinnedMoveId: pinnedMove.id,
        name: pinnedMove.name,
        trajectoryPath: pinnedMove.trajectoryPath,
        activatedAt: new Date().toISOString(),
        homePosition:
          suspendedHold.trajectoryPath === pinnedMove.trajectoryPath
            ? suspendedHold.homePosition
            : null,
        restoreArmSource: suspendedHold.restoreArmSource ?? "leader",
      };
      this.keyboardOverrideArmHold = null;

      const holdReplay: ReplayRequest = {
        ...pinnedMove,
        target: "pi",
        homeMode: "none",
        includeBase: false,
        autoVexPositioning: false,
        startTimeS: 0,
      };
      this.logActivity(`Returning to held arm pose: ${pinnedMove.name}.`);
      try {
        await this.ensureCommandReadyHost(config.settings, "return to held arm pose");
        return await this.startReplayLocked(holdReplay, null, {
          returnToActiveHold: false,
        });
      } catch (error) {
        this.activeArmHold = previousHold;
        this.keyboardOverrideArmHold = suspendedHold;
        throw error;
      }
    });
  }

  async saveChainLink(payload: SaveChainLinkRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      const id = typeof payload.id === "string" && payload.id.trim() ? payload.id.trim() : crypto.randomUUID();
      const items = (Array.isArray(payload.items) ? payload.items : [])
        .map((item) => {
          const replay = normalizeReplayRequest(item, config.settings.trajectories);
          return {
            id: typeof item.id === "string" && item.id.trim() ? item.id.trim() : crypto.randomUUID(),
            name:
              typeof item.name === "string" && item.name.trim()
                ? item.name.trim()
                : replay.trajectoryPath.split("/").pop() ?? "Recording",
            ...replay,
          };
        })
        .filter((item) => item.trajectoryPath);

      if (!items.length) {
        throw new Error("Add at least one recording before saving a Chain-link.");
      }

      const chainLink: ChainLink = {
        id,
        name: typeof payload.name === "string" && payload.name.trim() ? payload.name.trim() : "Chain-link",
        confirmAfterEach: payload.confirmAfterEach === false ? false : true,
        items,
      };
      const existingIndex = config.chainLinks.findIndex((chain) => chain.id === id);
      const chainLinks =
        existingIndex === -1
          ? [...config.chainLinks, chainLink]
          : config.chainLinks.map((chain) => (chain.id === id ? chainLink : chain));

      this.logActivity(`Saving Chain-link ${chainLink.name}.`);
      this.configStore.saveChainLinks(chainLinks);
      this.lastError = null;
      return this.getState();
    });
  }

  async deleteChainLink(id: string): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const config = this.configStore.getConfig();
      this.logActivity("Removing Chain-link.");
      this.configStore.saveChainLinks(config.chainLinks.filter((chain) => chain.id !== id));
      this.lastError = null;
      return this.getState();
    });
  }

  private async runExclusive<T>(operation: () => Promise<T>): Promise<T> {
    const previous = this.queue;
    let release: () => void = () => undefined;
    this.queue = new Promise<void>((resolve) => {
      release = resolve;
    });

    await previous;

    this.exclusiveOperationInFlight = true;
    try {
      return await operation();
    } catch (error) {
      this.noteError(error);
      throw error;
    } finally {
      this.exclusiveOperationInFlight = false;
      release();
    }
  }

  private async withRemoteHandshakeRetries<T>(
    label: string,
    task: (markHandshakeComplete: () => void) => Promise<T>,
  ): Promise<T> {
    const maxAttempts = REMOTE_HANDSHAKE_RETRY_DELAYS_MS.length + 1;

    for (let attempt = 1; attempt <= maxAttempts; attempt += 1) {
      let handshakeComplete = false;
      try {
        return await task(() => {
          handshakeComplete = true;
        });
      } catch (error) {
        if (
          handshakeComplete ||
          !isTransientRemoteTransportError(error) ||
          attempt === maxAttempts
        ) {
          throw error;
        }

        const delayMs = REMOTE_HANDSHAKE_RETRY_DELAYS_MS[attempt - 1] ?? 0;
        this.logActivity(
          `${label} SSH handshake dropped (${errorMessage(error)}); retrying ${attempt + 1}/${maxAttempts}.`,
        );
        await sleep(delayMs);
      }
    }

    throw new Error(`${label} failed before the SSH handshake could complete.`);
  }

  private noteError(error: unknown): void {
    if (isTransientRemoteTransportError(error)) {
      if (Date.now() < this.suppressRemoteTransportErrorsUntil) {
        this.logActivity(`Suppressed reset transport error: ${errorMessage(error)}`);
        return;
      }
      this.lastError =
        "The Pi SSH connection dropped while the backend was talking to it. Reset Pi Connections, confirm the Pi is still reachable, then retry.";
      this.logActivity(`Error: ${this.lastError} (${errorMessage(error)})`);
      return;
    }
    this.lastError = errorMessage(error);
    this.logActivity(`Error: ${this.lastError}`);
  }

  private isPiConnectivityError(message: string | null): boolean {
    if (!message) {
      return false;
    }
    return (
      message.includes("Pi SSH connection") ||
      message.includes("Pi is not reachable") ||
      message.includes("Reset Pi Connections")
    );
  }

  private noteBackgroundRemoteError(error: unknown): void {
    if (isTransientRemoteTransportError(error)) {
      return;
    }
    if (isIncompleteJsonOutputError(error)) {
      if (Date.now() >= this.suppressRemoteTransportErrorsUntil) {
        this.logActivity("Background Pi refresh returned incomplete JSON; retrying on the next refresh.");
      }
      return;
    }
    this.noteError(error);
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
    const configuredHosts = [settings.pi.host, settings.pi.fallbackHost].filter(Boolean);
    const candidates = [
      this.lastResolvedHost && configuredHosts.includes(this.lastResolvedHost)
        ? this.lastResolvedHost
        : null,
      ...configuredHosts,
    ].filter((host, index, hosts): host is string => Boolean(host) && hosts.indexOf(host) === index);

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
    readyPattern?: RegExp,
  ): Promise<void> {
    if (!readyPattern) {
      const exited = await runner.waitForExit(timeoutMs);
      if (exited) {
        throw new Error(this.formatStartupExit(label, exited));
      }
      return;
    }

    const deadline = Date.now() + timeoutMs;
    while (Date.now() <= deadline) {
      const snapshot = runner.getSnapshot();
      if (snapshot.logs.some((line) => readyPattern.test(line))) {
        return;
      }

      const exited = await runner.waitForExit(150);
      if (exited) {
        throw new Error(this.formatStartupExit(label, exited));
      }
    }

    throw new Error(`${label} did not report ready within ${Math.round(timeoutMs / 1000)} seconds.`);
  }

  private isServiceActive(snapshot: ServiceSnapshot): boolean {
    return snapshot.state === "starting" || snapshot.state === "running" || snapshot.state === "stopping";
  }

  private normalizeRecordingInputMode(
    value: StartRecordingRequest["inputMode"],
  ): "auto" | "leader" | "keyboard" | "free-teach" {
    if (value === "leader" || value === "keyboard" || value === "free-teach") {
      return value;
    }
    return "auto";
  }

  private normalizeBetaRecordingType(
    value: unknown,
    fallback: CreateMoveRecordingVersionRequest["recordingType"] = "keyboardControl",
  ): NonNullable<CreateMoveRecordingVersionRequest["recordingType"]> {
    if (
      value === "leaderArm" ||
      value === "followerHandGuide" ||
      value === "keyboardFromActiveHold"
    ) {
      return value;
    }
    return fallback ?? "keyboardControl";
  }

  private normalizeMoveRecordingSafetyStatus(
    value: unknown,
  ): NonNullable<CreateMoveRecordingVersionRequest["safetyStatus"]> {
    if (value === "clear" || value === "latched") {
      return value;
    }
    return "unknown";
  }

  private async startKeyboardControlSession(
    settings: AppSettings,
    host: string,
    meta: Record<string, unknown> = {},
    syncVexTelemetry = true,
    options: {
      allowLeader?: boolean;
      disableArmInput?: boolean;
      armSource?: "leader" | "keyboard" | "none";
      baseSource?: "keyboard" | "vexController";
    } = {},
  ): Promise<void> {
    assertHelperExists(HOST_SCRIPT);
    assertHelperExists(POWER_SCRIPT);
    assertHelperExists(RUNTIME_SCRIPT);
    assertHelperExists(KEYBOARD_TELEOP_SCRIPT);
    await this.ensureRemoteHelpers(settings, host);
    if (syncVexTelemetry && settings.vex.autoRunTelemetry) {
      await this.syncVexTelemetryProgramOnPi(
        settings,
        host,
        "start keyboard backup control",
        false,
      );
    }
    await this.writeRemoteTorqueLimits(settings, host);
    const keyboardJointLimitsJson = await this.loadKeyboardJointLimitsJson(settings, host);
    this.rememberVexControlModeFromSnapshot(this.teleopRunner.getSnapshot());
    const requestedArmSource = options.disableArmInput ? "none" : options.armSource;
    const allowLeader =
      options.allowLeader !== false &&
      requestedArmSource !== "keyboard" &&
      requestedArmSource !== "none";
    const leader = allowLeader ? await this.getLeaderStatus(settings) : null;
    if (leader?.connected) {
      this.acceptLeaderPoseForFollower("Leader arm control restore selected");
      this.logActivity(leader.message);
    }
    const leaderConnected = Boolean(leader?.connected);
    if (requestedArmSource === "leader" && !leaderConnected) {
      throw new Error("Leader arm restore was requested, but the leader arm is not connected.");
    }
    const armSource = requestedArmSource ?? (leaderConnected ? "leader" : "keyboard");
    if (armSource === "keyboard") {
      this.markLeaderPoseStale("keyboard arm authority selected for restored control");
    }
    const baseSource = this.normalizeLiveBaseSource(options.baseSource);
    const inputLabel = this.liveInputLabel(armSource, baseSource, leaderConnected);

    await this.clearRemoteHostCommand(settings, host);
    await this.hostRunner.start(
      this.buildHostScript(settings, true, {
        vexLiveBaseControl: baseSource === "keyboard",
        requireVexLiveBaseControl: false,
        releaseVexControllerBase: baseSource === "vexController",
      }),
      this.toConnectConfig(settings, host),
      "keyboard-control",
      {
        host,
        hotkeysArmed: true,
        keyboardBackup: true,
        input: inputLabel,
        leaderPort: leaderConnected ? leader?.expectedPort : null,
        armSource,
        baseSource,
        keyboardBaseInput: baseSource === "keyboard" ? "enabled" : "disabled",
        armInput: options.disableArmInput ? "disabled" : "enabled",
        vexLiveBaseControl: baseSource === "keyboard",
        ...meta,
      },
    );

    try {
      await this.waitForHostReady(host, "Keyboard backup host");
      await this.teleopRunner.start(
        this.buildKeyboardTeleopScript(
          settings,
          host,
          keyboardJointLimitsJson,
          leaderConnected ? leader : null,
          {
            disableArmInput: Boolean(options.disableArmInput),
            disableBaseInput: baseSource === "vexController",
            armSource,
          },
        ),
        "keyboard-control",
        {
          host,
          input: inputLabel,
          leaderPort: leaderConnected ? leader?.expectedPort : null,
          armSource,
          baseSource,
          keyboardBaseInput: baseSource === "keyboard" ? "enabled" : "disabled",
          armInput: options.disableArmInput ? "disabled" : "enabled",
          vexControlMode: baseSource === "keyboard" ? this.vexKeyboardControlMode : undefined,
          ...meta,
        },
      );
      await this.waitForLocalProcessReady(
        baseSource === "keyboard"
          ? leaderConnected ? "Leader arm + keyboard base teleop" : "Keyboard/base teleop"
          : leaderConnected ? "Leader arm + VEX controller base teleop" : "VEX controller base teleop",
        this.teleopRunner,
        KEYBOARD_TELEOP_READY_TIMEOUT_MS,
        / is live\./,
      );
    } catch (error) {
      await this.teleopRunner.stop("Keyboard backup control failed to launch.");
      await this.hostRunner.stop("Keyboard backup control failed to launch.");
      throw error;
    }
  }

  private async returnToActiveArmHoldPose(
    settings: AppSettings,
    actionLabel: string,
    reason: string,
  ): Promise<string> {
    const activeArmHold = this.activeArmHold;
    if (!activeArmHold) {
      throw new Error("No active arm hold pose is available.");
    }

    this.logActivity(`Replaying held arm recording ${activeArmHold.name} ${reason}.`);
    this.markLeaderPoseStale(`follower replayed held arm pose ${activeArmHold.name}`);
    const host = await this.ensureCommandReadyHost(settings, actionLabel);
    await this.teleopRunner.stop(`Stopping live command stream before replaying held arm pose ${reason}.`);
    const replayLogBaseline = this.warmHostReplayLogCounts();
    const requestedAtMs = Date.now();
    await this.sendWarmHostReplay(
      settings,
      host,
      this.buildActiveHoldReplayRequest(settings, activeArmHold),
      null,
    );
    await this.waitForWarmHostReplayStopped(requestedAtMs, 120000, replayLogBaseline);

    this.logActivity(`Held arm pose ${activeArmHold.name} reached ${reason}.`);
    return host;
  }

  private buildActiveHoldReplayRequest(
    settings: AppSettings,
    activeArmHold: ActiveArmHold,
  ): ReplayRequest {
    const pinnedHold = this.configStore
      .getConfig()
      .pinnedMoves.find((move) => move.id === activeArmHold.pinnedMoveId);
    return {
      trajectoryPath: activeArmHold.trajectoryPath,
      target: "pi",
      vexReplayMode: pinnedHold?.vexReplayMode ?? "ecu",
      homeMode: "none",
      speed: pinnedHold?.speed ?? settings.trajectories.defaultReplaySpeed,
      autoVexPositioning: false,
      vexPositioningSpeed: pinnedHold?.vexPositioningSpeed ?? DEFAULT_VEX_POSITIONING_SPEED,
      vexPositioningTimeoutS: pinnedHold?.vexPositioningTimeoutS ?? DEFAULT_VEX_POSITIONING_TIMEOUT_S,
      vexPositioningXyToleranceM: pinnedHold?.vexPositioningXyToleranceM ?? DEFAULT_VEX_POSITIONING_XY_TOLERANCE_M,
      vexPositioningHeadingToleranceDeg:
        pinnedHold?.vexPositioningHeadingToleranceDeg ?? DEFAULT_VEX_POSITIONING_HEADING_TOLERANCE_DEG,
      vexPositioningXyTrimToleranceM:
        pinnedHold?.vexPositioningXyTrimToleranceM ?? DEFAULT_VEX_POSITIONING_XY_TRIM_TOLERANCE_M,
      vexPositioningHeadingTrimToleranceDeg:
        pinnedHold?.vexPositioningHeadingTrimToleranceDeg ?? DEFAULT_VEX_POSITIONING_HEADING_TRIM_TOLERANCE_DEG,
      includeBase: false,
      holdFinalS: pinnedHold?.holdFinalS ?? settings.trajectories.defaultHoldFinalS,
      startTimeS: 0,
    };
  }

  private async restoreBaseOnlyKeyboardControlForActiveHold(
    settings: AppSettings,
    host: string,
    meta: Record<string, unknown>,
    reason: string,
  ): Promise<void> {
    const holdName = this.activeArmHold?.name ?? "active hold";
    this.logActivity(`Restoring base-only keyboard control while holding ${holdName} ${reason}.`);
    const warmHost = this.tryGetWarmHostForHomeCommand();
    if (warmHost === host && this.warmHostHasVexLiveBaseControl()) {
      await this.ensureLocalBaseOnlyKeyboardControlForWarmHost(
        settings,
        host,
        {
          ...meta,
          activeArmHold: this.activeArmHold?.name ?? null,
        },
        reason,
      );
      return;
    }

    await this.startKeyboardControlSession(
      settings,
      host,
      {
        ...meta,
        activeArmHold: this.activeArmHold?.name ?? null,
      },
      true,
      {
        allowLeader: false,
        disableArmInput: true,
      },
    );
  }

  private localTeleopIsBaseOnlyForHost(host: string): boolean {
    const snapshot = this.teleopRunner.getSnapshot();
    return (
      snapshot.state === "running" &&
      snapshot.meta.host === host &&
      snapshot.meta.armSource === "none" &&
      snapshot.meta.armInput === "disabled"
    );
  }

  private async ensureLocalBaseOnlyKeyboardControlForWarmHost(
    settings: AppSettings,
    host: string,
    meta: Record<string, unknown>,
    reason: string,
  ): Promise<void> {
    if (this.localTeleopIsBaseOnlyForHost(host)) {
      return;
    }
    await this.startLocalBaseOnlyKeyboardControlForWarmHost(settings, host, meta, reason);
  }

  private warmHostHasVexLiveBaseControl(): boolean {
    const hostSnapshot = this.hostRunner.getSnapshot();
    return (
      hostSnapshot.state === "running" &&
      this.hostSupportsInlineReplay(hostSnapshot.mode) &&
      hostSnapshot.meta.vexLiveBaseControl === true
    );
  }

  private async startLocalBaseOnlyKeyboardControlForWarmHost(
    settings: AppSettings,
    host: string,
    meta: Record<string, unknown>,
    reason: string,
  ): Promise<void> {
    this.rememberVexControlModeFromSnapshot(this.teleopRunner.getSnapshot());
    await this.teleopRunner.stop(`Restoring base-only keyboard control ${reason}.`);
    await this.teleopRunner.start(
      this.buildKeyboardTeleopScript(settings, host, "{}", null, {
        disableArmInput: true,
        armSource: "none",
      }),
      "keyboard-control",
      {
        host,
        input: "base-only",
        keyboardBackup: true,
        leaderPort: null,
        armSource: "none",
        armInput: "disabled",
        vexControlMode: this.vexKeyboardControlMode,
        ...meta,
      },
    );
    await this.waitForLocalProcessReady(
      "Keyboard/base teleop",
      this.teleopRunner,
      KEYBOARD_TELEOP_READY_TIMEOUT_MS,
      / is live\./,
    );
  }

  private async restoreControlAfterHoldRelease(
    settings: AppSettings,
    releasedHoldName: string,
    requestedArmSource: "leader" | "keyboard" | "none" | null,
  ): Promise<void> {
    let armSource = requestedArmSource;
    let leader: LeaderStatus | null = null;

    if (armSource === null) {
      leader = await this.getLeaderStatus(settings);
      armSource = leader.connected ? "leader" : null;
    }
    if (!armSource || armSource === "none") {
      this.logActivity(
        `Arm hold released: ${releasedHoldName}. Base keyboard control remains active.`,
      );
      return;
    }

    const hostSnapshot = this.hostRunner.getSnapshot();
    const host = this.getActiveHostAddress();
    if (hostSnapshot.state !== "running" || !this.hostSupportsInlineReplay(hostSnapshot.mode) || !host) {
      this.logActivity(
        `Arm hold released: ${releasedHoldName}. ${armSource} control was not restored because no warm Pi host is running.`,
      );
      return;
    }

    if (armSource === "leader") {
      leader = leader ?? (await this.getLeaderStatus(settings));
      if (!leader.connected) {
        this.logActivity(
          `Arm hold released: ${releasedHoldName}. Leader control was requested but the leader arm is not connected; base keyboard control remains active.`,
        );
        return;
      }
      this.acceptLeaderPoseForFollower("Arm hold released; restoring leader arm control");
      this.logActivity(leader.message);
    } else {
      this.markLeaderPoseStale("keyboard arm authority restored after arm hold release");
    }

    const mode =
      hostSnapshot.mode === "control" || hostSnapshot.mode === "keyboard-control"
        ? hostSnapshot.mode
        : "keyboard-control";
    this.rememberVexControlModeFromSnapshot(this.teleopRunner.getSnapshot());
    await this.teleopRunner.stop(`Restoring ${armSource} control after arm hold release.`);
    await this.teleopRunner.start(
      this.buildKeyboardTeleopScript(
        settings,
        host,
        "{}",
        armSource === "leader" ? leader : null,
        { armSource },
      ),
      mode,
      {
        host,
        input: armSource === "leader" ? "leader+keyboard-base" : "keyboard",
        keyboardBackup: true,
        leaderPort: armSource === "leader" ? leader?.expectedPort ?? null : null,
        armSource,
        armInput: "enabled",
        vexControlMode: this.vexKeyboardControlMode,
        activeArmHoldReleased: releasedHoldName,
      },
    );
    await this.waitForLocalProcessReady(
      armSource === "leader" ? "Leader arm + keyboard base teleop" : "Keyboard/base teleop",
      this.teleopRunner,
      KEYBOARD_TELEOP_READY_TIMEOUT_MS,
      / is live\./,
    );
    this.logActivity(`Arm hold released: ${releasedHoldName}; restored ${armSource} control.`);
  }

  private async runLocalLeaderMimicPoseScript(script: string, timeoutMs = 30000): Promise<void> {
    this.logActivity("Starting local leader mimic.");
    await new Promise<void>((resolve, reject) => {
      const child = spawn("/bin/zsh", ["-lc", script], {
        stdio: ["ignore", "pipe", "pipe"],
        env: process.env,
      });
      let settled = false;
      let stdoutBuffer = "";
      let stderrBuffer = "";
      const outputTail: string[] = [];

      const finish = (error?: Error): void => {
        if (settled) {
          return;
        }
        settled = true;
        clearTimeout(timeout);
        if (error) {
          reject(error);
          return;
        }
        resolve();
      };
      const recordLine = (source: "stdout" | "stderr", line: string): void => {
        const trimmed = line.trim();
        if (!trimmed) {
          return;
        }
        outputTail.push(trimmed);
        outputTail.splice(0, Math.max(0, outputTail.length - 8));
        this.logActivity(`[leader mimic ${source}] ${trimmed}`);
      };
      const consume = (source: "stdout" | "stderr", chunk: Buffer): void => {
        const next = (source === "stdout" ? stdoutBuffer : stderrBuffer) + chunk.toString("utf8");
        const lines = next.split(/\r?\n/);
        const remainder = lines.pop() ?? "";
        for (const line of lines) {
          recordLine(source, line);
        }
        if (source === "stdout") {
          stdoutBuffer = remainder;
        } else {
          stderrBuffer = remainder;
        }
      };

      const timeout = setTimeout(() => {
        const tail = outputTail.length ? ` Last output: ${outputTail.join(" | ")}` : "";
        this.logActivity("Local leader mimic timed out; stopping helper.");
        child.kill("SIGTERM");
        setTimeout(() => {
          if (!child.killed) {
            child.kill("SIGKILL");
          }
        }, 2000).unref();
        finish(new Error(`Timed out waiting for local leader mimic after ${timeoutMs / 1000}s.${tail}`));
      }, timeoutMs);
      timeout.unref();

      child.stdout.on("data", (chunk: Buffer) => consume("stdout", chunk));
      child.stderr.on("data", (chunk: Buffer) => consume("stderr", chunk));
      child.once("error", (error) => finish(error));
      child.once("close", (code, signal) => {
        recordLine("stdout", stdoutBuffer);
        recordLine("stderr", stderrBuffer);
        if (settled) {
          return;
        }
        if (code === 0) {
          finish();
          return;
        }
        const tail = outputTail.length ? ` Last output: ${outputTail.join(" | ")}` : "";
        finish(new Error(`Local leader mimic exited with code ${code ?? "null"}${signal ? ` (${signal})` : ""}.${tail}`));
      });
    });
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

  private async waitForWarmHostReplayStopped(
    requestedAtMs: number,
    timeoutMs: number,
    replayLogBaseline?: { starts: number; finishes: number },
  ): Promise<void> {
    const deadline = Date.now() + timeoutMs;
    let sawRequestedReplayStart = false;
    while (Date.now() <= deadline) {
      const hostSnapshot = this.hostRunner.getSnapshot();
      const snapshot = this.deriveWarmHostReplaySnapshot(hostSnapshot);
      if (replayLogBaseline) {
        const counts = this.warmHostReplayLogCounts(hostSnapshot);
        sawRequestedReplayStart ||= counts.starts > replayLogBaseline.starts;
        if (
          sawRequestedReplayStart &&
          snapshot.state !== "running" &&
          counts.finishes > replayLogBaseline.finishes &&
          counts.finishes >= counts.starts
        ) {
          return;
        }
        await sleep(100);
        continue;
      }
      if (snapshot.state !== "running") {
        const stoppedAtMs = snapshot.stoppedAt ? Date.parse(snapshot.stoppedAt) : Date.now();
        if (!Number.isFinite(stoppedAtMs) || stoppedAtMs >= requestedAtMs - 1000) {
          return;
        }
      }
      await sleep(100);
    }
    throw new Error("Timed out waiting for the warm-host replay to stop before returning to hold.");
  }

  private warmHostReplayLogCounts(
    snapshot: ServiceSnapshot = this.hostRunner.getSnapshot(),
  ): { starts: number; finishes: number } {
    let starts = 0;
    let finishes = 0;
    for (const line of snapshot.logs) {
      if (/\[replay\] start\b/.test(line)) {
        starts += 1;
      } else if (/\[replay\] (complete|stopped|interrupted|error)\b/.test(line)) {
        finishes += 1;
      }
    }
    return { starts, finishes };
  }

  private markReplayControlRestorePending(
    replay: ReplayRequest,
    restoreArmSource: "leader" | "keyboard" | "none" | null,
    returnToActiveHold = false,
  ): void {
    this.replayControlRestore = {
      requestedAtMs: Date.now(),
      target: replay.target,
      trajectoryPath: replay.trajectoryPath,
      restoreArmSource,
      returnToActiveHold,
    };
  }

  private clearReplayControlRestore(): void {
    this.replayControlRestore = null;
  }

  private maybeRestoreControlAfterReplay(
    settings: AppSettings,
    replaySnapshot: ServiceSnapshot,
    localReplaySnapshot: ServiceSnapshot,
    warmHostReplaySnapshot: ServiceSnapshot,
  ): void {
    if (
      !this.replayControlRestore ||
      this.replayControlRestoreInFlight ||
      this.exclusiveOperationInFlight
    ) {
      return;
    }

    const completed =
      this.replayCompletedAfterRestoreRequest(replaySnapshot) ||
      this.replayCompletedAfterRestoreRequest(localReplaySnapshot) ||
      this.replayCompletedAfterRestoreRequest(warmHostReplaySnapshot);
    if (!completed) {
      return;
    }

    const restore = this.replayControlRestore;
    this.replayControlRestore = null;
    this.replayControlRestoreInFlight = this.runExclusive(async () => {
      const restoreBaseOnly = Boolean(this.activeArmHold) || restore.restoreArmSource === "none";
      const restoreArmSource = restoreBaseOnly ? "none" : restore.restoreArmSource;
      if (!restoreBaseOnly && restoreArmSource === null) {
        return;
      }
      this.logActivity(
        restoreBaseOnly
          ? `Replay finished; restoring base-only keyboard control${this.activeArmHold ? ` while holding ${this.activeArmHold.name}` : ""}.`
          : `Replay finished; restoring ${restoreArmSource} control for ${restore.trajectoryPath}.`,
      );
      const host =
        restore.returnToActiveHold && this.activeArmHold
          ? await this.returnToActiveArmHoldPose(
              settings,
              "restore held arm pose after replay",
              "after replay",
            )
          : await this.preparePi(settings, "restore keyboard base after replay");
      const restoreMeta = {
        restoredAfterReplay: true,
        replayTarget: restore.target,
        trajectoryPath: restore.trajectoryPath,
        activeArmHold: restoreBaseOnly ? this.activeArmHold?.name ?? null : null,
        restoredArmSource: restoreArmSource,
      };
      if (restoreBaseOnly && this.activeArmHold) {
        await this.restoreBaseOnlyKeyboardControlForActiveHold(
          settings,
          host,
          restoreMeta,
          "after replay",
        );
        this.lastError = null;
        return;
      }
      await this.startKeyboardControlSession(
        settings,
        host,
        restoreMeta,
        true,
        {
          allowLeader: restoreArmSource === "leader",
          disableArmInput: restoreBaseOnly,
          armSource: restoreArmSource ?? undefined,
        },
      );
      this.lastError = null;
    })
      .catch((error) => {
        this.noteError(error);
      })
      .finally(() => {
        this.replayControlRestoreInFlight = null;
      });
  }

  private replayCompletedAfterRestoreRequest(snapshot: ServiceSnapshot): boolean {
    if (!this.replayControlRestore) {
      return false;
    }
    const isReplaySnapshot =
      snapshot.mode === "replay" || snapshot.meta.transport === "warm-host";
    if (!isReplaySnapshot || snapshot.state !== "idle" || !snapshot.stoppedAt) {
      return false;
    }
    const stoppedAtMs = Date.parse(snapshot.stoppedAt);
    if (!Number.isFinite(stoppedAtMs) || stoppedAtMs < this.replayControlRestore.requestedAtMs) {
      return false;
    }
    return snapshot.exitCode === null || snapshot.exitCode === 0;
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

  private buildKeyboardTeleopScript(
    settings: AppSettings,
    host: string,
    jointLimitsJson: string,
    leader?: LeaderStatus | null,
    options: {
      disableArmInput?: boolean;
      disableBaseInput?: boolean;
      armSource?: "leader" | "keyboard" | "none";
    } = {},
  ): string {
    assertHelperExists(KEYBOARD_TELEOP_SCRIPT);
    const ecuLinearSpeed = KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS;
    const ecuTurnSpeed = KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS;
    const driveLinearSpeed = settings.vex.tuning.maxLinearSpeedMps;
    const driveTurnSpeed = settings.vex.tuning.maxTurnSpeedDps;
    return [
      this.buildMacActivation(settings),
      `python ${shellQuote(KEYBOARD_TELEOP_SCRIPT)} \\`,
      `  --remote-host ${shellQuote(host)} \\`,
      `  --robot-id ${shellQuote(settings.host.robotId)} \\`,
      `  --fps ${KEYBOARD_TELEOP_FPS} \\`,
      `  --print-every ${KEYBOARD_TELEOP_PRINT_EVERY} \\`,
      `  --base-linear-speed ${ecuLinearSpeed} \\`,
      `  --base-turn-speed ${ecuTurnSpeed} \\`,
      `  --drive-base-linear-speed ${driveLinearSpeed} \\`,
      `  --drive-base-turn-speed ${driveTurnSpeed} \\`,
      `  --initial-vex-control-mode ${this.vexKeyboardControlMode} \\`,
      `  --leader-port ${shellQuote(leader?.expectedPort ?? "off")} \\`,
      `  --arm-source ${shellQuote(options.armSource ?? (leader ? "leader" : "keyboard"))} \\`,
      `  --vex-keyboard-calibration-json ${shellQuote(JSON.stringify(settings.vex.keyboardCalibration))} \\`,
      `  --joint-limits-json ${shellQuote(jointLimitsJson)}${options.disableArmInput ? " \\\n  --disable-arm-input" : ""}${options.disableBaseInput ? " \\\n  --disable-base-input" : ""}`,
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
      inertial: structuredClone(settings.vex.inertial),
      motors: structuredClone(settings.vex.motors),
      controls: structuredClone(settings.vex.controls),
      keyboardCalibration: structuredClone(settings.vex.keyboardCalibration),
      pin5Servo: structuredClone(settings.vex.pin5Servo),
      manualIdleStoppingMode: settings.vex.manualIdleStoppingMode,
      tuning: {
        ...structuredClone(settings.vex.tuning),
        ecuLinearSpeedMps: KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS,
        ecuTurnSpeedDps: KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS,
      },
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

  private buildKeyboardJointLimitsScript(settings: AppSettings): string {
    return `
${this.buildPiActivation(settings)}
export PYTHONPATH=${shellQuote(`${settings.pi.remoteHelperDir}/scripts`)}:$PYTHONPATH
export LEKIWI_UI_ROBOT_ID=${shellQuote(settings.host.robotId)}
export LEKIWI_UI_ROBOT_PORT=${shellQuote(settings.pi.robotPort)}
python - <<'PY'
import json
import os

from lekiwi_runtime import build_normalized_arm_position_limits
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

fields = getattr(LeKiwiConfig, "__dataclass_fields__", {})
kwargs = {
    "id": os.environ["LEKIWI_UI_ROBOT_ID"],
    "port": os.environ["LEKIWI_UI_ROBOT_PORT"],
}
optional_values = {
    "use_degrees": True,
    "cameras": {},
    "enable_base": False,
}
for name, value in optional_values.items():
    if name in fields:
        kwargs[name] = value

robot = LeKiwi(LeKiwiConfig(**kwargs))
limits = build_normalized_arm_position_limits(robot, preserve_continuous_wrist_roll=True)
print(json.dumps(limits, sort_keys=True, separators=(",", ":")))
PY
`.trim();
  }

  private async loadKeyboardJointLimitsJson(
    settings: AppSettings,
    host: string,
  ): Promise<string> {
    try {
      const { stdout } = await this.execRemoteScript(
        this.buildKeyboardJointLimitsScript(settings),
        host,
        settings,
      );
      if (!stdout) {
        return "{}";
      }

      const parsed = JSON.parse(stdout);
      if (!parsed || typeof parsed !== "object" || Array.isArray(parsed)) {
        return "{}";
      }

      return JSON.stringify(parsed);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.logActivity(
        `Keyboard joint-limit preload skipped; Pi host clamps remain active. ${message}`,
      );
      return "{}";
    }
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

  private buildVexGyroZeroScript(settings: AppSettings): string {
    return `
export PYTHONPATH=${shellQuote(`${settings.pi.remoteHelperDir}/scripts`)}:$PYTHONPATH
export LEKIWI_VEX_CONTROL_CONFIG=${shellQuote(JSON.stringify(this.buildVexControlConfig(settings)))}
export LEKIWI_VEX_TELEMETRY_SLOT=${shellQuote(String(settings.vex.telemetrySlot))}
export LEKIWI_VEX_TELEMETRY_NAME=${shellQuote(settings.vex.telemetryProgramName)}
python3 - <<'PY'
import json
import logging
import os
import time

from vex_base_bridge import (
    VexBaseBridge,
    VexBaseTelemetryManager,
    ensure_vex_command_stream,
    normalize_vex_control_config,
)

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("vex_gyro_zero")

control_config = normalize_vex_control_config(json.loads(os.environ["LEKIWI_VEX_CONTROL_CONFIG"]))
bridge = VexBaseBridge(
    requested_port="auto",
    baudrate=115200,
    stale_after_s=0.35,
    command_timeout_s=0.35,
    logger=logger,
)
telemetry = VexBaseTelemetryManager(
    requested_vexcom_path="auto",
    telemetry_slot=int(os.environ["LEKIWI_VEX_TELEMETRY_SLOT"]),
    cache_dir="~/.lekiwi-vex/programs",
    logger=logger,
)

result = {
    "enabled": bridge.enabled or telemetry.enabled,
    "commandStream": False,
    "success": False,
    "status": telemetry.status_message,
    "gyro": None,
}

try:
    bridge.connect()
    result["enabled"] = bridge.enabled or telemetry.enabled
    result["commandStream"] = ensure_vex_command_stream(
        bridge,
        telemetry,
        control_config,
        program_name=os.environ["LEKIWI_VEX_TELEMETRY_NAME"],
        logger=logger,
        startup_delay_s=1.0,
        telemetry_timeout_s=6.0,
    )
    if not result["commandStream"]:
        result["status"] = "VEX Brain did not accept the USB gyro-zero command stream."
    else:
        result["success"] = bridge.set_pose_origin(ttl_ms=1200, timeout_s=2.0)
        time.sleep(0.2)
        bridge.poll()
        gyro = bridge.gyro_status_snapshot()
        result["gyro"] = gyro
        if result["success"]:
            result["status"] = gyro.get("message") or "VEX gyro origin zeroed."
        else:
            result["status"] = "Timed out waiting for the VEX Brain to confirm gyro zero."
finally:
    bridge.close()

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

  private buildRobotSensorStatus(
    label: string,
    axis: string | null,
    state: RobotSensorState,
    message: string,
    overrides: Partial<RobotSensorStatus> = {},
  ): RobotSensorStatus {
    return {
      label,
      axis,
      state,
      connected: false,
      value: null,
      unit: null,
      updatedAt: null,
      source: null,
      message,
      ...overrides,
    };
  }

  private normalizeRobotSensorState(value: unknown): RobotSensorState {
    return value === "online" ||
      value === "stale" ||
      value === "waiting" ||
      value === "missing" ||
      value === "idle"
      ? value
      : "idle";
  }

  private normalizeRobotSensorStatus(
    candidate: unknown,
    label: string,
    axis: string | null,
  ): RobotSensorStatus {
    if (!candidate || typeof candidate !== "object") {
      return this.buildRobotSensorStatus(label, axis, "idle", `${label} status is unavailable.`);
    }

    const payload = candidate as Record<string, unknown>;
    const updatedAt =
      typeof payload.updated_at === "string"
        ? payload.updated_at
        : typeof payload.updatedAt === "string"
          ? payload.updatedAt
          : null;
    const value =
      typeof payload.value === "number" && Number.isFinite(payload.value) ? payload.value : null;
    const unit = typeof payload.unit === "string" ? payload.unit : null;
    const source = typeof payload.source === "string" ? payload.source : null;
    const message =
      typeof payload.message === "string" && payload.message.trim()
        ? payload.message.trim()
        : `${label} status is unavailable.`;

    return this.buildRobotSensorStatus(
      label,
      axis,
      this.normalizeRobotSensorState(payload.state),
      message,
      {
        connected: Boolean(payload.connected),
        value,
        unit,
        updatedAt,
        source,
      },
    );
  }

  private coerceRobotSensorFreshness(status: RobotSensorStatus): RobotSensorStatus {
    if (!status.updatedAt || status.state === "missing" || status.state === "idle") {
      return status;
    }

    const updatedAtMs = Date.parse(status.updatedAt);
    if (!Number.isFinite(updatedAtMs)) {
      return status;
    }

    const ageMs = Date.now() - updatedAtMs;
    if (ageMs <= LIVE_SENSOR_STATUS_STALE_MS || status.state === "stale") {
      return status;
    }

    const ageS = Math.max(1, Math.round(ageMs / 1000));
    return {
      ...status,
      state: "stale",
      message: `${status.message} Last update ${ageS}s ago.`,
    };
  }

  private parseRobotSensorLogLine(
    line: string,
  ): { observedAtMs: number; payload: Record<string, unknown> } | null {
    const cleaned = this.cleanServiceLogLine(line);
    if (!cleaned.startsWith(SENSOR_STATUS_LOG_PREFIX)) {
      return null;
    }

    const rawPayload = cleaned.slice(SENSOR_STATUS_LOG_PREFIX.length).trim();
    try {
      const payload = JSON.parse(rawPayload);
      if (!payload || typeof payload !== "object") {
        return null;
      }

      const payloadRecord = payload as Record<string, unknown>;
      const payloadTimestamp =
        typeof payloadRecord.timestamp === "string" ? Date.parse(payloadRecord.timestamp) : Number.NaN;
      const logTimestamp = this.parseServiceLogTimestamp(line);
      const observedAtMs = Number.isFinite(payloadTimestamp)
        ? payloadTimestamp
        : logTimestamp
          ? Date.parse(logTimestamp)
          : 0;

      return {
        observedAtMs: Number.isFinite(observedAtMs) ? observedAtMs : 0,
        payload: payloadRecord,
      };
    } catch {
      return null;
    }
  }

  private extractLatestRobotSensorPayload(
    ...snapshots: ServiceSnapshot[]
  ): Record<string, unknown> | null {
    return this.extractLatestRobotSensorReading(...snapshots)?.payload ?? null;
  }

  private extractLatestRobotSensorReading(
    ...snapshots: ServiceSnapshot[]
  ): { observedAtMs: number; payload: Record<string, unknown> } | null {
    let latest: { observedAtMs: number; payload: Record<string, unknown> } | null = null;

    for (const snapshot of snapshots) {
      const match = [...snapshot.logs]
        .reverse()
        .map((line) => this.parseRobotSensorLogLine(line))
        .find((candidate): candidate is { observedAtMs: number; payload: Record<string, unknown> } => {
          return candidate !== null;
        });
      if (!match) {
        continue;
      }
      if (latest === null || match.observedAtMs >= latest.observedAtMs) {
        latest = match;
      }
    }

    return latest;
  }

  private buildFallbackGyroStatus(
    piReachable: boolean,
    vexBrain: VexBrainStatus,
  ): RobotSensorStatus {
    if (!piReachable) {
      return this.buildRobotSensorStatus(
        "Gyro",
        "rotation",
        "missing",
        "Pi is offline, so gyro status is unavailable.",
        { unit: "deg" },
      );
    }

    if (vexBrain.telemetryActive) {
      return this.buildRobotSensorStatus(
        "Gyro",
        "rotation",
        "online",
        vexBrain.message,
        {
          connected: true,
          unit: "deg",
          source: vexBrain.source,
        },
      );
    }

    if (vexBrain.connected) {
      return this.buildRobotSensorStatus(
        "Gyro",
        "rotation",
        "waiting",
        vexBrain.message,
        {
          connected: true,
          unit: "deg",
          source: vexBrain.source,
        },
      );
    }

    return this.buildRobotSensorStatus(
      "Gyro",
      "rotation",
      "missing",
      vexBrain.message,
      { unit: "deg" },
    );
  }

  private buildFallbackDistanceSensorStatus(
    label: string,
    axis: string,
    piReachable: boolean,
    liveServiceActive: boolean,
  ): RobotSensorStatus {
    if (!piReachable) {
      return this.buildRobotSensorStatus(
        label,
        axis,
        "missing",
        "Pi is offline, so sensor status is unavailable.",
        { unit: "m" },
      );
    }

    if (liveServiceActive) {
      return this.buildRobotSensorStatus(
        label,
        axis,
        "waiting",
        `Waiting for live ${axis.toUpperCase()} distance telemetry from the Pi.`,
        { unit: "m" },
      );
    }

    return this.buildRobotSensorStatus(
      label,
      axis,
      "idle",
      `Start Control, Recording, Replay, or Pi dataset capture to stream ${label}.`,
      { unit: "m" },
    );
  }

  private getRobotSensorsStatus(
    piReachable: boolean,
    vexBrain: VexBrainStatus,
    hostSnapshot: ServiceSnapshot,
    replaySnapshot: ServiceSnapshot,
    datasetCaptureSnapshot: ServiceSnapshot,
  ): RobotSensorsStatus {
    const liveServiceActive = [hostSnapshot, replaySnapshot, datasetCaptureSnapshot].some(
      (snapshot) => snapshot.state !== "idle",
    );
    const livePayload = liveServiceActive
      ? this.extractLatestRobotSensorPayload(hostSnapshot, replaySnapshot, datasetCaptureSnapshot)
      : null;

    const gyro = livePayload?.gyro
      ? this.coerceRobotSensorFreshness(
          this.normalizeRobotSensorStatus(livePayload.gyro, "Gyro", "rotation"),
        )
      : this.buildFallbackGyroStatus(piReachable, vexBrain);
    const x = livePayload?.x
      ? this.coerceRobotSensorFreshness(
          this.normalizeRobotSensorStatus(livePayload.x, "Sensor 1", "x"),
        )
      : this.buildFallbackDistanceSensorStatus("Sensor 1", "x", piReachable, liveServiceActive);
    const y = livePayload?.y
      ? this.coerceRobotSensorFreshness(
          this.normalizeRobotSensorStatus(livePayload.y, "Sensor 2", "y"),
        )
      : this.buildFallbackDistanceSensorStatus("Sensor 2", "y", piReachable, liveServiceActive);

    return { gyro, x, y };
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
    const availablePorts = this.deduplicateLeaderPorts(await this.listLocalLeaderPorts());

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

    const autoDetectedPort = availablePorts.length === 1 ? availablePorts[0] : null;

    if (!expectedPort) {
      if (autoDetectedPort) {
        return {
          teleoperateScriptPath,
          expectedPort: autoDetectedPort,
          connected: true,
          availablePorts,
          message: `Leader arm detected on ${autoDetectedPort}. The UI will use the detected port.`,
        };
      }
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

    if (autoDetectedPort) {
      return {
        teleoperateScriptPath,
        expectedPort: autoDetectedPort,
        connected: true,
        availablePorts,
        message: `Leader arm detected on ${autoDetectedPort}. teleoperate.py expects ${expectedPort}, so the UI will use the detected port instead.`,
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

  private deduplicateLeaderPorts(ports: string[]): string[] {
    const grouped = new Map<string, string[]>();
    for (const port of ports) {
      const key = this.normalizeLeaderPortKey(port);
      grouped.set(key, [...(grouped.get(key) ?? []), port]);
    }

    return [...grouped.values()]
      .map((options) => {
        return (
          options.find((item) => path.basename(item).startsWith("tty.")) ??
          options.slice().sort()[0]
        );
      })
      .sort();
  }

  private normalizeLeaderPortKey(port: string): string {
    const name = path.basename(port);
    if (name.startsWith("tty.")) {
      return name.slice(4);
    }
    if (name.startsWith("cu.")) {
      return name.slice(3);
    }
    return name;
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

  private buildHostScript(
    settings: AppSettings,
    includeUiCommandPath = false,
    options: {
      vexLiveBaseControl?: boolean;
      requireVexLiveBaseControl?: boolean;
      releaseVexControllerBase?: boolean;
    } = {},
  ): string {
    const uiCommandFlag = includeUiCommandPath
      ? ` \\\n  --ui-command-path ${shellQuote(this.getRemoteHostCommandPath(settings))}`
      : "";
    const saferServoModeFlag = this.buildSaferServoModeFlag(settings);
    const vexBaseFlags = this.buildVexBaseFlags(settings);
    const vexLiveBaseControlFlag = options.vexLiveBaseControl
      ? ` \\\n  --vex-live-base-control true`
      : "";
    const requireVexLiveBaseControlFlag = options.requireVexLiveBaseControl
      ? ` \\\n  --require-vex-live-base-control true`
      : "";
    const releaseVexControllerBaseFlag = options.releaseVexControllerBase
      ? ` \\\n  --release-vex-controller-base true`
      : "";
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
      `  --loop-hz ${CONTROL_LOOP_HZ} \\`,
      `  --enable-base false \\`,
      `${vexBaseFlags}${uiCommandFlag}${saferServoModeFlag}${vexLiveBaseControlFlag}${requireVexLiveBaseControlFlag}${releaseVexControllerBaseFlag}`,
    ].join("\n");
  }

  private buildRecordScript(
    settings: AppSettings,
    trajectoryPath: string,
    label: string,
    recordingInput: "leader" | "keyboard" | "free-teach",
  ): string {
    const labelFlag = label ? ` \\\n  --label ${shellQuote(label)}` : "";
    const saferServoModeFlag = this.buildSaferServoModeFlag(settings);
    const vexBaseFlags = this.buildVexBaseFlags(settings);
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
      `${vexBaseFlags} \\`,
      `  --output ${shellQuote(trajectoryPath)} \\`,
      `  --recording-mode ${shellQuote(recordingInput)}${labelFlag}${saferServoModeFlag}`,
    ].join("\n");
  }

  private buildReplayScript(
    settings: AppSettings,
    replay: ReplayRequest,
    homePosition: ArmHomePosition | null,
    options: { recaptureUltrasonicStream?: boolean } = {},
  ): string {
    const includeBaseFlag = replay.includeBase ? " \\\n  --include-base" : "";
    const autoVexPositioningFlag = replay.autoVexPositioning
      ? ""
      : " \\\n  --auto-vex-positioning false";
    const vexPositioningTimeoutFlag =
      ` \\\n  --vex-positioning-timeout-s ${replay.vexPositioningTimeoutS}`;
    const vexPositioningTolerancesFlags =
      ` \\\n  --vex-positioning-speed ${replay.vexPositioningSpeed}` +
      ` \\\n  --vex-positioning-xy-tolerance-m ${replay.vexPositioningXyToleranceM}` +
      ` \\\n  --vex-positioning-heading-tolerance-deg ${replay.vexPositioningHeadingToleranceDeg}` +
      ` \\\n  --vex-positioning-xy-trim-tolerance-m ${replay.vexPositioningXyTrimToleranceM}` +
      ` \\\n  --vex-positioning-heading-trim-tolerance-deg ${replay.vexPositioningHeadingTrimToleranceDeg}`;
    const recaptureUltrasonicFlag = options.recaptureUltrasonicStream
      ? " \\\n  --recapture-ultrasonic-stream"
      : "";
    const startTimeFlag = replay.startTimeS
      ? ` \\\n  --start-time-s ${replay.startTimeS}`
      : "";
    const homeFlags =
      replay.homeMode !== "none" && isFiniteHomePosition(homePosition)
        ? ` \\\n  --home-mode ${shellQuote(replay.homeMode)} \\\n  --home-position-json ${shellQuote(JSON.stringify(homePosition.joints))}`
        : "";
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
      `  --vex-replay-mode ${shellQuote(replay.vexReplayMode)} \\`,
      `  --hold-final-s ${replay.holdFinalS}${startTimeFlag}${includeBaseFlag}${autoVexPositioningFlag}${vexPositioningTimeoutFlag}${vexPositioningTolerancesFlags}${recaptureUltrasonicFlag}${homeFlags}${saferServoModeFlag}`,
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
      `  --hold-final-s ${replay.holdFinalS} \\`,
      `  --start-time-s ${replay.startTimeS ?? 0}`,
    ].join("\n");
  }

  private buildLocalLeaderMimicPoseScript(
    settings: AppSettings,
    leader: LeaderStatus,
    homePosition: ArmHomePosition,
  ): string {
    assertHelperExists(LOCAL_LEADER_MIMIC_POSE_SCRIPT);
    return [
      this.buildMacActivation(settings),
      `python ${shellQuote(LOCAL_LEADER_MIMIC_POSE_SCRIPT)} \\`,
      `  --robot-id ${shellQuote(LOCAL_LEADER_ROBOT_ID)} \\`,
      `  --robot-port ${shellQuote(leader.expectedPort ?? "auto")} \\`,
      `  --home-position-json ${shellQuote(JSON.stringify(homePosition.joints))} \\`,
      "  --fps 60 \\",
      "  --settle-s 0.5",
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
export PYTHONPATH=${shellQuote(`${settings.pi.remoteHelperDir}/scripts`)}:$PYTHONPATH

python3 - <<'PY' || true
import glob
import time

try:
    import serial
except Exception as exc:
    print(f"VEX base release skipped: pyserial unavailable: {exc}")
    raise SystemExit(0)

preferred_suffixes = ("if02", "if00")

def rank(path):
    for index, suffix in enumerate(preferred_suffixes):
        if suffix in path:
            return index
    return len(preferred_suffixes)

accepted_ports = []
for port in sorted(glob.glob("/dev/serial/by-id/*VEX_Robotics_V5_Brain*"), key=rank):
    handle = None
    try:
        handle = serial.Serial(port, baudrate=115200, timeout=0.05, write_timeout=0.25)
        for command in (b"!release\\n", b"!telemetry off\\n", b"!telemetry off\\n"):
            handle.write(command)
            handle.flush()
            time.sleep(0.08)
        accepted_ports.append(port)
        print(f"Sent VEX base release and telemetry shutdown on {port}.")
    except Exception as exc:
        print(f"Failed VEX base release/telemetry shutdown on {port}: {exc}")
    finally:
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

if accepted_ports:
    print(f"Sent VEX base shutdown commands on {', '.join(accepted_ports)}.")
else:
    print("VEX base release skipped: no V5 Brain serial port accepted shutdown commands.")
PY

python - <<'PY'
import os
from lekiwi_runtime import resolve_lekiwi_robot_port
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
robot_port = resolve_lekiwi_robot_port(os.environ.get("LEKIWI_ROBOT_PORT", "auto"))
motor_names = [
  "shoulder_pan",
  "shoulder_lift",
  "elbow_flex",
  "wrist_flex",
  "wrist_roll",
  "gripper",
]
motors = {
  "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
  "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
  "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
  "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
  "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_0_100),
  "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}
bus = FeetechMotorsBus(port=robot_port, motors=motors)
try:
    bus.connect()
    disabled = []
    failed = []
    for motor_name in motor_names:
      try:
        bus.disable_torque(motor_name)
        disabled.append(motor_name)
      except Exception as exc:
        failed.append(f"{motor_name}: {exc}")
finally:
    bus.disconnect(disable_torque=False)

print(f"Disabled torque on: {', '.join(disabled) if disabled else 'none'}")
if failed:
    print("Failed to disable torque on:")
    for item in failed:
      print(f"  - {item}")
PY
`;
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

  private canUseWarmHostVexCommand(): boolean {
    const hostSnapshot = this.hostRunner.getSnapshot();
    return (
      hostSnapshot.state === "running" &&
      this.hostSupportsInlineReplay(hostSnapshot.mode)
    );
  }

  private async ensureCommandReadyHost(
    settings: AppSettings,
    actionLabel: string,
  ): Promise<string> {
    const existingHost = this.tryGetWarmHostForHomeCommand();
    if (existingHost) {
      return existingHost;
    }

    const host = await this.preparePi(settings, actionLabel);
    assertHelperExists(HOST_SCRIPT);
    assertHelperExists(POWER_SCRIPT);
    assertHelperExists(RUNTIME_SCRIPT);
    await this.ensureRemoteHelpers(settings, host);
    if (settings.vex.autoRunTelemetry) {
      await this.syncVexTelemetryProgramOnPi(settings, host, actionLabel, false);
    }
    await this.writeRemoteTorqueLimits(settings, host);

    await this.stopRobotExclusiveProcesses(`Preparing command-ready Pi host for ${actionLabel}.`);
    await this.clearRemoteHostCommand(settings, host);

    await this.hostRunner.start(
      this.buildHostScript(settings, true, {
        vexLiveBaseControl: true,
        requireVexLiveBaseControl: false,
      }),
      this.toConnectConfig(settings, host),
      "keyboard-control",
      {
        host,
        hotkeysArmed: true,
        commandReady: true,
        vexLiveBaseControl: true,
      },
    );

    try {
      await this.waitForHostReady(host, "Command-ready Pi host");
    } catch (error) {
      await this.hostRunner.stop("Command-ready Pi host failed to launch.");
      throw error;
    }

    return host;
  }

  private tryGetWarmHostForHomeCommand(): string | null {
    const hostSnapshot = this.hostRunner.getSnapshot();
    if (
      hostSnapshot.state !== "running" ||
      !this.hostSupportsInlineReplay(hostSnapshot.mode)
    ) {
      return null;
    }

    return this.getActiveHostAddress();
  }

  private async zeroVexGyroViaWarmHost(settings: AppSettings, host: string): Promise<void> {
    const requestId = `zero-vex-gyro-${Date.now()}-${Math.random().toString(36).slice(2)}`;
    await this.writeRemoteHostCommand(settings, host, {
      command: "zero-vex-gyro",
      requestId,
    });
    const result = await this.waitForWarmHostVexCommandResult(requestId, 6000);
    if (!result) {
      throw new Error("Timed out waiting for the running Pi host to confirm VEX gyro zero.");
    }
    if (!result.success) {
      throw new Error(result.status || "The running Pi host failed to zero the VEX gyro.");
    }
  }

  private async waitForWarmHostVexCommandResult(
    requestId: string,
    timeoutMs: number,
  ): Promise<{ success: boolean; status: string } | null> {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() <= deadline) {
      const parsed = this.findWarmHostVexCommandResult(requestId);
      if (parsed) {
        return parsed;
      }
      await sleep(100);
    }
    return this.findWarmHostVexCommandResult(requestId);
  }

  private findWarmHostVexCommandResult(
    requestId: string,
  ): { success: boolean; status: string } | null {
    const marker = "[vex-command] ";
    const logs = this.hostRunner.getSnapshot().logs;
    for (let index = logs.length - 1; index >= 0; index -= 1) {
      const line = logs[index] ?? "";
      const markerIndex = line.indexOf(marker);
      if (markerIndex < 0) {
        continue;
      }
      const jsonText = line.slice(markerIndex + marker.length).trim();
      try {
        const payload = JSON.parse(jsonText) as Record<string, unknown>;
        if (
          payload.command === "zero-vex-gyro" &&
          payload.request_id === requestId
        ) {
          return {
            success: payload.success === true,
            status:
              typeof payload.status === "string" && payload.status.trim()
                ? payload.status.trim()
                : "VEX gyro zero command finished.",
          };
        }
      } catch {
        continue;
      }
    }
    return null;
  }

  private async waitForWarmHostHomeCommandResult(
    requestId: string,
    timeoutMs: number,
  ): Promise<{ success: boolean; status: string; homePosition: ArmHomePosition | null } | null> {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() <= deadline) {
      const parsed = this.findWarmHostHomeCommandResult(requestId);
      if (parsed) {
        return parsed;
      }
      await sleep(100);
    }
    return this.findWarmHostHomeCommandResult(requestId);
  }

  private findWarmHostHomeCommandResult(
    requestId: string,
  ): { success: boolean; status: string; homePosition: ArmHomePosition | null } | null {
    const logs = this.hostRunner.getSnapshot().logs;
    for (let index = logs.length - 1; index >= 0; index -= 1) {
      const line = logs[index] ?? "";
      const markerIndex = line.indexOf(`${HOME_COMMAND_LOG_PREFIX} `);
      if (markerIndex < 0) {
        continue;
      }
      const jsonText = line.slice(markerIndex + HOME_COMMAND_LOG_PREFIX.length + 1).trim();
      try {
        const payload = JSON.parse(jsonText) as Record<string, unknown>;
        if (payload.request_id !== requestId) {
          continue;
        }
        const positions = payload.positions;
        const homePosition =
          positions && typeof positions === "object"
            ? {
                capturedAt:
                  typeof payload.captured_at === "string" && payload.captured_at.trim()
                    ? payload.captured_at
                    : new Date().toISOString(),
                joints: Object.fromEntries(
                  ARM_HOME_JOINT_KEYS.map((key) => [
                    key,
                    Number((positions as Record<string, unknown>)[key]),
                  ]),
                ),
              }
            : null;
        return {
          success: payload.success === true,
          status:
            typeof payload.status === "string" && payload.status.trim()
              ? payload.status.trim()
              : "Home command finished.",
          homePosition,
        };
      } catch {
        continue;
      }
    }
    return null;
  }

  private async sendWarmHostReplay(
    settings: AppSettings,
    host: string,
    replay: ReplayRequest,
    homePosition: ArmHomePosition | null,
  ): Promise<void> {
    await this.writeRemoteHostCommand(settings, host, {
      command: "replay",
      trajectoryPath: replay.trajectoryPath,
      speed: replay.speed,
      holdFinalS: replay.holdFinalS,
      startTimeS: replay.startTimeS ?? 0,
      includeBase: replay.includeBase,
      autoVexPositioning: replay.autoVexPositioning,
      vexPositioningSpeed: replay.vexPositioningSpeed,
      vexPositioningTimeoutS: replay.vexPositioningTimeoutS,
      vexPositioningXyToleranceM: replay.vexPositioningXyToleranceM,
      vexPositioningHeadingToleranceDeg: replay.vexPositioningHeadingToleranceDeg,
      vexPositioningXyTrimToleranceM: replay.vexPositioningXyTrimToleranceM,
      vexPositioningHeadingTrimToleranceDeg: replay.vexPositioningHeadingTrimToleranceDeg,
      vexReplayMode: replay.vexReplayMode,
      homeMode: replay.homeMode,
      homePosition,
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
          startTimeS: number;
          includeBase: boolean;
          autoVexPositioning: boolean;
          vexPositioningSpeed: number;
          vexPositioningTimeoutS: number;
          vexPositioningXyToleranceM: number;
          vexPositioningHeadingToleranceDeg: number;
          vexPositioningXyTrimToleranceM: number;
          vexPositioningHeadingTrimToleranceDeg: number;
          vexReplayMode: "drive" | "ecu";
          homeMode: ArmHomeMode;
          homePosition: ArmHomePosition | null;
        }
      | { command: "stop-replay" }
      | { command: "zero-vex-gyro"; requestId: string }
      | { command: "capture-home"; requestId: string }
      | { command: "go-home"; requestId: string; homePosition: ArmHomePosition },
  ): Promise<void> {
    let payload: Record<string, unknown>;
    if (command.command === "replay") {
      payload = {
        command: "replay",
        trajectory_path: command.trajectoryPath,
        speed: command.speed,
        hold_final_s: command.holdFinalS,
        start_time_s: command.startTimeS,
        include_base: command.includeBase,
        auto_vex_positioning: command.autoVexPositioning,
        vex_positioning_speed: command.vexPositioningSpeed,
        vex_positioning_timeout_s: command.vexPositioningTimeoutS,
        vex_positioning_xy_tolerance_m: command.vexPositioningXyToleranceM,
        vex_positioning_heading_tolerance_deg: command.vexPositioningHeadingToleranceDeg,
        vex_positioning_xy_trim_tolerance_m: command.vexPositioningXyTrimToleranceM,
        vex_positioning_heading_trim_tolerance_deg:
          command.vexPositioningHeadingTrimToleranceDeg,
        vex_replay_mode: command.vexReplayMode,
        home_mode: command.homeMode,
        home_position: isFiniteHomePosition(command.homePosition)
          ? command.homePosition.joints
          : null,
        requested_at: new Date().toISOString(),
      };
    } else if (command.command === "zero-vex-gyro") {
      payload = {
        command: "zero-vex-gyro",
        request_id: command.requestId,
        requested_at: new Date().toISOString(),
      };
    } else if (command.command === "capture-home") {
      payload = {
        command: "capture-home",
        request_id: command.requestId,
        requested_at: new Date().toISOString(),
      };
    } else if (command.command === "go-home") {
      payload = {
        command: "go-home",
        request_id: command.requestId,
        home_position: command.homePosition.joints,
        requested_at: new Date().toISOString(),
      };
    } else {
      payload = {
        command: "stop-replay",
        requested_at: new Date().toISOString(),
      };
    }
    const serialized = JSON.stringify(payload);
    const warmHostCommandStatus = this.sendWarmHostCommandInput(
      host,
      serialized,
      String(payload.command ?? "command"),
    );
    if (warmHostCommandStatus === "sent") {
      return;
    }
    if (warmHostCommandStatus === "failed") {
      throw new Error(
        "The running Pi host command channel is unavailable. Stop and start Live Control once to reopen it.",
      );
    }

    const commandPath = this.getRemoteHostCommandPath(settings);
    const script = `
mkdir -p ${shellQuote(path.posix.dirname(commandPath))}
cat > ${shellQuote(commandPath)} <<'JSON'
${serialized}
JSON
`.trim();
    await this.execRemoteScript(script, host, settings);
  }

  private sendWarmHostCommandInput(
    host: string,
    serialized: string,
    label: string,
  ): "sent" | "unavailable" | "failed" {
    const hostSnapshot = this.hostRunner.getSnapshot();
    if (
      hostSnapshot.state !== "running" ||
      !this.hostSupportsInlineReplay(hostSnapshot.mode)
    ) {
      return "unavailable";
    }

    const activeHost = this.getActiveHostAddress();
    if (activeHost && activeHost !== host) {
      return "unavailable";
    }

    return this.hostRunner.sendInput(`${serialized}\n`, `warm-host ${label}`)
      ? "sent"
      : "failed";
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
        this.fastPut(sftp, SENSOR_REPLAY_SCRIPT, `${helperDir}/lekiwi_sensor_replay.py`),
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
    const validatedPath = await this.validateRemoteRecordingPath(settings, host, recordingPath);
    const rawText = await this.downloadRemoteRecordingText(
      settings,
      host,
      validatedPath,
    );
    try {
      return this.buildRecordingDetailFromPayload(validatedPath, JSON.parse(rawText));
    } catch (error) {
      if (isIncompleteJsonOutputError(error)) {
        throw new Error(
          `Recording JSON is incomplete or corrupted: ${validatedPath}. If this file was created during a dropped connection, record it again or delete the partial file.`,
        );
      }
      throw error;
    }
  }

  private async validateRemoteRecordingPath(
    settings: AppSettings,
    host: string,
    recordingPath: string,
  ): Promise<string> {
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
print(json.dumps({"path": str(path)}))
PY
`.trim();

    const { stdout } = await this.execRemoteScript(script, host, settings);
    const parsed = JSON.parse(stdout) as { path?: unknown };
    if (typeof parsed.path !== "string" || !parsed.path.trim()) {
      throw new Error("Remote recording path validation returned an invalid response.");
    }
    return parsed.path;
  }

  private async downloadRemoteRecordingText(
    settings: AppSettings,
    host: string,
    remotePath: string,
  ): Promise<string> {
    await fs.promises.mkdir(LOCAL_RECORDING_DETAIL_DIR, { recursive: true });
    const localPath = path.join(
      LOCAL_RECORDING_DETAIL_DIR,
      `${path.basename(remotePath, ".json")}-${crypto.randomUUID()}.json`,
    );
    try {
      await this.withSftp(settings, host, async (sftp) => {
        await this.fastGet(sftp, remotePath, localPath);
      });
      return await fs.promises.readFile(localPath, "utf8");
    } finally {
      await fs.promises.rm(localPath, { force: true }).catch(() => undefined);
    }
  }

  private async writeRecordingServoAdjustment(
    settings: AppSettings,
    host: string,
    recordingPath: string,
    timeS: number,
    values: Record<string, number>,
  ): Promise<void> {
    const script = `
export LEKIWI_TRAJECTORY_DIR=${shellQuote(settings.trajectories.remoteDir)}
export LEKIWI_RECORDING_PATH=${shellQuote(recordingPath)}
export LEKIWI_ADJUST_TIME_S=${shellQuote(String(timeS))}
export LEKIWI_SERVO_VALUES_JSON=${shellQuote(JSON.stringify(values))}
python - <<'PY'
from datetime import datetime
import json
import os
from pathlib import Path

ARM_STATE_KEYS = (
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
)

root = Path(os.environ["LEKIWI_TRAJECTORY_DIR"]).expanduser().resolve()
path = Path(os.environ["LEKIWI_RECORDING_PATH"]).expanduser().resolve()
time_s = float(os.environ["LEKIWI_ADJUST_TIME_S"])
values = json.loads(os.environ["LEKIWI_SERVO_VALUES_JSON"])

if root not in path.parents:
    raise SystemExit("Recording path is outside the trajectory directory.")
if not path.is_file():
    raise SystemExit(f"Recording does not exist: {path}")
if not isinstance(values, dict):
    raise SystemExit("Servo values payload is invalid.")

normalized_values = {}
for key in ARM_STATE_KEYS:
    value = values.get(key)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"Servo values payload is missing numeric {key}.")
    normalized_values[key] = float(value)

payload = json.loads(path.read_text())
if not isinstance(payload, dict):
    raise SystemExit("Recording payload is invalid.")

def nearest_index(points):
    if not isinstance(points, list):
        return None
    best_index = None
    best_distance = None
    for index, item in enumerate(points):
        if not isinstance(item, dict):
            continue
        raw_t_s = item.get("t_s")
        if not isinstance(raw_t_s, (int, float)):
            continue
        distance = abs(float(raw_t_s) - time_s)
        if best_distance is None or distance < best_distance:
            best_index = index
            best_distance = distance
    return best_index

def update_point(points, value_key):
    index = nearest_index(points)
    if index is None:
        return None
    item = points[index]
    target = item.get(value_key)
    if not isinstance(target, dict):
        target = {}
        item[value_key] = target
    for key, value in normalized_values.items():
        target[key] = value
    return float(item.get("t_s", time_s))

sample_time_s = update_point(payload.get("samples"), "state")
command_time_s = update_point(payload.get("command_samples"), "action")
if sample_time_s is None and command_time_s is None:
    raise SystemExit("Recording does not contain any editable servo samples.")

payload["arm_state_keys"] = list(ARM_STATE_KEYS)
adjustments = payload.get("servo_adjustments")
if not isinstance(adjustments, list):
    adjustments = []
adjustments.append(
    {
        "time_s": round(time_s, 6),
        "sample_time_s": round(sample_time_s, 6) if sample_time_s is not None else None,
        "command_time_s": round(command_time_s, 6) if command_time_s is not None else None,
        "values": normalized_values,
        "adjusted_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    }
)
payload["servo_adjustments"] = adjustments[-100:]

tmp_path = path.with_name(f".{path.name}.tmp")
tmp_path.write_text(json.dumps(payload, indent=2) + "\\n")
tmp_path.replace(path)
PY
`.trim();

    await this.execRemoteScript(script, host, settings);
  }

  private async readRecordingStartHomePosition(
    settings: AppSettings,
    host: string,
    recordingPath: string,
  ): Promise<ArmHomePosition> {
    return this.readRecordingHomePosition(settings, host, recordingPath, "start");
  }

  private async readRecordingEndHomePosition(
    settings: AppSettings,
    host: string,
    recordingPath: string,
  ): Promise<ArmHomePosition> {
    return this.readRecordingHomePosition(settings, host, recordingPath, "end");
  }

  private async readRecordingHomePosition(
    settings: AppSettings,
    host: string,
    recordingPath: string,
    samplePosition: "start" | "end",
  ): Promise<ArmHomePosition> {
    const script = `
export LEKIWI_TRAJECTORY_DIR=${shellQuote(settings.trajectories.remoteDir)}
export LEKIWI_RECORDING_PATH=${shellQuote(recordingPath)}
export LEKIWI_SAMPLE_POSITION=${shellQuote(samplePosition)}
python - <<'PY'
import json
import os
from datetime import datetime, timezone
from pathlib import Path

ARM_HOME_KEYS = ${JSON.stringify(ARM_HOME_JOINT_KEYS)}

root = Path(os.environ["LEKIWI_TRAJECTORY_DIR"]).expanduser().resolve()
path = Path(os.environ["LEKIWI_RECORDING_PATH"]).expanduser().resolve()
sample_position = os.environ["LEKIWI_SAMPLE_POSITION"]

if root not in path.parents:
    raise SystemExit("Recording path is outside the trajectory directory.")
if not path.is_file():
    raise SystemExit(f"Recording does not exist: {path}")

payload = json.loads(path.read_text())
if not isinstance(payload, dict):
    raise SystemExit("Recording payload is invalid.")

samples = payload.get("samples")
if not isinstance(samples, list) or not samples:
    raise SystemExit("Recording does not contain any state samples.")

sample = samples[-1] if sample_position == "end" else samples[0]
sample_label = "final" if sample_position == "end" else "first"
if not isinstance(sample, dict):
    raise SystemExit(f"Recording {sample_label} sample is invalid.")

state = sample.get("state")
if not isinstance(state, dict):
    raise SystemExit(f"Recording {sample_label} sample does not contain state.")

joints = {}
for key in ARM_HOME_KEYS:
    value = state.get(key)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"Recording {sample_label} sample is missing numeric {key}.")
    joints[key] = float(value)

print(json.dumps({
    "capturedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    "joints": joints,
}))
PY
`.trim();

    const { stdout } = await this.execRemoteScript(script, host, settings);
    return JSON.parse(stdout) as ArmHomePosition;
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

    return this.withRemoteHandshakeRetries(
      "Remote command",
      (markHandshakeComplete) => new Promise((resolve, reject) => {
        const client = new Client();
        this.trackRemoteClient(client);
        let stdout = "";
        let stderr = "";
        let settled = false;

        const fail = (error: Error) => {
          if (settled) {
            return;
          }
          settled = true;
          client.destroy();
          reject(error);
        };
        const succeed = (value: { stdout: string; stderr: string }) => {
          if (settled) {
            return;
          }
          settled = true;
          resolve(value);
        };
        const handleError = (error: Error) => {
          fail(error);
        };
        const cleanupErrorHandler = () => {
          client.off("error", handleError);
        };

        client.once("ready", () => {
          markHandshakeComplete();
          client.exec(`/bin/bash -lc ${shellQuote(script)}`, (error, channel) => {
            if (error) {
              fail(error);
              return;
            }

            channel.on("data", (chunk: Buffer) => {
              stdout += chunk.toString("utf8");
            });

            channel.stderr.on("data", (chunk: Buffer) => {
              stderr += chunk.toString("utf8");
            });

            channel.once("close", (code: number | undefined | null) => {
              if (code && code !== 0) {
                fail(new Error(stderr.trim() || stdout.trim() || `Remote command failed with code ${code}.`));
                return;
              }
              client.end();
              succeed({ stdout: stdout.trim(), stderr: stderr.trim() });
            });
          });
        });

        client.on("error", handleError);
        client.once("close", cleanupErrorHandler);
        client.once("end", cleanupErrorHandler);
        client.connect(connection);
      }),
    );
  }

  private async withSftp<T>(
    settings: AppSettings,
    host: string,
    callback: (sftp: SFTPWrapper) => Promise<T>,
  ): Promise<T> {
    const connection = this.toConnectConfig(settings, host);

    return this.withRemoteHandshakeRetries(
      "SFTP session",
      (markHandshakeComplete) => new Promise<T>((resolve, reject) => {
        const client = new Client();
        this.trackRemoteClient(client);
        let settled = false;

        const fail = (error: Error) => {
          if (settled) {
            return;
          }
          settled = true;
          client.destroy();
          reject(error);
        };
        const succeed = (value: T) => {
          if (settled) {
            return;
          }
          settled = true;
          resolve(value);
        };
        const handleError = (error: Error) => {
          fail(error);
        };
        const cleanupErrorHandler = () => {
          client.off("error", handleError);
        };

        client.once("ready", () => {
          markHandshakeComplete();
          client.sftp(async (error, sftp) => {
            if (error || !sftp) {
              fail(error ?? new Error("SFTP could not be opened."));
              return;
            }

            try {
              const result = await callback(sftp);
              client.end();
              succeed(result);
            } catch (callbackError) {
              fail(callbackError instanceof Error ? callbackError : new Error(String(callbackError)));
            }
          });
        });

        client.on("error", handleError);
        client.once("close", cleanupErrorHandler);
        client.once("end", cleanupErrorHandler);
        client.connect(connection);
      }),
    );
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
      SENSOR_REPLAY_SCRIPT,
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

  private async duplicateRemoteRecordingFile(
    settings: AppSettings,
    host: string,
    recordingPath: string,
  ): Promise<{ path: string; label: string }> {
    const script = `
export LEKIWI_TRAJECTORY_DIR=${shellQuote(settings.trajectories.remoteDir)}
export LEKIWI_RECORDING_PATH=${shellQuote(recordingPath)}
python - <<'PY'
import json
import os
import re
from datetime import datetime, timezone
from pathlib import Path

root = Path(os.environ["LEKIWI_TRAJECTORY_DIR"]).expanduser().resolve()
requested = Path(os.environ["LEKIWI_RECORDING_PATH"]).expanduser()

try:
    path = requested.resolve(strict=True)
except FileNotFoundError:
    raise SystemExit(f"Recording does not exist: {requested}")

if path.parent != root:
    raise SystemExit("Recording path is outside the trajectory directory.")
if path.suffix != ".json":
    raise SystemExit("Only trajectory JSON recordings can be duplicated.")
if not path.is_file():
    raise SystemExit(f"Recording is not a file: {path}")

payload = json.loads(path.read_text())
if not isinstance(payload, dict):
    raise SystemExit("Recording payload is invalid.")

def base_copy_label(value):
    normalized = value.strip()
    match = re.match(r"^(?P<base>.+?) copy(?: (?P<index>\\d+))?$", normalized, flags=re.IGNORECASE)
    return match.group("base").strip() if match else normalized

def base_copy_stem(value):
    match = re.match(r"^(?P<base>.+?)-copy(?:-(?P<index>\\d+))?$", value, flags=re.IGNORECASE)
    return match.group("base") if match else value

def existing_label_values():
    labels = set()
    for candidate in root.glob("*.json"):
        try:
            candidate_payload = json.loads(candidate.read_text())
        except Exception:
            candidate_payload = None
        if isinstance(candidate_payload, dict):
            raw_label = candidate_payload.get("label")
            if isinstance(raw_label, str) and raw_label.strip():
                labels.add(raw_label.strip().lower())
                continue
        labels.add(candidate.stem.lower())
    return labels

source_label = payload.get("label")
source_name = source_label.strip() if isinstance(source_label, str) and source_label.strip() else path.stem
label_base = base_copy_label(source_name)
labels = existing_label_values()
next_label = f"{label_base} copy"
label_index = 2
while next_label.lower() in labels:
    next_label = f"{label_base} copy {label_index}"
    label_index += 1

stem_base = base_copy_stem(path.stem)
next_path = root / f"{stem_base}-copy.json"
path_index = 2
while next_path.exists():
    next_path = root / f"{stem_base}-copy-{path_index}.json"
    path_index += 1

duplicated_from = payload.get("duplicated_from")
root_source_path = None
if isinstance(duplicated_from, dict):
    raw_root_source = duplicated_from.get("root_source_path")
    if isinstance(raw_root_source, str) and raw_root_source.strip():
        root_source_path = raw_root_source.strip()

payload["label"] = next_label
payload["duplicated_from"] = {
    "source_path": str(path),
    "source_label": source_name,
    "root_source_path": root_source_path or str(path),
    "duplicated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
}

next_path.write_text(json.dumps(payload, indent=2) + "\\n")
print(json.dumps({"path": str(next_path), "label": next_label}))
PY
`.trim();

    const { stdout } = await this.execRemoteScript(script, host, settings);
    return JSON.parse(stdout) as { path: string; label: string };
  }

  private async deleteRemoteRecordingFile(
    settings: AppSettings,
    host: string,
    recordingPath: string,
  ): Promise<void> {
    const script = `
export LEKIWI_TRAJECTORY_DIR=${shellQuote(settings.trajectories.remoteDir)}
export LEKIWI_RECORDING_PATH=${shellQuote(recordingPath)}
python - <<'PY'
import json
import os
from pathlib import Path

root = Path(os.environ["LEKIWI_TRAJECTORY_DIR"]).expanduser().resolve()
requested = Path(os.environ["LEKIWI_RECORDING_PATH"]).expanduser()

try:
    path = requested.resolve(strict=True)
except FileNotFoundError:
    raise SystemExit(f"Recording does not exist: {requested}")

if path.parent != root:
    raise SystemExit("Recording path is outside the trajectory directory.")
if path.suffix != ".json":
    raise SystemExit("Only trajectory JSON recordings can be deleted.")
if not path.is_file():
    raise SystemExit(f"Recording is not a file: {path}")

path.unlink()
print(json.dumps({"ok": True, "path": str(path)}))
PY
`.trim();

    await this.execRemoteScript(script, host, settings);
  }

  private async markRecordingStartGyroZero(
    settings: AppSettings,
    host: string,
    recordingPath: string,
  ): Promise<void> {
    const script = `
export LEKIWI_TRAJECTORY_DIR=${shellQuote(settings.trajectories.remoteDir)}
export LEKIWI_RECORDING_PATH=${shellQuote(recordingPath)}
python - <<'PY'
import json
import os
from datetime import datetime, timezone
from pathlib import Path

GYRO_ROTATION_KEY = "vex_inertial_rotation.deg"
GYRO_HEADING_KEY = "vex_inertial_heading.deg"
GYRO_RATE_KEY = "vex_inertial_rate_z.dps"
POSE_EPOCH_KEY = "vex_pose_epoch"

root = Path(os.environ["LEKIWI_TRAJECTORY_DIR"]).expanduser().resolve()
path = Path(os.environ["LEKIWI_RECORDING_PATH"]).expanduser().resolve()

if root not in path.parents:
    raise SystemExit("Recording path is outside the trajectory directory.")
if not path.is_file():
    raise SystemExit(f"Recording does not exist: {path}")

payload = json.loads(path.read_text())
if not isinstance(payload, dict):
    raise SystemExit("Recording payload is invalid.")

samples = payload.get("samples")
if not isinstance(samples, list) or not samples:
    raise SystemExit("Recording does not contain any samples.")

first_sample = samples[0]
if not isinstance(first_sample, dict):
    raise SystemExit("Recording first sample is invalid.")

state = first_sample.get("state")
if not isinstance(state, dict):
    raise SystemExit("Recording first sample does not contain state.")

state[GYRO_ROTATION_KEY] = 0.0
state[GYRO_HEADING_KEY] = 0.0
state[GYRO_RATE_KEY] = 0.0
state[POSE_EPOCH_KEY] = 1.0

def append_unique_list(key, values):
    existing = payload.get(key)
    if not isinstance(existing, list):
        existing = []
    normalized = [item for item in existing if isinstance(item, str)]
    for value in values:
        if value not in normalized:
            normalized.append(value)
    payload[key] = normalized

append_unique_list("vex_inertial_state_keys", [GYRO_ROTATION_KEY, GYRO_HEADING_KEY, GYRO_RATE_KEY])
append_unique_list("vex_pose_state_keys", [POSE_EPOCH_KEY])
append_unique_list("vex_state_keys", [GYRO_ROTATION_KEY, GYRO_HEADING_KEY, GYRO_RATE_KEY, POSE_EPOCH_KEY])
payload["gyro_zero_reference"] = {
    "type": "manual-recording-start",
    "heading_deg": 0.0,
    "pose_epoch": 1,
    "marked_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
}

path.write_text(json.dumps(payload, indent=2) + "\\n")
print(json.dumps({"ok": True, "path": str(path)}))
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
    await this.withRemoteHandshakeRetries(
      "Remote dataset download",
      (markHandshakeComplete) => new Promise<void>((resolve, reject) => {
        const client = new Client();
        this.trackRemoteClient(client);
        let settled = false;

        const fail = (error: Error) => {
          if (settled) {
            return;
          }
          settled = true;
          client.destroy();
          reject(error);
        };
        const succeed = () => {
          if (settled) {
            return;
          }
          settled = true;
          resolve();
        };
        const handleError = (error: Error) => {
          fail(error);
        };
        const cleanupErrorHandler = () => {
          client.off("error", handleError);
        };

        client.once("ready", () => {
          markHandshakeComplete();
          const remoteCommand = [
            `if [ ! -d ${shellQuote(remoteDir)} ]; then`,
            `  echo "Missing directory: ${remoteDir}" >&2`,
            "  exit 1",
            "fi",
            `tar -C ${shellQuote(remoteDir)} -czf - .`,
          ].join("\n");

          client.exec(`/bin/bash -lc ${shellQuote(remoteCommand)}`, (error, channel) => {
            if (error) {
              fail(error);
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
              if (code && code !== 0) {
                fail(new Error(`Local tar extraction failed with code ${code}.`));
                return;
              }
              client.end();
              succeed();
            });

            channel.once("close", (code: number | undefined | null) => {
              if (code && code !== 0) {
                fail(new Error(`Remote dataset archive failed with code ${code}.`));
              }
            });
          });
        });

        client.on("error", handleError);
        client.once("close", cleanupErrorHandler);
        client.once("end", cleanupErrorHandler);
        client.connect(connection);
      }),
    );
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
    await this.withRemoteHandshakeRetries(
      "Remote dataset upload",
      (markHandshakeComplete) => new Promise<void>((resolve, reject) => {
        const client = new Client();
        this.trackRemoteClient(client);
        let settled = false;

        const fail = (error: Error) => {
          if (settled) {
            return;
          }
          settled = true;
          client.destroy();
          reject(error);
        };
        const succeed = () => {
          if (settled) {
            return;
          }
          settled = true;
          resolve();
        };
        const handleError = (error: Error) => {
          fail(error);
        };
        const cleanupErrorHandler = () => {
          client.off("error", handleError);
        };

        client.once("ready", () => {
          markHandshakeComplete();
          const remoteCommand = [
            `rm -rf ${shellQuote(remoteDir)}`,
            `mkdir -p ${shellQuote(remoteDir)}`,
            `tar -xzf - -C ${shellQuote(remoteDir)}`,
          ].join("\n");

          client.exec(`/bin/bash -lc ${shellQuote(remoteCommand)}`, (error, channel) => {
            if (error) {
              fail(error);
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
                fail(new Error(`Local tar archive failed with code ${code}.`));
              }
            });

            channel.once("close", (code: number | undefined | null) => {
              if (code && code !== 0) {
                fail(new Error(`Remote extract failed with code ${code}.`));
                return;
              }
              client.end();
              succeed();
            });
          });
        });

        client.on("error", handleError);
        client.once("close", cleanupErrorHandler);
        client.once("end", cleanupErrorHandler);
        client.connect(connection);
      }),
    );
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

  private trackRemoteClient(client: Client): void {
    this.activeRemoteClients.add(client);
    const untrack = () => {
      this.activeRemoteClients.delete(client);
    };
    client.once("close", untrack);
    client.once("end", untrack);
    client.once("error", untrack);
  }

  private closeActiveRemoteClients(reason: string): number {
    const clients = [...this.activeRemoteClients];
    if (!clients.length) {
      return 0;
    }

    this.logActivity(reason);
    for (const client of clients) {
      try {
        client.destroy();
      } catch {
        // The SSH client may already be closing.
      }
      this.activeRemoteClients.delete(client);
    }
    return clients.length;
  }

  private logActivity(message: string): void {
    const stamped = `${new Date().toLocaleTimeString()} ${message}`;
    this.activityLog.push(stamped);
    if (this.activityLog.length > MAX_ACTIVITY_LINES) {
      this.activityLog.splice(0, this.activityLog.length - MAX_ACTIVITY_LINES);
    }
  }
}
