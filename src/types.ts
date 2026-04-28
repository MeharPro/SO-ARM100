export interface HotspotSettings {
  ssid: string;
}

export interface PiSettings {
  host: string;
  fallbackHost: string;
  username: string;
  password: string;
  projectDir: string;
  condaScript: string;
  robotPort: string;
  remoteHelperDir: string;
}

export interface MacSettings {
  projectDir: string;
  condaScript: string;
}

export interface HostSettings {
  connectionTimeS: number;
  robotId: string;
  camerasJson: string;
  baseMaxRawVelocity: number;
  baseWheelTorqueLimit: number;
  enableBase: boolean;
  saferServoMode: boolean;
  armTorqueLimits: Record<string, number>;
}

export interface TrajectorySettings {
  remoteDir: string;
  defaultReplaySpeed: number;
  defaultHoldFinalS: number;
}

export type VexControllerAxis = "axis1" | "axis2" | "axis3" | "axis4";
export type VexReplayMode = "drive" | "ecu";

export interface VexDriveMotorSettings {
  port: number;
  reversed: boolean;
}

export interface VexDriveControls {
  forwardAxis: VexControllerAxis;
  strafeAxis: VexControllerAxis;
  turnAxis: VexControllerAxis;
  invertForward: boolean;
  invertStrafe: boolean;
  invertTurn: boolean;
}

export interface VexDriveTuning {
  deadbandPercent: number;
  maxLinearSpeedMps: number;
  maxTurnSpeedDps: number;
}

export interface VexInertialSettings {
  port: number;
}

export interface VexSettings {
  controlPresetVersion: number;
  telemetrySlot: number;
  replaySlot: number;
  autoRunTelemetry: boolean;
  telemetryProgramName: string;
  inertial: VexInertialSettings;
  motors: {
    frontRight: VexDriveMotorSettings;
    frontLeft: VexDriveMotorSettings;
    rearRight: VexDriveMotorSettings;
    rearLeft: VexDriveMotorSettings;
  };
  controls: VexDriveControls;
  tuning: VexDriveTuning;
}

export type TrainingCaptureMode = "leader" | "free-teach" | "leader-as-follower";

export type ReplayTarget = "pi" | "leader";
export type ArmHomeMode = "none" | "start" | "end" | "both";

export interface ArmHomePosition {
  capturedAt: string;
  joints: Record<string, number>;
}

export interface RecordingReplayOptions {
  homeMode: ArmHomeMode;
  speed: number;
  autoVexPositioning: boolean;
  vexPositioningSpeed: number;
  vexPositioningTimeoutS: number;
  vexPositioningXyToleranceM: number;
  vexPositioningHeadingToleranceDeg: number;
  vexPositioningXyTrimToleranceM: number;
  vexPositioningHeadingTrimToleranceDeg: number;
}

export interface PolicyBenchmarkResult {
  targetFps: number;
  measuredAt: string;
  iterations: number;
  averageLatencyMs: number;
  p95LatencyMs: number;
  maxLatencyMs: number;
  effectiveFps: number;
  peakRssMb: number | null;
  passed: boolean;
}

export interface TrainingArtifact {
  datasetEpisodeCount: number | null;
  datasetCameraKeys: string[];
  lastDatasetEpisodeIndex: number | null;
  lastDatasetSyncAt: string | null;
  lastTrainingStartedAt: string | null;
  lastTrainingCompletedAt: string | null;
  lastCheckpointPath: string | null;
  availableCheckpointPaths: string[];
  deployedCheckpointPath: string | null;
  deployedAt: string | null;
  latestEvalDatasetPath: string | null;
  latestEvalAt: string | null;
  benchmark: PolicyBenchmarkResult | null;
}

export interface TrainingProfile {
  id: string;
  name: string;
  task: string;
  captureMode: TrainingCaptureMode;
  numEpisodes: number;
  episodeTimeS: number;
  resetTimeS: number;
  fps: number;
  camerasMode: string;
  policyType: "act";
  piDatasetPath: string;
  macDatasetPath: string;
  macTrainOutputDir: string;
  selectedCheckpointPath: string;
  piDeployPath: string;
  piEvalDatasetPath: string;
  artifacts: TrainingArtifact;
}

export interface TrainingSettings {
  defaultPolicyType: "act";
  localDevice: "mps";
  benchmarkIterations: number;
  deployBenchmarkMargin: number;
}

export interface TrainingConfig {
  settings: TrainingSettings;
  profiles: TrainingProfile[];
  selectedProfileId: string | null;
}

export interface AppSettings {
  hotspot: HotspotSettings;
  pi: PiSettings;
  mac: MacSettings;
  host: HostSettings;
  trajectories: TrajectorySettings;
  vex: VexSettings;
}

export interface ServiceSnapshot {
  label: string;
  state: "idle" | "starting" | "running" | "stopping" | "error";
  detail: string;
  pid: number | null;
  exitCode: number | null;
  startedAt: string | null;
  stoppedAt: string | null;
  mode: string | null;
  logs: string[];
  meta: Record<string, unknown>;
}

export interface PinnedMove {
  id: string;
  name: string;
  trajectoryPath: string;
  target: ReplayTarget;
  vexReplayMode: VexReplayMode;
  homeMode: ArmHomeMode;
  speed: number;
  autoVexPositioning: boolean;
  vexPositioningSpeed: number;
  vexPositioningTimeoutS: number;
  vexPositioningXyToleranceM: number;
  vexPositioningHeadingToleranceDeg: number;
  vexPositioningXyTrimToleranceM: number;
  vexPositioningHeadingTrimToleranceDeg: number;
  includeBase: boolean;
  holdFinalS: number;
  keyBinding: string;
}

export interface ChainLinkItem extends ReplayRequest {
  id: string;
  name: string;
}

export interface ChainLink {
  id: string;
  name: string;
  confirmAfterEach: boolean;
  items: ChainLinkItem[];
}

export interface RecordingEntry {
  path: string;
  name: string;
  fileName: string;
  label: string | null;
  size: number;
  modifiedAt: string;
  durationS: number | null;
  dummy?: boolean;
}

export interface RecordingDetailRequest {
  path: string;
}

export interface SetArmHomeFromRecordingRequest {
  path: string;
}

export interface DeleteRecordingRequest {
  path: string;
}

export interface SetRecordingReplayOptionsRequest {
  path: string;
  options: RecordingReplayOptions;
}

export interface AdjustRecordingServoRequest {
  path: string;
  timeS: number;
  values: Record<string, number>;
}

export type RecordingInputMode = "auto" | "leader" | "keyboard" | "free-teach";

export interface StartRecordingRequest {
  label: string;
  inputMode?: RecordingInputMode;
}

export interface RecordingTimelinePoint {
  tS: number;
  values: Record<string, number>;
}

export interface RecordingDetail {
  path: string;
  durationS: number;
  sampleCount: number;
  commandSampleCount: number;
  armKeys: string[];
  baseKeys: string[];
  sensorKeys: string[];
  timelineSource: "commands" | "state";
  samples: RecordingTimelinePoint[];
  commandSamples: RecordingTimelinePoint[];
}

export interface RenameRecordingRequest {
  path: string;
  label: string;
}

export interface DuplicateRecordingRequest {
  path: string;
}

export interface MarkRecordingGyroZeroRequest {
  path: string;
}

export interface ResetRecordingUltrasonicPositionRequest {
  path: string;
}

export interface TrimRecordingRequest {
  path: string;
  trimStartS: number;
  trimEndS: number;
}

export interface LeaderStatus {
  teleoperateScriptPath: string;
  expectedPort: string | null;
  connected: boolean;
  availablePorts: string[];
  message: string;
}

export interface VexBrainStatus {
  connected: boolean;
  telemetryActive: boolean;
  consolePort: string | null;
  commPort: string | null;
  source: string | null;
  message: string;
}

export type RobotSensorState = "online" | "stale" | "waiting" | "missing" | "idle";

export interface RobotSensorStatus {
  label: string;
  axis: string | null;
  state: RobotSensorState;
  connected: boolean;
  value: number | null;
  unit: string | null;
  updatedAt: string | null;
  source: string | null;
  message: string;
}

export interface RobotSensorsStatus {
  gyro: RobotSensorStatus;
  x: RobotSensorStatus;
  y: RobotSensorStatus;
}

export interface DashboardState {
  settings: AppSettings;
  pinnedMoves: PinnedMove[];
  chainLinks: ChainLink[];
  homePosition: ArmHomePosition | null;
  recordingReplayOptions: Record<string, RecordingReplayOptions>;
  training: TrainingConfig & {
    selectedProfile: TrainingProfile | null;
  };
  wifi: {
    device: string | null;
    ssid: string | null;
    connected: boolean;
    message: string;
  };
  piReachable: boolean;
  resolvedPiHost: string | null;
  leader: LeaderStatus;
  vexBrain: VexBrainStatus;
  robotSensors: RobotSensorsStatus;
  recordings: RecordingEntry[];
  lastError: string | null;
  activityLog: string[];
  services: {
    host: ServiceSnapshot;
    teleop: ServiceSnapshot;
    replay: ServiceSnapshot;
    piCalibration: ServiceSnapshot;
    macCalibration: ServiceSnapshot;
    datasetCapture: ServiceSnapshot;
    datasetSync: ServiceSnapshot;
    training: ServiceSnapshot;
    deployment: ServiceSnapshot;
    policyBenchmark: ServiceSnapshot;
    policyEval: ServiceSnapshot;
  };
}

export interface ServoCalibrationRequest {
  servoId: string;
}

export interface ReplayRequest {
  trajectoryPath: string;
  target: ReplayTarget;
  vexReplayMode: VexReplayMode;
  homeMode: ArmHomeMode;
  speed: number;
  autoVexPositioning: boolean;
  vexPositioningSpeed: number;
  vexPositioningTimeoutS: number;
  vexPositioningXyToleranceM: number;
  vexPositioningHeadingToleranceDeg: number;
  vexPositioningXyTrimToleranceM: number;
  vexPositioningHeadingTrimToleranceDeg: number;
  includeBase: boolean;
  holdFinalS: number;
}

export interface UpdatePinnedMoveRequest extends Partial<ReplayRequest> {
  name?: string;
  keyBinding?: string;
}

export interface SaveChainLinkRequest {
  id?: string;
  name: string;
  confirmAfterEach?: boolean;
  items: ChainLinkItem[];
}
