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
  armTorqueLimits: Record<string, number>;
}

export interface TrajectorySettings {
  remoteDir: string;
  defaultReplaySpeed: number;
  defaultHoldFinalS: number;
}

export interface AppSettings {
  hotspot: HotspotSettings;
  pi: PiSettings;
  mac: MacSettings;
  host: HostSettings;
  trajectories: TrajectorySettings;
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
  speed: number;
  includeBase: boolean;
  holdFinalS: number;
  keyBinding: string;
}

export interface RecordingEntry {
  path: string;
  name: string;
  fileName: string;
  label: string | null;
  size: number;
  modifiedAt: string;
  durationS: number | null;
}

export interface RenameRecordingRequest {
  path: string;
  label: string;
}

export interface LeaderStatus {
  teleoperateScriptPath: string;
  expectedPort: string | null;
  connected: boolean;
  availablePorts: string[];
  message: string;
}

export interface DashboardState {
  settings: AppSettings;
  pinnedMoves: PinnedMove[];
  wifi: {
    device: string | null;
    ssid: string | null;
    connected: boolean;
    message: string;
  };
  piReachable: boolean;
  resolvedPiHost: string | null;
  leader: LeaderStatus;
  recordings: RecordingEntry[];
  lastError: string | null;
  activityLog: string[];
  services: {
    host: ServiceSnapshot;
    teleop: ServiceSnapshot;
    replay: ServiceSnapshot;
    piCalibration: ServiceSnapshot;
    macCalibration: ServiceSnapshot;
  };
}
