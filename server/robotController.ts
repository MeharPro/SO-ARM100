import fs from "node:fs";
import path from "node:path";

import { Client, type ConnectConfig, type SFTPWrapper } from "ssh2";

import { ConfigStore } from "./configStore.js";
import { defaultConfig } from "./defaultConfig.js";
import { LocalProcessRunner, RemoteProcessRunner } from "./processRunners.js";
import {
  getWifiStatus,
  isTcpReachable,
  shellQuote,
  sleep,
} from "./system.js";
import type {
  AppSettings,
  CreatePinnedMoveRequest,
  DashboardState,
  LeaderStatus,
  PinnedMove,
  RenameRecordingRequest,
  RecordingEntry,
  ReplayRequest,
  ServiceSnapshot,
  TorqueLimitsRequest,
} from "./types.js";

const ROOT_DIR = process.cwd();
const HOST_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_host.py");
const POWER_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_power.py");
const RUNTIME_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_runtime.py");
const RECORD_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_record_trajectory.py");
const REPLAY_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_replay_trajectory.py");
const UI_TELEOP_SCRIPT = path.join(ROOT_DIR, "scripts", "lekiwi_ui_teleop.py");
const MAX_ACTIVITY_LINES = 220;
const HOST_READY_TIMEOUT_MS = 15000;
const HOST_READY_PORTS = [5555, 5556];
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
    speed: payload.speed > 0 ? payload.speed : defaults.defaultReplaySpeed,
    includeBase: false,
    holdFinalS:
      payload.holdFinalS >= 0 ? payload.holdFinalS : defaults.defaultHoldFinalS,
  };
}

export class RobotController {
  private readonly configStore = new ConfigStore(defaultConfig, ROOT_DIR);
  private readonly hostRunner = new RemoteProcessRunner("Pi host");
  private readonly teleopRunner = new LocalProcessRunner("Mac teleop");
  private readonly replayRunner = new RemoteProcessRunner("Replay");
  private readonly piCalibrationRunner = new RemoteProcessRunner("Pi calibration");
  private readonly macCalibrationRunner = new LocalProcessRunner("Mac calibration");
  private lastError: string | null = null;
  private cachedRecordings: RecordingEntry[] = [];
  private lastRecordingRefresh = 0;
  private lastResolvedHost: string | null = null;
  private activityLog: string[] = [];
  private queue: Promise<void> = Promise.resolve();
  private readonly remoteHelperSyncCache = new Map<string, string>();
  private readonly remoteTorqueCache = new Map<string, string>();

  async getState(): Promise<DashboardState> {
    const { settings, pinnedMoves } = this.configStore.getConfig();
    const wifi = await getWifiStatus();
    const resolvedPiHost = await this.findReachablePiHost(settings);
    const piReachable = Boolean(resolvedPiHost);
    const leader = await this.getLeaderStatus(settings);
    this.lastResolvedHost = resolvedPiHost;

    if (resolvedPiHost && Date.now() - this.lastRecordingRefresh > 15000) {
      try {
        this.cachedRecordings = await this.fetchRecordings(settings, resolvedPiHost, true);
        this.lastRecordingRefresh = Date.now();
      } catch (error) {
        this.noteError(error);
      }
    }

    return {
      settings,
      pinnedMoves,
      wifi,
      piReachable,
      resolvedPiHost,
      leader,
      recordings: this.cachedRecordings,
      lastError: this.lastError,
      activityLog: [...this.activityLog],
      services: {
        host: this.hostRunner.getSnapshot(),
        teleop: this.teleopRunner.getSnapshot(),
        replay: this.replayRunner.getSnapshot(),
        piCalibration: this.piCalibrationRunner.getSnapshot(),
        macCalibration: this.macCalibrationRunner.getSnapshot(),
      },
    };
  }

  async saveSettings(settings: AppSettings): Promise<DashboardState> {
    return this.runExclusive(async () => {
      this.logActivity("Saving updated dashboard settings.");
      this.configStore.saveSettings(settings);
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
      await this.writeRemoteTorqueLimits(settings, host);

      await this.replayRunner.stop("Stopping replay before live control.");
      await this.teleopRunner.stop("Restarting teleop.");
      await this.hostRunner.stop("Restarting Pi host.");
      await this.piCalibrationRunner.stop("Live control needs the robot exclusively.");
      await this.macCalibrationRunner.stop("Live control needs the leader arm exclusively.");

      await this.hostRunner.start(
        this.buildHostScript(settings),
        this.toConnectConfig(settings, host),
        "control",
        { host },
      );

      try {
        await this.waitForHostReady(host, "Pi host");
        await this.teleopRunner.start(this.buildTeleopScript(settings, host, leader), "control");
      } catch (error) {
        await this.hostRunner.stop("Teleop failed to launch, stopping the Pi host.");
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
      await this.hostRunner.stop("Emergency stop requested.");
      await this.piCalibrationRunner.stop("Emergency stop requested.");
      await this.macCalibrationRunner.stop("Emergency stop requested.");
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

      await this.teleopRunner.stop("Pi calibration needs the robot exclusively.");
      await this.replayRunner.stop("Pi calibration needs the robot exclusively.");
      await this.hostRunner.stop("Pi calibration needs the robot exclusively.");
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

  async startMacCalibration(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      this.logActivity("Mac leader calibration requested.");
      const leader = await this.ensureLeaderConnected(settings);

      await this.teleopRunner.stop("Mac calibration needs the leader arm exclusively.");
      await this.replayRunner.stop("Mac calibration requested.");
      await this.hostRunner.stop("Mac calibration requested.");
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
      await this.writeRemoteTorqueLimits(settings, host);

      await this.replayRunner.stop("Recording takes over the robot.");
      await this.teleopRunner.stop("Restarting teleop for recording.");
      await this.hostRunner.stop("Replacing the live host with the recorder.");
      await this.piCalibrationRunner.stop("Recording needs the robot exclusively.");
      await this.macCalibrationRunner.stop("Recording needs the leader arm exclusively.");

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

  async startReplay(payload: ReplayRequest): Promise<DashboardState> {
    return this.runExclusive(async () => {
      const { settings } = this.configStore.getConfig();
      const replay = normalizeReplayRequest(payload, settings.trajectories);
      if (!replay.trajectoryPath) {
        throw new Error("Choose a saved trajectory before starting replay.");
      }
      this.logActivity(`Starting replay for ${replay.trajectoryPath}.`);
      const host = await this.preparePi(settings, "start replay");

      assertHelperExists(REPLAY_SCRIPT);
      assertHelperExists(RUNTIME_SCRIPT);
      await this.ensureRemoteHelpers(settings, host);
      await this.writeRemoteTorqueLimits(settings, host);

      await this.teleopRunner.stop("Replay takes control away from teleop.");
      await this.hostRunner.stop("Replay needs exclusive access.");
      await this.replayRunner.stop("Restarting replay.");
      await this.piCalibrationRunner.stop("Replay needs the robot exclusively.");
      await this.macCalibrationRunner.stop("Replay requested.");

      await this.replayRunner.start(
        this.buildReplayScript(settings, replay),
        this.toConnectConfig(settings, host),
        "replay",
        { ...replay },
      );

      this.lastError = null;
      return this.getState();
    });
  }

  async stopReplay(): Promise<DashboardState> {
    return this.runExclusive(async () => {
      this.logActivity("Stop replay requested.");
      await this.replayRunner.stop("Stopping replay.");
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

  private async waitForHostReady(host: string, label: string): Promise<void> {
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

      const exited = await this.hostRunner.waitForExit(250);
      if (exited) {
        throw new Error(this.formatStartupExit(label, exited));
      }
    }

    const exited = await this.hostRunner.waitForExit(0);
    if (exited) {
      throw new Error(this.formatStartupExit(label, exited));
    }
    throw new Error(
      `${label} did not open ports ${HOST_READY_PORTS.join(", ")} on ${host} within ${Math.round(
        HOST_READY_TIMEOUT_MS / 1000,
      )} seconds.`,
    );
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
      "lerobot-calibrate \\",
      "  --robot.type=lekiwi \\",
      `  --robot.id=${shellQuote(settings.host.robotId)} \\`,
      `  --robot.port=${shellQuote(settings.pi.robotPort)} \\`,
      `  --robot.cameras=${shellQuote("{}")} \\`,
      "  --robot.enable_base=false \\",
      "  --robot.use_degrees=true",
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

  private buildHostScript(settings: AppSettings): string {
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
      `  --enable-base false`,
    ].join("\n");
  }

  private buildRecordScript(
    settings: AppSettings,
    trajectoryPath: string,
    label: string,
  ): string {
    const labelFlag = label ? ` \\\n  --label ${shellQuote(label)}` : "";
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
      `  --output ${shellQuote(trajectoryPath)}${labelFlag}`,
    ].join("\n");
  }

  private buildReplayScript(settings: AppSettings, replay: ReplayRequest): string {
    const includeBaseFlag = replay.includeBase ? " \\\n  --include-base" : "";
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
      `  --input ${shellQuote(replay.trajectoryPath)} \\`,
      `  --speed ${replay.speed} \\`,
      `  --hold-final-s ${replay.holdFinalS}${includeBaseFlag}`,
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
        this.fastPut(sftp, RECORD_SCRIPT, `${helperDir}/lekiwi_record_trajectory.py`),
        this.fastPut(sftp, REPLAY_SCRIPT, `${helperDir}/lekiwi_replay_trajectory.py`),
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

  private async computeRemoteHelperFingerprint(settings: AppSettings): Promise<string> {
    const localScripts = [
      HOST_SCRIPT,
      POWER_SCRIPT,
      RUNTIME_SCRIPT,
      RECORD_SCRIPT,
      REPLAY_SCRIPT,
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

  private logActivity(message: string): void {
    const stamped = `${new Date().toLocaleTimeString()} ${message}`;
    this.activityLog.push(stamped);
    if (this.activityLog.length > MAX_ACTIVITY_LINES) {
      this.activityLog.splice(0, this.activityLog.length - MAX_ACTIVITY_LINES);
    }
  }
}
