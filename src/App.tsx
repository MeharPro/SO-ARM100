import { type MouseEvent, useEffect, useMemo, useRef, useState } from "react";

import type {
  ArmHomeMode,
  AppSettings,
  DashboardState,
  DeleteRecordingRequest,
  DuplicateRecordingRequest,
  MarkRecordingGyroZeroRequest,
  PinnedMove,
  RecordingDetail,
  RecordingDetailRequest,
  RecordingReplayOptions,
  ReplayTarget,
  RecordingInputMode,
  RecordingEntry,
  RenameRecordingRequest,
  RecordingTimelinePoint,
  ResetRecordingUltrasonicPositionRequest,
  SetArmHomeFromRecordingRequest,
  SetRecordingReplayOptionsRequest,
  ServiceSnapshot,
  ServoCalibrationRequest,
  TrainingArtifact,
  TrainingProfile,
  TrimRecordingRequest,
  VexReplayMode,
} from "./types";

const POLL_MS = 2500;
const SECTIONS = [
  { key: "overview", label: "Overview" },
  { key: "recordings", label: "Recordings" },
  { key: "pins", label: "Pinned Moves" },
  { key: "training", label: "Training" },
  { key: "settings", label: "Settings" },
  { key: "logs", label: "Logs" },
] as const;

type SectionKey = (typeof SECTIONS)[number]["key"];

const SERVO_LABELS: Record<string, string> = {
  arm_shoulder_pan: "Shoulder pan",
  arm_shoulder_lift: "Shoulder lift",
  arm_elbow_flex: "Elbow flex",
  arm_wrist_flex: "Wrist flex",
  arm_wrist_roll: "Wrist roll",
  arm_gripper: "Gripper",
};

const ARM_SERVO_IDS = [
  "arm_shoulder_pan",
  "arm_shoulder_lift",
  "arm_elbow_flex",
  "arm_wrist_flex",
  "arm_wrist_roll",
  "arm_gripper",
] as const;
const SERVO_TRACE_COLORS: Record<string, string> = {
  arm_shoulder_pan: "#ea580c",
  arm_shoulder_lift: "#ca8a04",
  arm_elbow_flex: "#16a34a",
  arm_wrist_flex: "#0891b2",
  arm_wrist_roll: "#2563eb",
  arm_gripper: "#c026d3",
};

const TORQUE_LIMIT_MIN = 0;
const TORQUE_LIMIT_MAX = 1000;
const DEFAULT_ARM_TORQUE_LIMITS = Object.fromEntries(
  ARM_SERVO_IDS.map((id) => [id, TORQUE_LIMIT_MAX]),
) as Record<string, number>;
const RECOMMENDED_SAFE_ARM_TORQUE_LIMITS: Record<string, number> = {
  arm_shoulder_pan: 500,
  arm_shoulder_lift: 650,
  arm_elbow_flex: 600,
  arm_wrist_flex: 450,
  arm_wrist_roll: 250,
  arm_gripper: 1000,
};
const LOCAL_TRAINING_ROOT = "/Users/meharkhanna/robot-arm/output";
const VEX_AXIS_OPTIONS = [
  { value: "axis1", label: "Axis 1, right stick horizontal" },
  { value: "axis2", label: "Axis 2, right stick vertical" },
  { value: "axis3", label: "Axis 3, left stick vertical" },
  { value: "axis4", label: "Axis 4, left stick horizontal" },
] as const;
const VEX_REPLAY_MODE_OPTIONS = [
  { value: "ecu", label: "ECU mode" },
  { value: "drive", label: "Drive mode" },
] as const;
const HOME_MODE_OPTIONS: Array<{ value: ArmHomeMode; label: string }> = [
  { value: "none", label: "Do not go home" },
  { value: "start", label: "Go home before replay" },
  { value: "end", label: "Go home after replay" },
  { value: "both", label: "Go home before and after" },
];
const SENSOR_STATUS_LOG_PREFIX = "[sensor-status]";
const VEX_POSITION_LOG_PREFIX = "[vex-position]";

interface ServoTemperature {
  id: string;
  label: string;
  temperatureC: number | null;
}

interface SensorTelemetryEntry {
  observedAtMs: number;
  displayTime: string;
  serviceLabel: string;
  source: string | null;
  line: string;
  rawLine: string;
}

interface VexPositionStatusEntry {
  observedAtMs: number;
  displayTime: string;
  serviceLabel: string;
  status: string;
  reason: string | null;
  timeoutS: number | null;
  message: string;
  rawLine: string;
}

interface PowerTelemetry {
  sourceLabel: string | null;
  timestamp: string | null;
  batteryPercent: number | null;
  voltage: number | null;
  voltageMin: number | null;
  voltageMax: number | null;
  totalPowerW: number | null;
  totalCurrentA: number | null;
  motorsReporting: number | null;
  motorsExpected: number | null;
  peakMotor: string | null;
  peakCurrentA: number | null;
  hottestMotor: string | null;
  hottestTemperatureC: number | null;
  servoTemperatures: ServoTemperature[];
  rawLine: string;
}

function formatBytes(bytes: number): string {
  if (bytes < 1024) {
    return `${bytes} B`;
  }
  if (bytes < 1024 * 1024) {
    return `${(bytes / 1024).toFixed(1)} KB`;
  }
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

function prettyTimestamp(value: string | null): string {
  if (!value) {
    return "not yet";
  }
  return new Date(value).toLocaleString();
}

function formatClockDuration(totalSeconds: number): string {
  const safeSeconds = Math.max(0, Math.floor(totalSeconds));
  const hours = Math.floor(safeSeconds / 3600);
  const minutes = Math.floor((safeSeconds % 3600) / 60);
  const seconds = safeSeconds % 60;

  if (hours > 0) {
    return [hours, minutes, seconds].map((value) => String(value).padStart(2, "0")).join(":");
  }
  return [minutes, seconds].map((value) => String(value).padStart(2, "0")).join(":");
}

function formatDurationSeconds(value: number | null): string {
  return value === null ? "n/a" : formatClockDuration(value);
}

function clampPlaybackTime(value: number, durationS: number): number {
  if (!Number.isFinite(value) || durationS <= 0) {
    return 0;
  }
  return Math.min(Math.max(value, 0), durationS);
}

function formatSecondsInput(value: number): string {
  return Math.max(0, value)
    .toFixed(2)
    .replace(/\.00$/, "")
    .replace(/(\.\d)0$/, "$1");
}

function formatElapsedSince(startedAt: string | null, nowMs: number): string {
  if (!startedAt) {
    return "00:00";
  }

  const startedMs = Date.parse(startedAt);
  if (!Number.isFinite(startedMs)) {
    return "00:00";
  }

  return formatClockDuration((nowMs - startedMs) / 1000);
}

function parseLocalLogTimestamp(line: string): number | null {
  const match = line.match(/^(\d{1,2}):(\d{2}):(\d{2})\s(AM|PM)\b/);
  if (!match) {
    return null;
  }

  let hours = Number(match[1]) % 12;
  const minutes = Number(match[2]);
  const seconds = Number(match[3]);
  if (match[4] === "PM") {
    hours += 12;
  }

  const now = new Date();
  now.setHours(hours, minutes, seconds, 0);
  return now.getTime();
}

function normalizeHotkeyText(value: string): string {
  const raw = value.trim().toUpperCase();
  if (!raw) {
    return "";
  }
  return raw.replace(/\s+/g, "").replace(/COMMAND/g, "META");
}

function keyboardEventPrimaryKey(event: KeyboardEvent): string {
  const code = event.code;
  if (/^Digit\d$/.test(code)) {
    return code.slice(5);
  }
  if (/^Key[A-Z]$/.test(code)) {
    return code.slice(3);
  }
  if (/^Numpad\d$/.test(code)) {
    return `NUM${code.slice(6)}`;
  }

  const codeMap: Record<string, string> = {
    Space: "SPACE",
    Escape: "ESC",
    Enter: "ENTER",
    Tab: "TAB",
    Backspace: "BACKSPACE",
    ArrowUp: "UP",
    ArrowDown: "DOWN",
    ArrowLeft: "LEFT",
    ArrowRight: "RIGHT",
    Minus: "-",
    Equal: "=",
    BracketLeft: "[",
    BracketRight: "]",
    Backslash: "\\",
    Semicolon: ";",
    Quote: "'",
    Comma: ",",
    Period: ".",
    Slash: "/",
    Backquote: "`",
    NumpadAdd: "NUM+",
    NumpadSubtract: "NUM-",
    NumpadMultiply: "NUM*",
    NumpadDivide: "NUM/",
    NumpadDecimal: "NUM.",
    NumpadEnter: "NUMENTER",
  };
  if (codeMap[code]) {
    return codeMap[code];
  }

  let key = event.key.toUpperCase();
  if (key === " ") {
    key = "SPACE";
  }
  if (key === "ESCAPE") {
    key = "ESC";
  }
  return key;
}

function normalizeKeyboardEvent(event: KeyboardEvent): string {
  const parts: string[] = [];
  if (event.ctrlKey) {
    parts.push("CTRL");
  }
  if (event.metaKey) {
    parts.push("META");
  }
  if (event.altKey) {
    parts.push("ALT");
  }
  if (event.shiftKey) {
    parts.push("SHIFT");
  }

  const key = keyboardEventPrimaryKey(event);
  if (["CONTROL", "META", "ALT", "SHIFT"].includes(key)) {
    return parts.join("+");
  }

  parts.push(key);
  return parts.join("+");
}

async function request<T>(url: string, init?: RequestInit): Promise<T> {
  const response = await fetch(url, {
    headers: {
      "Content-Type": "application/json",
    },
    ...init,
  });

  const payload = await response.json().catch(() => null);
  if (!response.ok) {
    throw new Error(payload?.error ?? "Request failed.");
  }

  return payload as T;
}

function statusTone(state: string): string {
  if (state === "running") {
    return "good";
  }
  if (state === "error") {
    return "bad";
  }
  if (state === "starting" || state === "stopping") {
    return "warn";
  }
  return "muted";
}

function robotSensorTone(sensor: DashboardState["robotSensors"]["gyro"] | null | undefined): string {
  if (!sensor) {
    return "muted";
  }
  if (sensor.state === "online") {
    return "good";
  }
  if (sensor.state === "missing") {
    return "bad";
  }
  if (sensor.state === "waiting" || sensor.state === "stale") {
    return "warn";
  }
  return "muted";
}

function formatRobotSensorValue(sensor: DashboardState["robotSensors"]["gyro"] | null | undefined): string {
  if (!sensor || typeof sensor.value !== "number" || !Number.isFinite(sensor.value)) {
    return "n/a";
  }
  if (sensor.unit === "m") {
    return formatDistanceMeters(sensor.value);
  }
  if (sensor.unit === "deg") {
    return `${sensor.value.toFixed(1)} deg`;
  }
  return sensor.value.toFixed(2);
}

function formatRobotSensorSource(source: string | null | undefined): string {
  return source ? source.replace(/-/g, " ") : "n/a";
}

function cleanServiceLogLine(line: string): string {
  return line.replace(/^\d{1,2}:\d{2}:\d{2}\s[AP]M\s\[[^\]]+\]\s*/, "");
}

function getRecordingInputSource(
  service: ServiceSnapshot | null | undefined,
): "keyboard" | "leader" | "free-teach" | "control" {
  const input = service?.meta?.input;
  if (input === "keyboard") {
    return "keyboard";
  }
  if (input === "leader") {
    return "leader";
  }
  if (input === "free-teach") {
    return "free-teach";
  }
  return "control";
}

function getRecordingWaitingLabel(service: ServiceSnapshot | null | undefined): string {
  const inputSource = getRecordingInputSource(service);
  if (inputSource === "keyboard") {
    return "Waiting for keyboard input";
  }
  if (inputSource === "leader") {
    return "Waiting for leader input";
  }
  if (inputSource === "free-teach") {
    return "Hand-guide recording";
  }
  return "Waiting for control input";
}

function getRecordingArmedLabel(service: ServiceSnapshot | null | undefined): string {
  const inputSource = getRecordingInputSource(service);
  if (inputSource === "keyboard") {
    return "Recorder is armed and ready for keyboard input.";
  }
  if (inputSource === "leader") {
    return "Recorder is armed and ready for leader input.";
  }
  if (inputSource === "free-teach") {
    return "Follower arm torque is disabled; move the arm by hand.";
  }
  return "Recorder is armed and ready.";
}

function formatSensorTelemetryReading(sensor: unknown): string {
  if (!sensor || typeof sensor !== "object") {
    return "n/a";
  }

  const payload = sensor as {
    state?: unknown;
    value?: unknown;
    unit?: unknown;
  };
  const state = typeof payload.state === "string" ? payload.state : "unknown";
  const value = typeof payload.value === "number" && Number.isFinite(payload.value) ? payload.value : null;
  const unit = typeof payload.unit === "string" ? payload.unit : null;
  if (value === null) {
    return state;
  }
  if (unit === "m") {
    return `${state} ${formatDistanceMeters(value)}`;
  }
  if (unit === "deg") {
    return `${state} ${value.toFixed(1)} deg`;
  }
  return `${state} ${value.toFixed(2)}`;
}

function parseSensorTelemetryEntries(services: ServiceSnapshot[]): SensorTelemetryEntry[] {
  const entries: SensorTelemetryEntry[] = [];
  for (const service of services) {
    for (const rawLine of service.logs) {
      const cleaned = cleanServiceLogLine(rawLine);
      if (!cleaned.startsWith(SENSOR_STATUS_LOG_PREFIX)) {
        continue;
      }

      const payloadText = cleaned.slice(SENSOR_STATUS_LOG_PREFIX.length).trim();
      try {
        const payload = JSON.parse(payloadText) as {
          timestamp?: unknown;
          source?: unknown;
          gyro?: unknown;
          x?: unknown;
          y?: unknown;
        };
        const observedAtMs =
          typeof payload.timestamp === "string"
            ? Date.parse(payload.timestamp)
            : parseLocalLogTimestamp(rawLine);
        const safeObservedAtMs = Number.isFinite(observedAtMs) ? observedAtMs : 0;
        const displayTime = safeObservedAtMs ? new Date(safeObservedAtMs).toLocaleTimeString() : "unknown time";
        const source = typeof payload.source === "string" ? payload.source : null;
        entries.push({
          observedAtMs: safeObservedAtMs,
          displayTime,
          serviceLabel: service.label,
          source,
          line:
            `${displayTime} [${service.label}] ` +
            `gyro=${formatSensorTelemetryReading(payload.gyro)} ` +
            `x=${formatSensorTelemetryReading(payload.x)} ` +
            `y=${formatSensorTelemetryReading(payload.y)}` +
            (source ? ` source=${formatRobotSensorSource(source)}` : ""),
          rawLine,
        });
      } catch {
        entries.push({
          observedAtMs: parseLocalLogTimestamp(rawLine) ?? 0,
          displayTime: "unknown time",
          serviceLabel: service.label,
          source: null,
          line: `${service.label}: ${cleaned}`,
          rawLine,
        });
      }
    }
  }

  return entries
    .sort((left, right) => left.observedAtMs - right.observedAtMs)
    .slice(-60);
}

function defaultVexPositionMessage(status: string, reason: string | null): string {
  if (status === "aligned") {
    return "VEX start positioning completed before arm replay.";
  }
  if (status === "skipped" && (reason === "timeout" || reason === "not-aligned")) {
    return "VEX start positioning stopped, so arm replay was aborted.";
  }
  if (status === "skipped") {
    return "VEX start positioning skipped.";
  }
  return "VEX start positioning status updated.";
}

function parseLatestVexPositionStatus(services: ServiceSnapshot[]): VexPositionStatusEntry | null {
  let latest: VexPositionStatusEntry | null = null;

  for (const service of services) {
    for (const rawLine of service.logs) {
      const cleaned = cleanServiceLogLine(rawLine);
      if (!cleaned.startsWith(VEX_POSITION_LOG_PREFIX)) {
        continue;
      }

      const payloadText = cleaned.slice(VEX_POSITION_LOG_PREFIX.length).trim();
      let entry: VexPositionStatusEntry;
      try {
        const payload = JSON.parse(payloadText) as {
          status?: unknown;
          reason?: unknown;
          timeout_s?: unknown;
          message?: unknown;
        };
        const status = typeof payload.status === "string" ? payload.status : "unknown";
        const reason = typeof payload.reason === "string" ? payload.reason : null;
        const timeoutS =
          typeof payload.timeout_s === "number" && Number.isFinite(payload.timeout_s) ? payload.timeout_s : null;
        const observedAtMs = parseLocalLogTimestamp(rawLine) ?? 0;
        const displayTime = observedAtMs ? new Date(observedAtMs).toLocaleTimeString() : "unknown time";
        entry = {
          observedAtMs,
          displayTime,
          serviceLabel: service.label,
          status,
          reason,
          timeoutS,
          message:
            typeof payload.message === "string" && payload.message.trim()
              ? payload.message.trim()
              : defaultVexPositionMessage(status, reason),
          rawLine,
        };
      } catch {
        const observedAtMs = parseLocalLogTimestamp(rawLine) ?? 0;
        entry = {
          observedAtMs,
          displayTime: observedAtMs ? new Date(observedAtMs).toLocaleTimeString() : "unknown time",
          serviceLabel: service.label,
          status: "unknown",
          reason: null,
          timeoutS: null,
          message: cleaned,
          rawLine,
        };
      }

      if (!latest || entry.observedAtMs >= latest.observedAtMs) {
        latest = entry;
      }
    }
  }

  return latest;
}

function latestLogLines(lines: string[], count: number): string[] {
  return lines.slice(Math.max(lines.length - count, 0));
}

function parseNumber(value: string | undefined): number | null {
  if (!value) {
    return null;
  }
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function labelServo(id: string | null): string {
  if (!id) {
    return "n/a";
  }
  return SERVO_LABELS[id] ?? id.replace(/^arm_/, "").replace(/_/g, " ");
}

function labelSensor(key: string): string {
  if (key === "ultrasonic_sensor_1.distance_m") {
    return "X ultrasonic distance";
  }
  if (key === "ultrasonic_sensor_2.distance_m") {
    return "Y ultrasonic distance";
  }
  if (key === "ultrasonic_sensor_1.position_m") {
    return "X ultrasonic position";
  }
  if (key === "ultrasonic_sensor_2.position_m") {
    return "Y ultrasonic position";
  }
  return key.replace(/_/g, " ");
}

function isLocalLeaderCaptureMode(captureMode: TrainingProfile["captureMode"] | null | undefined): boolean {
  return captureMode === "leader-as-follower";
}

function describeCaptureMode(profile: TrainingProfile | null | undefined): string {
  if (!profile) {
    return "Select a training profile first.";
  }
  if (profile.captureMode === "leader-as-follower") {
    return "Leader-as-follower mode on the Mac. The leader arm is recorded locally as an arm-only LeKiwi-compatible stand-in.";
  }
  if (profile.captureMode === "free-teach") {
    return "Free-teach mode on the Pi with follower torque disabled.";
  }
  return "Leader mode on the Pi using the follower arm plus the Mac-attached leader arm.";
}

function captureModeLabel(captureMode: TrainingProfile["captureMode"]): string {
  if (captureMode === "leader-as-follower") {
    return "leader as follower";
  }
  return captureMode;
}

function replayTargetLabel(target: ReplayTarget): string {
  return target === "leader" ? "Leader arm" : "Pi follower";
}

function homeModeLabel(mode: ArmHomeMode): string {
  if (mode === "start") {
    return "Home before";
  }
  if (mode === "end") {
    return "Home after";
  }
  if (mode === "both") {
    return "Home before + after";
  }
  return "No home";
}

function vexReplayModeLabel(mode: VexReplayMode): string {
  return mode === "ecu" ? "ECU mode" : "Drive mode";
}

function vexStatusLabel(vexBrain: DashboardState["vexBrain"] | null | undefined): string {
  if (!vexBrain?.connected) {
    return "missing";
  }
  if (vexBrain.telemetryActive) {
    return vexBrain.source === "replay" ? "replay" : "ready";
  }
  return "detected";
}

function recordingHasReplayableBaseReference(detail: RecordingDetail | null | undefined): boolean {
  if (!detail || !detail.baseKeys.length) {
    return false;
  }

  const timeline = detail.commandSamples.length ? detail.commandSamples : detail.samples;
  return timeline.some((point) =>
    detail.baseKeys.some((key) => {
      const value = point.values[key];
      return typeof value === "number" && Math.abs(value) > 1e-4;
    }),
  );
}

function emptyTrainingArtifact(): TrainingArtifact {
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

function createTrainingDraft(base?: TrainingProfile | null): TrainingProfile {
  if (base) {
    return {
      ...base,
      id: "",
      name: `${base.name} Copy`,
      selectedCheckpointPath: "",
      artifacts: emptyTrainingArtifact(),
    };
  }

  return {
    id: "",
    name: "New Task",
    task: "Pick up the wooden piece from the table, close the gripper around it, and lift it.",
    captureMode: "leader",
    numEpisodes: 50,
    episodeTimeS: 20,
    resetTimeS: 10,
    fps: 30,
    camerasMode: "default",
    policyType: "act",
    piDatasetPath: "/home/pi/lerobot-datasets/new-task",
    macDatasetPath: `${LOCAL_TRAINING_ROOT}/training-datasets/new-task`,
    macTrainOutputDir: `${LOCAL_TRAINING_ROOT}/training-runs/new-task`,
    selectedCheckpointPath: "",
    piDeployPath: "/home/pi/lerobot-training/new-task/deployed-policy",
    piEvalDatasetPath: "/home/pi/lerobot-eval/new-task",
    artifacts: emptyTrainingArtifact(),
  };
}

function findRecordingPoint(
  points: RecordingTimelinePoint[],
  targetS: number,
): RecordingTimelinePoint | null {
  if (!points.length) {
    return null;
  }

  let low = 0;
  let high = points.length - 1;
  const safeTargetS = Math.max(0, targetS);

  while (low <= high) {
    const mid = Math.floor((low + high) / 2);
    const point = points[mid];
    if (!point) {
      break;
    }
    if (point.tS <= safeTargetS) {
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }

  return points[Math.max(0, high)] ?? points[0] ?? null;
}

function evenlySampleTimeline(
  points: RecordingTimelinePoint[],
  maxPoints: number,
): RecordingTimelinePoint[] {
  if (points.length <= maxPoints) {
    return points;
  }

  const sampled: RecordingTimelinePoint[] = [];
  const lastIndex = points.length - 1;
  const step = lastIndex / Math.max(maxPoints - 1, 1);
  let previousIndex = -1;

  for (let index = 0; index < maxPoints; index += 1) {
    const nextIndex = Math.min(lastIndex, Math.round(index * step));
    const point = points[nextIndex];
    if (nextIndex !== previousIndex && point) {
      sampled.push(point);
      previousIndex = nextIndex;
    }
  }

  const finalPoint = points[lastIndex];
  if (finalPoint && sampled.at(-1) !== finalPoint) {
    sampled.push(finalPoint);
  }

  return sampled;
}

function formatServoPosition(value: number | undefined): string {
  return Number.isFinite(value) ? `${value.toFixed(1)} deg` : "n/a";
}

function formatDistanceMeters(value: number | undefined): string {
  return Number.isFinite(value) ? `${(value * 100).toFixed(1)} cm` : "n/a";
}

function recordingTimelineCopy(source: RecordingDetail["timelineSource"]): string {
  return source === "commands"
    ? "Using recorded servo command timing for the playhead."
    : "Legacy recording. The playhead is following follower-state samples because command timestamps were not saved.";
}

function RecordingPreviewPanel({
  detail,
  loading,
  error,
  playheadS,
  playing,
  trimStartS,
  trimEndS,
  onScrub,
  onTogglePlayback,
  onReset,
  onSetTrimStart,
  onSetTrimEnd,
}: {
  detail: RecordingDetail | null;
  loading: boolean;
  error: string;
  playheadS: number;
  playing: boolean;
  trimStartS: number;
  trimEndS: number;
  onScrub: (timeS: number) => void;
  onTogglePlayback: () => void;
  onReset: () => void;
  onSetTrimStart: () => void;
  onSetTrimEnd: () => void;
}) {
  const timelinePoints = useMemo(
    () => (detail ? (detail.commandSamples.length ? detail.commandSamples : detail.samples) : []),
    [detail],
  );
  const durationS = detail?.durationS ?? timelinePoints.at(-1)?.tS ?? 0;
  const sampledPoints = useMemo(() => evenlySampleTimeline(timelinePoints, 260), [timelinePoints]);
  const currentPoint = useMemo(
    () => findRecordingPoint(timelinePoints, playheadS),
    [playheadS, timelinePoints],
  );
  const currentSensorPoint = useMemo(
    () => findRecordingPoint(detail?.samples ?? [], playheadS),
    [detail?.samples, playheadS],
  );

  const displayArmKeys = useMemo(
    () =>
      detail?.armKeys.length
        ? detail.armKeys
        : ARM_SERVO_IDS.filter((key) => currentPoint?.values[key] !== undefined),
    [currentPoint?.values, detail?.armKeys],
  );

  const tracePaths = useMemo(
    () =>
      displayArmKeys.map((key) => {
        const values = sampledPoints.map((point) => point.values[key] ?? 0);
        const min = values.length ? Math.min(...values) : 0;
        const max = values.length ? Math.max(...values) : 0;
        const span = max - min;
        const points = sampledPoints
          .map((point) => {
            const x = durationS > 0 ? (point.tS / durationS) * 100 : 0;
            const value = point.values[key] ?? 0;
            const y = span < 1e-6 ? 50 : 92 - (((value - min) / span) * 84);
            return `${x.toFixed(3)},${y.toFixed(3)}`;
          })
          .join(" ");
        return {
          key,
          color: SERVO_TRACE_COLORS[key] ?? "#2563eb",
          points,
        };
      }),
    [displayArmKeys, durationS, sampledPoints],
  );

  const safePlayheadS = clampPlaybackTime(playheadS, durationS);
  const safeTrimStartS = clampPlaybackTime(trimStartS, durationS);
  const candidateTrimEndS = Number.isFinite(trimEndS) ? trimEndS : durationS;
  const safeTrimEndS = clampPlaybackTime(candidateTrimEndS, durationS);
  const effectiveTrimEndS =
    safeTrimEndS > safeTrimStartS ? safeTrimEndS : durationS;
  const keepWindowS = Math.max(effectiveTrimEndS - safeTrimStartS, 0);
  const playheadPct = durationS > 0 ? (safePlayheadS / durationS) * 100 : 0;
  const trimStartPct = durationS > 0 ? (safeTrimStartS / durationS) * 100 : 0;
  const trimEndPct = durationS > 0 ? (effectiveTrimEndS / durationS) * 100 : 100;

  const handleSeekFromChart = (event: MouseEvent<HTMLDivElement>) => {
    if (durationS <= 0) {
      return;
    }

    const bounds = event.currentTarget.getBoundingClientRect();
    const ratio = Math.min(Math.max((event.clientX - bounds.left) / bounds.width, 0), 1);
    onScrub(ratio * durationS);
  };

  return (
    <div className="recording-preview">
      <div className="recording-preview-head">
        <div>
          <h3>Recording Preview</h3>
          <p className="card-note">
            {detail ? recordingTimelineCopy(detail.timelineSource) : "Load a recording to scrub and trim it here."}
          </p>
        </div>
        <div className="button-cluster inline recording-preview-buttons">
          <button disabled={!detail || durationS <= 0} onClick={onTogglePlayback}>
            {playing ? "Pause" : "Play"}
          </button>
          <button disabled={!detail || durationS <= 0} onClick={onReset}>
            Restart
          </button>
        </div>
      </div>

      {loading ? <div className="empty-state">Loading recording timeline...</div> : null}
      {!loading && error ? <div className="empty-state">{error}</div> : null}
      {!loading && !error && detail && !timelinePoints.length ? (
        <div className="empty-state">This recording does not contain timeline samples yet.</div>
      ) : null}

      {!loading && !error && detail && timelinePoints.length ? (
        <>
          <div className="recording-preview-chart-shell">
            <div
              className="recording-preview-chart"
              onClick={handleSeekFromChart}
              role="presentation"
            >
              <svg viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true">
                <rect x="0" y="0" width="100" height="100" className="recording-preview-chart-bg" />
                <rect
                  x="0"
                  y="0"
                  width={trimStartPct}
                  height="100"
                  className="recording-preview-chart-mask"
                />
                <rect
                  x={trimEndPct}
                  y="0"
                  width={Math.max(100 - trimEndPct, 0)}
                  height="100"
                  className="recording-preview-chart-mask"
                />
                <rect
                  x={trimStartPct}
                  y="0"
                  width={Math.max(trimEndPct - trimStartPct, 0)}
                  height="100"
                  className="recording-preview-chart-window"
                />
                {tracePaths.map((trace) =>
                  trace.points ? (
                    <polyline
                      key={trace.key}
                      fill="none"
                      points={trace.points}
                      stroke={trace.color}
                      strokeWidth="1.4"
                      strokeLinejoin="round"
                      strokeLinecap="round"
                    />
                  ) : null,
                )}
                <line
                  x1={trimStartPct}
                  y1="0"
                  x2={trimStartPct}
                  y2="100"
                  className="recording-preview-boundary start"
                />
                <line
                  x1={trimEndPct}
                  y1="0"
                  x2={trimEndPct}
                  y2="100"
                  className="recording-preview-boundary end"
                />
                <line
                  x1={playheadPct}
                  y1="0"
                  x2={playheadPct}
                  y2="100"
                  className="recording-preview-playhead"
                />
              </svg>
            </div>

            <div className="recording-preview-stats">
              <article>
                <span>Playhead</span>
                <strong>{formatClockDuration(safePlayheadS)}</strong>
                <small>{safePlayheadS.toFixed(2)}s</small>
              </article>
              <article>
                <span>Keep Window</span>
                <strong>{formatClockDuration(keepWindowS)}</strong>
                <small>
                  {formatSecondsInput(safeTrimStartS)}s to {formatSecondsInput(effectiveTrimEndS)}s
                </small>
              </article>
              <article>
                <span>{detail.timelineSource === "commands" ? "Command Points" : "Follower Samples"}</span>
                <strong>
                  {detail.timelineSource === "commands"
                    ? detail.commandSampleCount
                    : detail.sampleCount}
                </strong>
                <small>
                  {detail.timelineSource === "commands"
                    ? `${detail.sampleCount} follower samples also recorded`
                    : "Command timing unavailable in this file"}
                </small>
              </article>
            </div>
          </div>

          <label className="recording-preview-scrubber">
            Timeline
            <input
              type="range"
              min="0"
              max={Math.max(durationS, 0)}
              step="0.01"
              value={safePlayheadS}
              onChange={(event) => onScrub(Number(event.target.value))}
            />
          </label>

          <div className="button-cluster inline recording-preview-actions">
            <button disabled={durationS <= 0} onClick={onSetTrimStart}>
              Set Trim Start to Playhead
            </button>
            <button disabled={durationS <= 0} onClick={onSetTrimEnd}>
              Set Trim End to Playhead
            </button>
          </div>

          <div className="recording-preview-legend">
            {displayArmKeys.map((key) => (
              <span key={key} className="recording-preview-legend-item">
                <span
                  className="recording-preview-swatch"
                  style={{ backgroundColor: SERVO_TRACE_COLORS[key] ?? "#2563eb" }}
                />
                {labelServo(key)}
              </span>
            ))}
          </div>

          <div className="recording-preview-poses">
            {displayArmKeys.map((key) => (
              <article key={key}>
                <span>{labelServo(key)}</span>
                <strong>{formatServoPosition(currentPoint?.values[key])}</strong>
              </article>
            ))}
            {detail.sensorKeys.map((key) => (
              <article key={key}>
                <span>{labelSensor(key)}</span>
                <strong>{formatDistanceMeters(currentSensorPoint?.values[key])}</strong>
                <small>sensor sample</small>
              </article>
            ))}
          </div>
        </>
      ) : null}
    </div>
  );
}

function parsePowerTelemetry(
  sources: Array<{ label: string; logs: string[] }>,
): PowerTelemetry | null {
  const latest = sources
    .flatMap((source, sourceIndex) =>
      source.logs
        .map((rawLine, lineIndex) => ({
          rawLine,
          sourceLabel: source.label,
          sourceIndex,
          lineIndex,
          sampledAt: Date.parse(rawLine.match(/\[power\]\s+(\S+)/)?.[1] ?? ""),
        }))
        .filter((entry) => entry.rawLine.includes("[power]")),
    )
    .sort((left, right) => {
      const leftSampledAt = Number.isFinite(left.sampledAt) ? left.sampledAt : Number.NEGATIVE_INFINITY;
      const rightSampledAt = Number.isFinite(right.sampledAt) ? right.sampledAt : Number.NEGATIVE_INFINITY;
      return (
        leftSampledAt - rightSampledAt ||
        left.sourceIndex - right.sourceIndex ||
        left.lineIndex - right.lineIndex
      );
    })
    .at(-1);

  if (!latest) {
    return null;
  }
  const rawLine = latest.rawLine;

  const motorsMatch = rawLine.match(/motors=(\d+)\/(\d+)/);
  const totalMatch = rawLine.match(/total≈([\d.]+)W/);
  const currentMatch = rawLine.match(/current≈([\d.]+)A/);
  const voltageMatch = rawLine.match(/voltage≈([\d.]+)V\s*\(([\d.]+)-([\d.]+)\)/);
  const batteryMatch = rawLine.match(/battery≈([\d.]+)%/);
  const peakMatch = rawLine.match(/peak=([a-zA-Z0-9_]+)@([\d.]+)A/);
  const hottestMatch = rawLine.match(/hottest=([a-zA-Z0-9_]+)@([\d.]+)C/);
  const tempsMatch = rawLine.match(/temps=([^\s]+)/);

  const tempsByServo = new Map<string, number>();
  if (tempsMatch) {
    for (const item of tempsMatch[1].split(",")) {
      const match = item.match(/^([a-zA-Z0-9_]+)@([\d.]+)C$/);
      const temperatureC = parseNumber(match?.[2]);
      if (match?.[1] && temperatureC !== null) {
        tempsByServo.set(match[1], temperatureC);
      }
    }
  }

  const hottestTemperatureC = parseNumber(hottestMatch?.[2]);
  if (hottestMatch?.[1] && hottestTemperatureC !== null && !tempsByServo.has(hottestMatch[1])) {
    tempsByServo.set(hottestMatch[1], hottestTemperatureC);
  }

  return {
    sourceLabel: latest.sourceLabel,
    timestamp: rawLine.match(/\[power\]\s+(\S+)/)?.[1] ?? null,
    batteryPercent: parseNumber(batteryMatch?.[1]),
    voltage: parseNumber(voltageMatch?.[1]),
    voltageMin: parseNumber(voltageMatch?.[2]),
    voltageMax: parseNumber(voltageMatch?.[3]),
    totalPowerW: parseNumber(totalMatch?.[1]),
    totalCurrentA: parseNumber(currentMatch?.[1]),
    motorsReporting: parseNumber(motorsMatch?.[1]),
    motorsExpected: parseNumber(motorsMatch?.[2]),
    peakMotor: peakMatch?.[1] ?? null,
    peakCurrentA: parseNumber(peakMatch?.[2]),
    hottestMotor: hottestMatch?.[1] ?? null,
    hottestTemperatureC,
    servoTemperatures: ARM_SERVO_IDS.map((id) => ({
      id,
      label: labelServo(id),
      temperatureC: tempsByServo.get(id) ?? null,
    })),
    rawLine,
  };
}

function batteryTone(percent: number | null): string {
  if (percent === null) {
    return "muted";
  }
  if (percent <= 20) {
    return "bad";
  }
  if (percent <= 35) {
    return "warn";
  }
  return "good";
}

function temperatureTone(temperatureC: number | null): string {
  if (temperatureC === null) {
    return "muted";
  }
  if (temperatureC >= 70) {
    return "bad";
  }
  if (temperatureC >= 55) {
    return "warn";
  }
  return "good";
}

function formatMetric(value: number | null, suffix: string, digits = 1): string {
  return value === null ? "n/a" : `${value.toFixed(digits)}${suffix}`;
}

function BatteryGauge({ percent }: { percent: number | null }) {
  const safePercent = percent === null ? 0 : Math.min(Math.max(percent, 0), 100);
  return (
    <div className={`battery-gauge ${batteryTone(percent)}`}>
      <span className="battery-fill" style={{ width: `${safePercent}%` }} />
      <span className="battery-label">{percent === null ? "n/a" : `${Math.round(percent)}%`}</span>
    </div>
  );
}

function clampTorqueLimit(value: number): number {
  if (!Number.isFinite(value)) {
    return TORQUE_LIMIT_MAX;
  }
  return Math.min(TORQUE_LIMIT_MAX, Math.max(TORQUE_LIMIT_MIN, Math.round(value)));
}

function PowerTelemetryPanel({
  telemetry,
  torqueLimits,
  calibrationBusy,
  calibratingServoId,
  controlsDisabled,
  onApplyRecommendedTorqueLimits,
  onServoCalibrationStart,
  onTorqueLimitChange,
}: {
  telemetry: PowerTelemetry | null;
  torqueLimits: Record<string, number>;
  calibrationBusy: boolean;
  calibratingServoId: string | null;
  controlsDisabled: boolean;
  onApplyRecommendedTorqueLimits: () => void;
  onServoCalibrationStart: (servoId: string) => void;
  onTorqueLimitChange: (servoId: string, value: number) => void;
}) {
  return (
    <section className="power-panel">
      <div className="power-summary">
        <div>
          <p className="card-kicker">Power</p>
          <h3>Battery and Servo Temperatures</h3>
          <p className="card-note">
            {telemetry?.timestamp
              ? `Last power sample: ${new Date(telemetry.timestamp).toLocaleTimeString()} from ${telemetry.sourceLabel ?? "the active process"}.`
              : "Waiting for the next power sample from the active robot process."}
          </p>
        </div>
        <BatteryGauge percent={telemetry?.batteryPercent ?? null} />
      </div>

      <div className="button-cluster inline">
        <button
          disabled={controlsDisabled || calibrationBusy}
          onClick={onApplyRecommendedTorqueLimits}
        >
          Apply Safer Torque Preset
        </button>
      </div>

      <div className="power-metrics">
        <div>
          <span>Voltage</span>
          <strong>{formatMetric(telemetry?.voltage ?? null, "V", 2)}</strong>
          <small>
            {telemetry?.voltageMin !== null && telemetry?.voltageMin !== undefined
              ? `${telemetry.voltageMin.toFixed(2)}-${telemetry.voltageMax?.toFixed(2) ?? "n/a"}V`
              : "range n/a"}
          </small>
        </div>
        <div>
          <span>Draw</span>
          <strong>{formatMetric(telemetry?.totalPowerW ?? null, "W", 1)}</strong>
          <small>{formatMetric(telemetry?.totalCurrentA ?? null, "A", 2)}</small>
        </div>
        <div>
          <span>Motors</span>
          <strong>
            {telemetry?.motorsReporting ?? "n/a"}/{telemetry?.motorsExpected ?? "n/a"}
          </strong>
          <small>reporting</small>
        </div>
        <div>
          <span>Peak Current</span>
          <strong>{labelServo(telemetry?.peakMotor ?? null)}</strong>
          <small>{formatMetric(telemetry?.peakCurrentA ?? null, "A", 2)}</small>
        </div>
      </div>

      <div className="servo-temperature-grid">
        {(telemetry?.servoTemperatures.length
          ? telemetry.servoTemperatures
          : ARM_SERVO_IDS.map((id) => ({ id, label: labelServo(id), temperatureC: null }))
        ).map((servo) => (
          <article key={servo.id} className={`servo-temperature ${temperatureTone(servo.temperatureC)}`}>
            <span>{servo.label}</span>
            <div className="servo-controls">
              <label className="servo-torque-control">
                <small>Torque</small>
                <input
                  type="range"
                  min={TORQUE_LIMIT_MIN}
                  max={TORQUE_LIMIT_MAX}
                  step="10"
                  disabled={controlsDisabled || calibrationBusy}
                  value={torqueLimits[servo.id] ?? TORQUE_LIMIT_MAX}
                  onChange={(event) =>
                    onTorqueLimitChange(servo.id, Number(event.target.value))
                  }
                />
                <small>{torqueLimits[servo.id] ?? TORQUE_LIMIT_MAX}</small>
              </label>
              <button
                className="servo-calibrate-button"
                disabled={controlsDisabled || calibrationBusy}
                onClick={() => onServoCalibrationStart(servo.id)}
              >
                {calibratingServoId === servo.id ? "Calibrating..." : "Calibrate"}
              </button>
            </div>
            <strong>{servo.temperatureC === null ? "--" : `${Math.round(servo.temperatureC)}C`}</strong>
          </article>
        ))}
      </div>
    </section>
  );
}

function logSortKey(line: string): number {
  const match = line.match(/^(\d{1,2}):(\d{2}):(\d{2})\s(AM|PM)\b/);
  if (!match) {
    return Number.MAX_SAFE_INTEGER;
  }

  let hours = Number(match[1]) % 12;
  const minutes = Number(match[2]);
  const seconds = Number(match[3]);
  if (match[4] === "PM") {
    hours += 12;
  }

  return hours * 3600 + minutes * 60 + seconds;
}

function mergeLiveConsoleLines(groups: string[][], count: number): string[] {
  return groups
    .flatMap((lines, groupIndex) =>
      lines.map((line, lineIndex) => ({
        line,
        order: groupIndex * 1000 + lineIndex,
        sortKey: logSortKey(line),
      })),
    )
    .sort((left, right) => left.sortKey - right.sortKey || left.order - right.order)
    .slice(-count)
    .map((entry) => entry.line);
}

function preferredLogServiceLabel(services: ServiceSnapshot[]): string {
  return (
    services.find((service) => service.state === "running")?.label ??
    services.find((service) => service.state !== "idle")?.label ??
    services[0]?.label ??
    ""
  );
}

async function writeClipboardText(text: string): Promise<void> {
  if (navigator.clipboard?.writeText) {
    await navigator.clipboard.writeText(text);
    return;
  }

  const textarea = document.createElement("textarea");
  textarea.value = text;
  textarea.setAttribute("readonly", "");
  textarea.style.position = "fixed";
  textarea.style.opacity = "0";
  document.body.appendChild(textarea);
  textarea.select();
  document.execCommand("copy");
  textarea.remove();
}

export default function App() {
  const [state, setState] = useState<DashboardState | null>(null);
  const [activeSection, setActiveSection] = useState<SectionKey>("overview");
  const [settingsDraft, setSettingsDraft] = useState<AppSettings | null>(null);
  const [settingsDirty, setSettingsDirty] = useState(false);
  const [trainingDraft, setTrainingDraft] = useState<TrainingProfile | null>(null);
  const [trainingDirty, setTrainingDirty] = useState(false);
  const [recordLabel, setRecordLabel] = useState("");
  const [recordInputMode, setRecordInputMode] = useState<RecordingInputMode>("auto");
  const [selectedRecording, setSelectedRecording] = useState<string>("");
  const [recordingNameDraft, setRecordingNameDraft] = useState("");
  const [trimStartDraft, setTrimStartDraft] = useState("0");
  const [trimEndDraft, setTrimEndDraft] = useState("");
  const [pinName, setPinName] = useState("");
  const [pinHotkey, setPinHotkey] = useState("");
  const [pinSpeed, setPinSpeed] = useState(1);
  const [pinHoldFinal, setPinHoldFinal] = useState(0.5);
  const [replayTarget, setReplayTarget] = useState<ReplayTarget>("pi");
  const [torqueDraft, setTorqueDraft] = useState<Record<string, number>>({});
  const [pendingAction, setPendingAction] = useState<string | null>(null);
  const [toast, setToast] = useState<string>("");
  const [backendError, setBackendError] = useState<string>("");
  const [recordingClockMs, setRecordingClockMs] = useState(() => Date.now());
  const [recordingDetail, setRecordingDetail] = useState<RecordingDetail | null>(null);
  const [recordingDetailLoading, setRecordingDetailLoading] = useState(false);
  const [recordingDetailError, setRecordingDetailError] = useState("");
  const [recordingDetailReloadToken, setRecordingDetailReloadToken] = useState(0);
  const [replayIncludeBasePreference, setReplayIncludeBasePreference] = useState<boolean | null>(null);
  const [vexReplayMode, setVexReplayMode] = useState<VexReplayMode>("ecu");
  const [playbackTimeS, setPlaybackTimeS] = useState(0);
  const [playbackPlaying, setPlaybackPlaying] = useState(false);
  const [selectedLogServiceLabel, setSelectedLogServiceLabel] = useState("");
  const toastTimer = useRef<number | null>(null);
  const torqueTimers = useRef<Record<string, number>>({});
  const playbackAnchorRef = useRef<{ startedAtMs: number; baseTimeS: number } | null>(null);
  const stateRequestController = useRef<AbortController | null>(null);

  const loadState = async () => {
    const controller = new AbortController();
    stateRequestController.current?.abort();
    stateRequestController.current = controller;
    try {
      const next = await request<DashboardState>("/api/state", { signal: controller.signal });
      if (stateRequestController.current !== controller) {
        return;
      }
      setBackendError("");
      setState(next);
      if (!settingsDirty || !settingsDraft) {
        setSettingsDraft(next.settings);
      }
    } catch (error) {
      if (stateRequestController.current !== controller) {
        return;
      }
      if (error instanceof Error && error.name === "AbortError") {
        return;
      }
      setBackendError(error instanceof Error ? error.message : "Backend state is unavailable.");
    }
  };

  useEffect(() => {
    void loadState();
    const interval = window.setInterval(() => {
      void loadState().catch(() => undefined);
    }, POLL_MS);
    return () => {
      window.clearInterval(interval);
      stateRequestController.current?.abort();
    };
  }, []);

  useEffect(() => {
    return () => {
      for (const timer of Object.values(torqueTimers.current)) {
        window.clearTimeout(timer);
      }
    };
  }, []);

  useEffect(() => {
    if (!state) {
      return;
    }
    if (!state.recordings.length) {
      if (selectedRecording) {
        setSelectedRecording("");
        setRecordingDetail(null);
      }
      return;
    }

    const stillExists = state.recordings.some((item) => item.path === selectedRecording);
    if (!selectedRecording || !stillExists) {
      setSelectedRecording(state.recordings[0].path);
    }
  }, [selectedRecording, state]);

  useEffect(() => {
    if (!state) {
      return;
    }

    const activeRecording = state.recordings.find((item) => item.path === selectedRecording);
    if (activeRecording && !pinName) {
      setPinName(activeRecording.name);
    }
  }, [pinName, selectedRecording, state]);

  useEffect(() => {
    const listener = (event: KeyboardEvent) => {
      if (!state?.pinnedMoves.length) {
        return;
      }

      const target = event.target as HTMLElement | null;
      const tagName = target?.tagName?.toLowerCase();
      if (
        target?.isContentEditable ||
        tagName === "input" ||
        tagName === "textarea" ||
        tagName === "select"
      ) {
        return;
      }

      const normalized = normalizeKeyboardEvent(event);
      const match = state.pinnedMoves.find(
        (item) => normalizeHotkeyText(item.keyBinding) === normalized,
      );
      if (!match) {
        return;
      }

      event.preventDefault();
      void handleTriggerPinnedMove(match);
    };

    window.addEventListener("keydown", listener);
    return () => window.removeEventListener("keydown", listener);
  }, [state]);

  const selectedRecordingEntry = useMemo<RecordingEntry | undefined>(
    () => state?.recordings.find((item) => item.path === selectedRecording),
    [selectedRecording, state?.recordings],
  );
  const selectedRecordingReplayOptions: RecordingReplayOptions = useMemo(() => {
    const savedOptions = state?.recordingReplayOptions[selectedRecording];
    return {
      homeMode: savedOptions?.homeMode ?? "none",
      speed: savedOptions?.speed ?? state?.settings.trajectories.defaultReplaySpeed ?? 1,
      autoVexPositioning: savedOptions?.autoVexPositioning ?? true,
    };
  }, [
    selectedRecording,
    state?.recordingReplayOptions,
    state?.settings.trajectories.defaultReplaySpeed,
  ]);
  const selectedRecordingHomeMode = selectedRecordingReplayOptions.homeMode;
  const selectedTrainingProfile = state?.training.selectedProfile ?? null;
  const selectedRecordingHasBaseReference = useMemo(
    () => recordingHasReplayableBaseReference(recordingDetail),
    [recordingDetail],
  );
  const effectiveReplayIncludeBase =
    replayTarget === "pi" ? (replayIncludeBasePreference ?? false) : false;

  useEffect(() => {
    setRecordingNameDraft(selectedRecordingEntry?.name ?? "");
  }, [selectedRecordingEntry?.name, selectedRecordingEntry?.path]);

  useEffect(() => {
    setReplayIncludeBasePreference(null);
  }, [selectedRecordingEntry?.path]);

  useEffect(() => {
    setPinSpeed(selectedRecordingReplayOptions.speed);
  }, [selectedRecordingEntry?.path, selectedRecordingReplayOptions.speed]);

  useEffect(() => {
    setTrimStartDraft("0");
    setTrimEndDraft(
      selectedRecordingEntry?.durationS !== null && selectedRecordingEntry?.durationS !== undefined
        ? String(selectedRecordingEntry.durationS)
        : "",
    );
  }, [selectedRecordingEntry?.durationS, selectedRecordingEntry?.path]);

  useEffect(() => {
    if (!selectedRecordingEntry) {
      setRecordingDetail(null);
      setRecordingDetailLoading(false);
      setRecordingDetailError("");
      setPlaybackPlaying(false);
      playbackAnchorRef.current = null;
      setPlaybackTimeS(0);
      return;
    }

    const payload: RecordingDetailRequest = {
      path: selectedRecordingEntry.path,
    };
    let cancelled = false;

    setRecordingDetailLoading(true);
    setRecordingDetailError("");
    setPlaybackPlaying(false);
    playbackAnchorRef.current = null;
    setPlaybackTimeS(0);

    void request<RecordingDetail>("/api/recordings/detail", {
      method: "POST",
      body: JSON.stringify(payload),
    })
      .then((detail) => {
        if (cancelled) {
          return;
        }
        setRecordingDetail(detail);
        setRecordingDetailLoading(false);
      })
      .catch((error) => {
        if (cancelled) {
          return;
        }
        setRecordingDetail(null);
        setRecordingDetailLoading(false);
        setRecordingDetailError(
          error instanceof Error ? error.message : "Could not load the recording timeline.",
        );
      });

    return () => {
      cancelled = true;
    };
  }, [recordingDetailReloadToken, selectedRecordingEntry?.modifiedAt, selectedRecordingEntry?.path]);

  useEffect(() => {
    if (!recordingDetail) {
      return;
    }

    setPlaybackTimeS((current) => clampPlaybackTime(current, recordingDetail.durationS));
  }, [recordingDetail]);

  useEffect(() => {
    if (!playbackPlaying || !recordingDetail) {
      return;
    }

    let frameId = 0;
    const tick = (now: number) => {
      const anchor = playbackAnchorRef.current;
      if (!anchor) {
        return;
      }

      const nextTimeS = clampPlaybackTime(
        anchor.baseTimeS + (now - anchor.startedAtMs) / 1000,
        recordingDetail.durationS,
      );
      setPlaybackTimeS(nextTimeS);

      if (nextTimeS >= recordingDetail.durationS) {
        playbackAnchorRef.current = null;
        setPlaybackPlaying(false);
        return;
      }

      frameId = window.requestAnimationFrame(tick);
    };

    frameId = window.requestAnimationFrame(tick);
    return () => window.cancelAnimationFrame(frameId);
  }, [playbackPlaying, recordingDetail]);

  useEffect(() => {
    if (!selectedTrainingProfile) {
      setTrainingDraft(null);
      setTrainingDirty(false);
      return;
    }

    if (!trainingDirty || trainingDraft?.id !== selectedTrainingProfile.id) {
      setTrainingDraft(selectedTrainingProfile);
      setTrainingDirty(false);
    }
  }, [selectedTrainingProfile, trainingDirty, trainingDraft?.id]);

  const recordingService = state?.services.host ?? null;
  const isRecordingActive = Boolean(
    recordingService?.mode === "recording" &&
      recordingService.startedAt &&
      ["starting", "running", "stopping"].includes(recordingService.state),
  );
  const recordingStartedLog = useMemo(
    () =>
      recordingService?.logs.find((line) => line.includes("Recording started.")) ?? null,
    [recordingService?.logs],
  );
  const recordingStartedMs = useMemo(
    () => (recordingStartedLog ? parseLocalLogTimestamp(recordingStartedLog) : null),
    [recordingStartedLog],
  );
  const recordingHasCapturedMotion = Boolean(recordingStartedLog);
  const serviceInMotionSensitiveState = (service: ServiceSnapshot): boolean =>
    ["starting", "running", "stopping"].includes(service.state);
  const hostBlocksVexGyroZero = Boolean(
    state &&
      serviceInMotionSensitiveState(state.services.host) &&
      state.services.host.mode !== "keyboard-control",
  );
  const vexGyroZeroBusy = Boolean(
    state &&
      (hostBlocksVexGyroZero ||
        [
          state.services.replay,
          state.services.piCalibration,
          state.services.datasetCapture,
          state.services.policyEval,
        ].some(serviceInMotionSensitiveState)),
  );

  useEffect(() => {
    if (!isRecordingActive) {
      return;
    }

    setRecordingClockMs(Date.now());
    const interval = window.setInterval(() => {
      setRecordingClockMs(Date.now());
    }, 1000);
    return () => window.clearInterval(interval);
  }, [isRecordingActive, recordingService?.startedAt]);

  const selectedMovePayload = useMemo(
    () => ({
      trajectoryPath: selectedRecording,
      target: replayTarget,
      vexReplayMode,
      homeMode: replayTarget === "pi" ? selectedRecordingHomeMode : "none",
      autoVexPositioning:
        replayTarget === "pi" ? selectedRecordingReplayOptions.autoVexPositioning : false,
      includeBase: effectiveReplayIncludeBase,
      speed: pinSpeed,
      holdFinalS: pinHoldFinal,
    }),
    [
      effectiveReplayIncludeBase,
      pinHoldFinal,
      pinSpeed,
      replayTarget,
      selectedRecording,
      selectedRecordingHomeMode,
      selectedRecordingReplayOptions.autoVexPositioning,
      vexReplayMode,
    ],
  );
  const recordingDurationS = recordingDetail?.durationS ?? selectedRecordingEntry?.durationS ?? 0;
  const trimStartValueS = parseNumber(trimStartDraft) ?? 0;
  const trimEndValueS = parseNumber(trimEndDraft) ?? recordingDurationS;

  const liveConsoleLines = useMemo(() => {
    if (!state) {
      return [];
    }

    return mergeLiveConsoleLines(
      [
        latestLogLines(state.activityLog, 40),
        latestLogLines(state.services.host.logs, 30),
        latestLogLines(state.services.teleop.logs, 30),
        latestLogLines(state.services.replay.logs, 30),
        latestLogLines(state.services.piCalibration.logs, 30),
        latestLogLines(state.services.macCalibration.logs, 30),
        latestLogLines(state.services.datasetCapture.logs, 30),
        latestLogLines(state.services.datasetSync.logs, 30),
        latestLogLines(state.services.training.logs, 30),
        latestLogLines(state.services.deployment.logs, 30),
        latestLogLines(state.services.policyBenchmark.logs, 30),
        latestLogLines(state.services.policyEval.logs, 30),
      ],
      100,
    );
  }, [state]);

  const powerTelemetry = useMemo(
    () =>
      state
        ? parsePowerTelemetry([
            { label: state.services.host.label, logs: state.services.host.logs },
            { label: state.services.replay.label, logs: state.services.replay.logs },
          ])
        : null,
    [state],
  );
  const sensorTelemetryEntries = useMemo(
    () =>
      state
        ? parseSensorTelemetryEntries([
            state.services.host,
            state.services.replay,
            state.services.datasetCapture,
          ])
        : [],
    [state],
  );
  const sensorTelemetryLines = useMemo(
    () => sensorTelemetryEntries.map((entry) => entry.line),
    [sensorTelemetryEntries],
  );
  const latestVexPositionStatus = useMemo(
    () =>
      state
        ? parseLatestVexPositionStatus([
            state.services.host,
            state.services.replay,
          ])
        : null,
    [state],
  );

  const torqueLimits = useMemo(
    () => ({
      ...DEFAULT_ARM_TORQUE_LIMITS,
      ...(state?.settings.host.armTorqueLimits ?? {}),
      ...torqueDraft,
    }),
    [state?.settings.host.armTorqueLimits, torqueDraft],
  );

  const serviceSnapshots = useMemo(
    () =>
      state
        ? [
            state.services.host,
            state.services.teleop,
            state.services.replay,
            state.services.piCalibration,
            state.services.macCalibration,
            state.services.datasetCapture,
            state.services.datasetSync,
            state.services.training,
            state.services.deployment,
            state.services.policyBenchmark,
            state.services.policyEval,
          ]
        : [],
    [state],
  );
  const hotkeysArmed =
    state?.services.host.state === "running" && state.services.host.mode === "keyboard-control";
  const keyboardBackupActive =
    state?.services.teleop.state === "running" && state.services.teleop.mode === "keyboard-control";
  const warmReplayReady =
    state?.services.host.state === "running" &&
    (state.services.host.mode === "control" || state.services.host.mode === "keyboard-control");
  const warmReplayFromControl =
    state?.services.host.state === "running" && state.services.host.mode === "control";
  const homeCommandReady = Boolean(warmReplayReady);
  const hasArmHomePosition = Boolean(state?.homePosition);
  const recordingUltrasonicResetReady = Boolean(selectedRecordingEntry);

  const activeSectionMeta = useMemo(
    () => SECTIONS.find((section) => section.key === activeSection) ?? SECTIONS[0],
    [activeSection],
  );

  useEffect(() => {
    if (!serviceSnapshots.length) {
      setSelectedLogServiceLabel("");
      return;
    }

    const stillExists = serviceSnapshots.some((service) => service.label === selectedLogServiceLabel);
    if (!selectedLogServiceLabel || !stillExists) {
      setSelectedLogServiceLabel(preferredLogServiceLabel(serviceSnapshots));
    }
  }, [selectedLogServiceLabel, serviceSnapshots]);

  const selectedLogService = useMemo(
    () =>
      serviceSnapshots.find((service) => service.label === selectedLogServiceLabel) ??
      serviceSnapshots[0] ??
      null,
    [selectedLogServiceLabel, serviceSnapshots],
  );

  const flashToast = (message: string) => {
    setToast(message);
    if (toastTimer.current) {
      window.clearTimeout(toastTimer.current);
    }
    toastTimer.current = window.setTimeout(() => setToast(""), 2200);
  };

  const seekPlayback = (nextTimeS: number) => {
    const clampedTimeS = clampPlaybackTime(nextTimeS, recordingDurationS);
    setPlaybackTimeS(clampedTimeS);
    if (playbackPlaying) {
      playbackAnchorRef.current = {
        startedAtMs: performance.now(),
        baseTimeS: clampedTimeS,
      };
    }
  };

  const handleTogglePlayback = () => {
    if (!recordingDetail || recordingDetail.durationS <= 0) {
      return;
    }

    if (playbackPlaying) {
      playbackAnchorRef.current = null;
      setPlaybackPlaying(false);
      return;
    }

    const nextBaseTimeS =
      playbackTimeS >= recordingDetail.durationS ? 0 : clampPlaybackTime(playbackTimeS, recordingDetail.durationS);
    setPlaybackTimeS(nextBaseTimeS);
    playbackAnchorRef.current = {
      startedAtMs: performance.now(),
      baseTimeS: nextBaseTimeS,
    };
    setPlaybackPlaying(true);
  };

  const handleSetTrimStartFromPlayhead = () => {
    const nextStartS = clampPlaybackTime(playbackTimeS, recordingDurationS);
    const nextEndS =
      trimEndValueS > nextStartS ? clampPlaybackTime(trimEndValueS, recordingDurationS) : recordingDurationS;
    setTrimStartDraft(formatSecondsInput(nextStartS));
    setTrimEndDraft(formatSecondsInput(nextEndS));
  };

  const handleSetTrimEndFromPlayhead = () => {
    const nextEndS = clampPlaybackTime(playbackTimeS, recordingDurationS);
    const nextStartS =
      trimStartValueS < nextEndS ? clampPlaybackTime(trimStartValueS, recordingDurationS) : 0;
    setTrimStartDraft(formatSecondsInput(nextStartS));
    setTrimEndDraft(formatSecondsInput(nextEndS));
  };

  const handleCopyLiveConsole = async () => {
    const text = liveConsoleLines.join("\n");
    if (!text) {
      flashToast("No live console output to copy.");
      return;
    }

    try {
      await writeClipboardText(text);
      flashToast("Live console copied.");
    } catch {
      flashToast("Could not copy live console.");
    }
  };

  const handleCopySelectedLogs = async () => {
    const text = selectedLogService?.logs.join("\n") ?? "";
    if (!text) {
      flashToast("No selected log output to copy.");
      return;
    }

    try {
      await writeClipboardText(text);
      flashToast(`${selectedLogService?.label ?? "Process"} logs copied.`);
    } catch {
      flashToast("Could not copy the selected logs.");
    }
  };

  const handleCopySensorTelemetry = async () => {
    const text = sensorTelemetryLines.join("\n");
    if (!text) {
      flashToast("No sensor telemetry to copy.");
      return;
    }

    try {
      await writeClipboardText(text);
      flashToast("Sensor telemetry copied.");
    } catch {
      flashToast("Could not copy sensor telemetry.");
    }
  };

  const mutate = async (
    label: string,
    url: string,
    init?: RequestInit,
  ): Promise<DashboardState | null> => {
    try {
      setPendingAction(label);
      const next = await request<DashboardState>(url, init);
      setBackendError("");
      setState(next);
      if (!settingsDirty) {
        setSettingsDraft(next.settings);
      }
      return next;
    } catch (error) {
      flashToast(error instanceof Error ? error.message : "Request failed.");
      return null;
    } finally {
      setPendingAction(null);
    }
  };

  const updateSettingsField = <K extends keyof AppSettings>(
    section: K,
    value: AppSettings[K],
  ) => {
    setSettingsDraft((current) => {
      if (!current) {
        return current;
      }
      return {
        ...current,
        [section]: value,
      };
    });
    setSettingsDirty(true);
  };

  const handleSaveSettings = async (showToast = true) => {
    if (!settingsDraft) {
      return null;
    }

    const next = await mutate("save-settings", "/api/settings", {
      method: "POST",
      body: JSON.stringify(settingsDraft),
    });
    if (next) {
      setSettingsDraft(next.settings);
      setSettingsDirty(false);
      if (showToast) {
        flashToast("Settings saved.");
      }
    }
    return next;
  };

  const handleSyncVexTelemetry = async () => {
    if (settingsDirty) {
      const saved = await handleSaveSettings(false);
      if (!saved) {
        return;
      }
    }

    const next = await mutate("sync-vex-telemetry", "/api/vex/telemetry/sync", {
      method: "POST",
    });
    if (next) {
      setSettingsDraft(next.settings);
      setSettingsDirty(false);
      flashToast("VEX telemetry synced.");
    }
  };

  const handleZeroVexGyro = async () => {
    if (settingsDirty) {
      const saved = await handleSaveSettings(false);
      if (!saved) {
        return;
      }
    }

    const next = await mutate("zero-vex-gyro", "/api/vex/gyro/zero", {
      method: "POST",
    });
    if (next) {
      setSettingsDraft(next.settings);
      setSettingsDirty(false);
      flashToast("VEX gyro zeroed.");
    }
  };

  const handleSetArmHome = async () => {
    const next = await mutate("set-arm-home", "/api/arm/home/set", {
      method: "POST",
    });
    if (next) {
      flashToast("Arm home position saved.");
    }
  };

  const handleGoArmHome = async () => {
    const next = await mutate("go-arm-home", "/api/arm/home/go", {
      method: "POST",
    });
    if (next) {
      flashToast("Arm moved home.");
    }
  };

  const handleSetArmHomeFromRecordingStart = async () => {
    if (!selectedRecordingEntry) {
      flashToast("Select a recording first.");
      return;
    }

    const payload: SetArmHomeFromRecordingRequest = {
      path: selectedRecordingEntry.path,
    };
    const next = await mutate("set-recording-start-home", "/api/recordings/home/set-from-start", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    if (next) {
      flashToast("Recording start saved as arm home.");
    }
  };

  const updateTrainingField = <K extends keyof TrainingProfile>(
    field: K,
    value: TrainingProfile[K],
  ) => {
    setTrainingDraft((current) => {
      if (!current) {
        return current;
      }
      return {
        ...current,
        [field]: value,
      };
    });
    setTrainingDirty(true);
  };

  const handleSaveTrainingProfile = async () => {
    if (!trainingDraft) {
      flashToast("Create or select a training profile first.");
      return;
    }

    const next = await mutate("save-training-profile", "/api/training/profiles", {
      method: "POST",
      body: JSON.stringify(trainingDraft),
    });

    if (next) {
      setTrainingDraft(next.training.selectedProfile);
      setTrainingDirty(false);
      flashToast("Training profile saved.");
    }
  };

  const handleCreateTrainingProfile = () => {
    setTrainingDraft(createTrainingDraft(selectedTrainingProfile));
    setTrainingDirty(true);
    setActiveSection("training");
  };

  const handleDeleteTrainingProfile = async () => {
    if (!selectedTrainingProfile) {
      flashToast("Select a training profile first.");
      return;
    }

    const next = await mutate(
      "delete-training-profile",
      `/api/training/profiles/${selectedTrainingProfile.id}`,
      { method: "DELETE" },
    );

    if (next) {
      setTrainingDraft(next.training.selectedProfile);
      setTrainingDirty(false);
      flashToast("Training profile removed.");
    }
  };

  const handleSelectTrainingProfile = async (profileId: string) => {
    const next = await mutate("select-training-profile", "/api/training/profiles/select", {
      method: "POST",
      body: JSON.stringify({ id: profileId }),
    });

    if (next) {
      setTrainingDraft(next.training.selectedProfile);
      setTrainingDirty(false);
    }
  };

  const mutateTrainingProfileAction = async (
    label: string,
    url: string,
    successMessage: string,
  ) => {
    if (!selectedTrainingProfile) {
      flashToast("Save and select a training profile first.");
      return;
    }

    const next = await mutate(label, url, {
      method: "POST",
      body: JSON.stringify({ profileId: selectedTrainingProfile.id }),
    });

    if (next) {
      flashToast(successMessage);
    }
  };

  const sendTorqueLimit = async (servoId: string, value: number) => {
    try {
      const next = await request<DashboardState>("/api/robot/torque-limits", {
        method: "POST",
        body: JSON.stringify({ limits: { [servoId]: value } }),
      });
      setState(next);
      setSettingsDraft((current) =>
        current
          ? {
              ...current,
              host: {
                ...current.host,
                armTorqueLimits: next.settings.host.armTorqueLimits,
              },
            }
          : next.settings,
      );
      setTorqueDraft((current) => {
        const nextDraft = { ...current };
        delete nextDraft[servoId];
        return nextDraft;
      });
    } catch (error) {
      flashToast(error instanceof Error ? error.message : "Could not update torque.");
    }
  };

  const handleTorqueLimitChange = (servoId: string, value: number) => {
    const limit = clampTorqueLimit(value);
    setTorqueDraft((current) => ({ ...current, [servoId]: limit }));

    if (torqueTimers.current[servoId]) {
      window.clearTimeout(torqueTimers.current[servoId]);
    }
    torqueTimers.current[servoId] = window.setTimeout(() => {
      delete torqueTimers.current[servoId];
      void sendTorqueLimit(servoId, limit);
    }, 350);
  };

  const handleApplyRecommendedTorqueLimits = async () => {
    const next = await mutate("recommended-torque-preset", "/api/robot/torque-limits", {
      method: "POST",
      body: JSON.stringify({ limits: RECOMMENDED_SAFE_ARM_TORQUE_LIMITS }),
    });
    if (!next) {
      return;
    }
    setTorqueDraft({});
    setSettingsDraft((current) =>
      current
        ? {
            ...current,
            host: {
              ...current.host,
              armTorqueLimits: next.settings.host.armTorqueLimits,
            },
          }
        : next.settings,
    );
    flashToast("Safer torque preset applied.");
  };

  const handleServoCalibrationStart = async (servoId: string) => {
    const payload: ServoCalibrationRequest = { servoId };
    const next = await mutate("pi-servo-calibration", "/api/calibration/pi/servo", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    if (next) {
      flashToast(`${labelServo(servoId)} calibration started.`);
    }
  };

  const handleTriggerPinnedMove = async (move: PinnedMove) => {
    const next = await mutate(
      `trigger-${move.id}`,
      `/api/pinned-moves/${move.id}/trigger`,
      { method: "POST" },
    );
    if (next) {
      flashToast(`Triggered ${move.name}.`);
    }
  };

  const handleRenameRecording = async () => {
    if (!selectedRecordingEntry) {
      flashToast("Select a recording first.");
      return;
    }

    const payload: RenameRecordingRequest = {
      path: selectedRecordingEntry.path,
      label: recordingNameDraft.trim(),
    };
    const next = await mutate("rename-recording", "/api/recordings/rename", {
      method: "POST",
      body: JSON.stringify(payload),
    });

    if (next) {
      flashToast("Recording name saved.");
    }
  };

  const handleDuplicateRecording = async () => {
    if (!selectedRecordingEntry) {
      flashToast("Select a recording first.");
      return;
    }

    const existingPaths = new Set(state?.recordings.map((item) => item.path) ?? []);
    const payload: DuplicateRecordingRequest = {
      path: selectedRecordingEntry.path,
    };
    const next = await mutate("duplicate-recording", "/api/recordings/duplicate", {
      method: "POST",
      body: JSON.stringify(payload),
    });

    if (next) {
      const duplicatedRecording =
        next.recordings.find((item) => !existingPaths.has(item.path)) ?? next.recordings[0];
      if (duplicatedRecording) {
        setSelectedRecording(duplicatedRecording.path);
        setRecordingNameDraft(duplicatedRecording.name);
        setPlaybackTimeS(0);
      }
      setRecordingDetail(null);
      setRecordingDetailError("");
      flashToast("Recording duplicated.");
    }
  };

  const handleDeleteRecording = async () => {
    if (!selectedRecordingEntry) {
      flashToast("Select a recording first.");
      return;
    }

    const confirmed = window.confirm(
      `Delete ${selectedRecordingEntry.name}? This removes the trajectory JSON from the Pi.`,
    );
    if (!confirmed) {
      return;
    }

    const payload: DeleteRecordingRequest = {
      path: selectedRecordingEntry.path,
    };
    const next = await mutate("delete-recording", "/api/recordings", {
      method: "DELETE",
      body: JSON.stringify(payload),
    });

    if (next) {
      setSelectedRecording(next.recordings[0]?.path ?? "");
      setRecordingDetail(null);
      setRecordingDetailError("");
      flashToast("Recording deleted.");
    }
  };

  const handleTrimRecording = async () => {
    if (!selectedRecordingEntry) {
      flashToast("Select a recording first.");
      return;
    }

    const trimStartS = Number(trimStartDraft);
    const trimEndS = Number(trimEndDraft);
    if (!Number.isFinite(trimStartS) || !Number.isFinite(trimEndS) || trimStartS < 0 || trimEndS <= trimStartS) {
      flashToast("Trim end must be greater than trim start.");
      return;
    }

    const payload: TrimRecordingRequest = {
      path: selectedRecordingEntry.path,
      trimStartS,
      trimEndS,
    };
    const next = await mutate("trim-recording", "/api/recordings/trim", {
      method: "POST",
      body: JSON.stringify(payload),
    });

    if (next) {
      const updatedRecording = next.recordings.find((item) => item.path === selectedRecordingEntry.path);
      setTrimStartDraft("0");
      setTrimEndDraft(
        updatedRecording?.durationS !== null && updatedRecording?.durationS !== undefined
          ? String(updatedRecording.durationS)
          : "",
      );
      flashToast("Recording trimmed.");
    }
  };

  const handleMarkRecordingGyroZero = async () => {
    if (!selectedRecordingEntry) {
      flashToast("Select a recording first.");
      return;
    }

    const payload: MarkRecordingGyroZeroRequest = {
      path: selectedRecordingEntry.path,
    };
    const next = await mutate("mark-recording-gyro-zero", "/api/recordings/mark-gyro-zero", {
      method: "POST",
      body: JSON.stringify(payload),
    });

    if (next) {
      flashToast("Recording start marked as gyro zero.");
    }
  };

  const handleResetRecordingUltrasonicPosition = async () => {
    if (!selectedRecordingEntry) {
      flashToast("Select a recording first.");
      return;
    }

    const payload: ResetRecordingUltrasonicPositionRequest = {
      path: selectedRecordingEntry.path,
    };
    const next = await mutate(
      "reset-recording-ultrasonic-position",
      "/api/recordings/reset-ultrasonic-position",
      {
        method: "POST",
        body: JSON.stringify(payload),
      },
    );

    if (next) {
      flashToast("X/Y ultrasonic recapture replay started.");
    }
  };

  const saveSelectedRecordingReplayOptions = async (
    options: RecordingReplayOptions,
    successMessage?: string,
  ): Promise<DashboardState | null> => {
    if (!selectedRecordingEntry) {
      flashToast("Select a recording first.");
      return null;
    }

    const payload: SetRecordingReplayOptionsRequest = {
      path: selectedRecordingEntry.path,
      options,
    };

    try {
      const next = await request<DashboardState>("/api/recordings/replay-options", {
        method: "POST",
        body: JSON.stringify(payload),
      });
      setBackendError("");
      setState(next);
      if (!settingsDirty) {
        setSettingsDraft(next.settings);
      }
      if (successMessage) {
        flashToast(successMessage);
      }
      return next;
    } catch (error) {
      flashToast(error instanceof Error ? error.message : "Request failed.");
      return null;
    }
  };

  const handleRecordingHomeModeChange = async (homeMode: ArmHomeMode) => {
    await saveSelectedRecordingReplayOptions(
      {
        ...selectedRecordingReplayOptions,
        homeMode,
      },
      "Replay home option saved.",
    );
  };

  const handleRecordingSpeedChange = (speed: number) => {
    setPinSpeed(speed);
    if (!Number.isFinite(speed) || speed <= 0) {
      return;
    }
    void saveSelectedRecordingReplayOptions({
      ...selectedRecordingReplayOptions,
      speed,
    });
  };

  const handleRecordingAutoVexPositioningChange = async (autoVexPositioning: boolean) => {
    await saveSelectedRecordingReplayOptions(
      {
        ...selectedRecordingReplayOptions,
        autoVexPositioning,
      },
      "Auto VEX positioning option saved.",
    );
  };

  const handleCreatePin = async () => {
    if (!selectedRecording) {
      flashToast("Select a recording first.");
      return;
    }

    const next = await mutate("create-pin", "/api/pinned-moves", {
      method: "POST",
      body: JSON.stringify({
        name: pinName.trim() || selectedRecordingEntry?.name || "Pinned move",
        keyBinding: normalizeHotkeyText(pinHotkey),
        ...selectedMovePayload,
      }),
    });

    if (next) {
      flashToast("Pinned move saved.");
      setPinName("");
      setPinHotkey("");
    }
  };

  const disabled = pendingAction !== null;
  const piCalibrationBusy = state
    ? ["starting", "running", "stopping"].includes(state.services.piCalibration.state)
    : false;
  const activePiServoCalibrationId =
    state?.services.piCalibration.mode === "pi-servo-calibration" &&
    typeof state.services.piCalibration.meta.servoId === "string"
      ? state.services.piCalibration.meta.servoId
      : null;

  return (
    <div className="app-frame">
      <aside className="sidebar">
        <div className="sidebar-brand">
          <p className="eyebrow">Robot Arm Console</p>
          <h1>Robert</h1>
          <p className="sidebar-copy">
            Start control, record motion, replay saved moves, and watch the
            robot processes from one place.
          </p>
        </div>

        <figure className="sidebar-visual minion-visual">
          <img
            className="minion-avatar-image"
            src="/minion-avatar.png"
            alt="Cute yellow robot avatar"
          />
        </figure>

        <div className="sidebar-status">
          <span className={`status-pill ${state?.piReachable ? "good" : "bad"}`}>
            Pi: {state?.resolvedPiHost ?? "offline"}
          </span>
          <span className={`status-pill ${state?.leader.connected ? "good" : "bad"}`}>
            Leader: {state?.leader.connected ? "ready" : "missing"}
          </span>
          <span className={`status-pill ${state?.vexBrain.connected ? "good" : "bad"}`}>
            VEX: {vexStatusLabel(state?.vexBrain)}
          </span>
          <span className={`status-pill ${statusTone(state?.services.host.state ?? "idle")}`}>
            Host: {state?.services.host.state ?? "idle"}
          </span>
          <span className={`status-pill ${statusTone(state?.services.teleop.state ?? "idle")}`}>
            Teleop: {state?.services.teleop.state ?? "idle"}
          </span>
        </div>

        <nav className="sidebar-nav">
          {SECTIONS.map((section) => (
            <button
              key={section.key}
              className={activeSection === section.key ? "menu-button active" : "menu-button"}
              onClick={() => setActiveSection(section.key)}
            >
              {section.label}
            </button>
          ))}
        </nav>

        <div className="sidebar-footer">
          <strong>Leader check</strong>
          <span>{state?.leader.message ?? "Checking leader status..."}</span>
          <strong>VEX check</strong>
          <span>{state?.vexBrain.message ?? "Checking VEX Brain status..."}</span>
        </div>
      </aside>

      <div className="content-stage">
        <header className="stage-header">
          <div>
            <p className="card-kicker">Dashboard</p>
            <h2>{activeSectionMeta.label}</h2>
          </div>
          <p className="card-note">
            {activeSection === "overview" &&
              "Start or stop control, watch live output, and confirm the robot and leader are both ready."}
            {activeSection === "recordings" &&
              "Record follower motion on the Pi, replay saved trajectories, and pin them for reuse."}
            {activeSection === "pins" &&
              "Launch saved movements instantly from the dashboard or with keyboard shortcuts."}
            {activeSection === "training" &&
              "Create task profiles, capture either on the Pi or directly from the leader arm on the Mac, train ACT locally, then deploy and benchmark policies on the Pi."}
            {activeSection === "settings" &&
              "Adjust connection defaults. The expected hotspot stays visible here as a reference only."}
            {activeSection === "logs" &&
              "Inspect raw stdout and stderr from the Pi host, teleop relay, replay runner, and calibration commands."}
          </p>
        </header>

        {toast ? <div className="toast">{toast}</div> : null}
        {backendError ? (
          <div className="alert-banner">
            <strong>Backend state unavailable:</strong> {backendError}
          </div>
        ) : null}
        {state?.lastError ? (
          <div className="alert-banner">
            <strong>Last backend error:</strong> {state.lastError}
          </div>
        ) : null}

        <div className="stage-body">
          {activeSection === "overview" && (
            <section className="card stage-panel">
              <div className="card-head">
                <div>
                  <p className="card-kicker">Quick Actions</p>
                  <h2>Connect and Control</h2>
                </div>
                <p className="card-note">
                  The backend retries the Pi five times, auto-detects a single attached leader-arm port when the scripted path is stale, and streams the action output below.
                </p>
              </div>

              <div className="button-cluster">
                <button
                  className="primary"
                  disabled={disabled}
                  onClick={() => void mutate("start-control", "/api/robot/start-control", { method: "POST" })}
                >
                  Start Control
                </button>
                <button
                  disabled={disabled}
                  onClick={() => void mutate("stop-control", "/api/robot/stop-control", { method: "POST" })}
                >
                  Stop Control
                </button>
                <button
                  disabled={disabled || keyboardBackupActive}
                  onClick={() =>
                    void mutate("start-keyboard-control", "/api/robot/start-keyboard-control", {
                      method: "POST",
                    })
                  }
                >
                  Start Keyboard Backup
                </button>
                <button
                  disabled={disabled || hotkeysArmed}
                  onClick={() => void mutate("start-hotkeys", "/api/robot/start-hotkeys", { method: "POST" })}
                >
                  Arm Hotkeys
                </button>
                <button
                  disabled={disabled || !homeCommandReady}
                  onClick={() => void handleSetArmHome()}
                >
                  Set Arm Home
                </button>
                <button
                  disabled={disabled || !homeCommandReady || !hasArmHomePosition}
                  onClick={() => void handleGoArmHome()}
                >
                  Go Home
                </button>
                <button
                  className="danger"
                  disabled={disabled}
                  onClick={() => void mutate("emergency-stop", "/api/robot/emergency-stop", { method: "POST" })}
                >
                  Emergency Stop + Torque Off
                </button>
              </div>
              <div className="mode-strip">
                <span className={`status-pill ${keyboardBackupActive ? "good" : "muted"}`}>
                  {keyboardBackupActive ? "keyboard backup live" : "keyboard backup off"}
                </span>
                <p className="card-note">
                  Keyboard backup runs without the leader arm. Arm: <code>Q/A</code> shoulder pan, <code>W/S</code> shoulder lift, <code>E/D</code> elbow, <code>R/F</code> wrist flex, <code>T/G</code> wrist roll, <code>Z/X</code> gripper. Base: <code>I/K</code> forward/back, <code>J/L</code> strafe, <code>U/O</code> turn. Press <code>Esc</code> in the keyboard teleop process or use Stop Control here.
                </p>
              </div>
              <div className="mode-strip">
                <span className={`status-pill ${hotkeysArmed ? "good" : "muted"}`}>
                  {hotkeysArmed ? "hotkeys armed" : "hotkeys disarmed"}
                </span>
                <p className="card-note">
                  Arm hotkeys keeps the Pi host live without teleop so pinned Pi replays can start immediately.
                </p>
              </div>
              <div className="mode-strip">
                <span className={`status-pill ${hasArmHomePosition ? "good" : "muted"}`}>
                  {hasArmHomePosition ? "home saved" : "home not set"}
                </span>
                <p className="card-note">
                  Set Arm Home captures the current follower pose from the running Pi host. Go Home uses that saved pose without needing the VEX Brain.
                </p>
              </div>

              <div className="calibration-panel">
                <div>
                  <p className="card-kicker">Calibration</p>
                  <h3>Calibrate Arm and Leader</h3>
                  <p className="card-note">
                    Use the live console prompts. Send <code>c</code> to force a fresh full-arm follower calibration, then Enter at each step. Individual follower servos can also be started from the cards below.
                  </p>
                </div>
                <div className="calibration-grid">
                  <article>
                    <div>
                      <strong>Pi follower</strong>
                      <span className={`status-pill ${statusTone(state?.services.piCalibration.state ?? "idle")}`}>
                        {state?.services.piCalibration.state ?? "idle"}
                      </span>
                    </div>
                    <div className="button-cluster inline">
                      <button
                        className="primary"
                        disabled={disabled}
                        onClick={() => void mutate("pi-calibration", "/api/calibration/pi/start", { method: "POST" })}
                      >
                        Start Pi Calibration
                      </button>
                      <button
                        disabled={disabled}
                        onClick={() =>
                          void mutate("pi-calibration-c", "/api/calibration/pi/input", {
                            method: "POST",
                            body: JSON.stringify({ input: "c" }),
                          })
                        }
                      >
                        Send c
                      </button>
                      <button
                        disabled={disabled}
                        onClick={() =>
                          void mutate("pi-calibration-enter", "/api/calibration/pi/input", {
                            method: "POST",
                            body: JSON.stringify({ input: "enter" }),
                          })
                        }
                      >
                        Enter
                      </button>
                      <button
                        disabled={disabled}
                        onClick={() => void mutate("pi-calibration-stop", "/api/calibration/pi/stop", { method: "POST" })}
                      >
                        Stop
                      </button>
                    </div>
                  </article>
                  <article>
                    <div>
                      <strong>Mac leader</strong>
                      <span className={`status-pill ${statusTone(state?.services.macCalibration.state ?? "idle")}`}>
                        {state?.services.macCalibration.state ?? "idle"}
                      </span>
                    </div>
                    <div className="button-cluster inline">
                      <button
                        className="primary"
                        disabled={disabled}
                        onClick={() => void mutate("mac-calibration", "/api/calibration/mac/start", { method: "POST" })}
                      >
                        Start Mac Calibration
                      </button>
                      <button
                        disabled={disabled}
                        onClick={() =>
                          void mutate("mac-calibration-c", "/api/calibration/mac/input", {
                            method: "POST",
                            body: JSON.stringify({ input: "c" }),
                          })
                        }
                      >
                        Send c
                      </button>
                      <button
                        disabled={disabled}
                        onClick={() =>
                          void mutate("mac-calibration-enter", "/api/calibration/mac/input", {
                            method: "POST",
                            body: JSON.stringify({ input: "enter" }),
                          })
                        }
                      >
                        Enter
                      </button>
                      <button
                        disabled={disabled}
                        onClick={() => void mutate("mac-calibration-stop", "/api/calibration/mac/stop", { method: "POST" })}
                      >
                        Stop
                      </button>
                    </div>
                  </article>
                </div>
              </div>

              <PowerTelemetryPanel
                telemetry={powerTelemetry}
                torqueLimits={torqueLimits}
                calibrationBusy={piCalibrationBusy}
                calibratingServoId={activePiServoCalibrationId}
                controlsDisabled={disabled}
                onApplyRecommendedTorqueLimits={handleApplyRecommendedTorqueLimits}
                onServoCalibrationStart={handleServoCalibrationStart}
                onTorqueLimitChange={handleTorqueLimitChange}
              />

              <div className="inline-console">
                <div className="inline-console-head">
                  <h3>Live Console</h3>
                  <div className="inline-console-actions">
                    <button
                      className="console-copy-button"
                      disabled={!liveConsoleLines.length}
                      onClick={() => void handleCopyLiveConsole()}
                    >
                      Copy Live Console
                    </button>
                    <span className={`status-pill ${pendingAction ? "warn" : "muted"}`}>
                      {pendingAction ? `running: ${pendingAction}` : "idle"}
                    </span>
                  </div>
                </div>
                <pre>
                  {liveConsoleLines.length
                    ? liveConsoleLines.join("\n")
                    : "No action output yet."}
                </pre>
              </div>

              <div className="status-grid">
                {state ? (
                  <>
                    {serviceSnapshots.map((service) => (
                      <article key={service.label} className="status-card">
                        <div className="status-card-head">
                          <h3>{service.label}</h3>
                          <span className={`status-pill ${statusTone(service.state)}`}>
                            {service.state}
                          </span>
                        </div>
                        <p>{service.detail}</p>
                        <dl>
                          <div>
                            <dt>Mode</dt>
                            <dd>{service.mode ?? "none"}</dd>
                          </div>
                          <div>
                            <dt>Started</dt>
                            <dd>{prettyTimestamp(service.startedAt)}</dd>
                          </div>
                          <div>
                            <dt>PID / Exit</dt>
                            <dd>
                              {service.pid ?? "n/a"} / {service.exitCode ?? "n/a"}
                            </dd>
                          </div>
                        </dl>
                      </article>
                    ))}
                    <article className="status-card">
                      <div className="status-card-head">
                        <h3>Leader Arm</h3>
                        <span className={`status-pill ${state.leader.connected ? "good" : "bad"}`}>
                          {state.leader.connected ? "ready" : "missing"}
                        </span>
                      </div>
                      <p>{state.leader.message}</p>
                      <dl>
                        <div>
                          <dt>Expected Port</dt>
                          <dd>{state.leader.expectedPort ?? "not found in teleoperate.py"}</dd>
                        </div>
                        <div>
                          <dt>Available Ports</dt>
                          <dd>
                            {state.leader.availablePorts.length
                              ? state.leader.availablePorts.join(", ")
                              : "none"}
                          </dd>
                        </div>
                        <div>
                          <dt>Wi-Fi</dt>
                          <dd>{state.wifi.ssid ?? "offline"}</dd>
                        </div>
                      </dl>
                    </article>
                    <article className="status-card">
                      <div className="status-card-head">
                        <h3>Gyro</h3>
                        <span className={`status-pill ${robotSensorTone(state.robotSensors.gyro)}`}>
                          {state.robotSensors.gyro.state}
                        </span>
                      </div>
                      <p>{state.robotSensors.gyro.message}</p>
                      <dl>
                        <div>
                          <dt>Axis</dt>
                          <dd>{state.robotSensors.gyro.axis ?? "rotation"}</dd>
                        </div>
                        <div>
                          <dt>Value</dt>
                          <dd>{formatRobotSensorValue(state.robotSensors.gyro)}</dd>
                        </div>
                        <div>
                          <dt>Updated</dt>
                          <dd>{prettyTimestamp(state.robotSensors.gyro.updatedAt)}</dd>
                        </div>
                        <div>
                          <dt>Source</dt>
                          <dd>{formatRobotSensorSource(state.robotSensors.gyro.source)}</dd>
                        </div>
                      </dl>
                    </article>
                    <article className="status-card">
                      <div className="status-card-head">
                        <h3>Sensor 1 (X)</h3>
                        <span className={`status-pill ${robotSensorTone(state.robotSensors.x)}`}>
                          {state.robotSensors.x.state}
                        </span>
                      </div>
                      <p>{state.robotSensors.x.message}</p>
                      <dl>
                        <div>
                          <dt>Axis</dt>
                          <dd>{state.robotSensors.x.axis?.toUpperCase() ?? "X"}</dd>
                        </div>
                        <div>
                          <dt>Distance</dt>
                          <dd>{formatRobotSensorValue(state.robotSensors.x)}</dd>
                        </div>
                        <div>
                          <dt>Updated</dt>
                          <dd>{prettyTimestamp(state.robotSensors.x.updatedAt)}</dd>
                        </div>
                        <div>
                          <dt>Source</dt>
                          <dd>{formatRobotSensorSource(state.robotSensors.x.source)}</dd>
                        </div>
                      </dl>
                    </article>
                    <article className="status-card">
                      <div className="status-card-head">
                        <h3>Sensor 2 (Y)</h3>
                        <span className={`status-pill ${robotSensorTone(state.robotSensors.y)}`}>
                          {state.robotSensors.y.state}
                        </span>
                      </div>
                      <p>{state.robotSensors.y.message}</p>
                      <dl>
                        <div>
                          <dt>Axis</dt>
                          <dd>{state.robotSensors.y.axis?.toUpperCase() ?? "Y"}</dd>
                        </div>
                        <div>
                          <dt>Distance</dt>
                          <dd>{formatRobotSensorValue(state.robotSensors.y)}</dd>
                        </div>
                        <div>
                          <dt>Updated</dt>
                          <dd>{prettyTimestamp(state.robotSensors.y.updatedAt)}</dd>
                        </div>
                        <div>
                          <dt>Source</dt>
                          <dd>{formatRobotSensorSource(state.robotSensors.y.source)}</dd>
                        </div>
                      </dl>
                    </article>
                  </>
                ) : null}
              </div>
            </section>
          )}

          {activeSection === "recordings" && (
            <section className="card stage-panel">
              <div className="card-head">
                <div>
                  <p className="card-kicker">Record & Replay</p>
                  <h2>Movement Library</h2>
                </div>
                <p className="card-note">
                  Record exact follower motion on the Pi, then replay or pin it.
                </p>
              </div>

              <div className="mode-strip compact">
                <span className={`status-pill ${warmReplayReady ? "good" : "muted"}`}>
                  {warmReplayReady ? "No-drop Pi replay ready" : "No-drop Pi replay inactive"}
                </span>
                <p className="card-note">
                  {warmReplayFromControl
                    ? "Replay Selected stops the leader relay and runs the saved motion on the already-connected Pi host."
                    : warmReplayReady
                      ? "Pinned Pi replays will use the warm host instead of launching a fresh remote replay process."
                      : "Start Control or Arm Hotkeys first if you want Pi replays to keep the robot host connected."}
                </p>
              </div>

              <div className="record-toolbar">
                <label>
                  Recording label
                  <input
                    value={recordLabel}
                    onChange={(event) => setRecordLabel(event.target.value)}
                    placeholder="Pick up cube"
                  />
                </label>
                <label>
                  Recording mode
                  <select
                    value={recordInputMode}
                    onChange={(event) => setRecordInputMode(event.target.value as RecordingInputMode)}
                    disabled={disabled}
                  >
                    <option value="auto">Auto</option>
                    <option value="leader">Leader arm</option>
                    <option value="keyboard">Keyboard</option>
                    <option value="free-teach">Hand-guide</option>
                  </select>
                </label>
                <div className="button-cluster inline">
                  <button
                    className="primary"
                    disabled={disabled}
                    onClick={() =>
                      void mutate("start-recording", "/api/recordings/start", {
                        method: "POST",
                        body: JSON.stringify({ label: recordLabel, inputMode: recordInputMode }),
                      })
                    }
                  >
                    {recordInputMode === "free-teach" ? "Start Hand-Guide Recording" : "Start Recording"}
                  </button>
                  <button
                    disabled={disabled}
                    onClick={() => void mutate("stop-recording", "/api/recordings/stop", { method: "POST" })}
                  >
                    Stop Recording
                  </button>
                  <button
                    disabled={disabled || !state || vexGyroZeroBusy}
                    onClick={() => void handleZeroVexGyro()}
                  >
                    Zero VEX Gyro
                  </button>
                  <button
                    disabled={disabled}
                    onClick={() =>
                      void mutate("refresh-recordings", "/api/recordings/refresh", { method: "POST" })
                    }
                  >
                    Refresh List
                  </button>
                </div>
              </div>

              {isRecordingActive && recordingService ? (
                <div className="recording-live-strip" role="status" aria-live="polite">
                  <div className="recording-live-status">
                    <span className="recording-live-dot" />
                    <span className="recording-live-label">
                      {recordingHasCapturedMotion ? "REC" : "ARMED"}
                    </span>
                    {recordingHasCapturedMotion ? (
                      <strong className="recording-live-timer">
                        {recordingStartedMs
                          ? formatClockDuration((recordingClockMs - recordingStartedMs) / 1000)
                          : formatElapsedSince(recordingService.startedAt, recordingClockMs)}
                      </strong>
                    ) : (
                      <strong className="recording-live-waiting">
                        {getRecordingWaitingLabel(recordingService)}
                      </strong>
                    )}
                  </div>
                  <span className="recording-live-meta">
                    {recordingHasCapturedMotion
                      ? `Started ${prettyTimestamp(recordingService.startedAt)}`
                      : getRecordingArmedLabel(recordingService)}
                  </span>
                </div>
              ) : null}

              <div className="recordings-layout">
                <div className="recordings-list">
                  {state?.recordings.length ? (
                    state.recordings.map((recording) => (
                      <button
                        key={recording.path}
                        className={`recording-row ${recording.path === selectedRecording ? "active" : ""}`}
                        onClick={() => setSelectedRecording(recording.path)}
                      >
                        <span className="recording-name">{recording.name}</span>
                        <span className="recording-meta">{recording.fileName}</span>
                        <span>
                          {formatBytes(recording.size)}
                          {recording.durationS !== null ? ` • ${formatDurationSeconds(recording.durationS)}` : ""}
                        </span>
                        <span>{prettyTimestamp(recording.modifiedAt)}</span>
                      </button>
                    ))
                  ) : (
                    <div className="empty-state">
                      No recordings cached yet. Refresh after the Pi is reachable.
                    </div>
                  )}
                </div>

                <div className="replay-panel">
                  <h3>Replay Selected Move</h3>
                  <p className="card-note">
                    {selectedRecordingEntry
                      ? `${selectedRecordingEntry.name} • ${formatBytes(selectedRecordingEntry.size)}${
                          selectedRecordingEntry.durationS !== null
                            ? ` • ${formatDurationSeconds(selectedRecordingEntry.durationS)}`
                            : ""
                        }`
                      : "Select a recording to replay or pin."}
                  </p>
                  <div className="selected-recording-editor">
                    <label>
                      Recording name
                      <input
                        value={recordingNameDraft}
                        disabled={!selectedRecordingEntry}
                        onChange={(event) => setRecordingNameDraft(event.target.value)}
                        placeholder="Desk pickup"
                      />
                    </label>
                    <div className="selected-recording-actions">
                      <button
                        disabled={
                          disabled ||
                          !selectedRecordingEntry ||
                          !recordingNameDraft.trim() ||
                          recordingNameDraft.trim() === selectedRecordingEntry.name
                        }
                        onClick={() => void handleRenameRecording()}
                      >
                        Save Name
                      </button>
                      <button
                        disabled={disabled || !selectedRecordingEntry}
                        title="Copy this recording so it can get its own X/Y ultrasonic recapture."
                        onClick={() => void handleDuplicateRecording()}
                      >
                        Duplicate
                      </button>
                      <button
                        disabled={disabled || !selectedRecordingEntry}
                        onClick={() => void handleMarkRecordingGyroZero()}
                      >
                        Mark Gyro Zero
                      </button>
                      <button
                        disabled={disabled || !selectedRecordingEntry}
                        title="Save the first recorded follower pose as the arm home position."
                        onClick={() => void handleSetArmHomeFromRecordingStart()}
                      >
                        Save Start as Home
                      </button>
                      <button
                        disabled={disabled || !homeCommandReady || !hasArmHomePosition}
                        title="Move the follower arm to the saved home position without moving the gripper."
                        onClick={() => void handleGoArmHome()}
                      >
                        Go Home Position
                      </button>
                      <button
                        disabled={disabled || !selectedRecordingEntry || !recordingUltrasonicResetReady}
                        title={
                          recordingUltrasonicResetReady
                            ? "Replay the whole recording and overwrite its X/Y ultrasonic stream from live sensor readings."
                            : "Select a recording to replay and recapture its X/Y ultrasonic stream."
                        }
                        onClick={() => void handleResetRecordingUltrasonicPosition()}
                      >
                        Replay + Recapture X/Y
                      </button>
                      <button
                        className="danger"
                        disabled={disabled || !selectedRecordingEntry}
                        onClick={() => void handleDeleteRecording()}
                      >
                        Delete
                      </button>
                    </div>
                  </div>
                  <RecordingPreviewPanel
                    detail={recordingDetail}
                    loading={recordingDetailLoading}
                    error={recordingDetailError}
                    playheadS={playbackTimeS}
                    playing={playbackPlaying}
                    trimStartS={trimStartValueS}
                    trimEndS={trimEndValueS}
                    onScrub={seekPlayback}
                    onTogglePlayback={handleTogglePlayback}
                    onReset={() => seekPlayback(0)}
                    onSetTrimStart={handleSetTrimStartFromPlayhead}
                    onSetTrimEnd={handleSetTrimEndFromPlayhead}
                  />
                  <div className="recording-trim-editor">
                    <div className="form-grid compact">
                      <label>
                        Trim Start (s)
                        <input
                          type="number"
                          min="0"
                          step="0.1"
                          value={trimStartDraft}
                          disabled={!selectedRecordingEntry}
                          onChange={(event) => setTrimStartDraft(event.target.value)}
                        />
                      </label>
                      <label>
                        Trim End (s)
                        <input
                          type="number"
                          min="0"
                          step="0.1"
                          value={trimEndDraft}
                          disabled={!selectedRecordingEntry}
                          onChange={(event) => setTrimEndDraft(event.target.value)}
                        />
                      </label>
                    </div>
                    <div className="trim-editor-actions">
                      <p className="card-note">
                        Scrub the preview, set trim start and end from the playhead, then keep only that action window.
                      </p>
                      <button
                        disabled={
                          disabled ||
                          !selectedRecordingEntry ||
                          !trimEndDraft.trim() ||
                          !Number.isFinite(Number(trimStartDraft)) ||
                          !Number.isFinite(Number(trimEndDraft)) ||
                          Number(trimStartDraft) < 0 ||
                          Number(trimEndDraft) <= Number(trimStartDraft)
                        }
                        onClick={() => void handleTrimRecording()}
                      >
                        Trim Selected
                      </button>
                    </div>
                  </div>
                  <div className="form-grid compact">
                    <label>
                      Replay target
                      <select
                        value={replayTarget}
                        onChange={(event) =>
                          setReplayTarget(event.target.value === "leader" ? "leader" : "pi")
                        }
                      >
                        <option value="pi">Pi follower</option>
                        <option value="leader">Leader arm</option>
                      </select>
                    </label>
                    <label>
                      Speed
                      <input
                        type="number"
                        min="0.1"
                        step="0.1"
                        value={pinSpeed}
                        disabled={!selectedRecordingEntry}
                        onChange={(event) => handleRecordingSpeedChange(Number(event.target.value))}
                      />
                    </label>
                    <label>
                      Hold Final (s)
                      <input
                        type="number"
                        min="0"
                        step="0.1"
                        value={pinHoldFinal}
                        onChange={(event) => setPinHoldFinal(Number(event.target.value))}
                      />
                    </label>
                    <label>
                      Go home
                      <select
                        value={selectedRecordingHomeMode}
                        disabled={!selectedRecordingEntry || replayTarget !== "pi" || !hasArmHomePosition}
                        onChange={(event) =>
                          void handleRecordingHomeModeChange(event.target.value as ArmHomeMode)
                        }
                      >
                        {HOME_MODE_OPTIONS.map((option) => (
                          <option key={option.value} value={option.value}>
                            {option.label}
                          </option>
                        ))}
                      </select>
                    </label>
                    <label className="checkbox-row">
                      <input
                        type="checkbox"
                        checked={selectedRecordingReplayOptions.autoVexPositioning}
                        disabled={!selectedRecordingEntry || replayTarget !== "pi"}
                        onChange={(event) =>
                          void handleRecordingAutoVexPositioningChange(event.target.checked)
                        }
                      />
                      <span>Auto VEX positioning</span>
                    </label>
                    <label className="checkbox-row">
                      <input
                        type="checkbox"
                        checked={effectiveReplayIncludeBase}
                        disabled={replayTarget !== "pi"}
                        onChange={(event) => setReplayIncludeBasePreference(event.target.checked)}
                      />
                      <span>
                        Replay VEX base {replayTarget === "pi" ? "with this move" : "(Pi follower only)"}
                      </span>
                    </label>
                    <label>
                      VEX replay mode
                      <select
                        value={vexReplayMode}
                        disabled={replayTarget !== "pi" || !effectiveReplayIncludeBase}
                        onChange={(event) =>
                          setVexReplayMode(event.target.value === "ecu" ? "ecu" : "drive")
                        }
                      >
                        {VEX_REPLAY_MODE_OPTIONS.map((option) => (
                          <option key={option.value} value={option.value}>
                            {option.label}
                          </option>
                        ))}
                      </select>
                    </label>
                    {replayTarget === "pi" &&
                    replayIncludeBasePreference === null &&
                    selectedRecordingHasBaseReference &&
                    selectedRecordingReplayOptions.autoVexPositioning ? (
                      <p className="card-note">
                        This recording contains VEX base motion. Leave base replay off unless you intentionally want to replay base movement; Auto VEX positioning can still use the gyro and ultrasonic sensors.
                      </p>
                    ) : null}
                    {replayTarget === "pi" && !selectedRecordingReplayOptions.autoVexPositioning ? (
                      <p className="card-note">
                        Auto VEX positioning is off for this recording.
                      </p>
                    ) : null}
                    {replayTarget === "pi" && !hasArmHomePosition ? (
                      <p className="card-note">
                        Save the selected recording start as Home, or set Home from Overview, before enabling automatic home movement for recordings.
                      </p>
                    ) : null}
                    {replayTarget === "pi" && hasArmHomePosition && selectedRecordingHomeMode !== "none" ? (
                      <p className="card-note">
                        This recording will {HOME_MODE_OPTIONS.find((option) => option.value === selectedRecordingHomeMode)?.label.toLowerCase()}.
                      </p>
                    ) : null}
                    {replayTarget === "pi" && effectiveReplayIncludeBase ? (
                      <p className="card-note">
                        {vexReplayMode === "ecu"
                          ? "ECU mode uploads recorded VEX wheel targets to the Brain and reapplies HOLD on idle VEX motors."
                          : "Drive mode uploads recorded VEX base velocities to the Brain."}
                      </p>
                    ) : null}
                  </div>
                  {latestVexPositionStatus ? (
                    <div className="mode-strip compact">
                      <span
                        className={`status-pill ${
                          latestVexPositionStatus.status === "aligned"
                            ? "good"
                            : latestVexPositionStatus.status === "skipped"
                              ? "warn"
                              : "muted"
                        }`}
                      >
                        VEX positioning {latestVexPositionStatus.status}
                      </span>
                      <p className="card-note">
                        {latestVexPositionStatus.message}
                        {latestVexPositionStatus.timeoutS !== null
                          ? ` Limit ${latestVexPositionStatus.timeoutS.toFixed(1)}s.`
                          : ""}
                      </p>
                    </div>
                  ) : null}
                  <div className="button-cluster inline">
                    <button
                      className="primary"
                      disabled={disabled || !selectedRecording}
                      onClick={() =>
                        void mutate("start-replay", "/api/replays/start", {
                          method: "POST",
                          body: JSON.stringify(selectedMovePayload),
                        })
                      }
                    >
                      {replayTarget === "leader" ? "Replay on Leader" : "Replay Selected"}
                    </button>
                    <button
                      disabled={disabled}
                      onClick={() => void mutate("stop-replay", "/api/replays/stop", { method: "POST" })}
                    >
                      Stop Replay
                    </button>
                  </div>

                  <div className="pin-builder">
                    <h3>Pin This Move</h3>
                    <div className="form-grid compact">
                      <label>
                        Button label
                        <input
                          value={pinName}
                          onChange={(event) => setPinName(event.target.value)}
                          placeholder="Desk pickup"
                        />
                      </label>
                      <label>
                        Hotkey
                        <input
                          value={pinHotkey}
                          onChange={(event) => setPinHotkey(event.target.value)}
                          onBlur={() => setPinHotkey((current) => normalizeHotkeyText(current))}
                          placeholder="SHIFT+1"
                        />
                      </label>
                    </div>
                    <button
                      className="primary"
                      disabled={disabled || !selectedRecording}
                      onClick={() => void handleCreatePin()}
                    >
                      Pin Move to Dashboard
                    </button>
                  </div>
                </div>
              </div>
            </section>
          )}

          {activeSection === "pins" && (
            <section className="card stage-panel">
              <div className="card-head">
                <div>
                  <p className="card-kicker">Pinned Moves</p>
                  <h2>Keyboard Launch Pad</h2>
                </div>
                <div className="pin-head-actions">
                  <span className={`status-pill ${hotkeysArmed ? "good" : "muted"}`}>
                    {hotkeysArmed ? "instant Pi hotkeys ready" : "Pi hotkeys not armed"}
                  </span>
                  <div className="button-cluster inline">
                    <button
                      className="primary"
                      disabled={disabled || hotkeysArmed}
                      onClick={() => void mutate("start-hotkeys", "/api/robot/start-hotkeys", { method: "POST" })}
                    >
                      Arm Hotkeys
                    </button>
                    <button
                      disabled={disabled || !hotkeysArmed}
                      onClick={() => void mutate("stop-control", "/api/robot/stop-control", { method: "POST" })}
                    >
                      Disarm
                    </button>
                  </div>
                </div>
              </div>
              <p className="card-note">
                Hotkeys fire when the page is focused and you are not typing in a field. Arm hotkeys to keep the Pi ready for fast pinned-move launches.
              </p>
              <div className="pin-grid">
                {state?.pinnedMoves.length ? (
                  state.pinnedMoves.map((move) => (
                    <article key={move.id} className="pin-card">
                      <div className="pin-card-head">
                        <div>
                          <h3>{move.name}</h3>
                          <p>{move.trajectoryPath}</p>
                        </div>
                        <span className="hotkey-chip">{move.keyBinding || "no hotkey"}</span>
                      </div>
                      <p className="pin-meta">
                        {replayTargetLabel(move.target)} • Speed {move.speed} • Hold {move.holdFinalS}s • {homeModeLabel(move.homeMode)} •{" "}
                        {move.includeBase
                          ? `VEX base + arm (${vexReplayModeLabel(move.vexReplayMode)})`
                          : "Arm only"}
                        {move.target === "pi" && !move.autoVexPositioning
                          ? " • VEX positioning off"
                          : ""}
                      </p>
                      <div className="button-cluster inline">
                        <button
                          className="primary"
                          disabled={disabled}
                          onClick={() => void handleTriggerPinnedMove(move)}
                        >
                          Run
                        </button>
                        <button
                          disabled={disabled}
                          onClick={() =>
                            void mutate(`delete-${move.id}`, `/api/pinned-moves/${move.id}`, {
                              method: "DELETE",
                            })
                          }
                        >
                          Remove
                        </button>
                      </div>
                    </article>
                  ))
                ) : (
                  <div className="empty-state">
                    Pin your frequent motions here so you can replay them with one click or a hotkey.
                  </div>
                )}
              </div>
            </section>
          )}

          {activeSection === "training" && (
            <section className="card stage-panel">
              <div className="card-head">
                <div>
                  <p className="card-kicker">Training</p>
                  <h2>Pi and Leader Training Workflow</h2>
                </div>
                <p className="card-note">
                  Save profile edits before capture, sync, training, deploy, or evaluation.
                </p>
              </div>

              <div className="recordings-layout">
                <div className="recordings-list">
                  {state?.training.profiles.length ? (
                    state.training.profiles.map((profile) => (
                      <button
                        key={profile.id}
                        className={`recording-row ${profile.id === selectedTrainingProfile?.id ? "active" : ""}`}
                        onClick={() => void handleSelectTrainingProfile(profile.id)}
                      >
                        <span className="recording-name">{profile.name}</span>
                        <span className="recording-meta">{captureModeLabel(profile.captureMode)}</span>
                        <span>
                          {profile.artifacts.datasetEpisodeCount ?? 0} episode
                          {(profile.artifacts.datasetEpisodeCount ?? 0) === 1 ? "" : "s"}
                        </span>
                        <span>{profile.artifacts.benchmark?.passed ? "benchmark pass" : "benchmark pending"}</span>
                      </button>
                    ))
                  ) : (
                    <div className="empty-state">
                      No training profiles yet. Create one to start dataset capture on the Pi or on the leader arm.
                    </div>
                  )}
                </div>

                <div className="replay-panel">
                  <div className="card-head">
                    <div>
                      <p className="card-kicker">Profiles</p>
                      <h3>{trainingDraft?.name || "Training profile"}</h3>
                    </div>
                    <div className="button-cluster inline">
                      <button onClick={handleCreateTrainingProfile} disabled={disabled}>
                        New Profile
                      </button>
                      <button
                        className="primary"
                        disabled={disabled || !trainingDraft}
                        onClick={() => void handleSaveTrainingProfile()}
                      >
                        Save Profile
                      </button>
                      <button
                        disabled={disabled || !selectedTrainingProfile}
                        onClick={() => void handleDeleteTrainingProfile()}
                      >
                        Delete
                      </button>
                    </div>
                  </div>

                  {trainingDraft ? (
                    <>
                      <div className="form-grid">
                        <label>
                          Name
                          <input
                            value={trainingDraft.name}
                            onChange={(event) => updateTrainingField("name", event.target.value)}
                          />
                        </label>
                        <label>
                          Capture mode
                          <select
                            value={trainingDraft.captureMode}
                            onChange={(event) =>
                              updateTrainingField(
                                "captureMode",
                                event.target.value === "free-teach"
                                  ? "free-teach"
                                  : event.target.value === "leader-as-follower"
                                    ? "leader-as-follower"
                                    : "leader",
                              )
                            }
                          >
                            <option value="leader">Leader</option>
                            <option value="free-teach">Free-teach</option>
                            <option value="leader-as-follower">Leader as follower (Mac)</option>
                          </select>
                        </label>
                        <label>
                          Episode count
                          <input
                            type="number"
                            min="1"
                            value={trainingDraft.numEpisodes}
                            onChange={(event) => updateTrainingField("numEpisodes", Number(event.target.value))}
                          />
                        </label>
                        <label>
                          Episode time (s)
                          <input
                            type="number"
                            min="1"
                            value={trainingDraft.episodeTimeS}
                            onChange={(event) => updateTrainingField("episodeTimeS", Number(event.target.value))}
                          />
                        </label>
                        <label>
                          Reset time (s)
                          <input
                            type="number"
                            min="0"
                            value={trainingDraft.resetTimeS}
                            onChange={(event) => updateTrainingField("resetTimeS", Number(event.target.value))}
                          />
                        </label>
                        <label>
                          Capture FPS
                          <input
                            type="number"
                            min="1"
                            value={trainingDraft.fps}
                            onChange={(event) => updateTrainingField("fps", Number(event.target.value))}
                          />
                        </label>
                        <label>
                          Cameras
                          <input
                            value={trainingDraft.camerasMode}
                            onChange={(event) => updateTrainingField("camerasMode", event.target.value)}
                          />
                        </label>
                        <label>
                          Pi dataset path
                          <input
                            value={trainingDraft.piDatasetPath}
                            onChange={(event) => updateTrainingField("piDatasetPath", event.target.value)}
                          />
                        </label>
                        <label>
                          Mac dataset path
                          <input
                            value={trainingDraft.macDatasetPath}
                            onChange={(event) => updateTrainingField("macDatasetPath", event.target.value)}
                          />
                        </label>
                        <label>
                          Mac train output
                          <input
                            value={trainingDraft.macTrainOutputDir}
                            onChange={(event) => updateTrainingField("macTrainOutputDir", event.target.value)}
                          />
                        </label>
                        <label>
                          Selected checkpoint
                          <select
                            value={trainingDraft.selectedCheckpointPath}
                            onChange={(event) =>
                              updateTrainingField("selectedCheckpointPath", event.target.value)
                            }
                          >
                            <option value="">Latest discovered checkpoint</option>
                            {(selectedTrainingProfile?.artifacts.availableCheckpointPaths ?? []).map((checkpoint) => (
                              <option key={checkpoint} value={checkpoint}>
                                {checkpoint}
                              </option>
                            ))}
                          </select>
                        </label>
                        <label>
                          Pi deploy path
                          <input
                            value={trainingDraft.piDeployPath}
                            onChange={(event) => updateTrainingField("piDeployPath", event.target.value)}
                          />
                        </label>
                        <label>
                          Pi eval root
                          <input
                            value={trainingDraft.piEvalDatasetPath}
                            onChange={(event) => updateTrainingField("piEvalDatasetPath", event.target.value)}
                          />
                        </label>
                      </div>

                      <label>
                        Task instruction
                        <textarea
                          value={trainingDraft.task}
                          onChange={(event) => updateTrainingField("task", event.target.value)}
                          rows={3}
                        />
                      </label>
                    </>
                  ) : null}
                </div>
              </div>

              <div className="status-grid">
                <article className="status-card">
                  <div className="status-card-head">
                    <h3>Capture Dataset</h3>
                    <span className={`status-pill ${statusTone(state?.services.datasetCapture.state ?? "idle")}`}>
                      {state?.services.datasetCapture.state ?? "idle"}
                    </span>
                  </div>
                  <p>
                    {describeCaptureMode(selectedTrainingProfile)}
                  </p>
                  <p className="pin-meta">
                    Episodes {selectedTrainingProfile?.artifacts.datasetEpisodeCount ?? 0}
                    {" • "}
                    Last episode {selectedTrainingProfile?.artifacts.lastDatasetEpisodeIndex ?? "n/a"}
                  </p>
                  <div className="button-cluster inline">
                    <button
                      className="primary"
                      disabled={disabled || trainingDirty || !selectedTrainingProfile}
                      onClick={() =>
                        void mutateTrainingProfileAction(
                          "training-capture-start",
                          "/api/training/capture/start",
                          "Dataset capture started.",
                        )
                      }
                    >
                      Start Capture
                    </button>
                    <button
                      disabled={disabled}
                      onClick={() =>
                        void mutate("training-capture-stop", "/api/training/capture/stop", {
                          method: "POST",
                        })
                      }
                    >
                      Stop Capture
                    </button>
                  </div>
                </article>

                <article className="status-card">
                  <div className="status-card-head">
                    <h3>Train on Mac</h3>
                    <span className={`status-pill ${statusTone(state?.services.training.state ?? "idle")}`}>
                      {state?.services.training.state ?? "idle"}
                    </span>
                  </div>
                  <p>
                    {isLocalLeaderCaptureMode(selectedTrainingProfile?.captureMode)
                      ? "The dataset is already local on the Mac in leader-as-follower mode. Refresh metadata here, then launch ACT training on Apple silicon with MPS."
                      : "Sync the Pi dataset locally, then launch ACT training on Apple silicon with MPS."}
                  </p>
                  <p className="pin-meta">
                    Last sync {selectedTrainingProfile?.artifacts.lastDatasetSyncAt ? prettyTimestamp(selectedTrainingProfile.artifacts.lastDatasetSyncAt) : "not yet"}
                  </p>
                  <div className="button-cluster inline">
                    <button
                      disabled={disabled || trainingDirty || !selectedTrainingProfile}
                      onClick={() =>
                        void mutateTrainingProfileAction(
                          "training-sync-start",
                          "/api/training/sync/start",
                          isLocalLeaderCaptureMode(selectedTrainingProfile?.captureMode)
                            ? "Local dataset metadata refreshed."
                            : "Dataset sync finished.",
                        )
                      }
                    >
                      {isLocalLeaderCaptureMode(selectedTrainingProfile?.captureMode)
                        ? "Refresh Local Dataset"
                        : "Sync Dataset"}
                    </button>
                    <button
                      className="primary"
                      disabled={disabled || trainingDirty || !selectedTrainingProfile}
                      onClick={() =>
                        void mutateTrainingProfileAction(
                          "training-run-start",
                          "/api/training/run/start",
                          "Training started.",
                        )
                      }
                    >
                      Start Training
                    </button>
                    <button
                      disabled={disabled}
                      onClick={() => void mutate("training-run-stop", "/api/training/run/stop", { method: "POST" })}
                    >
                      Stop Training
                    </button>
                  </div>
                  <p className="card-note">
                    Latest checkpoint: {selectedTrainingProfile?.artifacts.lastCheckpointPath ?? "none yet"}
                  </p>
                </article>

                <article className="status-card">
                  <div className="status-card-head">
                    <h3>Deploy & Benchmark</h3>
                    <span className={`status-pill ${statusTone(state?.services.policyBenchmark.state ?? "idle")}`}>
                      {selectedTrainingProfile?.artifacts.benchmark?.passed ? "ready" : state?.services.policyBenchmark.state ?? "idle"}
                    </span>
                  </div>
                  <p>
                    Deploy the chosen checkpoint to the Pi, then gate autonomous runs on the benchmark result.
                  </p>
                  <p className="pin-meta">
                    {selectedTrainingProfile?.artifacts.benchmark
                      ? `${selectedTrainingProfile.artifacts.benchmark.effectiveFps.toFixed(2)} fps effective • avg ${selectedTrainingProfile.artifacts.benchmark.averageLatencyMs.toFixed(2)} ms`
                      : "No benchmark result yet."}
                  </p>
                  <div className="button-cluster inline">
                    <button
                      disabled={disabled || trainingDirty || !selectedTrainingProfile}
                      onClick={() =>
                        void mutateTrainingProfileAction(
                          "training-deploy",
                          "/api/training/deploy",
                          "Checkpoint deployed to the Pi.",
                        )
                      }
                    >
                      Deploy Checkpoint
                    </button>
                    <button
                      className="primary"
                      disabled={disabled || trainingDirty || !selectedTrainingProfile}
                      onClick={() =>
                        void mutateTrainingProfileAction(
                          "training-benchmark",
                          "/api/training/benchmark",
                          "Policy benchmark completed.",
                        )
                      }
                    >
                      Run Benchmark
                    </button>
                  </div>
                </article>

                <article className="status-card">
                  <div className="status-card-head">
                    <h3>Evaluate on Pi</h3>
                    <span className={`status-pill ${statusTone(state?.services.policyEval.state ?? "idle")}`}>
                      {state?.services.policyEval.state ?? "idle"}
                    </span>
                  </div>
                  <p>
                    Autonomous evaluation runs entirely on the Pi and writes eval datasets back to the Pi.
                  </p>
                  <p className="pin-meta">
                    Last eval root {selectedTrainingProfile?.artifacts.latestEvalDatasetPath ?? "not yet"}
                  </p>
                  <div className="button-cluster inline">
                    <button
                      className="primary"
                      disabled={
                        disabled ||
                        trainingDirty ||
                        !selectedTrainingProfile ||
                        !selectedTrainingProfile.artifacts.benchmark?.passed
                      }
                      onClick={() =>
                        void mutateTrainingProfileAction(
                          "training-eval-start",
                          "/api/training/eval/start",
                          "Policy evaluation started.",
                        )
                      }
                    >
                      Start Eval
                    </button>
                    <button
                      disabled={disabled}
                      onClick={() => void mutate("training-eval-stop", "/api/training/eval/stop", { method: "POST" })}
                    >
                      Stop Eval
                    </button>
                  </div>
                </article>
              </div>
            </section>
          )}

          {activeSection === "settings" && (
            <section className="card stage-panel">
              <div className="card-head">
                <div>
                  <p className="card-kicker">Settings</p>
                  <h2>Connection Defaults</h2>
                </div>
                <p className="card-note">
                  Saved locally in <code>.lekiwi-ui/config.json</code>.
                </p>
              </div>

              {settingsDraft ? (
                <>
                  <div className="form-grid">
                    <label>
                      Expected hotspot
                      <input value={settingsDraft.hotspot.ssid} readOnly />
                    </label>
                    <label>
                      Pi host
                      <input
                        value={settingsDraft.pi.host}
                        onChange={(event) =>
                          updateSettingsField("pi", {
                            ...settingsDraft.pi,
                            host: event.target.value,
                          })
                        }
                      />
                    </label>
                    <label>
                      Pi fallback host
                      <input
                        value={settingsDraft.pi.fallbackHost}
                        onChange={(event) =>
                          updateSettingsField("pi", {
                            ...settingsDraft.pi,
                            fallbackHost: event.target.value,
                          })
                        }
                      />
                    </label>
                    <label>
                      Pi password
                      <input
                        value={settingsDraft.pi.password}
                        onChange={(event) =>
                          updateSettingsField("pi", {
                            ...settingsDraft.pi,
                            password: event.target.value,
                          })
                        }
                      />
                    </label>
                    <label>
                      Pi lerobot directory
                      <input
                        value={settingsDraft.pi.projectDir}
                        onChange={(event) =>
                          updateSettingsField("pi", {
                            ...settingsDraft.pi,
                            projectDir: event.target.value,
                          })
                        }
                      />
                    </label>
                    <label>
                      Mac lerobot directory
                      <input
                        value={settingsDraft.mac.projectDir}
                        onChange={(event) =>
                          updateSettingsField("mac", {
                            ...settingsDraft.mac,
                            projectDir: event.target.value,
                          })
                        }
                      />
                    </label>
                    <label>
                      Current Wi-Fi
                      <input value={state?.wifi.ssid ?? "offline"} readOnly />
                    </label>
                    <label>
                      Robot ID
                      <input
                        value={settingsDraft.host.robotId}
                        onChange={(event) =>
                          updateSettingsField("host", {
                            ...settingsDraft.host,
                            robotId: event.target.value,
                          })
                        }
                      />
                    </label>
                    <label>
                      Host connection time (s)
                      <input
                        type="number"
                        min="60"
                        value={settingsDraft.host.connectionTimeS}
                        onChange={(event) =>
                          updateSettingsField("host", {
                            ...settingsDraft.host,
                            connectionTimeS: Number(event.target.value),
                          })
                        }
                      />
                    </label>
                    <label>
                      Cameras JSON
                      <input
                        value={settingsDraft.host.camerasJson}
                        onChange={(event) =>
                          updateSettingsField("host", {
                            ...settingsDraft.host,
                            camerasJson: event.target.value,
                          })
                        }
                      />
                    </label>
                    <label className="checkbox-row settings-toggle">
                      <input
                        type="checkbox"
                        checked={settingsDraft.host.saferServoMode}
                        onChange={(event) =>
                          updateSettingsField("host", {
                            ...settingsDraft.host,
                            saferServoMode: event.target.checked,
                          })
                        }
                      />
                      <span>Enable safer SO-101 servo mode</span>
                    </label>
                  </div>

                  <div className="settings-note">
                    VEX Brain status: {state?.vexBrain.message ?? "Checking VEX Brain status..."}
                  </div>
                  <p className="card-note">
                    Base recordings now read turn-rate telemetry from the VEX inertial sensor and expect it on the
                    configured smart port.
                  </p>

                  <div className="form-grid">
                    <label>
                      VEX telemetry program
                      <input
                        value={settingsDraft.vex.telemetryProgramName}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            telemetryProgramName: event.target.value,
                          })
                        }
                      />
                    </label>
                    <label>
                      Telemetry slot
                      <input
                        type="number"
                        min="1"
                        max="8"
                        value={settingsDraft.vex.telemetrySlot}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            telemetrySlot: Number(event.target.value),
                          })
                        }
                      />
                    </label>
                    <label>
                      Replay slot
                      <input
                        type="number"
                        min="1"
                        max="8"
                        value={settingsDraft.vex.replaySlot}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            replaySlot: Number(event.target.value),
                          })
                        }
                      />
                    </label>
                    <label>
                      Inertial sensor port
                      <input
                        type="number"
                        min="1"
                        max="21"
                        value={settingsDraft.vex.inertial.port}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            inertial: {
                              ...settingsDraft.vex.inertial,
                              port: Number(event.target.value),
                            },
                          })
                        }
                      />
                    </label>
                    <label>
                      Deadband (%)
                      <input
                        type="number"
                        min="0"
                        max="30"
                        value={settingsDraft.vex.tuning.deadbandPercent}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            tuning: {
                              ...settingsDraft.vex.tuning,
                              deadbandPercent: Number(event.target.value),
                            },
                          })
                        }
                      />
                    </label>
                    <label>
                      Max linear speed (m/s)
                      <input
                        type="number"
                        min="0.05"
                        step="0.01"
                        value={settingsDraft.vex.tuning.maxLinearSpeedMps}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            tuning: {
                              ...settingsDraft.vex.tuning,
                              maxLinearSpeedMps: Number(event.target.value),
                            },
                          })
                        }
                      />
                    </label>
                    <label>
                      Max turn speed (deg/s)
                      <input
                        type="number"
                        min="5"
                        step="1"
                        value={settingsDraft.vex.tuning.maxTurnSpeedDps}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            tuning: {
                              ...settingsDraft.vex.tuning,
                              maxTurnSpeedDps: Number(event.target.value),
                            },
                          })
                        }
                      />
                    </label>
                    <label>
                      Forward axis
                      <select
                        value={settingsDraft.vex.controls.forwardAxis}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            controls: {
                              ...settingsDraft.vex.controls,
                              forwardAxis: event.target.value as AppSettings["vex"]["controls"]["forwardAxis"],
                            },
                          })
                        }
                      >
                        {VEX_AXIS_OPTIONS.map((option) => (
                          <option key={option.value} value={option.value}>
                            {option.label}
                          </option>
                        ))}
                      </select>
                    </label>
                    <label>
                      Strafe axis
                      <select
                        value={settingsDraft.vex.controls.strafeAxis}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            controls: {
                              ...settingsDraft.vex.controls,
                              strafeAxis: event.target.value as AppSettings["vex"]["controls"]["strafeAxis"],
                            },
                          })
                        }
                      >
                        {VEX_AXIS_OPTIONS.map((option) => (
                          <option key={option.value} value={option.value}>
                            {option.label}
                          </option>
                        ))}
                      </select>
                    </label>
                    <label>
                      Turn axis
                      <select
                        value={settingsDraft.vex.controls.turnAxis}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            controls: {
                              ...settingsDraft.vex.controls,
                              turnAxis: event.target.value as AppSettings["vex"]["controls"]["turnAxis"],
                            },
                          })
                        }
                      >
                        {VEX_AXIS_OPTIONS.map((option) => (
                          <option key={option.value} value={option.value}>
                            {option.label}
                          </option>
                        ))}
                      </select>
                    </label>
                    <label>
                      Front right port
                      <input
                        type="number"
                        min="1"
                        max="21"
                        value={settingsDraft.vex.motors.frontRight.port}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            motors: {
                              ...settingsDraft.vex.motors,
                              frontRight: {
                                ...settingsDraft.vex.motors.frontRight,
                                port: Number(event.target.value),
                              },
                            },
                          })
                        }
                      />
                    </label>
                    <label>
                      Front left port
                      <input
                        type="number"
                        min="1"
                        max="21"
                        value={settingsDraft.vex.motors.frontLeft.port}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            motors: {
                              ...settingsDraft.vex.motors,
                              frontLeft: {
                                ...settingsDraft.vex.motors.frontLeft,
                                port: Number(event.target.value),
                              },
                            },
                          })
                        }
                      />
                    </label>
                    <label>
                      Rear right port
                      <input
                        type="number"
                        min="1"
                        max="21"
                        value={settingsDraft.vex.motors.rearRight.port}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            motors: {
                              ...settingsDraft.vex.motors,
                              rearRight: {
                                ...settingsDraft.vex.motors.rearRight,
                                port: Number(event.target.value),
                              },
                            },
                          })
                        }
                      />
                    </label>
                    <label>
                      Rear left port
                      <input
                        type="number"
                        min="1"
                        max="21"
                        value={settingsDraft.vex.motors.rearLeft.port}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            motors: {
                              ...settingsDraft.vex.motors,
                              rearLeft: {
                                ...settingsDraft.vex.motors.rearLeft,
                                port: Number(event.target.value),
                              },
                            },
                          })
                        }
                      />
                    </label>
                    <label className="checkbox-row settings-toggle">
                      <input
                        type="checkbox"
                        checked={settingsDraft.vex.autoRunTelemetry}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            autoRunTelemetry: event.target.checked,
                          })
                        }
                      />
                      <span>Auto-run VEX telemetry when control, hotkeys, recording, or Pi capture starts</span>
                    </label>
                    <label className="checkbox-row settings-toggle">
                      <input
                        type="checkbox"
                        checked={settingsDraft.vex.controls.invertForward}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            controls: {
                              ...settingsDraft.vex.controls,
                              invertForward: event.target.checked,
                            },
                          })
                        }
                      />
                      <span>Invert forward axis</span>
                    </label>
                    <label className="checkbox-row settings-toggle">
                      <input
                        type="checkbox"
                        checked={settingsDraft.vex.controls.invertStrafe}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            controls: {
                              ...settingsDraft.vex.controls,
                              invertStrafe: event.target.checked,
                            },
                          })
                        }
                      />
                      <span>Invert strafe axis</span>
                    </label>
                    <label className="checkbox-row settings-toggle">
                      <input
                        type="checkbox"
                        checked={settingsDraft.vex.controls.invertTurn}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            controls: {
                              ...settingsDraft.vex.controls,
                              invertTurn: event.target.checked,
                            },
                          })
                        }
                      />
                      <span>Invert turn axis</span>
                    </label>
                    <label className="checkbox-row settings-toggle">
                      <input
                        type="checkbox"
                        checked={settingsDraft.vex.motors.frontRight.reversed}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            motors: {
                              ...settingsDraft.vex.motors,
                              frontRight: {
                                ...settingsDraft.vex.motors.frontRight,
                                reversed: event.target.checked,
                              },
                            },
                          })
                        }
                      />
                      <span>Front right reversed</span>
                    </label>
                    <label className="checkbox-row settings-toggle">
                      <input
                        type="checkbox"
                        checked={settingsDraft.vex.motors.frontLeft.reversed}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            motors: {
                              ...settingsDraft.vex.motors,
                              frontLeft: {
                                ...settingsDraft.vex.motors.frontLeft,
                                reversed: event.target.checked,
                              },
                            },
                          })
                        }
                      />
                      <span>Front left reversed</span>
                    </label>
                    <label className="checkbox-row settings-toggle">
                      <input
                        type="checkbox"
                        checked={settingsDraft.vex.motors.rearRight.reversed}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            motors: {
                              ...settingsDraft.vex.motors,
                              rearRight: {
                                ...settingsDraft.vex.motors.rearRight,
                                reversed: event.target.checked,
                              },
                            },
                          })
                        }
                      />
                      <span>Rear right reversed</span>
                    </label>
                    <label className="checkbox-row settings-toggle">
                      <input
                        type="checkbox"
                        checked={settingsDraft.vex.motors.rearLeft.reversed}
                        onChange={(event) =>
                          updateSettingsField("vex", {
                            ...settingsDraft.vex,
                            motors: {
                              ...settingsDraft.vex.motors,
                              rearLeft: {
                                ...settingsDraft.vex.motors.rearLeft,
                                reversed: event.target.checked,
                              },
                            },
                          })
                        }
                      />
                      <span>Rear left reversed</span>
                    </label>
                  </div>

                  <div className="settings-note">
                    When enabled, live control, recording, replay, and Pi dataset capture keep wrist roll continuous across the 180-degree seam for exact leader tracking, switch the follower wrist into continuous rotation mode, and still clamp the other arm joints before bad command jumps hit the servos. Start or restart the Pi-side robot process after saving for it to take effect.
                  </div>

                  <div className="settings-note">
                    Legacy LeKiwi base control stays off in this build. The Pi host, replay, calibration, and dataset capture paths all run arm-only, while the VEX Brain/controller remains the base drive path.
                  </div>

                  <div className="button-cluster inline">
                    <button
                      disabled={disabled}
                      onClick={() =>
                        updateSettingsField("host", {
                          ...settingsDraft.host,
                          armTorqueLimits: { ...RECOMMENDED_SAFE_ARM_TORQUE_LIMITS },
                        })
                      }
                    >
                      Load Safer Torque Preset
                    </button>
                    <button
                      disabled={disabled || !state?.piReachable}
                      onClick={() => void handleSyncVexTelemetry()}
                    >
                      Save + Run VEX Telemetry
                    </button>
                    <button
                      className="primary"
                      disabled={disabled || !settingsDirty}
                      onClick={() => void handleSaveSettings()}
                    >
                      Save Settings
                    </button>
                    <button
                      disabled={disabled}
                      onClick={() => {
                        if (state) {
                          setSettingsDraft(state.settings);
                          setSettingsDirty(false);
                        }
                      }}
                    >
                      Revert Draft
                    </button>
                  </div>
                </>
              ) : null}
            </section>
          )}

          {activeSection === "logs" && (
            <section className="card stage-panel">
              <div className="card-head">
                <div>
                  <p className="card-kicker">Logs</p>
                  <h2>Per-Process Output</h2>
                </div>
                <p className="card-note">
                  Raw stdout and stderr from the Pi host, teleop process, replay runner, and calibration commands, plus parsed gyro and ultrasonic telemetry from the Pi sensor feed.
                </p>
              </div>

              <div className="logs-shell">
                <div className="logs-service-list">
                  {serviceSnapshots.map((service) => (
                    <button
                      key={service.label}
                      className={
                        service.label === selectedLogService?.label
                          ? "logs-service-button active"
                          : "logs-service-button"
                      }
                      onClick={() => setSelectedLogServiceLabel(service.label)}
                    >
                      <div className="logs-service-row">
                        <strong>{service.label}</strong>
                        <span className={`status-pill ${statusTone(service.state)}`}>
                          {service.state}
                        </span>
                      </div>
                      <span className="logs-service-detail">{service.detail}</span>
                      <span className="logs-service-meta">
                        {service.logs.length} line{service.logs.length === 1 ? "" : "s"} •{" "}
                        {service.mode ?? "no mode"}
                      </span>
                    </button>
                  ))}
                </div>

                <div className="logs-viewer">
                  <div className="logs-viewer-head">
                    <div>
                      <p className="card-kicker">Selected Process</p>
                      <h3>{selectedLogService?.label ?? "No process selected"}</h3>
                      <p className="card-note">
                        {selectedLogService?.detail ?? "Choose a process from the left to inspect its output."}
                      </p>
                    </div>
                    <div className="inline-console-actions">
                      <button
                        className="console-copy-button"
                        disabled={!selectedLogService?.logs.length}
                        onClick={() => void handleCopySelectedLogs()}
                      >
                        Copy Selected Logs
                      </button>
                      <span className={`status-pill ${statusTone(selectedLogService?.state ?? "idle")}`}>
                        {selectedLogService?.state ?? "idle"}
                      </span>
                    </div>
                  </div>
                  <div className="logs-viewer-meta">
                    <span>Mode: {selectedLogService?.mode ?? "none"}</span>
                    <span>Started: {prettyTimestamp(selectedLogService?.startedAt ?? null)}</span>
                    <span>PID / Exit: {selectedLogService?.pid ?? "n/a"} / {selectedLogService?.exitCode ?? "n/a"}</span>
                  </div>
                  <pre className="logs-viewer-pre">
                    {selectedLogService?.logs.length
                      ? selectedLogService.logs.join("\n")
                      : "No logs yet."}
                  </pre>
                </div>
              </div>

              <div className="log-grid">
                <div className="sensor-feed">
                  <div className="log-panel-head">
                    <div>
                      <h3>Sensor Telemetry</h3>
                      <div className="logs-viewer-meta">
                        <span>Gyro: {state ? formatRobotSensorValue(state.robotSensors.gyro) : "n/a"}</span>
                        <span>X: {state ? formatRobotSensorValue(state.robotSensors.x) : "n/a"}</span>
                        <span>Y: {state ? formatRobotSensorValue(state.robotSensors.y) : "n/a"}</span>
                      </div>
                    </div>
                    <div className="inline-console-actions">
                      <button
                        className="console-copy-button"
                        disabled={!sensorTelemetryLines.length}
                        onClick={() => void handleCopySensorTelemetry()}
                      >
                        Copy Sensor Feed
                      </button>
                      <span className={`status-pill ${robotSensorTone(state?.robotSensors.gyro)}`}>
                        {state?.robotSensors.gyro.state ?? "idle"}
                      </span>
                    </div>
                  </div>
                  <pre>
                    {sensorTelemetryLines.length
                      ? sensorTelemetryLines.join("\n")
                      : "No sensor telemetry yet. Start Control, Recording, Replay, or Pi dataset capture to stream gyro, X, and Y updates here."}
                  </pre>
                </div>

                <div className="activity-feed">
                  <div className="log-panel-head">
                    <h3>Backend Activity</h3>
                    <span className={`status-pill ${pendingAction ? "warn" : "muted"}`}>
                      {pendingAction ? `running: ${pendingAction}` : "idle"}
                    </span>
                  </div>
                  <pre>
                    {state?.activityLog.length
                      ? state.activityLog.join("\n")
                      : "No backend activity yet."}
                  </pre>
                </div>
              </div>
            </section>
          )}
        </div>
      </div>
    </div>
  );
}
