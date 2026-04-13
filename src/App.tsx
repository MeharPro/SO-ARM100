import { useEffect, useMemo, useRef, useState } from "react";

import type {
  AppSettings,
  DashboardState,
  PinnedMove,
  ReplayTarget,
  RecordingEntry,
  RenameRecordingRequest,
  TrainingArtifact,
  TrainingProfile,
  TrimRecordingRequest,
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

const TORQUE_LIMIT_MIN = 0;
const TORQUE_LIMIT_MAX = 1000;
const DEFAULT_ARM_TORQUE_LIMITS = Object.fromEntries(
  ARM_SERVO_IDS.map((id) => [id, TORQUE_LIMIT_MAX]),
) as Record<string, number>;
const LOCAL_TRAINING_ROOT = "/Users/meharkhanna/robot-arm/output";

interface ServoTemperature {
  id: string;
  label: string;
  temperatureC: number | null;
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

  let key = event.key.toUpperCase();
  if (key === " ") {
    key = "SPACE";
  }
  if (key === "ESCAPE") {
    key = "ESC";
  }
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
  controlsDisabled,
  onTorqueLimitChange,
}: {
  telemetry: PowerTelemetry | null;
  torqueLimits: Record<string, number>;
  controlsDisabled: boolean;
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
            <label className="servo-torque-control">
              <small>Torque</small>
              <input
                type="range"
                min={TORQUE_LIMIT_MIN}
                max={TORQUE_LIMIT_MAX}
                step="10"
                disabled={controlsDisabled}
                value={torqueLimits[servo.id] ?? TORQUE_LIMIT_MAX}
                onChange={(event) =>
                  onTorqueLimitChange(servo.id, Number(event.target.value))
                }
              />
              <small>{torqueLimits[servo.id] ?? TORQUE_LIMIT_MAX}</small>
            </label>
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

export default function App() {
  const [state, setState] = useState<DashboardState | null>(null);
  const [activeSection, setActiveSection] = useState<SectionKey>("overview");
  const [settingsDraft, setSettingsDraft] = useState<AppSettings | null>(null);
  const [settingsDirty, setSettingsDirty] = useState(false);
  const [trainingDraft, setTrainingDraft] = useState<TrainingProfile | null>(null);
  const [trainingDirty, setTrainingDirty] = useState(false);
  const [recordLabel, setRecordLabel] = useState("");
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
  const toastTimer = useRef<number | null>(null);
  const torqueTimers = useRef<Record<string, number>>({});

  const loadState = async () => {
    try {
      const next = await request<DashboardState>("/api/state");
      setBackendError("");
      setState(next);
      if (!settingsDirty || !settingsDraft) {
        setSettingsDraft(next.settings);
      }
    } catch (error) {
      setBackendError(error instanceof Error ? error.message : "Backend state is unavailable.");
    }
  };

  useEffect(() => {
    void loadState();
    const interval = window.setInterval(() => {
      void loadState().catch(() => undefined);
    }, POLL_MS);
    return () => window.clearInterval(interval);
  }, []);

  useEffect(() => {
    return () => {
      for (const timer of Object.values(torqueTimers.current)) {
        window.clearTimeout(timer);
      }
    };
  }, []);

  useEffect(() => {
    if (!state?.recordings.length) {
      return;
    }

    const stillExists = state.recordings.some((item) => item.path === selectedRecording);
    if (!selectedRecording || !stillExists) {
      setSelectedRecording(state.recordings[0].path);
    }
  }, [selectedRecording, state?.recordings]);

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
  const selectedTrainingProfile = state?.training.selectedProfile ?? null;

  useEffect(() => {
    setRecordingNameDraft(selectedRecordingEntry?.name ?? "");
  }, [selectedRecordingEntry?.name, selectedRecordingEntry?.path]);

  useEffect(() => {
    setTrimStartDraft("0");
    setTrimEndDraft(
      selectedRecordingEntry?.durationS !== null && selectedRecordingEntry?.durationS !== undefined
        ? String(selectedRecordingEntry.durationS)
        : "",
    );
  }, [selectedRecordingEntry?.durationS, selectedRecordingEntry?.path]);

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
      recordingService?.logs.find((line) =>
        line.includes("First leader command received. Recording started."),
      ) ?? null,
    [recordingService?.logs],
  );
  const recordingStartedMs = useMemo(
    () => (recordingStartedLog ? parseLocalLogTimestamp(recordingStartedLog) : null),
    [recordingStartedLog],
  );
  const recordingHasCapturedMotion = Boolean(recordingStartedLog);

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
      includeBase: false,
      speed: pinSpeed,
      holdFinalS: pinHoldFinal,
    }),
    [pinHoldFinal, pinSpeed, replayTarget, selectedRecording],
  );

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

  const activeSectionMeta = useMemo(
    () => SECTIONS.find((section) => section.key === activeSection) ?? SECTIONS[0],
    [activeSection],
  );

  const flashToast = (message: string) => {
    setToast(message);
    if (toastTimer.current) {
      window.clearTimeout(toastTimer.current);
    }
    toastTimer.current = window.setTimeout(() => setToast(""), 2200);
  };

  const handleCopyLiveConsole = async () => {
    const text = liveConsoleLines.join("\n");
    if (!text) {
      flashToast("No live console output to copy.");
      return;
    }

    try {
      if (navigator.clipboard?.writeText) {
        await navigator.clipboard.writeText(text);
      } else {
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
      flashToast("Live console copied.");
    } catch {
      flashToast("Could not copy live console.");
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

  const handleSaveSettings = async () => {
    if (!settingsDraft) {
      return;
    }

    const next = await mutate("save-settings", "/api/settings", {
      method: "POST",
      body: JSON.stringify(settingsDraft),
    });
    if (next) {
      setSettingsDirty(false);
      flashToast("Settings saved.");
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
      if (!settingsDirty) {
        setSettingsDraft(next.settings);
      }
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
                  The backend retries the Pi five times, checks the leader-arm
                  port first, and streams the action output below.
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
                  className="danger"
                  disabled={disabled}
                  onClick={() => void mutate("emergency-stop", "/api/robot/emergency-stop", { method: "POST" })}
                >
                  Emergency Stop + Torque Off
                </button>
              </div>

              <div className="calibration-panel">
                <div>
                  <p className="card-kicker">Calibration</p>
                  <h3>Calibrate Arm and Leader</h3>
                  <p className="card-note">
                    Use the live console prompts. Send <code>c</code> to force a fresh calibration, then Enter at each step.
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
                controlsDisabled={disabled}
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

              <div className="record-toolbar">
                <label>
                  Recording label
                  <input
                    value={recordLabel}
                    onChange={(event) => setRecordLabel(event.target.value)}
                    placeholder="Pick up cube"
                  />
                </label>
                <div className="button-cluster inline">
                  <button
                    className="primary"
                    disabled={disabled}
                    onClick={() =>
                      void mutate("start-recording", "/api/recordings/start", {
                        method: "POST",
                        body: JSON.stringify({ label: recordLabel }),
                      })
                    }
                  >
                    Start Recording
                  </button>
                  <button
                    disabled={disabled}
                    onClick={() => void mutate("stop-recording", "/api/recordings/stop", { method: "POST" })}
                  >
                    Stop Recording
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
                      <strong className="recording-live-waiting">Waiting for leader input</strong>
                    )}
                  </div>
                  <span className="recording-live-meta">
                    {recordingHasCapturedMotion
                      ? `Started ${prettyTimestamp(recordingService.startedAt)}`
                      : "Recorder is armed and ready."}
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
                  </div>
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
                        Keep only the action window. This updates the selected recording in place.
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
                        onChange={(event) => setPinSpeed(Number(event.target.value))}
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
                  </div>
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
                <p className="card-note">
                  Hotkeys fire when the page is focused and you are not typing in a field.
                </p>
              </div>
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
                        {replayTargetLabel(move.target)} • Speed {move.speed} • Hold {move.holdFinalS}s • Arm only
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
                  </div>

                  <div className="settings-note">
                    Base control is disabled. This LeKiwi setup is arm-only, so replay and calibration always use <code>--robot.enable_base=false</code>.
                  </div>

                  <div className="button-cluster inline">
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
                  Raw stdout and stderr from the Pi host, teleop process, replay runner, and calibration commands.
                </p>
              </div>
              <PowerTelemetryPanel
                telemetry={powerTelemetry}
                torqueLimits={torqueLimits}
                controlsDisabled={disabled}
                onTorqueLimitChange={handleTorqueLimitChange}
              />
              <div className="log-grid">
                {state
                  ? serviceSnapshots.map(
                      (service) => (
                        <div key={service.label} className="log-panel">
                          <div className="log-panel-head">
                            <h3>{service.label}</h3>
                            <span className={`status-pill ${statusTone(service.state)}`}>
                              {service.state}
                            </span>
                          </div>
                          <pre>
                            {service.logs.length
                              ? service.logs.join("\n")
                              : "No logs yet."}
                          </pre>
                        </div>
                      ),
                    )
                  : null}
              </div>
            </section>
          )}
        </div>
      </div>
    </div>
  );
}
