import crypto from "node:crypto";
import fs from "node:fs";
import path from "node:path";

import type {
  ArmHomePosition,
  AppSettings,
  ChainLink,
  PinnedMove,
  RecordingReplayOptions,
  StoredConfig,
  TrainingConfig,
} from "./types.js";
import { normalizeTrainingConfig } from "./trainingUtils.js";

const ARM_HOME_JOINT_KEYS = [
  "arm_shoulder_pan.pos",
  "arm_shoulder_lift.pos",
  "arm_elbow_flex.pos",
  "arm_wrist_flex.pos",
  "arm_wrist_roll.pos",
  "arm_gripper.pos",
] as const;

export class ConfigStore {
  private readonly configDir: string;
  private readonly configPath: string;
  private readonly rootDir: string;
  private config: StoredConfig;

  constructor(private readonly defaults: StoredConfig, rootDir: string) {
    this.rootDir = rootDir;
    this.configDir = path.join(rootDir, ".lekiwi-ui");
    this.configPath = path.join(this.configDir, "config.json");
    this.config = this.load();
  }

  getConfig(): StoredConfig {
    return structuredClone(this.config);
  }

  saveSettings(settings: AppSettings): StoredConfig {
    this.config = {
      ...this.config,
      settings: this.normalizeSettings(settings),
    };
    this.persist();
    return this.getConfig();
  }

  savePinnedMoves(pinnedMoves: PinnedMove[]): StoredConfig {
    this.config = {
      ...this.config,
      pinnedMoves: structuredClone(pinnedMoves),
    };
    this.persist();
    return this.getConfig();
  }

  saveChainLinks(chainLinks: ChainLink[]): StoredConfig {
    this.config = {
      ...this.config,
      chainLinks: structuredClone(chainLinks),
    };
    this.persist();
    return this.getConfig();
  }

  saveHomePosition(homePosition: ArmHomePosition | null): StoredConfig {
    this.config = {
      ...this.config,
      homePosition: homePosition ? structuredClone(homePosition) : null,
    };
    this.persist();
    return this.getConfig();
  }

  saveRecordingReplayOptions(
    recordingReplayOptions: Record<string, RecordingReplayOptions>,
  ): StoredConfig {
    this.config = {
      ...this.config,
      recordingReplayOptions: structuredClone(recordingReplayOptions),
    };
    this.persist();
    return this.getConfig();
  }

  saveTraining(training: TrainingConfig): StoredConfig {
    this.config = {
      ...this.config,
      training: structuredClone(training),
    };
    this.persist();
    return this.getConfig();
  }

  private load(): StoredConfig {
    if (!fs.existsSync(this.configPath)) {
      return this.defaults;
    }

    try {
      const raw = JSON.parse(fs.readFileSync(this.configPath, "utf8")) as Partial<StoredConfig>;
      const rawVex = raw.settings?.vex;
      const rawVexControls = rawVex?.controls;
      const rawVexControlPresetVersion = Number(rawVex?.controlPresetVersion);
      const shouldUpgradeLegacyVexAxisDefaults =
        !Number.isFinite(rawVexControlPresetVersion) &&
        rawVexControls?.forwardAxis === "axis2" &&
        (rawVexControls?.strafeAxis ?? this.defaults.settings.vex.controls.strafeAxis) === "axis4" &&
        (rawVexControls?.turnAxis ?? this.defaults.settings.vex.controls.turnAxis) === "axis1" &&
        !Boolean(rawVexControls?.invertForward) &&
        !Boolean(rawVexControls?.invertStrafe) &&
        !Boolean(rawVexControls?.invertTurn);
      const settings = this.normalizeSettings({
        ...this.defaults.settings,
        ...raw.settings,
        hotspot: {
          ...this.defaults.settings.hotspot,
          ...raw.settings?.hotspot,
        },
        pi: {
          ...this.defaults.settings.pi,
          ...raw.settings?.pi,
        },
        mac: {
          ...this.defaults.settings.mac,
          ...raw.settings?.mac,
        },
        host: {
          ...this.defaults.settings.host,
          ...raw.settings?.host,
        },
        trajectories: {
          ...this.defaults.settings.trajectories,
          ...raw.settings?.trajectories,
        },
        vex: {
          ...this.defaults.settings.vex,
          ...raw.settings?.vex,
          inertial: {
            ...this.defaults.settings.vex.inertial,
            ...raw.settings?.vex?.inertial,
          },
          motors: {
            ...this.defaults.settings.vex.motors,
            ...raw.settings?.vex?.motors,
            frontRight: {
              ...this.defaults.settings.vex.motors.frontRight,
              ...raw.settings?.vex?.motors?.frontRight,
            },
            frontLeft: {
              ...this.defaults.settings.vex.motors.frontLeft,
              ...raw.settings?.vex?.motors?.frontLeft,
            },
            rearRight: {
              ...this.defaults.settings.vex.motors.rearRight,
              ...raw.settings?.vex?.motors?.rearRight,
            },
            rearLeft: {
              ...this.defaults.settings.vex.motors.rearLeft,
              ...raw.settings?.vex?.motors?.rearLeft,
            },
          },
          controls: {
            ...this.defaults.settings.vex.controls,
            ...raw.settings?.vex?.controls,
            ...(shouldUpgradeLegacyVexAxisDefaults ? { forwardAxis: "axis3" as const } : {}),
          },
          tuning: {
            ...this.defaults.settings.vex.tuning,
            ...raw.settings?.vex?.tuning,
          },
        },
      });
      return {
        settings,
        pinnedMoves: Array.isArray(raw.pinnedMoves)
          ? raw.pinnedMoves.map((move) => this.normalizePinnedMove(move))
          : this.defaults.pinnedMoves,
        chainLinks: Array.isArray(raw.chainLinks)
          ? raw.chainLinks.map((chain) => this.normalizeChainLink(chain))
          : this.defaults.chainLinks,
        homePosition: this.normalizeHomePosition(raw.homePosition),
        recordingReplayOptions: this.normalizeRecordingReplayOptions(
          raw.recordingReplayOptions,
          settings.trajectories.defaultReplaySpeed,
        ),
        training: normalizeTrainingConfig(raw.training, this.rootDir),
      };
    } catch {
      return this.defaults;
    }
  }

  private normalizePinnedMove(raw: Partial<PinnedMove> | undefined): PinnedMove {
    return {
      id: typeof raw?.id === "string" ? raw.id : crypto.randomUUID(),
      name: typeof raw?.name === "string" ? raw.name : "Pinned move",
      trajectoryPath: typeof raw?.trajectoryPath === "string" ? raw.trajectoryPath : "",
      target: raw?.target === "leader" ? "leader" : "pi",
      vexReplayMode: raw?.vexReplayMode === "drive" ? "drive" : "ecu",
      homeMode: this.normalizeHomeMode(raw?.homeMode),
      speed: this.normalizeReplaySpeed(raw?.speed),
      autoVexPositioning: raw?.autoVexPositioning === false ? false : true,
      includeBase: Boolean(raw?.includeBase),
      holdFinalS: Number(raw?.holdFinalS) >= 0 ? Number(raw?.holdFinalS) : 0.5,
      keyBinding: typeof raw?.keyBinding === "string" ? raw.keyBinding : "",
    };
  }

  private normalizeChainLink(raw: Partial<ChainLink> | undefined): ChainLink {
    const items = Array.isArray(raw?.items)
      ? raw.items
          .map((item) => this.normalizeChainLinkItem(item))
          .filter((item) => item.trajectoryPath.trim())
      : [];

    return {
      id: typeof raw?.id === "string" && raw.id.trim() ? raw.id : crypto.randomUUID(),
      name: typeof raw?.name === "string" && raw.name.trim() ? raw.name.trim() : "Chain-link",
      confirmAfterEach: raw?.confirmAfterEach === false ? false : true,
      items,
    };
  }

  private normalizeChainLinkItem(
    raw: Partial<ChainLink["items"][number]> | undefined,
  ): ChainLink["items"][number] {
    const target = raw?.target === "leader" ? "leader" : "pi";
    return {
      id: typeof raw?.id === "string" && raw.id.trim() ? raw.id : crypto.randomUUID(),
      name: typeof raw?.name === "string" && raw.name.trim() ? raw.name.trim() : "Recording",
      trajectoryPath: typeof raw?.trajectoryPath === "string" ? raw.trajectoryPath.trim() : "",
      target,
      vexReplayMode: raw?.vexReplayMode === "drive" ? "drive" : "ecu",
      homeMode: target === "pi" ? this.normalizeHomeMode(raw?.homeMode) : "none",
      speed: this.normalizeReplaySpeed(raw?.speed),
      autoVexPositioning: target === "pi" ? raw?.autoVexPositioning !== false : false,
      includeBase: target === "pi" ? Boolean(raw?.includeBase) : false,
      holdFinalS: Number(raw?.holdFinalS) >= 0 ? Number(raw?.holdFinalS) : 0.5,
    };
  }

  private normalizeReplaySpeed(
    value: unknown,
    fallback = this.defaults.settings.trajectories.defaultReplaySpeed,
  ): number {
    const numeric = Number(value);
    return Number.isFinite(numeric) && numeric > 0 ? numeric : fallback;
  }

  private normalizeHomeMode(value: unknown): RecordingReplayOptions["homeMode"] {
    return value === "start" || value === "end" || value === "both" ? value : "none";
  }

  private normalizeHomePosition(raw: unknown): ArmHomePosition | null {
    if (!raw || typeof raw !== "object") {
      return null;
    }
    const candidate = raw as Partial<ArmHomePosition>;
    if (!candidate.joints || typeof candidate.joints !== "object") {
      return null;
    }
    const joints: Record<string, number> = {};
    for (const key of ARM_HOME_JOINT_KEYS) {
      const value = candidate.joints[key];
      if (!Number.isFinite(Number(value))) {
        return null;
      }
      joints[key] = Number(value);
    }

    return {
      capturedAt:
        typeof candidate.capturedAt === "string" && candidate.capturedAt.trim()
          ? candidate.capturedAt
          : new Date(0).toISOString(),
      joints,
    };
  }

  private normalizeRecordingReplayOptions(
    raw: unknown,
    defaultReplaySpeed = this.defaults.settings.trajectories.defaultReplaySpeed,
  ): Record<string, RecordingReplayOptions> {
    if (!raw || typeof raw !== "object") {
      return {};
    }

    const normalized: Record<string, RecordingReplayOptions> = {};
    for (const [path, value] of Object.entries(raw as Record<string, unknown>)) {
      if (!path.trim() || !value || typeof value !== "object") {
        continue;
      }
      const candidate = value as Partial<RecordingReplayOptions>;
      normalized[path] = {
        homeMode: this.normalizeHomeMode(candidate.homeMode),
        speed: this.normalizeReplaySpeed(candidate.speed, defaultReplaySpeed),
        autoVexPositioning: candidate.autoVexPositioning === false ? false : true,
      };
    }
    return normalized;
  }

  private normalizeSettings(settings: AppSettings): AppSettings {
    const normalized = structuredClone(settings);
    const clampPort = (value: number) => {
      const numeric = Math.round(Number(value));
      if (!Number.isFinite(numeric)) {
        return 1;
      }
      return Math.min(21, Math.max(1, numeric));
    };
    const clampSlot = (value: number, fallback: number) => {
      const numeric = Math.round(Number(value));
      if (!Number.isFinite(numeric)) {
        return fallback;
      }
      return Math.min(8, Math.max(1, numeric));
    };
    const validAxes = new Set(["axis1", "axis2", "axis3", "axis4"]);

    return {
      ...normalized,
      host: {
        ...normalized.host,
        enableBase: false,
      },
      vex: {
        ...normalized.vex,
        controlPresetVersion: Number.isFinite(Number(normalized.vex.controlPresetVersion))
          ? Math.max(2, Math.round(Number(normalized.vex.controlPresetVersion)))
          : this.defaults.settings.vex.controlPresetVersion,
        telemetrySlot: clampSlot(normalized.vex.telemetrySlot, this.defaults.settings.vex.telemetrySlot),
        replaySlot: clampSlot(normalized.vex.replaySlot, this.defaults.settings.vex.replaySlot),
        autoRunTelemetry: Boolean(normalized.vex.autoRunTelemetry),
        telemetryProgramName:
          typeof normalized.vex.telemetryProgramName === "string" &&
          normalized.vex.telemetryProgramName.trim()
            ? normalized.vex.telemetryProgramName.trim()
            : this.defaults.settings.vex.telemetryProgramName,
        inertial: {
          port: clampPort(normalized.vex.inertial?.port ?? this.defaults.settings.vex.inertial.port),
        },
        motors: {
          frontRight: {
            port: clampPort(normalized.vex.motors.frontRight.port),
            reversed: Boolean(normalized.vex.motors.frontRight.reversed),
          },
          frontLeft: {
            port: clampPort(normalized.vex.motors.frontLeft.port),
            reversed: Boolean(normalized.vex.motors.frontLeft.reversed),
          },
          rearRight: {
            port: clampPort(normalized.vex.motors.rearRight.port),
            reversed: Boolean(normalized.vex.motors.rearRight.reversed),
          },
          rearLeft: {
            port: clampPort(normalized.vex.motors.rearLeft.port),
            reversed: Boolean(normalized.vex.motors.rearLeft.reversed),
          },
        },
        controls: {
          forwardAxis: validAxes.has(normalized.vex.controls.forwardAxis)
            ? normalized.vex.controls.forwardAxis
            : this.defaults.settings.vex.controls.forwardAxis,
          strafeAxis: validAxes.has(normalized.vex.controls.strafeAxis)
            ? normalized.vex.controls.strafeAxis
            : this.defaults.settings.vex.controls.strafeAxis,
          turnAxis: validAxes.has(normalized.vex.controls.turnAxis)
            ? normalized.vex.controls.turnAxis
            : this.defaults.settings.vex.controls.turnAxis,
          invertForward: Boolean(normalized.vex.controls.invertForward),
          invertStrafe: Boolean(normalized.vex.controls.invertStrafe),
          invertTurn: Boolean(normalized.vex.controls.invertTurn),
        },
        tuning: {
          deadbandPercent: Math.min(30, Math.max(0, Number(normalized.vex.tuning.deadbandPercent) || 0)),
          maxLinearSpeedMps: Math.min(2, Math.max(0.05, Number(normalized.vex.tuning.maxLinearSpeedMps) || 0.35)),
          maxTurnSpeedDps: Math.min(360, Math.max(5, Number(normalized.vex.tuning.maxTurnSpeedDps) || 90)),
        },
      },
    };
  }

  private persist(): void {
    fs.mkdirSync(this.configDir, { recursive: true });
    fs.writeFileSync(this.configPath, JSON.stringify(this.config, null, 2) + "\n", "utf8");
  }
}
