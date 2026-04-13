import crypto from "node:crypto";
import fs from "node:fs";
import path from "node:path";

import type {
  AppSettings,
  PinnedMove,
  StoredConfig,
  TrainingConfig,
} from "./types.js";
import { normalizeTrainingConfig } from "./trainingUtils.js";

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
      settings: structuredClone(settings),
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
      return {
        settings: {
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
        },
        pinnedMoves: Array.isArray(raw.pinnedMoves)
          ? raw.pinnedMoves.map((move) => this.normalizePinnedMove(move))
          : this.defaults.pinnedMoves,
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
      speed: Number(raw?.speed) > 0 ? Number(raw?.speed) : 1,
      includeBase: Boolean(raw?.includeBase),
      holdFinalS: Number(raw?.holdFinalS) >= 0 ? Number(raw?.holdFinalS) : 0.5,
      keyBinding: typeof raw?.keyBinding === "string" ? raw.keyBinding : "",
    };
  }

  private persist(): void {
    fs.mkdirSync(this.configDir, { recursive: true });
    fs.writeFileSync(this.configPath, JSON.stringify(this.config, null, 2) + "\n", "utf8");
  }
}
