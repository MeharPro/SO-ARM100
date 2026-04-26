import { spawn, type ChildProcessWithoutNullStreams } from "node:child_process";

import { Client, type ClientChannel, type ConnectConfig } from "ssh2";

import { shellQuote, sleep } from "./system.js";
import type { ServiceSnapshot } from "./types.js";

const MAX_LOG_LINES = 180;

function sanitizeLogText(value: string): string {
  return value.replace(/[\u0000-\u0008\u000b-\u001f\u007f]/g, "");
}

function createSnapshot(label: string): ServiceSnapshot {
  return {
    label,
    state: "idle",
    detail: "Idle",
    pid: null,
    exitCode: null,
    startedAt: null,
    stoppedAt: null,
    mode: null,
    logs: [],
    meta: {},
  };
}

function timestamp(): string {
  return new Date().toLocaleTimeString();
}

function pushLog(snapshot: ServiceSnapshot, stream: "stdout" | "stderr" | "system", line: string): void {
  const trimmed = sanitizeLogText(line).trimEnd();
  if (!trimmed) {
    return;
  }

  snapshot.logs.push(`${timestamp()} [${stream}] ${trimmed}`);
  if (snapshot.logs.length > MAX_LOG_LINES) {
    snapshot.logs.splice(0, snapshot.logs.length - MAX_LOG_LINES);
  }
}

function cloneSnapshot(snapshot: ServiceSnapshot): ServiceSnapshot {
  return structuredClone(snapshot);
}

function logCommand(snapshot: ServiceSnapshot, script: string): void {
  pushLog(snapshot, "system", "Command:");
  for (const line of script.split(/\r?\n/)) {
    pushLog(snapshot, "system", `  ${line}`);
  }
}

type TaskLogStream = "stdout" | "stderr" | "system";
type TaskLogger = (stream: TaskLogStream, line: string) => void;

export class LocalProcessRunner {
  private child: ChildProcessWithoutNullStreams | null = null;
  private exitPromise: Promise<void> | null = null;
  private stdoutBuffer = "";
  private stderrBuffer = "";
  private snapshot: ServiceSnapshot;

  constructor(label: string) {
    this.snapshot = createSnapshot(label);
  }

  getSnapshot(): ServiceSnapshot {
    return cloneSnapshot(this.snapshot);
  }

  sendInput(input: string, label: string): boolean {
    if (!this.child) {
      return false;
    }

    this.child.stdin.write(input);
    pushLog(this.snapshot, "system", `Sent input: ${label}.`);
    return true;
  }

  async waitForExit(timeoutMs: number): Promise<ServiceSnapshot | null> {
    if (!this.exitPromise) {
      return this.snapshot.stoppedAt ? cloneSnapshot(this.snapshot) : null;
    }

    let timeout: NodeJS.Timeout | null = null;
    try {
      return await Promise.race([
        this.exitPromise.then(() => cloneSnapshot(this.snapshot)),
        new Promise<null>((resolve) => {
          timeout = setTimeout(() => resolve(null), timeoutMs);
        }),
      ]);
    } finally {
      if (timeout) {
        clearTimeout(timeout);
      }
    }
  }

  async start(script: string, mode: string, meta: Record<string, unknown> = {}): Promise<void> {
    await this.stop("Replacing the existing process.");

    this.snapshot = {
      ...createSnapshot(this.snapshot.label),
      state: "starting",
      detail: "Launching local process.",
      startedAt: new Date().toISOString(),
      mode,
      meta,
    };
    pushLog(this.snapshot, "system", `Launching ${this.snapshot.label}.`);
    logCommand(this.snapshot, script);

    const child = spawn("/bin/zsh", ["-lc", script], {
      stdio: "pipe",
      env: process.env,
    });

    this.child = child;
    this.snapshot.pid = child.pid ?? null;
    this.exitPromise = new Promise((resolve) => {
      child.once("close", (code) => {
        this.flushBufferedLogs();
        this.snapshot.exitCode = code;
        this.snapshot.pid = null;
        this.snapshot.stoppedAt = new Date().toISOString();

        if (this.snapshot.state === "stopping") {
          this.snapshot.state = "idle";
          this.snapshot.detail = "Stopped.";
        } else if (code === 0 || code === null) {
          this.snapshot.state = "idle";
          this.snapshot.detail = "Exited cleanly.";
        } else {
          this.snapshot.state = "error";
          this.snapshot.detail = `Exited with code ${code}.`;
          pushLog(this.snapshot, "system", `${this.snapshot.label} exited with code ${code}.`);
        }

        this.child = null;
        this.exitPromise = null;
        resolve();
      });
    });

    child.once("spawn", () => {
      this.snapshot.state = "running";
      this.snapshot.detail = "Running.";
    });

    child.once("error", (error) => {
      this.snapshot.state = "error";
      this.snapshot.detail = error.message;
      pushLog(this.snapshot, "stderr", error.message);
    });

    child.stdout.on("data", (chunk: Buffer) => {
      this.stdoutBuffer += chunk.toString("utf8");
      this.stdoutBuffer = this.consumeBuffer(this.stdoutBuffer, "stdout");
    });

    child.stderr.on("data", (chunk: Buffer) => {
      this.stderrBuffer += chunk.toString("utf8");
      this.stderrBuffer = this.consumeBuffer(this.stderrBuffer, "stderr");
    });
  }

  async stop(reason = "Stopped by the UI."): Promise<void> {
    if (!this.child) {
      return;
    }

    this.snapshot.state = "stopping";
    this.snapshot.detail = reason;
    pushLog(this.snapshot, "system", reason);

    this.child.kill("SIGINT");
    await Promise.race([this.exitPromise, sleep(5000)]);

    if (this.child) {
      pushLog(this.snapshot, "system", "Escalating to SIGKILL.");
      this.child.kill("SIGKILL");
      await this.exitPromise;
    }
  }

  private consumeBuffer(buffer: string, stream: "stdout" | "stderr"): string {
    const parts = buffer.split(/\r?\n/);
    const remainder = parts.pop() ?? "";
    for (const part of parts) {
      pushLog(this.snapshot, stream, part);
    }
    return remainder;
  }

  private flushBufferedLogs(): void {
    if (this.stdoutBuffer) {
      pushLog(this.snapshot, "stdout", this.stdoutBuffer);
      this.stdoutBuffer = "";
    }
    if (this.stderrBuffer) {
      pushLog(this.snapshot, "stderr", this.stderrBuffer);
      this.stderrBuffer = "";
    }
  }
}

export class RemoteProcessRunner {
  private client: Client | null = null;
  private channel: ClientChannel | null = null;
  private exitPromise: Promise<void> | null = null;
  private stdoutBuffer = "";
  private stderrBuffer = "";
  private snapshot: ServiceSnapshot;

  constructor(label: string) {
    this.snapshot = createSnapshot(label);
  }

  getSnapshot(): ServiceSnapshot {
    return cloneSnapshot(this.snapshot);
  }

  sendInput(input: string, label: string): boolean {
    if (!this.channel) {
      return false;
    }

    this.channel.write(input);
    pushLog(this.snapshot, "system", `Sent input: ${label}.`);
    return true;
  }

  async waitForExit(timeoutMs: number): Promise<ServiceSnapshot | null> {
    if (!this.exitPromise) {
      return this.snapshot.stoppedAt ? cloneSnapshot(this.snapshot) : null;
    }

    let timeout: NodeJS.Timeout | null = null;
    try {
      return await Promise.race([
        this.exitPromise.then(() => cloneSnapshot(this.snapshot)),
        new Promise<null>((resolve) => {
          timeout = setTimeout(() => resolve(null), timeoutMs);
        }),
      ]);
    } finally {
      if (timeout) {
        clearTimeout(timeout);
      }
    }
  }

  async start(
    script: string,
    connection: ConnectConfig,
    mode: string,
    meta: Record<string, unknown> = {},
  ): Promise<void> {
    await this.stop("Replacing the existing remote process.");

    this.snapshot = {
      ...createSnapshot(this.snapshot.label),
      state: "starting",
      detail: `Connecting to ${connection.host}.`,
      startedAt: new Date().toISOString(),
      mode,
      meta,
    };
    pushLog(this.snapshot, "system", `Connecting to ${connection.host}.`);
    logCommand(this.snapshot, script);

    const client = new Client();
    this.client = client;

    await new Promise<void>((resolve, reject) => {
      let settled = false;

      const fail = (error: Error) => {
        if (settled) {
          return;
        }
        settled = true;
        this.snapshot.state = "error";
        this.snapshot.detail = error.message;
        pushLog(this.snapshot, "stderr", error.message);
        this.client = null;
        reject(error);
      };
      const handleError = (error: Error) => {
        fail(error);
      };
      const cleanupErrorHandler = () => {
        client.off("error", handleError);
      };

      client.once("ready", () => {
        client.exec(`/bin/bash -lc ${shellQuote(script)}`, (error, channel) => {
          if (error) {
            fail(error);
            return;
          }

          this.channel = channel;
          this.snapshot.state = "running";
          this.snapshot.detail = `Running on ${connection.host}.`;

          this.exitPromise = new Promise((exitResolve) => {
            channel.once("close", (code: number | undefined | null) => {
              this.flushBufferedLogs();
              this.snapshot.exitCode = typeof code === "number" ? code : null;
              this.snapshot.stoppedAt = new Date().toISOString();

              if (this.snapshot.state === "stopping") {
                this.snapshot.state = "idle";
                this.snapshot.detail = "Stopped.";
              } else if (code === 0 || code === null) {
                this.snapshot.state = "idle";
                this.snapshot.detail = "Exited cleanly.";
              } else {
                this.snapshot.state = "error";
                this.snapshot.detail = `Exited with code ${code}.`;
                pushLog(this.snapshot, "system", `${this.snapshot.label} exited with code ${code}.`);
              }

              this.channel = null;
              cleanupErrorHandler();
              client.end();
              this.client = null;
              this.exitPromise = null;
              exitResolve();
            });
          });

          channel.on("data", (chunk: Buffer) => {
            this.stdoutBuffer += chunk.toString("utf8");
            this.stdoutBuffer = this.consumeBuffer(this.stdoutBuffer, "stdout");
          });

          channel.stderr.on("data", (chunk: Buffer) => {
            this.stderrBuffer += chunk.toString("utf8");
            this.stderrBuffer = this.consumeBuffer(this.stderrBuffer, "stderr");
          });

          if (!settled) {
            settled = true;
            resolve();
          }
        });
      });

      client.on("error", handleError);
      client.once("close", cleanupErrorHandler);
      client.once("end", cleanupErrorHandler);
      client.connect(connection);
    });
  }

  async stop(reason = "Stopped by the UI."): Promise<void> {
    if (!this.client) {
      return;
    }

    this.snapshot.state = "stopping";
    this.snapshot.detail = reason;
    pushLog(this.snapshot, "system", reason);

    try {
      this.channel?.signal("INT");
    } catch {
      // The remote process may already be gone.
    }

    await Promise.race([this.exitPromise, sleep(5000)]);

    if (this.channel) {
      pushLog(this.snapshot, "system", "Closing the remote channel.");
      this.channel.close();
      this.client?.end();
      await this.exitPromise;
    }
  }

  private consumeBuffer(buffer: string, stream: "stdout" | "stderr"): string {
    const parts = buffer.split(/\r?\n/);
    const remainder = parts.pop() ?? "";
    for (const part of parts) {
      pushLog(this.snapshot, stream, part);
    }
    return remainder;
  }

  private flushBufferedLogs(): void {
    if (this.stdoutBuffer) {
      pushLog(this.snapshot, "stdout", this.stdoutBuffer);
      this.stdoutBuffer = "";
    }
    if (this.stderrBuffer) {
      pushLog(this.snapshot, "stderr", this.stderrBuffer);
      this.stderrBuffer = "";
    }
  }
}

export class TaskRunner {
  private snapshot: ServiceSnapshot;

  constructor(label: string) {
    this.snapshot = createSnapshot(label);
  }

  getSnapshot(): ServiceSnapshot {
    return cloneSnapshot(this.snapshot);
  }

  async run(
    mode: string,
    meta: Record<string, unknown>,
    task: (log: TaskLogger) => Promise<void>,
  ): Promise<void> {
    this.snapshot = {
      ...createSnapshot(this.snapshot.label),
      state: "starting",
      detail: "Starting task.",
      startedAt: new Date().toISOString(),
      mode,
      meta,
    };
    pushLog(this.snapshot, "system", `Starting ${this.snapshot.label}.`);

    const log: TaskLogger = (stream, line) => {
      pushLog(this.snapshot, stream, line);
    };

    try {
      this.snapshot.state = "running";
      this.snapshot.detail = "Running.";
      await task(log);
      this.snapshot.state = "idle";
      this.snapshot.detail = "Completed.";
    } catch (error) {
      this.snapshot.state = "error";
      this.snapshot.detail = error instanceof Error ? error.message : String(error);
      pushLog(this.snapshot, "stderr", this.snapshot.detail);
      throw error;
    } finally {
      this.snapshot.stoppedAt = new Date().toISOString();
    }
  }
}
