import net from "node:net";
import { execFile } from "node:child_process";
import { promisify } from "node:util";

import type { WifiStatus } from "./types.js";

const execFileAsync = promisify(execFile);

export function shellQuote(value: string): string {
  return `'${value.replace(/'/g, `'\\''`)}'`;
}

export function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export async function runZshScript(script: string): Promise<{ stdout: string; stderr: string }> {
  return execFileAsync("/bin/zsh", ["-lc", script], {
    maxBuffer: 1024 * 1024 * 8,
  });
}

export async function findWifiDevice(): Promise<string | null> {
  try {
    const { stdout } = await execFileAsync("/usr/sbin/networksetup", ["-listallhardwareports"]);
    const blocks = stdout.split(/\n\s*\n/);
    for (const block of blocks) {
      if (!block.includes("Hardware Port: Wi-Fi")) {
        continue;
      }

      const deviceMatch = block.match(/Device:\s*(\S+)/);
      if (deviceMatch) {
        return deviceMatch[1];
      }
    }
    return null;
  } catch {
    return null;
  }
}

export async function getWifiStatus(): Promise<WifiStatus> {
  const device = await findWifiDevice();
  if (!device) {
    return {
      device: null,
      ssid: null,
      connected: false,
      message: "Wi-Fi hardware was not detected on this Mac.",
    };
  }

  try {
    const { stdout } = await execFileAsync("/usr/sbin/networksetup", ["-getairportnetwork", device]);
    const currentMatch = stdout.match(/Current Wi-Fi Network:\s*(.+)/);
    if (currentMatch) {
      return {
        device,
        ssid: currentMatch[1].trim(),
        connected: true,
        message: `Connected on ${device}.`,
      };
    }

    return {
      device,
      ssid: null,
      connected: false,
      message: stdout.trim() || "Wi-Fi is not associated with a network.",
    };
  } catch (error) {
    return {
      device,
      ssid: null,
      connected: false,
      message: error instanceof Error ? error.message : "Wi-Fi status failed.",
    };
  }
}

export async function connectToWifi(ssid: string, password: string): Promise<WifiStatus> {
  const device = await findWifiDevice();
  if (!device) {
    throw new Error("Wi-Fi hardware was not detected on this Mac.");
  }

  await execFileAsync("/usr/sbin/networksetup", [
    "-setairportnetwork",
    device,
    ssid,
    password,
  ]);

  for (let attempt = 0; attempt < 12; attempt += 1) {
    const status = await getWifiStatus();
    if (status.ssid === ssid) {
      return status;
    }
    await sleep(1000);
  }

  throw new Error(`The Mac did not finish joining ${ssid}.`);
}

export async function isTcpReachable(
  host: string,
  port: number,
  timeoutMs = 1500,
): Promise<boolean> {
  return new Promise((resolve) => {
    const socket = net.createConnection({ host, port });

    const finish = (result: boolean) => {
      socket.removeAllListeners();
      socket.destroy();
      resolve(result);
    };

    socket.setTimeout(timeoutMs);
    socket.once("connect", () => finish(true));
    socket.once("timeout", () => finish(false));
    socket.once("error", () => finish(false));
  });
}
