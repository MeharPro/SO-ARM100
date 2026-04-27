const TRANSIENT_REMOTE_TRANSPORT_CODES = new Set([
  "econnreset",
  "econnaborted",
  "epipe",
  "etimedout",
]);

const TRANSIENT_REMOTE_TRANSPORT_MESSAGES = [
  "econnreset",
  "econnaborted",
  "epipe",
  "etimedout",
  "connection reset",
  "socket hang up",
  "socket closed",
  "client-socket disconnected",
  "no response from server",
  "connection lost before handshake",
  "timed out while waiting for handshake",
];

function errorCode(error: unknown): string {
  if (!error || typeof error !== "object" || !("code" in error)) {
    return "";
  }
  const code = (error as { code?: unknown }).code;
  return typeof code === "string" ? code.toLowerCase() : "";
}

export function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

export function isTransientRemoteTransportError(error: unknown): boolean {
  const code = errorCode(error);
  if (TRANSIENT_REMOTE_TRANSPORT_CODES.has(code)) {
    return true;
  }

  const message = errorMessage(error).toLowerCase();
  return TRANSIENT_REMOTE_TRANSPORT_MESSAGES.some((candidate) => message.includes(candidate));
}
