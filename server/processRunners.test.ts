import assert from "node:assert/strict";
import net from "node:net";
import test from "node:test";

import { RemoteProcessRunner } from "./processRunners.js";
import { isTransientRemoteTransportError } from "./transportErrors.js";

test("remote process runner rejects dropped SSH handshakes without an unhandled client error", async () => {
  const tcpServer = net.createServer((socket) => {
    socket.destroy();
  });

  await new Promise<void>((resolve) => {
    tcpServer.listen(0, "127.0.0.1", resolve);
  });

  const address = tcpServer.address();
  assert.ok(address && typeof address === "object");

  const runner = new RemoteProcessRunner("Dropped SSH handshake");
  try {
    await assert.rejects(
      runner.start(
        "true",
        {
          host: "127.0.0.1",
          port: address.port,
          username: "pi",
          password: "password",
          readyTimeout: 200,
          hostVerifier: () => true,
        },
        "test",
      ),
      (error) => error instanceof Error && isTransientRemoteTransportError(error),
    );
  } finally {
    await new Promise<void>((resolve, reject) => {
      tcpServer.close((error) => {
        if (error) {
          reject(error);
          return;
        }
        resolve();
      });
    });
  }
});
