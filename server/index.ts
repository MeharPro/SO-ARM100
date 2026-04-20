import express from "express";
import fs from "node:fs";
import path from "node:path";

import { RobotController } from "./robotController.js";
import type {
  AppSettings,
  BenchmarkPolicyRequest,
  CalibrationInputRequest,
  CreatePinnedMoveRequest,
  DeployTrainingCheckpointRequest,
  DeleteTrainingProfileRequest,
  RecordingDetailRequest,
  RenameRecordingRequest,
  ReplayRequest,
  SelectTrainingProfileRequest,
  StartPolicyEvalRequest,
  StartTrainingCaptureRequest,
  StartTrainingRunRequest,
  StartTrainingSyncRequest,
  ServoCalibrationRequest,
  TrainingProfile,
  TrimRecordingRequest,
  TorqueLimitsRequest,
} from "./types.js";

const app = express();
const controller = new RobotController();
const serverPort = Number(process.env.ROBOT_ARM_UI_PORT ?? 4318);
const rootDir = process.cwd();
const clientDir = path.join(rootDir, "dist", "client");

app.use(express.json({ limit: "1mb" }));

function asyncRoute(
  handler: (req: express.Request, res: express.Response) => Promise<void>,
) {
  return (req: express.Request, res: express.Response, next: express.NextFunction) => {
    void handler(req, res).catch(next);
  };
}

app.get(
  "/api/state",
  asyncRoute(async (_req, res) => {
    res.json(await controller.getState());
  }),
);

app.post(
  "/api/settings",
  asyncRoute(async (req, res) => {
    res.json(await controller.saveSettings(req.body as AppSettings));
  }),
);

app.post(
  "/api/robot/start-control",
  asyncRoute(async (_req, res) => {
    res.json(await controller.startControl());
  }),
);

app.post(
  "/api/robot/start-hotkeys",
  asyncRoute(async (_req, res) => {
    res.json(await controller.startKeyboardControl());
  }),
);

app.post(
  "/api/robot/stop-control",
  asyncRoute(async (_req, res) => {
    res.json(await controller.stopControl());
  }),
);

app.post(
  "/api/robot/emergency-stop",
  asyncRoute(async (_req, res) => {
    res.json(await controller.emergencyStop());
  }),
);

app.post(
  "/api/vex/telemetry/sync",
  asyncRoute(async (_req, res) => {
    res.json(await controller.syncVexTelemetryProgram());
  }),
);

app.post(
  "/api/robot/torque-limits",
  asyncRoute(async (req, res) => {
    res.json(await controller.setArmTorqueLimits(req.body as TorqueLimitsRequest));
  }),
);

app.post(
  "/api/calibration/pi/start",
  asyncRoute(async (_req, res) => {
    res.json(await controller.startPiCalibration());
  }),
);

app.post(
  "/api/calibration/pi/servo",
  asyncRoute(async (req, res) => {
    res.json(await controller.startPiServoCalibration(req.body as ServoCalibrationRequest));
  }),
);

app.post(
  "/api/calibration/pi/input",
  asyncRoute(async (req, res) => {
    const payload = req.body as CalibrationInputRequest;
    const input = payload.input === "c" ? "c" : "enter";
    res.json(await controller.sendPiCalibrationInput(input));
  }),
);

app.post(
  "/api/calibration/pi/stop",
  asyncRoute(async (_req, res) => {
    res.json(await controller.stopPiCalibration());
  }),
);

app.post(
  "/api/calibration/mac/start",
  asyncRoute(async (_req, res) => {
    res.json(await controller.startMacCalibration());
  }),
);

app.post(
  "/api/calibration/mac/input",
  asyncRoute(async (req, res) => {
    const payload = req.body as CalibrationInputRequest;
    const input = payload.input === "c" ? "c" : "enter";
    res.json(await controller.sendMacCalibrationInput(input));
  }),
);

app.post(
  "/api/calibration/mac/stop",
  asyncRoute(async (_req, res) => {
    res.json(await controller.stopMacCalibration());
  }),
);

app.post(
  "/api/recordings/start",
  asyncRoute(async (req, res) => {
    const label = typeof req.body?.label === "string" ? req.body.label : "";
    res.json(await controller.startRecording(label));
  }),
);

app.post(
  "/api/recordings/stop",
  asyncRoute(async (_req, res) => {
    res.json(await controller.stopRecording());
  }),
);

app.post(
  "/api/recordings/refresh",
  asyncRoute(async (_req, res) => {
    res.json(await controller.refreshRecordings());
  }),
);

app.post(
  "/api/recordings/detail",
  asyncRoute(async (req, res) => {
    res.json(await controller.getRecordingDetail(req.body as RecordingDetailRequest));
  }),
);

app.post(
  "/api/recordings/rename",
  asyncRoute(async (req, res) => {
    res.json(await controller.renameRecording(req.body as RenameRecordingRequest));
  }),
);

app.post(
  "/api/recordings/trim",
  asyncRoute(async (req, res) => {
    res.json(await controller.trimRecording(req.body as TrimRecordingRequest));
  }),
);

app.post(
  "/api/replays/start",
  asyncRoute(async (req, res) => {
    res.json(await controller.startReplay(req.body as ReplayRequest));
  }),
);

app.post(
  "/api/replays/stop",
  asyncRoute(async (_req, res) => {
    res.json(await controller.stopReplay());
  }),
);

app.post(
  "/api/pinned-moves",
  asyncRoute(async (req, res) => {
    res.json(await controller.createPinnedMove(req.body as CreatePinnedMoveRequest));
  }),
);

app.delete(
  "/api/pinned-moves/:id",
  asyncRoute(async (req, res) => {
    const id = Array.isArray(req.params.id) ? req.params.id[0] : req.params.id;
    res.json(await controller.deletePinnedMove(id));
  }),
);

app.post(
  "/api/pinned-moves/:id/trigger",
  asyncRoute(async (req, res) => {
    const id = Array.isArray(req.params.id) ? req.params.id[0] : req.params.id;
    res.json(await controller.triggerPinnedMove(id));
  }),
);

app.post(
  "/api/training/profiles",
  asyncRoute(async (req, res) => {
    res.json(await controller.saveTrainingProfile(req.body as TrainingProfile));
  }),
);

app.post(
  "/api/training/profiles/select",
  asyncRoute(async (req, res) => {
    res.json(await controller.selectTrainingProfile(req.body as SelectTrainingProfileRequest));
  }),
);

app.delete(
  "/api/training/profiles/:id",
  asyncRoute(async (req, res) => {
    const id = Array.isArray(req.params.id) ? req.params.id[0] : req.params.id;
    res.json(await controller.deleteTrainingProfile({ id } as DeleteTrainingProfileRequest));
  }),
);

app.post(
  "/api/training/capture/start",
  asyncRoute(async (req, res) => {
    res.json(await controller.startTrainingCapture(req.body as StartTrainingCaptureRequest));
  }),
);

app.post(
  "/api/training/capture/stop",
  asyncRoute(async (_req, res) => {
    res.json(await controller.stopTrainingCapture());
  }),
);

app.post(
  "/api/training/sync/start",
  asyncRoute(async (req, res) => {
    res.json(await controller.startTrainingSync(req.body as StartTrainingSyncRequest));
  }),
);

app.post(
  "/api/training/run/start",
  asyncRoute(async (req, res) => {
    res.json(await controller.startTrainingRun(req.body as StartTrainingRunRequest));
  }),
);

app.post(
  "/api/training/run/stop",
  asyncRoute(async (_req, res) => {
    res.json(await controller.stopTrainingRun());
  }),
);

app.post(
  "/api/training/deploy",
  asyncRoute(async (req, res) => {
    res.json(await controller.deployTrainingCheckpoint(req.body as DeployTrainingCheckpointRequest));
  }),
);

app.post(
  "/api/training/benchmark",
  asyncRoute(async (req, res) => {
    res.json(await controller.runPolicyBenchmark(req.body as BenchmarkPolicyRequest));
  }),
);

app.post(
  "/api/training/eval/start",
  asyncRoute(async (req, res) => {
    res.json(await controller.startPolicyEval(req.body as StartPolicyEvalRequest));
  }),
);

app.post(
  "/api/training/eval/stop",
  asyncRoute(async (_req, res) => {
    res.json(await controller.stopPolicyEval());
  }),
);

if (fs.existsSync(path.join(clientDir, "index.html"))) {
  app.use(express.static(clientDir));
  app.get("*", (_req, res) => {
    res.sendFile(path.join(clientDir, "index.html"));
  });
}

app.use(
  (
    error: unknown,
    _req: express.Request,
    res: express.Response,
    _next: express.NextFunction,
  ) => {
    const message = error instanceof Error ? error.message : "Unknown server error.";
    res.status(500).json({ error: message });
  },
);

app.listen(serverPort, () => {
  console.log(`Robot Arm UI backend listening on http://localhost:${serverPort}`);
});
