import { createServer } from "node:http";
import { semanticBriefing } from "./briefing.js";
import { createModelAdapters } from "./models.js";
import { candidateOverlay } from "./parking/overlay.js";
import { ParkingSessionStore } from "./parking/session-store.js";
import { sanitizeHistory } from "./parking/history.js";
import { CANDIDATE_COUNT } from "./parking/config.js";
import { prepareRequest } from "./parking/jev-request.js";
import { angleError, distance, roundPose } from "./parking/math.js";
import { navigationMeasurement } from "./parking/navigation.js";
import { scenarioTasks } from "./scenarios.js";
import { isLegal, stopAvailability } from "./parking/policy.js";
import { sanitizePose } from "./parking/pose.js";
import { decisionSelection, expandAnswers } from "./parking/selection.js";
import { readFile, stat } from "node:fs/promises";
import { extname, join, normalize } from "node:path";
import { fileURLToPath } from "node:url";
import {
  SENSOR, actualCollision, perceivedTrackCollision, nearestOccupied,
} from "./perception.js";

const root = fileURLToPath(new URL(".", import.meta.url));
const publicDir = join(root, "public");
await loadEnv(join(root, ".env"));

const config = {
  port: Number(process.env.PORT || 4173),
  llmBaseUrl: (process.env.LLM_BASE_URL || "http://127.0.0.1:4000/v1").replace(/\/$/, ""),
  llmApiKey: process.env.LLM_API_KEY || "",
  llmModel: process.env.LLM_MODEL || "gpt-5.6-sol",
  jevBaseUrl: (process.env.JEV_BASE_URL || "https://api.typesafe.ai/v1").replace(/\/$/, ""),
  jevApiKey: process.env.JEV_API_KEY || "",
  jevModel: process.env.JEV_MODEL || "jev-latest",
};

const sessionStore = new ParkingSessionStore();
const modelAdapters = createModelAdapters(config);

const mimeTypes = { ".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".svg": "image/svg+xml", ".json": "application/json; charset=utf-8" };
const staticRoots = [
  { prefix: "/vendor/three/", directory: join(root, "node_modules", "three", "build") },
  { prefix: "/scenarios.js", directory: root },
  { prefix: "/", directory: publicDir },
];

const server = createServer(async (req, res) => {
  try {
    if (req.method === "GET" && req.url === "/api/health") {
      return sendJson(res, 200, {
        ok: true,
        protocol: "shared-control-v6",
        architecture: "sensors -> shared one-step control generation -> model chooses -> safety veto -> execute exact selected path",
        candidateCount: CANDIDATE_COUNT,
        perception: { rangeFinderRays: SENSOR.ring.rays, shortRangeSensors: SENSOR.ultrasonic.count, cellM: 0.1, decisionLatencyS: SENSOR.decisionLatencyS },
        engines: {
          jev: { configured: Boolean(config.jevApiKey), model: config.jevModel },
          llm: { configured: Boolean(config.llmApiKey), model: config.llmModel, baseUrl: config.llmBaseUrl },
        },
      });
    }

    if (req.method === "POST" && req.url === "/api/decision") {
      const body = await readJsonBody(req);
      const stream = String(req.headers.accept || "").includes("application/x-ndjson");
      const started = performance.now();
      const scenario = String(body.scenario || "offset-bay");
      const pose = sanitizePose(body.pose);
      const target = sanitizePose(body.target || { x: 0, y: -2.45, heading: 90 });
      const history = sanitizeHistory(body.history);

      // Perception and route memory are isolated from model policy and control.
      const context = sessionStore.perceive({
        engine: body.engine, scenario, pose, target, history, reset: body.reset === true, runId: body.runId,
      });
      const { session, navigation, stageHistory, analysis, recovery, candidates, environment, scene } = context;

      // JevPilot exposes a freshly generated candidate fan before asking Jev
      // to choose.  Stream that same, collision-checked table to the browser
      // immediately so model latency never leaves the car without live paths.
      if (stream) {
        res.writeHead(200, {
          "Content-Type": "application/x-ndjson; charset=utf-8",
          "Cache-Control": "no-store",
          "X-Content-Type-Options": "nosniff",
        });
        writeNdjson(res, {
          type: "candidates",
          pose: roundPose(pose),
          scene,
          navigation,
          candidateLines: candidateOverlay(candidates, null, null, null),
        });
      }

      const stop = stopAvailability(candidates, pose, target);
      // Jev is a typed-choice model: it gets questions over a factored candidate
      // table. The language model gets the same facts written out in prose.
      const briefing = body.engine === "llm"
        ? semanticBriefing({
            scenario, task: scenarioTasks[scenario], pose, target, navigation, analysis, recovery,
            candidates, environment,
          })
        : null;
      const prepared = prepareRequest({
        scenario, pose, target, navigation, history: stageHistory, analysis, candidates, stop, recovery,
        environment,
      });
      const adapter = modelAdapters[body.engine];
      if (!adapter) return sendDecisionError(res, stream, 400, "Unknown engine");
      prepared.request.model = adapter.model;

      // The adapter owns representation and API details; it only returns a choice.
      // A completed task can stop locally. Every moving trajectory, including
      // a singleton safe set, still goes through the selected engine so the
      // comparison never substitutes a local planner decision.
      const localOnly = prepared.fixed.motion?.choice === "stop";
      let answers = {};
      let engineResult = { engine: adapter.id, model: adapter.model, rationale: "Single legal command; resolved locally without a model call.", usage: {} };
      if (!localOnly) {
        engineResult = await adapter.decide({ prepared, briefing, candidates: prepared.eligible });
        answers = engineResult.answers;
      }
      const expanded = expandAnswers(prepared, answers);
      const selection = decisionSelection(candidates, expanded);
      if (!selection) {
        return sendDecisionError(res, stream, 502, engineResult.model + " returned no usable decision", engineResult.raw ?? null);
      }

      // ---- the safety layer may only refuse, never substitute --------------
      const requested = candidates[selection.choice];
      if (!requested) {
        return sendDecisionError(res, stream, 502, engineResult.model + " returned a stale or unknown candidate", selection.choice);
      }
      // Safety admission can tighten between candidate generation and model
      // return as the perception session advances. Treat an absent/stale choice
      // as a refusal, and preserve its rollout in the overlay when available.
      const blocked = !isLegal(requested);
      const selected = blocked ? candidates.stop : requested;
      const segmentConflict = perceivedTrackCollision(session.state, selected.path, selected.control.duration);
      // Hidden world state is used only as the simulator's collision assertion.
      // A perception miss must fail closed before the UI animates the command.
      const truthHit = actualCollision(pose, selected.path, selected.control.duration, session.state.timeS);
      const executionBlocked = blocked || Boolean(segmentConflict) || Boolean(truthHit);
      const executable = executionBlocked ? candidates.stop : selected;
      const interventionReason = blocked
        ? "chosen command would collide according to your own map; holding position and asking again"
        : segmentConflict
          ? "the chosen command would collide with a detected object; holding position and asking again"
          : truthHit
            ? "onboard perception missed a real obstacle; emergency stop before motion"
          : null;

      // ---- ground truth scores what actually happened ----------------------
      // Use ground truth only as a simulator assertion. A command that reaches
      // this point should already be safe according to onboard perception. If
      // perception missed an obstacle, do not animate the car into it. Stop at
      // the last safe sample and expose the miss as an actual collision.
      const hit = truthHit;
      const executedPath = executable.path;
      const executedPose = executedPath.at(-1) || roundPose(pose);
      const executedDuration = executable.control.duration;
      const executedControl = executable.control;
      sessionStore.commit(session, executedPose, executedDuration);

      const result = {
        engine: engineResult.engine,
        model: engineResult.model,
        rationale: engineResult.rationale,
        decisionInterface: adapter.interface,
        decisionSource: localOnly ? "only_eligible_action" : "model",
        latencyMs: Math.round(performance.now() - started),

        requestedAction: requested.action,
        requestedControl: requested.control,
        requestedPlan: requested.plan || null,
        action: executable.action,
        control: executedControl,
        projection: executedPose,
        path: executedPath,
        projectedDistance: distance(executedPose, target),
        projectedAngleError: angleError(executedPose.heading, target.heading),
        projectedNavigationDistance: navigationMeasurement(navigation, executedPose).remainingM,
        navigation,
        clearance: hit ? Math.min(Number(requested.clearance) || 0, -0.001) : executable.clearance,
        startClearance: typeof executable.startClearance === "number" ? executable.startClearance : null,
        collision: Boolean(hit),
        actualCollision: Boolean(hit),
        perceivedCollision: requested.collision,
        collisionWith: hit ? hit.objectId : null,
        collisionAtS: hit ? hit.timeS : null,

        blocked: executionBlocked,
        safetyIntervention: executionBlocked,
        interventionReason,
        dangerousRequest: executionBlocked,
        recovery,
        motion: selection.motion,
        confidence: selection.confidence,
        riskScore: engineResult.riskScore ?? null,
        probabilities: selection.probabilities,
        historyAnalysis: analysis,
        usage: engineResult.usage || {},

        segments: executable.segments || 1,
        segmentsDetail: executable.segmentsDetail || [],
        plan: executable.plan || null,
        proposedBy: null,
        pathM: Number(executedControl.travelDistance) || 0,

        perceivedConflict: segmentConflict?.id || requested.predictedConflict || null,
        perceivedConflictInS: segmentConflict?.inS ?? requested.predictedConflictInS ?? null,
        perceptionMiss: Boolean(hit && !blocked && !segmentConflict),
        unknownFraction: typeof executable.unknownFraction === "number" ? executable.unknownFraction : null,
        candidateOrigin: roundPose(pose),
        scene,
        // A rejected request remains visible as a rejected candidate.  It must
        // never receive the bright "executed" treatment in the operator view.
        candidateLines: ensureCandidateVisible(candidateOverlay(
          candidates,
          executionBlocked ? null : requested.action,
          selection.probabilities,
          executionBlocked ? requested.action : null,
        ), requested, executionBlocked),
        chosenAction: executionBlocked ? null : requested.action,
        executedAction: executable.action,
        perceptionDebug: nearestOccupied(session.state, pose),
      };
      if (stream) {
        writeNdjson(res, { type: "decision", data: result });
        return res.end();
      }
      return sendJson(res, 200, result);
    }

    if (req.method !== "GET" && req.method !== "HEAD") return sendJson(res, 405, { error: "Method not allowed" });
    await serveStatic(req, res);
  } catch (error) {
    console.error(error);
    if (res.headersSent) {
      writeNdjson(res, { type: "error", error: error.message || "Unexpected error" });
      return res.end();
    }
    sendJson(res, error.status || 500, { error: error.message || "Unexpected error" });
  }
});

function ensureCandidateVisible(lines, requested, blocked) {
  if (!requested || lines.some((line) => line.id === requested.action)) return lines;
  const selected = candidateOverlay({ [requested.action]: requested }, blocked ? null : requested.action, null, blocked ? requested.action : null);
  return [...lines, ...selected];
}

server.listen(config.port, "127.0.0.1", () => console.log("Parking Lab running at http://127.0.0.1:" + config.port));

async function loadEnv(path) {
  try {
    const source = await readFile(path, "utf8");
    for (const line of source.split(/\r?\n/)) {
      const trimmed = line.trim();
      if (!trimmed || trimmed.startsWith("#")) continue;
      const index = trimmed.indexOf("=");
      if (index < 1) continue;
      const key = trimmed.slice(0, index).trim();
      const value = trimmed.slice(index + 1).trim().replace(/^['"]|['"]$/g, "");
      if (!(key in process.env)) process.env[key] = value;
    }
  } catch (error) { if (error.code !== "ENOENT") throw error; }
}

async function readJsonBody(req) {
  let body = "";
  for await (const chunk of req) { body += chunk; if (body.length > 4_000_000) throw new Error("Request body too large"); }
  try { return JSON.parse(body || "{}"); } catch { const error = new Error("Invalid JSON body"); error.status = 400; throw error; }
}

async function serveStatic(req, res) {
  const url = new URL(req.url, "http://localhost");
  const pathname = url.pathname === "/" ? "/index.html" : url.pathname;
  const selectedRoot = staticRoots.find(({ prefix }) => pathname.startsWith(prefix));
  if (!selectedRoot) return sendJson(res, 404, { error: "Not found" });
  const relativePath = selectedRoot.prefix.endsWith(".js")
    ? selectedRoot.prefix.slice(1)
    : pathname.slice(selectedRoot.prefix.length);
  const safePath = normalize(relativePath).replace(/^(\.\.[/\\])+/, "");
  let filePath = join(selectedRoot.directory, safePath);
  if (!filePath.startsWith(selectedRoot.directory)) return sendJson(res, 403, { error: "Forbidden" });
  try {
    const info = await stat(filePath);
    if (info.isDirectory()) filePath = join(filePath, "index.html");
    const content = await readFile(filePath);
    res.writeHead(200, { "Content-Type": mimeTypes[extname(filePath)] || "application/octet-stream", "Cache-Control": "no-store" });
    if (req.method === "HEAD") return res.end();
    res.end(content);
  } catch (error) {
    if (error.code === "ENOENT") return sendJson(res, 404, { error: "Not found" });
    throw error;
  }
}

function sendJson(res, status, data) { if (res.headersSent) return; res.writeHead(status, { "Content-Type": "application/json; charset=utf-8", "Cache-Control": "no-store" }); res.end(JSON.stringify(data)); }
function writeNdjson(res, data) { if (!res.destroyed && !res.writableEnded) res.write(JSON.stringify(data) + "\n"); }
function sendDecisionError(res, stream, status, message, raw = null) {
  const error = { error: message, ...(raw == null ? {} : { raw }) };
  if (!stream) return sendJson(res, status, error);
  writeNdjson(res, { type: "error", ...error });
  return res.end();
}
