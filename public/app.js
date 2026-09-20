import { isParked } from "./parking-goal.js";
import { createParkingScene } from "./parking3d.js";
import { scenarios } from "../scenarios.js";
import { DEFAULT_LANGUAGE, LANGUAGE_STORAGE_KEY, normalizeLanguage, translate } from "./i18n.js";

const select = document.querySelector("#scenario-select");
const runButton = document.querySelector("#run-button");
const resetButton = document.querySelector("#reset-button");
const customEditorHint = document.querySelector("#custom-editor-hint");
const customPoseLabel = document.querySelector("#custom-pose");
const toast = document.querySelector("#toast");
const engines = {
  jev: createEngineState("jev"),
  llm: createEngineState("llm"),
};
const scenes3d = {
  jev: createParkingScene(document.querySelector("#jev-scene"), { accent: 0xa7f542 }),
  llm: createParkingScene(document.querySelector("#llm-scene"), { accent: 0x75a7ff }),
};
let elapsedTimer = null;
let toastTimer = null;
let customStart = { ...scenarios.custom.start };
let language = normalizeLanguage(localStorage.getItem(LANGUAGE_STORAGE_KEY) || DEFAULT_LANGUAGE);

const metricLabels = {
  elapsed: "metric.elapsed", average: "metric.average", moves: "metric.decisions",
  path: "metric.path", shifts: "metric.shifts", steering: "metric.steering",
  clearance: "metric.clearance", dangerous: "metric.blocked",
  oscillations: "metric.oscillations", recoveries: "metric.recoveries",
  detected: "metric.detected", mapped: "metric.mapped", options: "metric.options",
};

init();

async function init() {
  bindControls();
  applyTranslations();
  resetSimulation();
  try {
    const response = await fetch("/api/health", { cache: "no-store" });
    const health = await response.json();
    if (!response.ok) throw new Error(health.error || "Health check failed");
    for (const key of Object.keys(engines)) {
      const data = health.engines?.[key];
      if (data?.model) document.querySelector(`#${key}-model`).textContent = data.model;
      if (!data?.configured) setStatus(key, "error", "status.notConfigured");
    }
  } catch {
    showToast(t("toast.disconnected"));
  }
}

function bindControls() {
  document.querySelectorAll(".language-switch button").forEach((button) => {
    button.addEventListener("click", () => setLanguage(button.dataset.language));
  });
  runButton.addEventListener("click", () => {
    if (anyEngineRunning()) stopAll();
    else {
      resetSimulation();
      startEngine("jev");
      startEngine("llm");
    }
  });
  for (const key of Object.keys(engines)) {
    document.querySelector(`#${key}-run-button`).addEventListener("click", () => {
      if (engines[key].running) stopEngine(key, "status.stopped");
      else startEngine(key);
    });
  }
  resetButton.addEventListener("click", resetSimulation);
  select.addEventListener("change", resetSimulation);
}

function startEngine(key) {
  const previous = engines[key];
  if (previous.running) return;
  if (previous.parked || previous.error) resetEngine(key);
  const engine = engines[key];
  engine.running = true;
  engine.runToken += 1;
  engine.startedAt = performance.now() - engine.elapsedMs;
  engine.error = null;
  if (isParked(engine.pose, engine.target)) { finishEngine(key, "parked"); return; }
  setStatus(key, "thinking", "status.move", { move: engine.moves + 1 });
  updateRunButtons();
  ensureElapsedTimer();
  runEngine(key, engine.runToken);
}

function stopEngine(key, statusKey = "status.stopped") {
  const engine = engines[key];
  if (engine.running) engine.elapsedMs = performance.now() - engine.startedAt;
  engine.running = false;
  engine.runToken += 1;
  engine.abortController?.abort();
  engine.abortController = null;
  scenes3d[key].stopDrive();
  engine.pose = scenes3d[key].getPose();
  if (!engine.parked && !engine.error) setStatus(key, "ready", statusKey);
  renderEngine(key, false);
  updateRunButtons();
  stopElapsedTimerIfIdle();
}

function stopAll() {
  stopEngine("jev", "status.stopped");
  stopEngine("llm", "status.stopped");
}

async function runEngine(key, token) {
  const engine = engines[key];
  let pendingDecision = null;
  try {
    while (isCurrentRun(key, token)) {
      if (!pendingDecision) {
        setStatus(key, "thinking", "status.move", { move: engine.moves + 1 });
        pendingDecision = requestDecision(key, engine, engine.pose, token);
      }
      const data = await pendingDecision;
      pendingDecision = null;
      if (!isCurrentRun(key, token)) return;

      recordDecision(engine, data);
      document.querySelector(`#${key}-model`).textContent = data.model || engine.model;
      renderEngine(key, false);
      addTrace(key, data, engine.moves);

      scenes3d[key].setScene(data.scene, []);
      applyCandidateBatch(key, engine, {
        pose: data.candidateOrigin || engine.pose,
        scene: data.scene,
        candidateLines: data.candidateLines,
      });
      if (data.actualCollision) {
        engine.error = t("action.emergency", { obstacle: data.collisionWith || t("action.unknownObstacle") });
        finishEngine(key, "collision");
        document.querySelector("#" + key + "-action").textContent = engine.error;
        showToast((key === "jev" ? "Jev" : "LLM") + ": " + engine.error);
        return;
      }

      // Receding-horizon control: the next model call starts from the validated
      // endpoint while the current control is still being animated. If it is
      // ready before this window ends, it takes over immediately. A slow model
      // remains visible as a safe pause instead of being hidden by a planner.
      const reachesTarget = isParked(data.projection, engine.target);
      if (!reachesTarget && !data.blocked) {
        pendingDecision = requestDecision(key, engine, data.projection, token);
      }
      engine.driving = true;
      const completed = await scenes3d[key].drivePath(
        data.path,
        data.control,
        () => !isCurrentRun(key, token),
      );
      engine.driving = false;
      if (engine.pendingPreview) {
        applyCandidateBatch(key, engine, engine.pendingPreview);
        engine.pendingPreview = null;
      }
      if (!completed || !isCurrentRun(key, token)) return;

      engine.pose = { ...data.projection };
      renderEngine(key, false);
      if (isParked(engine.pose, engine.target)) {
        finishEngine(key, "parked");
        return;
      }
      if (!pendingDecision) {
        setStatus(key, "thinking", "status.move", { move: engine.moves + 1 });
      }
    }
  } catch (error) {
    engine.abortController = null;
    if (error.name === "AbortError" || !isCurrentRun(key, token)) return;
    engine.error = error.message;
    finishEngine(key, "error");
    document.querySelector(`#${key}-action`).textContent = error.message;
    showToast(`${key === "jev" ? "Jev" : "LLM"}: ${error.message}`);
  }
}

async function requestDecision(key, engine, pose, token) {
  const controller = new AbortController();
  engine.abortController = controller;
  const response = await fetch("/api/decision", {
    method: "POST",
    headers: { "Content-Type": "application/json", Accept: "application/x-ndjson" },
    signal: controller.signal,
    body: JSON.stringify({
      engine: key,
      scenario: select.value,
      pose,
      target: engine.target,
      history: engine.history.slice(-10),
      runId: engine.runToken,
      // A browser reset starts a fresh perception/navigation session too.
      // Without this, a second run could inherit the previous run's route stage
      // when its first pose happened to match the server's expected pose.
      reset: engine.moves === 0,
    }),
  });
  if (!response.ok) {
    const data = await response.json().catch(() => ({}));
    if (engine.abortController === controller) engine.abortController = null;
    throw new Error(data.error || `Request failed (${response.status})`);
  }
  if (!response.body) throw new Error("Decision stream is unavailable");

  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let buffer = "";
  let decision = null;
  const consume = (line) => {
    if (!line.trim()) return;
    const event = JSON.parse(line);
    if (event.type === "error") throw new Error(event.error || "Decision failed");
    if (event.type === "candidates" && isCurrentRun(key, token)) {
      if (engine.driving) engine.pendingPreview = event;
      else applyCandidateBatch(key, engine, event);
    }
    if (event.type === "decision") decision = event.data;
  };
  try {
    while (true) {
      const { value, done } = await reader.read();
      buffer += decoder.decode(value || new Uint8Array(), { stream: !done });
      const lines = buffer.split("\n");
      buffer = lines.pop() || "";
      for (const line of lines) consume(line);
      if (done) break;
    }
    consume(buffer);
  } finally {
    if (engine.abortController === controller) engine.abortController = null;
  }
  if (!decision) throw new Error("Decision stream ended without a decision");
  return decision;
}

function applyCandidateBatch(key, engine, preview) {
  if (preview.scene) scenes3d[key].setScene(preview.scene, []);
  scenes3d[key].setCandidates(preview.candidateLines || [], preview.pose || engine.pose);
  renderEngine(key, false);
}

function recordDecision(engine, data) {
  const control = normalizeControl(data.control);
  const requestedControl = data.requestedControl ? normalizeControl(data.requestedControl) : null;
  const previousGear = engine.lastMotionGear;
  const previousSteer = engine.lastSteerDeg;

  engine.model = data.model || engine.model;
  engine.latencies.push(Number(data.latencyMs) || 0);
  engine.moves += 1;
  engine.lastDecision = data;
  engine.totalDistance += Number(data.pathM) || control.travelDistance;
  engine.steeringChange += Math.abs(control.steerDeg - previousSteer);
  engine.lastSteerDeg = control.steerDeg;
  if (control.gear === "forward" || control.gear === "reverse") {
    if (previousGear && previousGear !== control.gear) engine.gearChanges += 1;
    engine.lastMotionGear = control.gear;
  }
  if (Number.isFinite(Number(data.clearance))) {
    engine.minClearance = Math.min(engine.minClearance, Number(data.clearance));
  }
  if (data.blocked) engine.blockedCommands += 1;
  if (data.recovery?.active) engine.recoveries += 1;
  const oscillating = Boolean(data.historyAnalysis?.oscillating);
  if (oscillating && !engine.wasOscillating) engine.oscillations += 1;
  engine.wasOscillating = oscillating;

  engine.history.push({
    action: data.action,
    requestedAction: data.requestedAction,
    distance: Number(data.projectedDistance),
    navigationDistance: Number(data.projectedNavigationDistance),
    navigationStage: data.navigation?.stageId || null,
    navigationStageIndex: data.navigation?.stageIndex ?? 0,
    angleError: Number(data.projectedAngleError),
    pose: { ...data.projection },
    control,
    clearance: data.clearance,
    blocked: Boolean(data.blocked),
  });
}

function finishEngine(key, outcome) {
  const engine = engines[key];
  engine.elapsedMs = performance.now() - engine.startedAt;
  engine.running = false;
  engine.abortController = null;
  if (outcome === "parked") {
    engine.parked = true;
    setStatus(key, "parked", "status.parked");
  } else if (outcome === "collision") {
    setStatus(key, "error", "status.collision");
  } else {
    setStatus(key, "error", "status.apiError");
  }
  renderEngine(key, false);
  updateRunButtons();
  stopElapsedTimerIfIdle();
}

function resetSimulation() {
  for (const key of Object.keys(engines)) {
    const engine = engines[key];
    engine.abortController?.abort();
    engine.running = false;
    engine.runToken += 1;
    scenes3d[key].stopDrive();
  }
  const scenario = scenarios[select.value];
  const custom = Boolean(scenario.editable);
  customEditorHint.hidden = !custom;
  for (const key of Object.keys(scenes3d)) {
    scenes3d[key].setEditor(custom, (pose) => updateCustomStart(key, pose));
  }
  renderScenarioText();
  for (const key of Object.keys(engines)) resetEngine(key);
  updateRunButtons();
  stopElapsedTimerIfIdle();
}

function resetEngine(key) {
  const oldToken = engines[key].runToken + 1;
  engines[key].abortController?.abort();
  scenes3d[key].stopDrive();
  const scenario = scenarios[select.value];
  engines[key] = createEngineState(key);
  engines[key].runToken = oldToken;
  engines[key].pose = { ...(scenario.editable ? customStart : scenario.start) };
  engines[key].target = { ...scenario.target };
  scenes3d[key].setPose(engines[key].pose);
  scenes3d[key].setScene({ cells: [], tracks: [] }, []);
  scenes3d[key].setCandidates([]);

  const trace = document.querySelector(`#${key}-trace`);
  trace.innerHTML = `<li class="trace-empty">${escapeHtml(t("trace.empty"))}</li>`;
  document.querySelector(`#${key}-parked`).classList.remove("visible");
  document.querySelector(`#${key}-action`).textContent = t("action.waiting");
  document.querySelector(`#${key}-confidence-label`).textContent = "—";
  setStatus(key, "ready", "status.ready");
  renderEngine(key, false);
  if (scenario.editable) updateCustomPoseLabel();
}

function updateCustomStart(sourceKey, pose) {
  if (!scenarios[select.value]?.editable || anyEngineRunning()) return;
  customStart = {
    x: Math.round(Number(pose.x) * 100) / 100,
    y: Math.round(Number(pose.y) * 100) / 100,
    heading: Math.round(Number(pose.heading) * 10) / 10,
  };
  for (const key of Object.keys(engines)) {
    engines[key].pose = { ...customStart };
    engines[key].target = { ...scenarios.custom.target };
    if (key !== sourceKey) scenes3d[key].setPose(customStart);
    scenes3d[key].setCandidates([]);
    renderEngine(key, false);
  }
  updateCustomPoseLabel();
}

function updateCustomPoseLabel() {
  customPoseLabel.textContent = "x " + customStart.x.toFixed(2)
    + " · y " + customStart.y.toFixed(2)
    + " · " + t("controls.heading") + " " + customStart.heading.toFixed(1) + "°";
}

function renderEngine(key, syncScene = false) {
  const engine = engines[key];
  if (syncScene) scenes3d[key].setPose(engine.pose);
  const dist = distance(engine.pose, engine.target);
  const alignment = angleError(engine.pose.heading, engine.target.heading);
  const average = engine.latencies.length
    ? engine.latencies.reduce((sum, value) => sum + value, 0) / engine.latencies.length
    : 0;

  setMetric(key, "distance", `${dist.toFixed(2)}<small>m</small>`);
  setMetric(key, "moves", engine.moves);
  setMetric(key, "path", `${engine.totalDistance.toFixed(2)}<small>m</small>`);
  setMetric(key, "shifts", engine.gearChanges);
  setMetric(key, "steering", `${Math.round(engine.steeringChange)}<small>°</small>`);
  setMetric(key, "clearance", Number.isFinite(engine.minClearance) ? `${engine.minClearance.toFixed(2)}<small>m</small>` : "—");
  setMetric(key, "dangerous", engine.blockedCommands);
  setMetric(key, "oscillations", engine.oscillations);
  setMetric(key, "recoveries", engine.recoveries);
  setMetric(key, "detected", (engine.lastDecision?.scene?.tracks || []).length);
  setMetric(key, "mapped", (engine.lastDecision?.scene?.cells || []).length);
  setMetric(key, "options", (document.querySelector("#" + key + "-scene") || { dataset: {} }).dataset.candidateLines || 0);
  setMetric(key, "average", average ? `${(average / 1000).toFixed(2)}<small>s</small>` : "—");
  document.querySelector(`#${key}-alignment`).textContent = t("footer.alignment", { value: alignment.toFixed(1) });

  if (engine.lastDecision) {
    const data = engine.lastDecision;
    const tags = [data.recovery?.active ? t("action.recovery") : null].filter(Boolean);
    document.querySelector(`#${key}-action`).textContent = data.blocked
      ? t("action.blocked", { plan: requestedPlan(data) }) + (tags.length ? " · " + tags.join(" · ") : "")
      : t("action.executed", { plan: formatPlan(data) }) + (tags.length ? " · " + tags.join(" · ") : "");
    const confidence = Number.isFinite(Number(data.confidence)) ? t("confidence.value", { value: (Number(data.confidence) * 100).toFixed(0) }) : t("confidence.none");
    const risk = Number.isFinite(Number(data.riskScore)) ? t("risk.value", { value: Number(data.riskScore).toFixed(2) }) : t("risk.none");
    document.querySelector(`#${key}-confidence-label`).textContent = `${confidence} · ${risk}`;
    document.querySelector(`#${key}-last`).textContent = t("footer.last", { value: `${(Number(data.latencyMs || 0) / 1000).toFixed(2)}s` });
  } else {
    document.querySelector(`#${key}-action`).textContent = engine.error || t("action.waiting");
    document.querySelector(`#${key}-confidence-label`).textContent = "—";
    document.querySelector(`#${key}-last`).textContent = t("footer.last", { value: "—" });
  }
  document.querySelector(`#${key}-parked`).classList.toggle("visible", engine.parked);
  updateElapsedMetric(key);
}

function addTrace(key, data, move) {
  engines[key].traceItems.push({ data, move });
  renderTraceItem(key, data, move);
}

function renderTraceItem(key, data, move) {
  const trace = document.querySelector(`#${key}-trace`);
  trace.querySelector(".trace-empty")?.remove();
  const item = document.createElement("li");
  if (data.safetyIntervention) item.classList.add("intervened");
  const command = data.blocked ? requestedPlan(data) + " → held" : formatPlan(data);
  const rationale = [
    data.blocked ? t("trace.blocked", { reason: data.interventionReason || t("trace.illegal") }) : null,
    data.recovery?.active ? t("action.recovery") : null,
    data.rationale || "",
  ].filter(Boolean).join(" · ");
  item.innerHTML = `<span>#${String(move).padStart(2, "0")}</span><strong>${escapeHtml(command)}</strong><em>${escapeHtml(rationale)}</em><b>${(Number(data.latencyMs || 0) / 1000).toFixed(2)}s</b>`;
  trace.append(item);
  trace.scrollTop = trace.scrollHeight;
}

function ensureElapsedTimer() {
  if (elapsedTimer) return;
  elapsedTimer = setInterval(() => {
    for (const key of Object.keys(engines)) updateElapsedMetric(key);
  }, 100);
}

function stopElapsedTimerIfIdle() {
  if (anyEngineRunning() || !elapsedTimer) return;
  clearInterval(elapsedTimer);
  elapsedTimer = null;
}

function updateElapsedMetric(key) {
  const engine = engines[key];
  const elapsed = engine.running ? performance.now() - engine.startedAt : engine.elapsedMs;
  document.querySelector(`#${key}-elapsed`).innerHTML = `${(elapsed / 1000).toFixed(1)}<small>s</small>`;
}

function updateRunButtons() {
  for (const key of Object.keys(engines)) {
    const button = document.querySelector(`#${key}-run-button`);
    const running = engines[key].running;
    button.classList.toggle("running", running);
    button.textContent = t(running ? "controls.stopEngine" : "controls.runEngine", { name: displayName(key) });
  }
  const active = anyEngineRunning();
  runButton.classList.toggle("running", active);
  runButton.querySelector(".run-icon").textContent = active ? "■" : "▶";
  runButton.querySelector(".run-text").textContent = t(active ? "controls.stopAll" : "controls.startBoth");
  const editable = Boolean(scenarios[select.value]?.editable) && !active;
  for (const key of Object.keys(scenes3d)) {
    scenes3d[key].setEditor(editable, (pose) => updateCustomStart(key, pose));
  }
}

function setStatus(key, type, statusKey, params = {}) {
  const node = document.querySelector(`#${key}-status`);
  engines[key].status = { type, key: statusKey, params };
  node.className = `status ${type}`;
  node.innerHTML = `<i></i> ${escapeHtml(t(statusKey, params))}`;
}

function setMetric(key, name, html) {
  document.querySelector(`#${key}-${name}`).innerHTML = html;
}

function createEngineState(key) {
  return {
    key,
    model: key === "jev" ? "jev-latest" : "gpt-5.6-sol",
    running: false,
    runToken: 0,
    abortController: null,
    startedAt: 0,
    elapsedMs: 0,
    pose: { x: 0, y: 0, heading: 0 },
    target: { x: 0, y: 0, heading: 0 },
    moves: 0,
    history: [],
    lastDecision: null,
    parked: false,
    error: null,
    latencies: [],
    totalDistance: 0,
    gearChanges: 0,
    steeringChange: 0,
    lastMotionGear: null,
    lastSteerDeg: 0,
    minClearance: Infinity,
    blockedCommands: 0,
    recoveries: 0,
    oscillations: 0,
    wasOscillating: false,
    driving: false,
    pendingPreview: null,
    status: { type: "ready", key: "status.ready", params: {} },
    traceItems: [],
  };
}

function normalizeControl(control = {}) {
  return {
    gear: String(control.gear || "park"),
    direction: Number(control.direction) || 0,
    steerDeg: Number(control.steerDeg) || 0,
    targetSpeed: Math.max(0, Number(control.targetSpeed) || Math.abs(Number(control.speed)) || 0),
    validityS: Math.max(0, Number(control.validityS) || Number(control.duration) || 0),
    travelDistance: Math.max(0, Number(control.travelDistance) || 0),
    speed: Number(control.speed) || 0,
    duration: Math.max(0, Number(control.duration) || 0),
    steeringProfile: Array.isArray(control.steeringProfile) ? control.steeringProfile : [],
  };
}

function formatPlan(data) {
  return data.control ? formatControl(data.control) : (data.plan || t("action.invalid"));
}

// The model's own request, shown even when the safety layer refused to run it.
function requestedPlan(data) {
  return data.requestedControl ? formatControl(data.requestedControl) : (data.requestedPlan || t("action.invalid"));
}

function formatControl(control) {
  if (!control) return t("action.invalid");
  const value = normalizeControl(control);
  if (value.steeringProfile.length > 1) {
    return value.steeringProfile.map((segment) => {
      const gear = Number(segment.direction) < 0 ? "R" : "D";
      return `${gear}${signed(segment.steerDeg)}°@${Number(segment.targetSpeed).toFixed(2)}:${Number(segment.duration).toFixed(2)}s`;
    }).join("; " );
  }
  const gear = value.gear === "reverse" ? "R" : value.gear === "forward" ? "D" : "P";
  return `${gear} · ${t("action.steer")} ${signed(value.steerDeg)}° · ${value.targetSpeed.toFixed(2)}m/s · ${value.validityS.toFixed(2)}s`;
}

function signed(value) {
  const number = Number(value) || 0;
  return number > 0 ? `+${number.toFixed(0)}` : number.toFixed(0);
}

function t(key, params) {
  return translate(language, key, params);
}

function scenarioText(scenario) {
  return scenario.ui?.[language] || scenario.ui?.en || { label: scenario.label, description: scenario.description };
}

function renderScenarioText() {
  const scenario = scenarios[select.value];
  const copy = scenarioText(scenario);
  document.querySelectorAll(".scenario-name").forEach((node) => { node.textContent = copy.label.toUpperCase(); });
  document.querySelector("#scenario-description").textContent = copy.description;
  for (const option of select.options) option.textContent = scenarioText(scenarios[option.value]).label;
}

function setLanguage(nextLanguage) {
  language = normalizeLanguage(nextLanguage);
  localStorage.setItem(LANGUAGE_STORAGE_KEY, language);
  applyTranslations();
}

function applyTranslations() {
  document.documentElement.lang = language === "zh" ? "zh-CN" : "en";
  document.body.dataset.language = language;
  document.querySelectorAll("[data-i18n]").forEach((node) => { node.textContent = t(node.dataset.i18n); });
  document.querySelectorAll("[data-i18n-html]").forEach((node) => { node.innerHTML = t(node.dataset.i18nHtml); });
  document.querySelectorAll("[data-i18n-aria]").forEach((node) => { node.setAttribute("aria-label", t(node.dataset.i18nAria)); });
  document.querySelectorAll("[data-i18n-title]").forEach((node) => { node.title = t(node.dataset.i18nTitle); });
  document.querySelectorAll("[data-i18n-content]").forEach((node) => { node.content = t(node.dataset.i18nContent); });
  document.querySelectorAll("[data-engine-aria]").forEach((node) => { node.setAttribute("aria-label", t("scene.aria", { name: node.dataset.engineAria })); });
  document.querySelectorAll("[data-engine-trace]").forEach((node) => { node.setAttribute("aria-label", t("trace.aria", { name: node.dataset.engineTrace })); });
  document.querySelectorAll(".language-switch button").forEach((button) => {
    const active = button.dataset.language === language;
    button.classList.toggle("active", active);
    button.setAttribute("aria-pressed", String(active));
  });
  for (const [name, key] of Object.entries(metricLabels)) {
    document.querySelectorAll(`[id$="-${name}"]`).forEach((value) => {
      value.closest("div")?.querySelector(":scope > span")?.replaceChildren(t(key));
    });
  }
  renderScenarioText();
  if (scenarios[select.value]?.editable) updateCustomPoseLabel();
  for (const key of Object.keys(engines)) {
    const engine = engines[key];
    setStatus(key, engine.status.type, engine.status.key, engine.status.params);
    const trace = document.querySelector(`#${key}-trace`);
    trace.innerHTML = engine.traceItems.length ? "" : `<li class="trace-empty">${escapeHtml(t("trace.empty"))}</li>`;
    for (const item of engine.traceItems) renderTraceItem(key, item.data, item.move);
    renderEngine(key, false);
  }
  updateRunButtons();
}

function isCurrentRun(key, token) {
  return engines[key].running && engines[key].runToken === token;
}
function anyEngineRunning() { return Object.values(engines).some((engine) => engine.running); }
function displayName(key) { return key === "jev" ? "Jev" : "LLM"; }
function distance(a, b) { return Math.hypot(b.x - a.x, b.y - a.y); }
function angleError(a, b) { let value = (b - a) % 360; if (value > 180) value -= 360; if (value < -180) value += 360; return Math.abs(value); }
function escapeHtml(value) { const node = document.createElement("span"); node.textContent = String(value); return node.innerHTML; }
function showToast(message) { clearTimeout(toastTimer); toast.textContent = message; toast.classList.add("visible"); toastTimer = setTimeout(() => toast.classList.remove("visible"), 5000); }
