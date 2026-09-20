import { replyInstruction } from "./briefing.js";
import { clamp, round } from "./parking/math.js";

export function createModelAdapters(config) {
  return {
    jev: {
      id: "jev",
      model: config.jevModel,
      interface: "typed choice questions over a factored candidate table",
      decide: (input) => decideWithJev(config, input.prepared),
    },
    llm: {
      id: "llm",
      model: config.llmModel,
      interface: "semantic prose briefing",
      decide: (input) => decideWithLlm(config, input.briefing, input.candidates),
    },
  };
}

async function decideWithJev(config, prepared) {
  const response = await fetchJson(config.jevBaseUrl + "/systemone", {
    headers: authHeaders(config.jevApiKey),
    body: prepared.request,
  });
  return {
    engine: "jev", model: response.model || config.jevModel, answers: response.answers || {},
    confidence: response.answers?.motion?.confidence ?? null,
    riskScore: response.answers?.collision_risk?.score ?? null,
    rationale: "Typed Choice over the shared candidate table.", usage: response.usage || {}, raw: response,
  };
}

async function decideWithLlm(config, briefing, candidates) {
  const { text, order } = briefing;
  const payload = {
    model: config.llmModel,
    messages: [
      { role: "system", content: "You are an experienced driver parking a car. Reason in plain language about the situation, then answer with one JSON object." },
      { role: "user", content: [text, "", replyInstruction()].join("\n") },
    ],
    response_format: { type: "json_object" },
  };
  let response;
  try {
    response = await fetchJson(config.llmBaseUrl + "/chat/completions", { headers: authHeaders(config.llmApiKey), body: payload, timeoutMs: 120000 });
  } catch (error) {
    if (!/response_format|unsupported|400/i.test(error.message)) throw error;
    delete payload.response_format;
    response = await fetchJson(config.llmBaseUrl + "/chat/completions", { headers: authHeaders(config.llmApiKey), body: payload, timeoutMs: 120000 });
  }
  const parsed = parseModelJson(response.choices?.[0]?.message?.content || "{}");
  const chosen = resolveChoice(parsed.choice, order, candidates);
  const confidence = clamp(parsed.confidence ?? 0.5, 0, 1);
  return {
    engine: "llm", model: response.model || config.llmModel,
    answers: chosen ? {
      motion: { type: "choice", choice: "drive", probabilities: { drive: 1 } },
      vector: { type: "choice", choice: chosen, probabilities: { [chosen]: confidence } },
    } : {},
    confidence, riskScore: null,
    rationale: String(parsed.why || parsed.rationale || "No explanation given.").slice(0, 400),
    usage: response.usage || {}, raw: parsed,
  };
}

function resolveChoice(raw, order, candidates) {
  if (raw == null) return null;
  if (typeof raw === "number") return order[raw - 1] || null;
  const text = String(raw).trim();
  if (Object.hasOwn(candidates, text)) return text;
  const ordinal = text.match(/^#?(\d+)$/);
  if (ordinal) return order[Number(ordinal[1]) - 1] || null;
  const normalize = (value) => String(value || "").replace(/\s+/g, "").toUpperCase();
  const wanted = normalize(text);
  return order.find((id) => normalize(candidates[id].plan) === wanted) || null;
}

function authHeaders(key) {
  if (!key) throw new Error("Missing API key in .env");
  return { Authorization: "Bearer " + key };
}

async function fetchJson(url, { headers, body, timeoutMs = 45000 }) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json", ...headers }, body: JSON.stringify(body), signal: controller.signal });
    const raw = await response.text();
    let data;
    try { data = JSON.parse(raw); } catch { data = { raw }; }
    if (!response.ok) {
      const error = new Error(response.status + " " + (data.error?.message || data.detail || data.raw || response.statusText));
      error.status = response.status;
      throw error;
    }
    return data;
  } catch (error) {
    if (error.name === "AbortError") throw new Error("Request timed out after " + timeoutMs / 1000 + "s");
    throw error;
  } finally {
    clearTimeout(timer);
  }
}

function parseModelJson(content) {
  if (typeof content === "object" && content) return content;
  const cleaned = String(content).replace(/^```(?:json)?\s*/i, "").replace(/```\s*$/, "").trim();
  try { return JSON.parse(cleaned); } catch {
    const match = cleaned.match(/\{[\s\S]*\}/);
    if (!match) throw new Error("LLM did not return JSON");
    return JSON.parse(match[0]);
  }
}
