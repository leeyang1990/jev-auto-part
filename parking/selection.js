import { clamp } from "./math.js";

export function expandAnswers(prepared, answers = {}) {
  const result = { ...answers, ...prepared.fixed };
  if (result.vector) {
    result.vector = {
      ...result.vector,
      choice: prepared.aliases[result.vector.choice] ?? result.vector.choice,
      probabilities: Object.fromEntries(Object.entries(result.vector.probabilities || {})
        .map(([id, probability]) => [prepared.aliases[id] ?? id, probability])),
    };
  }
  return result;
}

export function decisionSelection(candidates, answers) {
  const motion = answers.motion;
  const vector = answers.vector;
  const stopChosen = motion?.choice === "stop";
  const choice = stopChosen ? "stop" : vector?.choice;
  if (!choice || !Object.hasOwn(candidates, choice)) return null;
  const drive = motion?.probabilities?.drive ?? (motion?.choice === "drive" ? 1 : 0);
  const motionProbability = clamp(motion?.probabilities?.[motion?.choice] ?? (motion?.choice ? 1 : 0), 0, 1);
  const vectorProbability = stopChosen ? 1 : clamp(vector?.probabilities?.[vector?.choice] ?? 1, 0, 1);
  const probabilities = Object.fromEntries(Object.keys(candidates).map((id) => [
    id, id === "stop" ? (motion?.probabilities?.stop ?? 0) : drive * (vector?.probabilities?.[id] ?? 0),
  ]));
  return {
    choice,
    motion: stopChosen ? "stop" : "drive",
    confidence: Number((motionProbability * vectorProbability).toFixed(4)),
    probabilities,
  };
}

export function validAnswers(candidates, answers) {
  if (!answers) return false;
  if (answers.motion && !weights(answers.motion, { drive: null, stop: null })) return false;
  if (answers.vector && !Object.hasOwn(candidates, answers.vector.choice)) return false;
  return Boolean(decisionSelection(candidates, answers));
}

function weights(question, criteria) {
  if (!question || !criteria) return false;
  const entries = Object.entries(criteria);
  if (entries.length === 1) return question.choice === entries[0][0] || !question.choice;
  return Object.hasOwn(criteria, question.choice)
    && (question.probabilities == null || Number.isFinite(question.probabilities[question.choice]));
}
