# Architecture

The demo uses one closed loop with explicit boundaries:

`HTTP/UI -> perception session -> shared one-step control generator -> model adapter -> safety gate -> exact control execution`

- `server.js` is the composition root and HTTP transport. It coordinates one decision; it contains no model protocol, vehicle dynamics, or perception state implementation.
- `models.js` contains the Jev typed-choice and LLM semantic adapters. Both adapters return only a candidate choice.
- `scenarios.js` is the shared source for browser starts, targets, labels, and model task text.
- `parking/session-store.js` owns per-run sensor memory and navigation-stage state.
- `parking/config.js` is the only source for vehicle geometry, tolerances, control horizons, and candidate dimensions.
- `parking/controller.js` builds a diverse one-step steering, speed, gear, and duration lattice. It has no model, prompt, history, or API knowledge.
- `parking/kinematics.js` rolls complete trajectories through a steering-rate-limited bicycle model and checks the full vehicle body.
- `parking/candidates.js` enriches controller rollouts with navigation and parking measurements.
- `parking/navigation.js` owns destination route stages and progress measurement; it never selects a control.
- `parking/policy.js` owns parking completion, safety eligibility, and history/recovery analysis.
- `parking/jev-request.js` serializes domain facts into Jev's factored typed-choice request.
- `parking/selection.js` maps model answers back to one candidate.
- `parking-decision.js` is only a compatibility facade for scripts and tests; runtime modules use the focused files above.
- `briefing.js` translates the same domain facts into prose for the LLM.
- `perception.js` owns simulated sensors, occupancy mapping, tracking, prediction, and collision assertions.
- `parking/overlay.js` converts candidate rollouts to the visual prediction-line DTO.
- `public/app.js` owns the browser run loop and view state; `public/parking3d.js` owns Three.js rendering and animation.

The honesty boundary is unchanged: the same generator, perceived world, candidate table, measurements, and safety gate are used for Jev and the LLM. Search cost is used only to keep generation bounded and diverse; it is never used after the table is built. Normal code may generate and measure candidates and refuse an unsafe choice. It does not replace the model's choice with a locally preferred control. The browser executes the exact path selected by the model.

Every selectable candidate contains one immediate control. The shared generator
offers steering, speed, gear, and duration choices to both engines. A terminal
rollout that satisfies both parking tolerances is preserved in the bounded table
so completion remains an explicit model-selectable outcome.

Loop recovery follows the same boundary. The session keeps a short memory of
physical poses visited during a detected recovery episode. A known return to
that state is temporarily removed from both engines' choice set while another
safe, novel control exists. Recovery ends only after two consecutive decisions
show objective progress, so a single excursion cannot immediately reopen the
same loop. No local score ranks the remaining controls and no escape control is
injected; Jev or the LLM still chooses gear, steering and speed.

When every safe endpoint is already tabu, a shared aspiration rule admits all
rollouts that improve the recovery episode's measured objective. When safe
terminal rollouts exist during recovery, it exposes every terminal choice and
withholds non-terminal wandering. These are common constraint filters over
measured outcomes; neither rule picks a winner or depends on engine identity.
