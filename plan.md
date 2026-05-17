## Plan: Dynamic SUMO Spawning from Lane-Class Counts

Build a dynamic demand path in Python that consumes per-tick lane/class counts (`north|south|east|west` × `car|3 wheeler|truck`) and injects vehicles into SUMO every second. Disable static XML demand when dynamic mode is enabled to avoid double-spawn. Keep adaptive signal control separate from spawning so visuals reflect injected demand directly.

**Steps**
1. **Phase 1 — Demand ownership switch**: add a clear mode gate so only one demand source is active at a time (`static_xml` vs `dynamic_python`). *blocks all later steps*
2. **Phase 1 — Disable static flows in dynamic mode**: route config uses a no-flow demand file (or zeroed flows) when dynamic mode is on, while preserving emergency vehicles if needed. *depends on 1*
3. **Phase 2 — Input contract**: define canonical in-memory payload for full-junction tick input, normalize lane keys and class labels (`3 wheeler` alias handling). *parallel with 2 after 1*
4. **Phase 2 — Type mapping**: map input classes to SUMO `vType` IDs (`car`, `threewheeler`, `truck`), with strict validation and error logging for unknown class names. *depends on 3*
5. **Phase 3 — Spawner module design**: create a dedicated runtime spawner component invoked once per second that converts counts into concrete spawn attempts per lane/class. *depends on 3,4*
6. **Phase 3 — Route strategy per lane**: predefine lane-origin route pools (straight/left/right weights) and deterministic selection policy so north inputs always originate from `north_in`, etc. *depends on 5*
7. **Phase 3 — Robust insertion policy**: define unique vehicle ID scheme, depart lane/speed policy, retry/backoff rules, and rejection accounting for failed insertions. *depends on 5,6*
8. **Phase 4 — Runner integration point**: call spawner in the once-per-second block after control updates; keep emergency preemption logic independent. *depends on 5-7*
9. **Phase 4 — Observability**: emit per-second spawn telemetry (requested vs inserted vs failed) per lane/class for terminal diagnostics and quick visual verification. *depends on 8*
10. **Phase 5 — YOLO-ready adapter seam**: keep source-agnostic interface (`CountInputProvider`) so current hardcoded/random generator can be swapped later with YOLO pipeline producer without touching spawner core. *parallel with 8/9*

**Relevant files**
- `simulation/runner.py` — integrate per-second spawner call in main loop and mode switch wiring.
- `simulation/interfaces/providers.py` — keep as temporary source provider (hardcoded/random) behind normalized payload contract.
- `simulation/core/config.py` — add demand-source mode flags, spawn interval, and toggles for diagnostics.
- `sumo_network/single_junction.rou.xml` — ensure required `vType`s exist (`car`, `threewheeler`, `truck`) and produce a dynamic-mode no-flow variant.
- `sumo_network/single_junction.sumocfg` — point to appropriate route file by mode.
- `simulation/logic/diagnostics.py` — print spawn diagnostics alongside control diagnostics.

**Verification**
1. Run dynamic mode with static flows disabled and fixed input (`north=0, south=50, east=0, west=0`) and confirm visible queues form only from south approaches.
2. Run one-minute scenario with mixed classes and verify all three class `vType`s appear visually and in TraCI lists.
3. Confirm per-second telemetry equality: `requested = inserted + failed` per lane/class.
4. Stress test high counts (e.g., 200/tick) and confirm graceful failures are logged without simulation crash.
5. Toggle input source hardcoded → random without changing spawner code path.

**Decisions**
- Dynamic mode owns demand exclusively (no XML flow overlap).
- Input is full-junction payload each tick.
- Current semantics: counts mean exact vehicles to inject each tick (not target queue).
- Spawn cadence: every 1 second.
- Add dedicated SUMO types for `3 wheeler` and `truck` now.

**Further Considerations**
1. Add optional safety cap per lane per second (recommended) to prevent unrealistic bursts during noisy upstream input.
2. Decide emergency-vehicle demand ownership: keep static scripted EVs or move EV spawning into dynamic pipeline later.
3. Decide whether route turn ratios should be static config or included in future pipeline payload.
