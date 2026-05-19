"""
ADAPTIVE TRAFFIC CONTROL SYSTEM — 4-Way Junction (India / LHT Edition)
Drive side: LEFT-HAND TRAFFIC (India, UK, Australia, etc.)

Conflict geometry (LHT):
  - LEFT turn  = near-side turn, no oncoming crossing → safe with straight ✅
  - RIGHT turn = crosses oncoming traffic             → needs protected phase ❌

Phase structure:
  NS       → North + South straight + left turns (bundled, no conflict)
  NS_RIGHT → North + South protected right-turn only
  EW       → East  + West  straight + left turns (bundled, no conflict)
  EW_RIGHT → East  + West  protected right-turn only

Fixes applied vs. previous version:
  1.  LHT conflict geometry (right-turn protection instead of left)
  2.  Async state-machine — no blocking time.sleep() in main loop
  3.  Left-turn sub-phase trigger bug fixed (used stale current_phase)
  4.  Extension loop now accepts fresh LaneData callback, not stale snapshot
  5.  PhaseWaitTracker warm-up: NS pre-seeded so first cycle isn't a coin-flip
  6.  Emergency handler auto-clears after configurable timeout
  7.  _validate_freshness now gates execution (returns stale-hold state)
  8.  MAX_WAIT starvation: starved opposing direction queued as next priority
  9.  GREEN_SCALE auto-derived from config so it reaches MAX_GREEN at CRITICAL_DENSITY
  10. _build_signal_state called BEFORE phase executes (correct hardware dispatch order)
"""

import asyncio
import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Awaitable, Callable, Optional

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("TrafficATC_India")


# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────

@dataclass
class JunctionConfig:
    # Timing
    MIN_GREEN: float = 15.0          # seconds — absolute floor for any phase
    MAX_GREEN: float = 60.0          # seconds — ceiling before forced switch
    YELLOW_TIME: float = 4.0         # amber clearance between phases
    ALL_RED_TIME: float = 2.0        # full-red buffer after yellow (safety)
    EXTENSION_STEP: float = 5.0      # seconds added per extension
    MAX_EXTENSIONS: int = 4          # hard cap on extensions per phase
    YOLO_INTERVAL: float = 5.0       # seconds between YOLO image captures

    # Thresholds
    MAX_WAIT: float = 120.0          # seconds — starvation override threshold
    HIGH_DENSITY_THRESHOLD: int = 15 # vehicles — triggers extension eligibility
    CRITICAL_DENSITY: int = 25       # vehicles — triggers MAX_GREEN immediately
    STALE_DATA_TIMEOUT: float = 15.0 # seconds — treat feed as failed after this
    EMERGENCY_TIMEOUT: float = 30.0  # seconds — auto-clear emergency after this

    # Weights
    WAIT_WEIGHT: float = 0.7         # how much historical wait influences priority

    @property
    def GREEN_SCALE(self) -> float:
        """Auto-derived so raw formula reaches MAX_GREEN exactly at CRITICAL_DENSITY."""
        return (self.MAX_GREEN - self.MIN_GREEN) / self.CRITICAL_DENSITY


# ─────────────────────────────────────────────
#  DATA STRUCTURES
# ─────────────────────────────────────────────

class Phase(Enum):
    NS        = auto()   # North + South straight + left (LHT: left is safe)
    NS_RIGHT  = auto()   # North + South protected right-turn
    EW        = auto()   # East  + West  straight + left (LHT: left is safe)
    EW_RIGHT  = auto()   # East  + West  protected right-turn
    ALL_RED   = auto()   # Transition buffer


@dataclass
class LaneData:
    """Density counts from YOLO for one approach direction."""
    straight: int = 0
    left_turn: int = 0   # LHT: safe — bundled with straight in main phase
    right_turn: int = 0  # LHT: crosses oncoming — needs protected phase
    timestamp: float = field(default_factory=time.time)

    @property
    def total(self) -> int:
        return self.straight + self.left_turn + self.right_turn

    @property
    def right_turn_ratio(self) -> float:
        """Fraction of traffic needing the protected right-turn phase."""
        if self.total == 0:
            return 0.0
        return self.right_turn / self.total


@dataclass
class PhaseWaitTracker:
    """
    Tracks real elapsed wait time using wall-clock timestamps.
    Far more accurate than accumulating opponent green_time.
    """
    _last_served: float = field(default_factory=time.time)

    def reset(self):
        self._last_served = time.time()

    def pre_seed(self, seconds_ago: float):
        """Warm-up helper: pretend this phase last ran N seconds ago."""
        self._last_served = time.time() - seconds_ago

    @property
    def elapsed(self) -> float:
        return time.time() - self._last_served


# ─────────────────────────────────────────────
#  CORE CONTROLLER  (async state-machine)
# ─────────────────────────────────────────────

# Type alias: a coroutine that returns fresh LaneData for (N, S, E, W)
SensorCallback = Callable[[], Awaitable[tuple[LaneData, LaneData, LaneData, LaneData]]]


class TrafficController:
    def __init__(self, config: JunctionConfig = JunctionConfig(),
                 sensor_cb: Optional[SensorCallback] = None):
        self.cfg = config
        self.sensor_cb = sensor_cb  # injected YOLO callback

        # Straight/left wait trackers (timestamp-based)
        self.wait = {
            Phase.NS: PhaseWaitTracker(),
            Phase.EW: PhaseWaitTracker(),
        }
        # Right-turn sub-phase wait tracked separately (LHT: right needs protection)
        self.right_wait = {
            Phase.NS_RIGHT: PhaseWaitTracker(),
            Phase.EW_RIGHT: PhaseWaitTracker(),
        }

        # FIX #5: Warm-up — seed NS as if it ran MIN_GREEN ago so EW isn't
        # unfairly prioritised on the very first cycle.
        self.wait[Phase.NS].pre_seed(self.cfg.MIN_GREEN)

        self.current_phase: Optional[Phase] = None
        self._priority_override: Optional[Phase] = None  # starvation queue
        self._emergency_direction: Optional[str] = None
        self._emergency_started: Optional[float] = None
        self._cycle_count: int = 0
        self._last_signal_state: dict = {"N": "RED", "S": "RED", "E": "RED", "W": "RED"}
        self._data_stale: bool = False

    # ── PUBLIC API ───────────────────────────

    async def run(self):
        """Main loop — call this from your asyncio entry point."""
        log.info("Traffic ATC (India/LHT) started.")
        while True:
            N, S, E, W = await self._fetch_sensor_data()
            await self.ingest(N, S, E, W)
            # Yield briefly before next YOLO frame
            await asyncio.sleep(self.cfg.YOLO_INTERVAL)

    async def ingest(self, N: LaneData, S: LaneData,
                     E: LaneData, W: LaneData) -> dict:
        """
        Main entry point each YOLO cycle.
        Returns the current signal state for hardware dispatch.
        FIX #10: signal state is dispatched BEFORE the phase executes.
        """
        # FIX #7: staleness gate — hold current state, do not proceed
        if not self._check_freshness(N, S, E, W):
            log.error("Stale YOLO data — holding current signal state.")
            return self._last_signal_state

        # FIX #6: auto-clear emergency after timeout
        self._maybe_clear_emergency()

        if self._emergency_direction:
            return await self._handle_emergency()

        phase = self._select_phase(N, S, E, W)
        green_time = self._compute_green_time(phase, N, S, E, W)

        # FIX #10: dispatch signal BEFORE sleeping
        signal_state = self._build_signal_state(phase)
        self._last_signal_state = signal_state
        log.info(f"Cycle {self._cycle_count + 1} | Phase: {phase.name} | "
                 f"Green: {green_time:.1f}s | "
                 f"W_NS: {self.wait[Phase.NS].elapsed:.1f}s | "
                 f"W_EW: {self.wait[Phase.EW].elapsed:.1f}s")
        self._dispatch_signals(signal_state)

        actual_green = await self._run_phase(phase, green_time)

        self._cycle_count += 1
        return signal_state

    def trigger_emergency(self, direction: str):
        """Called externally when emergency vehicle detected (siren sensor)."""
        self._emergency_direction = direction
        self._emergency_started = time.time()
        log.warning(f"🚨 EMERGENCY OVERRIDE: {direction} corridor")

    def clear_emergency(self):
        self._emergency_direction = None
        self._emergency_started = None

    # ── PHASE SELECTION ──────────────────────

    def _select_phase(self, N, S, E, W) -> Phase:
        """
        Multi-priority decision tree:
          1. Starvation override queue (FIX #8: tracks which side was skipped)
          2. Critical density emergency green
          3. Protected RIGHT-turn sub-phase (LHT fix: right, not left)
          4. Weighted priority score
        """
        cfg = self.cfg
        w_ns = self.wait[Phase.NS].elapsed
        w_ew = self.wait[Phase.EW].elapsed

        Q_NS = N.total + S.total
        Q_EW = E.total + W.total

        # Priority 1 — Starvation queue (FIX #8)
        if self._priority_override:
            chosen = self._priority_override
            self._priority_override = None
            log.info(f"  ↑ Starvation queue override → {chosen.name}")
            return chosen

        if w_ns >= cfg.MAX_WAIT and w_ew >= cfg.MAX_WAIT:
            # Both starved: serve longer-waiting, queue the other for NEXT cycle
            if w_ns >= w_ew:
                self._priority_override = Phase.EW
                return Phase.NS
            else:
                self._priority_override = Phase.NS
                return Phase.EW
        if w_ns >= cfg.MAX_WAIT:
            return Phase.NS
        if w_ew >= cfg.MAX_WAIT:
            return Phase.EW

        # Priority 2 — Critical density
        if Q_NS >= cfg.CRITICAL_DENSITY and Q_NS >= 2 * Q_EW:
            return Phase.NS
        if Q_EW >= cfg.CRITICAL_DENSITY and Q_EW >= 2 * Q_NS:
            return Phase.EW

        # Priority 3 — Protected RIGHT-turn sub-phase (LHT: right crosses oncoming)
        # FIX #3: trigger based on PREVIOUS phase (current_phase set at end of last cycle)
        ns_right_demand = N.right_turn + S.right_turn
        ew_right_demand = E.right_turn + W.right_turn
        ns_right_wait = self.right_wait[Phase.NS_RIGHT].elapsed
        ew_right_wait = self.right_wait[Phase.EW_RIGHT].elapsed

        if (ns_right_demand >= 5 and ns_right_wait >= 60
                and self.current_phase == Phase.NS):
            return Phase.NS_RIGHT
        if (ew_right_demand >= 5 and ew_right_wait >= 60
                and self.current_phase == Phase.EW):
            return Phase.EW_RIGHT

        # Priority 4 — Weighted priority score
        P_NS = Q_NS + cfg.WAIT_WEIGHT * w_ns
        P_EW = Q_EW + cfg.WAIT_WEIGHT * w_ew

        return Phase.NS if P_NS >= P_EW else Phase.EW

    # ── GREEN TIME COMPUTATION ────────────────

    def _compute_green_time(self, phase: Phase, N, S, E, W) -> float:
        cfg = self.cfg

        if phase == Phase.NS_RIGHT:
            Q = N.right_turn + S.right_turn
        elif phase == Phase.EW_RIGHT:
            Q = E.right_turn + W.right_turn
        elif phase == Phase.NS:
            Q = N.total + S.total
        else:
            Q = E.total + W.total

        raw = cfg.MIN_GREEN + cfg.GREEN_SCALE * Q
        return max(cfg.MIN_GREEN, min(raw, cfg.MAX_GREEN))

    # ── PHASE EXECUTION (async — no blocking sleep) ──

    async def _run_phase(self, phase: Phase, green_time: float) -> float:
        """
        FIX #2: fully async — uses asyncio.sleep, never blocks.
        FIX #4: fetches fresh sensor data in extension loop.
        """
        cfg = self.cfg
        extensions = 0
        total_green = green_time

        await asyncio.sleep(green_time)

        # Extension loop — fresh data each iteration (FIX #4)
        while extensions < cfg.MAX_EXTENSIONS:
            if total_green >= cfg.MAX_GREEN:
                log.warning(f"  MAX_GREEN ({cfg.MAX_GREEN}s) reached, forcing switch.")
                break

            N, S, E, W = await self._fetch_sensor_data()
            current_Q = self._current_queue(phase, N, S, E, W)

            if current_Q < cfg.HIGH_DENSITY_THRESHOLD:
                break

            log.info(f"  ↑ Extension {extensions + 1}/{cfg.MAX_EXTENSIONS} "
                     f"(queue={current_Q})")
            self._dispatch_signals(self._build_signal_state(phase))
            await asyncio.sleep(cfg.EXTENSION_STEP)
            total_green += cfg.EXTENSION_STEP
            extensions += 1

        # Yellow transition
        log.info(f"  → YELLOW for {cfg.YELLOW_TIME}s")
        self._dispatch_signals(self._build_signal_state(Phase.ALL_RED, yellow=True))
        await asyncio.sleep(cfg.YELLOW_TIME)

        # All-red buffer
        log.info(f"  → ALL_RED for {cfg.ALL_RED_TIME}s")
        self._dispatch_signals(self._build_signal_state(Phase.ALL_RED))
        await asyncio.sleep(cfg.ALL_RED_TIME)

        # Reset wait trackers
        parent = Phase.NS if phase in (Phase.NS, Phase.NS_RIGHT) else Phase.EW
        self.wait[parent].reset()
        if phase == Phase.NS_RIGHT:
            self.right_wait[Phase.NS_RIGHT].reset()
        elif phase == Phase.EW_RIGHT:
            self.right_wait[Phase.EW_RIGHT].reset()

        self.current_phase = phase  # FIX #3: set AFTER phase completes
        return total_green

    # ── EMERGENCY PRE-EMPTION ────────────────

    async def _handle_emergency(self) -> dict:
        phase = Phase.NS if self._emergency_direction == "NS" else Phase.EW
        self._dispatch_signals(self._build_signal_state(Phase.ALL_RED))
        await asyncio.sleep(self.cfg.ALL_RED_TIME)
        signal_state = self._build_signal_state(phase)
        self._dispatch_signals(signal_state)
        self._last_signal_state = signal_state
        log.warning(f"🚨 Emergency green: {phase.name}")
        return signal_state

    def _maybe_clear_emergency(self):
        """FIX #6: auto-clear emergency after EMERGENCY_TIMEOUT seconds."""
        if (self._emergency_direction and self._emergency_started
                and time.time() - self._emergency_started >= self.cfg.EMERGENCY_TIMEOUT):
            log.info("🚨 Emergency timeout — resuming normal operation.")
            self.clear_emergency()

    # ── STALENESS GUARD ──────────────────────

    def _check_freshness(self, *lanes: LaneData) -> bool:
        """FIX #7: returns False and gates execution if any feed is stale."""
        now = time.time()
        for lane in lanes:
            age = now - lane.timestamp
            if age > self.cfg.STALE_DATA_TIMEOUT:
                log.error(f"  ⚠ STALE YOLO DATA ({age:.1f}s old).")
                return False
        return True

    # ── HELPERS ──────────────────────────────

    def _current_queue(self, phase: Phase, N, S, E, W) -> int:
        if phase in (Phase.NS, Phase.NS_RIGHT):
            return N.total + S.total
        return E.total + W.total

    def _dispatch_signals(self, state: dict):
        """Replace with hardware GPIO / API calls in production."""
        log.debug(f"  Signal dispatch → {state}")

    def _build_signal_state(self, phase: Phase, yellow: bool = False) -> dict:
        """
        LHT signal map:
          GREEN      = straight + left permitted (no conflict in LHT)
          RIGHT_GREEN = protected right-turn only
          YELLOW     = clearing
          RED        = stop

        FIX #1: RIGHT_GREEN replaces LEFT_GREEN from the original RHT code.
        """
        all_red = {"N": "RED", "S": "RED", "E": "RED", "W": "RED"}

        if yellow:
            # Outgoing phase shows yellow on its directions
            if phase == Phase.NS:
                return {**all_red, "N": "YELLOW", "S": "YELLOW"}
            if phase == Phase.EW:
                return {**all_red, "E": "YELLOW", "W": "YELLOW"}
            return all_red

        states = {
            Phase.NS:        {**all_red, "N": "GREEN",       "S": "GREEN"},
            Phase.NS_RIGHT:  {**all_red, "N": "RIGHT_GREEN", "S": "RIGHT_GREEN"},
            Phase.EW:        {**all_red, "E": "GREEN",       "W": "GREEN"},
            Phase.EW_RIGHT:  {**all_red, "E": "RIGHT_GREEN", "W": "RIGHT_GREEN"},
            Phase.ALL_RED:   all_red,
        }
        return states.get(phase, all_red)

    async def _fetch_sensor_data(self) -> tuple[LaneData, LaneData, LaneData, LaneData]:
        """Calls the injected YOLO callback, or returns dummy data for testing."""
        if self.sensor_cb:
            return await self.sensor_cb()
        # ── Dummy sensor data — replace with real YOLO pipeline ──
        now = time.time()
        return (
            LaneData(straight=8,  left_turn=3, right_turn=2, timestamp=now),
            LaneData(straight=6,  left_turn=2, right_turn=1, timestamp=now),
            LaneData(straight=12, left_turn=1, right_turn=3, timestamp=now),
            LaneData(straight=5,  left_turn=4, right_turn=2, timestamp=now),
        )


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────

async def main():
    cfg = JunctionConfig()
    controller = TrafficController(cfg)
    await controller.run()


if __name__ == "__main__":
    asyncio.run(main())