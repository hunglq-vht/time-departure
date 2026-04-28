"""Batch flight intention listener — collects intentions and processes by priority every 10 s."""
from __future__ import annotations

import threading
import time
from typing import Callable, Dict, List, Optional

from .models import FlightIntention, SCRPConfig, SCRPResult, SystemState
from .scrp import resolve_conflict


class BatchListener:
    """Collects FlightIntentions and processes them in priority order (1=highest) every batch_interval seconds.

    Usage::

        listener = BatchListener(system_state, config, on_result=my_callback)
        listener.start()
        listener.submit(fi)          # thread-safe
        listener.update_state(new_state)
        listener.stop()
    """

    def __init__(
        self,
        system_state: SystemState,
        config: SCRPConfig,
        on_result: Optional[Callable[[str, SCRPResult], None]] = None,
        batch_interval: float = 10.0,
    ) -> None:
        self._state = system_state
        self._config = config
        self._on_result = on_result
        self._batch_interval = batch_interval

        self._pending: List[FlightIntention] = []
        self._results: Dict[str, SCRPResult] = {}
        self._lock = threading.Lock()

        self._thread: Optional[threading.Thread] = None
        self._running = False

    # ── public API ────────────────────────────────────────────────────────────

    def submit(self, fi: FlightIntention) -> None:
        """Enqueue a flight intention for the next batch window."""
        with self._lock:
            self._pending.append(fi)

    def start(self) -> None:
        """Start the background batch-processing thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="BatchListener", daemon=True)
        self._thread.start()

    def stop(self, timeout: Optional[float] = None) -> None:
        """Stop the background thread and wait for it to finish."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=timeout if timeout is not None else self._batch_interval + 1)
            self._thread = None

    def update_state(self, state: SystemState) -> None:
        """Replace the SystemState used for conflict resolution (thread-safe)."""
        with self._lock:
            self._state = state

    def get_results(self) -> Dict[str, SCRPResult]:
        """Return a snapshot of all results produced so far."""
        with self._lock:
            return dict(self._results)

    def process_now(self) -> Dict[str, SCRPResult]:
        """Immediately drain and process the current pending queue (synchronous, for testing)."""
        return self._process_batch()

    # ── internals ─────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        """Background loop: sleep batch_interval, then process pending intentions."""
        while self._running:
            time.sleep(self._batch_interval)
            if self._running:
                self._process_batch()

    def _process_batch(self) -> Dict[str, SCRPResult]:
        """Drain pending queue, sort by priority, resolve each intention in order."""
        with self._lock:
            batch = sorted(self._pending, key=lambda fi: fi.priority)
            self._pending = []
            state = self._state

        results: Dict[str, SCRPResult] = {}
        for fi in batch:
            result = resolve_conflict(
                fi,
                state.approved_plans,
                state.vertiport_state,
                state,
                self._config,
            )
            results[fi.drone_id] = result
            if self._on_result is not None:
                self._on_result(fi.drone_id, result)

        with self._lock:
            self._results.update(results)

        return results
