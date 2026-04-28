"""Tests for BatchListener — priority-ordered batch processing."""
from __future__ import annotations

from scrp.batch_listener import BatchListener
from scrp.models import ApproveResult, RejectResult, SCRPConfig
from tests.conftest import make_fi, make_lane, make_system_state, make_waypoint


def _simple_state():
    wps = [make_waypoint("P1", 0, 0), make_waypoint("P2", 100, 0), make_waypoint("P3", 200, 0)]
    lane = make_lane("lane1", wps)
    state = make_system_state()
    return state, lane


class TestBatchListenerProcessNow:
    def test_empty_batch_returns_empty(self):
        state, _ = _simple_state()
        listener = BatchListener(state, SCRPConfig())
        results = listener.process_now()
        assert results == {}

    def test_single_intention_processed(self):
        state, lane = _simple_state()
        listener = BatchListener(state, SCRPConfig())
        fi = make_fi(lane, v=10.0, t_des=1000.0, drone_id="drone_a")
        listener.submit(fi)
        results = listener.process_now()
        assert "drone_a" in results
        assert isinstance(results["drone_a"], ApproveResult)

    def test_results_stored_after_processing(self):
        state, lane = _simple_state()
        listener = BatchListener(state, SCRPConfig())
        fi = make_fi(lane, v=10.0, t_des=1000.0, drone_id="drone_a")
        listener.submit(fi)
        listener.process_now()
        assert "drone_a" in listener.get_results()

    def test_pending_cleared_after_processing(self):
        state, lane = _simple_state()
        listener = BatchListener(state, SCRPConfig())
        fi = make_fi(lane, v=10.0, t_des=1000.0, drone_id="drone_a")
        listener.submit(fi)
        listener.process_now()
        # second call finds nothing pending
        results = listener.process_now()
        assert results == {}


class TestBatchPriorityOrdering:
    def test_priority_1_processed_before_priority_3(self):
        state, lane = _simple_state()
        processing_order: list[str] = []

        def capture(drone_id: str, result) -> None:
            processing_order.append(drone_id)

        listener = BatchListener(state, SCRPConfig(), on_result=capture)

        fi_low = make_fi(lane, v=10.0, t_des=1000.0, drone_id="low_priority")
        fi_low.priority = 3
        fi_high = make_fi(lane, v=10.0, t_des=1000.0, drone_id="high_priority")
        fi_high.priority = 1
        fi_mid = make_fi(lane, v=10.0, t_des=1000.0, drone_id="mid_priority")
        fi_mid.priority = 2

        # submit in reverse priority order
        listener.submit(fi_low)
        listener.submit(fi_mid)
        listener.submit(fi_high)

        listener.process_now()

        assert processing_order == ["high_priority", "mid_priority", "low_priority"]

    def test_same_priority_all_processed(self):
        state, lane = _simple_state()
        listener = BatchListener(state, SCRPConfig())

        for i in range(3):
            fi = make_fi(lane, v=10.0, t_des=1000.0, drone_id=f"drone_{i}")
            fi.priority = 2
            listener.submit(fi)

        results = listener.process_now()
        assert len(results) == 3


class TestBatchListenerCallback:
    def test_on_result_called_for_each_fi(self):
        state, lane = _simple_state()
        called: list[str] = []

        listener = BatchListener(state, SCRPConfig(), on_result=lambda did, _: called.append(did))
        for name in ("alpha", "beta", "gamma"):
            listener.submit(make_fi(lane, v=10.0, t_des=1000.0, drone_id=name))

        listener.process_now()
        assert sorted(called) == ["alpha", "beta", "gamma"]

    def test_no_callback_no_error(self):
        state, lane = _simple_state()
        listener = BatchListener(state, SCRPConfig(), on_result=None)
        listener.submit(make_fi(lane, v=10.0, t_des=1000.0, drone_id="d1"))
        listener.process_now()  # must not raise


class TestBatchListenerStateUpdate:
    def test_update_state_used_in_next_batch(self):
        state, lane = _simple_state()
        listener = BatchListener(state, SCRPConfig())

        new_state = make_system_state(t_now=9999.0)
        listener.update_state(new_state)

        fi = make_fi(lane, v=10.0, t_des=1000.0, drone_id="d1")
        listener.submit(fi)
        results = listener.process_now()
        assert "d1" in results


class TestBatchListenerThread:
    def test_start_stop_no_deadlock(self):
        state, _ = _simple_state()
        listener = BatchListener(state, SCRPConfig(), batch_interval=0.05)
        listener.start()
        listener.stop(timeout=1.0)

    def test_start_idempotent(self):
        state, _ = _simple_state()
        listener = BatchListener(state, SCRPConfig(), batch_interval=0.05)
        listener.start()
        listener.start()  # second call must not spawn extra thread
        listener.stop(timeout=1.0)
