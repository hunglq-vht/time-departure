"""Tests for geometry module."""
from __future__ import annotations

import math

import pytest

from scrp.geometry import (
    common_waypoints,
    euclidean_distance,
    t_arrival_at_waypoint,
    t_land,
    waypoint_arrival_times,
)
from tests.conftest import make_fi, make_lane, make_waypoint


def test_t_land_single_segment():
    wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
    lane = make_lane("l", wps)
    fi = make_fi(lane, v=10.0, t_des=0.0)
    assert t_land(fi, 0.0) == pytest.approx(10.0)


def test_t_land_multi_segment():
    wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0), make_waypoint("C", 300, 0)]
    lane = make_lane("l", wps)
    fi = make_fi(lane, v=10.0, t_des=0.0)
    # seg1 = 100m at 10m/s = 10s; seg2 = 200m at 10m/s = 20s
    assert t_land(fi, 0.0) == pytest.approx(30.0)


def test_waypoint_arrival_times_length():
    wps = [make_waypoint(f"P{i}", i * 50, 0) for i in range(4)]
    lane = make_lane("l", wps)
    fi = make_fi(lane, v=10.0, t_des=100.0)
    times = waypoint_arrival_times(fi, 100.0)
    assert len(times) == 4
    assert times[0] == pytest.approx(100.0)


def test_t_arrival_at_waypoint_matches_arrival_times():
    wps = [make_waypoint(f"P{i}", i * 100, 0) for i in range(4)]
    lane = make_lane("l", wps)
    fi = make_fi(lane, v=10.0, t_des=0.0)
    all_times = waypoint_arrival_times(fi, 0.0)
    for k in range(4):
        assert t_arrival_at_waypoint(fi, k, 0.0) == pytest.approx(all_times[k])


def test_common_waypoints_shared():
    wps_a = [make_waypoint("P1"), make_waypoint("P2"), make_waypoint("P3")]
    wps_b = [make_waypoint("P2"), make_waypoint("P3"), make_waypoint("P4")]
    lane_a = make_lane("a", wps_a)
    lane_b = make_lane("b", wps_b)
    shared = common_waypoints(lane_a, lane_b)
    assert len(shared) == 2
    indices_a = [s[0] for s in shared]
    assert 1 in indices_a  # P2 is at index 1 in lane_a
    assert 2 in indices_a  # P3 is at index 2 in lane_a


def test_common_waypoints_none():
    wps_a = [make_waypoint("P1"), make_waypoint("P2")]
    wps_b = [make_waypoint("P3"), make_waypoint("P4")]
    lane_a = make_lane("a", wps_a)
    lane_b = make_lane("b", wps_b)
    assert common_waypoints(lane_a, lane_b) == []


def test_euclidean_distance():
    a = make_waypoint("A", 0, 0, 0)
    b = make_waypoint("B", 3, 4, 0)
    assert euclidean_distance(a, b) == pytest.approx(5.0)
