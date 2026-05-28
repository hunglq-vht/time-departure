"""Client interface for the external flight route service.

Callers that already hold a list of routes can skip this and pass them
directly to suggest_routes().  This module is for integrations where the
deconfliction service itself fetches routes from a remote endpoint.
"""
from __future__ import annotations

import json
from abc import ABC, abstractmethod
from typing import List
from urllib.request import urlopen

from .models import FlightRoute, Segment, Waypoint


class FlightRouteClient(ABC):
    """Abstract interface for the external route service."""

    @abstractmethod
    def get_routes(self) -> List[FlightRoute]:
        """Fetch all currently published flight routes."""


class HttpFlightRouteClient(FlightRouteClient):
    """Minimal HTTP client that calls a JSON REST endpoint.

    Expected response schema (list of objects):
    [
      {
        "route_id": "r1",
        "waypoints": [{"id": "w1", "x": 0, "y": 0, "z": 50}, ...],
        "take_off_vertiport_id": "vp_a",
        "landing_vertiport_id": "vp_b",
        "safety_score": 0.9,
        "average_wind_speed": 5.0,
        "max_drone_weight_kg": 25.0,
        "compatible_drone_sizes": ["A", "B"]
      },
      ...
    ]
    """

    def __init__(self, base_url: str, timeout_sec: float = 10.0) -> None:
        self._url = base_url.rstrip("/") + "/routes"
        self._timeout = timeout_sec

    def get_routes(self) -> List[FlightRoute]:
        with urlopen(self._url, timeout=self._timeout) as resp:
            data = json.loads(resp.read().decode())
        return [_route_from_dict(item) for item in data]


# ---------------------------------------------------------------------------
# Deserialisation helper
# ---------------------------------------------------------------------------

def _route_from_dict(d: dict) -> FlightRoute:
    waypoints = [
        Waypoint(id=w["id"], position=(float(w["x"]), float(w["y"]), float(w["z"])))
        for w in d["waypoints"]
    ]
    return FlightRoute(
        route_id=d["route_id"],
        waypoints=waypoints,
        take_off_vertiport_id=d["take_off_vertiport_id"],
        landing_vertiport_id=d["landing_vertiport_id"],
        safety_score=float(d["safety_score"]),
        average_wind_speed=float(d["average_wind_speed"]),
        max_drone_weight_kg=float(d["max_drone_weight_kg"]),
        compatible_drone_sizes=list(d["compatible_drone_sizes"]),
    )
