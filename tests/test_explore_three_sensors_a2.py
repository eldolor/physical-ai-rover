# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.237 criterion A2 (`docs/plans/ros-alignment/3.237-explore-three-sensors.md`,
amendment 1): each sensor sees the low things its geometry lets it, judged
by the test's own geometry, over 500 seeded poses in six houses (~80 s, two
full scans a pose: a slow-tier file)."""

import json

from tests.test_explore_three_sensors import SCATTERED, _a2_house


def test_a2_each_sensor_sees_what_its_geometry_lets_it(monkeypatch):
    """A2 (amendment 1): the lidar's scan unchanged by the low things; the
    floor band against the test's own march; the camera's low detections
    exactly the vertical rule's, among 3.32's visible. 500 poses a house,
    commanded tilts -30..30, all three pans."""
    totals = {}
    for house in ["complex_house_hazards"] + SCATTERED:
        c = _a2_house(house, monkeypatch)
        totals[house] = c
        assert c["camera_seen"] > 0 and c["camera_hidden"] > 0, (house, c)
    print(json.dumps(totals))
