# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
brain/explore_sight.py -- 3.227: the plain `explore` policy's eyes on frames
that carry no `detections`.

`ExploreAgent._hits` reads the simulator's `frame["detections"]` under
`sight_from: frame`, and a real camera's frame has none (3.76 E12). Under
`sight_from: perception` (3.80.G) it reads the scene's `_perception` instead
-- which the tiered policy fills and the plain policy never did. This is that
filler: a vision_fn returning the agent's own sensor scene (`sensed_scene`,
bound by MissionRunner once the agent exists) plus one frame's on-board
perception (`brain/perceive.py`'s pipeline -- YOLOE crops, CLIP, the match
gate -- or, in the simulator, `FrameReportedPipeline`).

It is the vision_fn, not something the agent calls beside it, so perception
runs inside B3.2's guard (MissionRunner `_guarded_vision`): a wedged model is
a vision failure counted like any other, never a hung tick.

**Identity.** Plain explore has no identity tier, so an arrival is never
`found` on perception alone (3.47). `confirm_arrival` answers undecided with
`no_identity_tier`, which MissionRunner ends `arrived_unconfirmed` -- unless
an `identity_fn` is given (the simulator's geometry, 3.98, for the sweep).
"""

from typing import Callable, Optional

# The verdict's flag MissionRunner ends `arrived_unconfirmed` on (3.227).
NO_IDENTITY_TIER = "no_identity_tier"


class PerceptionSight:
    """A plain explore mission's vision_fn on frames with no detections."""

    # Read by MissionRunner: run explore with `sight_from: perception`.
    explore_sight_from = "perception"

    def __init__(self, pipeline, identity_fn: Optional[Callable[[dict], dict]] = None):
        self.pipeline = pipeline
        self.identity_fn = identity_fn
        self._base: Optional[Callable[[dict], dict]] = None
        self.frames = 0                 # frames perceived
        self.detected = 0               # ...whose status was DETECTED

    def bind_base(self, base: Callable[[dict], dict]) -> None:
        """MissionRunner hands over the agent's `sensed_scene` once the agent
        exists (the vision_fn is built before it)."""
        self._base = base

    def __call__(self, frame: dict) -> dict:
        if self._base is None:
            raise RuntimeError("PerceptionSight has no base scene: MissionRunner binds "
                               "the agent's sensed_scene (bind_base) before the first tick")
        scene = dict(self._base(frame))
        # A wedged model raises out (PerceptionWedged): B3.2 counts it.
        perception = self.pipeline.perceive(frame).as_dict()
        self.frames += 1
        if perception.get("status") == "detected":
            self.detected += 1
        scene["_perception"] = perception
        return scene

    def confirm_arrival(self, frame: dict, local_ok: bool = True) -> dict:
        """The arrival's identity. With no `identity_fn`, undecided and never
        asked of anyone: the mission ends `arrived_unconfirmed` (3.227 A)."""
        said = self.identity_fn(frame) if self.identity_fn is not None else None
        if said is not None:
            # The simulator's geometry (brain/arrival.py `geometry_identity`):
            # True or False, judged free; None (not a sim frame) is no tier.
            return {"confirmed": bool(said), "cloud_called": False, "tier": "sim",
                    "reason": ("the simulator's geometry confirms the target" if said
                               else "the simulator's geometry says this is not the target")}
        return {"confirmed": False, "undecided": True, "cloud_called": False,
                NO_IDENTITY_TIER: True, "tier": "none",
                "reason": "the explore policy has no identity tier (tiered with "
                          "tier_search 'explore' confirms identity)"}

    def models(self) -> dict:
        """What looked, for status: the pipeline's detector and scorer names."""
        def name(m):
            return getattr(m, "weights", None) or getattr(m, "model_name", None) or (
                type(m).__name__ if m is not None else None)
        return {"detector": name(getattr(self.pipeline, "detector", None)),
                "scorer": name(getattr(self.pipeline, "scorer", None))}

    def close(self) -> None:
        close = getattr(self.pipeline, "close", None)
        if callable(close):
            close()
