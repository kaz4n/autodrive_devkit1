"""Deprecated no-op map manager.

The competition-facing stack intentionally does not depend on pre-built maps, saved laps, or
track fingerprints. This stub remains only to keep legacy imports from crashing.
"""

from __future__ import annotations

from typing import Optional

from .models import MapLocalizerOutput, TrackBoundaries, TrackMapRecord, VehicleState


class TrackMapManager:
    def __init__(self, *args, **kwargs) -> None:
        self._active_track_id = ''

    @property
    def active_track_id(self) -> str:
        return ''

    def active_record(self) -> Optional[TrackMapRecord]:
        return None

    def ensure_active_track(self, observation: TrackBoundaries, stamp: float) -> str:
        return ''

    def localize(self, prior_pose: VehicleState, observation: TrackBoundaries, stamp: float) -> MapLocalizerOutput:
        return MapLocalizerOutput(pose=prior_pose, corrected=False, track_id='')

    def update_map(self, pose: VehicleState, observation: TrackBoundaries, stamp: float) -> Optional[TrackMapRecord]:
        return None

    def get_local_boundary_prior(self, pose: VehicleState, lookahead_m: float) -> TrackBoundaries:
        return TrackBoundaries(stamp=pose.stamp)

    def list_tracks(self):
        return []

    def save_active(self, force: bool = False, stamp: float = 0.0) -> None:
        return None
