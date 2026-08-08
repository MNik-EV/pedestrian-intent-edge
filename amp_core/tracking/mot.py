"""Multi-object tracking with temporal dynamic/static classification."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from amp_core.common.types import BoundingBox, Detection, Timestamp, TrackedObject


@dataclass
class TrackerConfig:
    iou_threshold: float = 0.3
    max_age: int = 30
    min_hits: int = 3
    dynamic_speed_mps: float = 0.15
    dynamic_confirm_frames: int = 5
    pixels_per_meter: float = 120.0  # coarse geometric fallback


def _iou(a: BoundingBox, b: BoundingBox) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    if inter <= 0:
        return 0.0
    area_a = a.width * a.height
    area_b = b.width * b.height
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


@dataclass
class _TrackState:
    track_id: int
    class_name: str
    confidence: float
    bbox: BoundingBox
    age: int = 1
    hits: int = 1
    time_since_update: int = 0
    last_seen: Timestamp = field(default_factory=Timestamp.now)
    trajectory: list[tuple[float, float]] = field(default_factory=list)
    velocity_mps: float = 0.0
    motion_evidence: int = 0
    is_dynamic: bool = False
    distance_m: float | None = None
    bearing_deg: float | None = None


class MultiObjectTracker:
    """IOU association tracker with motion-evidence dynamic labeling."""

    def __init__(self, cfg: TrackerConfig | None = None) -> None:
        self.cfg = cfg or TrackerConfig()
        self._tracks: dict[int, _TrackState] = {}
        self._next_id = 1

    def update(
        self,
        detections: list[Detection],
        distances: dict[int, float] | None = None,
        bearings: dict[int, float] | None = None,
        dt: float = 0.1,
    ) -> list[TrackedObject]:
        distances = distances or {}
        bearings = bearings or {}

        # Greedy IOU matching
        track_ids = list(self._tracks.keys())
        unmatched_tracks = set(track_ids)
        unmatched_dets = set(range(len(detections)))
        matches: list[tuple[int, int]] = []

        pairs: list[tuple[float, int, int]] = []
        for ti, tid in enumerate(track_ids):
            for di, det in enumerate(detections):
                score = _iou(self._tracks[tid].bbox, det.bbox)
                if score >= self.cfg.iou_threshold:
                    pairs.append((score, ti, di))
        pairs.sort(reverse=True)
        used_t: set[int] = set()
        used_d: set[int] = set()
        for score, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            matches.append((track_ids[ti], di))
            unmatched_tracks.discard(track_ids[ti])
            unmatched_dets.discard(di)

        for tid, di in matches:
            det = detections[di]
            tr = self._tracks[tid]
            prev_cx, prev_cy = tr.bbox.cx, tr.bbox.cy
            tr.bbox = det.bbox
            tr.confidence = det.confidence
            tr.class_name = det.class_name
            tr.hits += 1
            tr.age += 1
            tr.time_since_update = 0
            tr.last_seen = det.timestamp
            tr.trajectory.append((det.bbox.cx, det.bbox.cy))
            # Image-plane velocity -> approximate m/s
            dx = det.bbox.cx - prev_cx
            dy = det.bbox.cy - prev_cy
            pix_speed = math.hypot(dx, dy) / max(dt, 1e-3)
            tr.velocity_mps = pix_speed / self.cfg.pixels_per_meter
            if tr.velocity_mps >= self.cfg.dynamic_speed_mps:
                tr.motion_evidence += 1
            else:
                tr.motion_evidence = max(0, tr.motion_evidence - 1)
            tr.is_dynamic = tr.motion_evidence >= self.cfg.dynamic_confirm_frames
            # Do NOT assume class==person implies dynamic
            if tid in distances:
                tr.distance_m = distances[tid]
            if tid in bearings:
                tr.bearing_deg = bearings[tid]

        for di in unmatched_dets:
            det = detections[di]
            tid = self._next_id
            self._next_id += 1
            self._tracks[tid] = _TrackState(
                track_id=tid,
                class_name=det.class_name,
                confidence=det.confidence,
                bbox=det.bbox,
                last_seen=det.timestamp,
                trajectory=[(det.bbox.cx, det.bbox.cy)],
                distance_m=distances.get(tid),
                bearing_deg=bearings.get(tid),
            )

        for tid in list(unmatched_tracks):
            tr = self._tracks[tid]
            tr.age += 1
            tr.time_since_update += 1
            if tr.time_since_update > self.cfg.max_age:
                del self._tracks[tid]

        return self.get_confirmed()

    def get_confirmed(self) -> list[TrackedObject]:
        out: list[TrackedObject] = []
        for tr in self._tracks.values():
            if tr.hits < self.cfg.min_hits and tr.time_since_update == 0:
                # still warming up; include with age for visibility in dashboard
                pass
            out.append(
                TrackedObject(
                    track_id=tr.track_id,
                    class_name=tr.class_name,
                    confidence=tr.confidence,
                    bbox=tr.bbox,
                    distance_m=tr.distance_m,
                    bearing_deg=tr.bearing_deg,
                    velocity_mps=tr.velocity_mps,
                    is_dynamic=tr.is_dynamic,
                    age=tr.age,
                    hits=tr.hits,
                    last_seen=tr.last_seen,
                    trajectory=list(tr.trajectory),
                )
            )
        return out
