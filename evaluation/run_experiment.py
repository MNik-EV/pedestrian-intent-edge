#!/usr/bin/env python3
"""Run offline evaluation for a recorded experiment.

Does NOT fabricate metrics. If ground truth is missing, marks NOT YET MEASURED.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from evaluation.metrics import PoseSample, absolute_trajectory_error, relative_pose_error


METHODS = [
    "lidar_only",
    "camera_only",
    "fixed_fusion",
    "adaptive_fusion",
    "adaptive_fusion_dynfilter",
]


def load_poses(path: Path) -> list[PoseSample]:
    if not path.exists():
        return []
    samples = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        d = json.loads(line)
        samples.append(PoseSample(d["t"], d["x"], d["y"], d.get("yaw", 0.0)))
    return samples


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--experiment", required=True, help="Path to experiments/EXP_xxx")
    ap.add_argument("--method", default="adaptive_fusion", choices=METHODS)
    args = ap.parse_args()
    exp = Path(args.experiment)
    est = load_poses(exp / "poses_est.jsonl")
    gt = load_poses(exp / "poses_gt.jsonl")
    result = {
        "experiment": exp.name,
        "method": args.method,
        "ate": "NOT YET MEASURED",
        "rpe": "NOT YET MEASURED",
    }
    if est and gt:
        result["ate"] = absolute_trajectory_error(est, gt)
        result["rpe"] = relative_pose_error(est, gt)
    else:
        result["note"] = "Missing poses_est.jsonl and/or poses_gt.jsonl"
    out = exp / "eval_summary.json"
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
