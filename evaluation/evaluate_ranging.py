#!/usr/bin/env python3
"""Evaluate real ranging measurements without inventing missing observations."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from evaluation.ranging_metrics import (  # noqa: E402
    RangingSummary,
    grouped_summaries,
    load_ranging_csv,
    summarize_ranging,
)


def _markdown_table(groups: dict[str, RangingSummary]) -> str:
    lines = [
        "| Group | n | MAE [m] | RMSE [m] | Bias [m] | P95 abs. [m] | Rel. MAE [%] |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name, summary in groups.items():
        lines.append(
            f"| {name} | {summary.n} | {summary.mae_m:.4f} | {summary.rmse_m:.4f} | "
            f"{summary.bias_m:.4f} | {summary.p95_abs_error_m:.4f} | "
            f"{summary.relative_mae_percent:.2f} |"
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="measured CSV file")
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    samples = load_ranging_csv(args.input)
    if not samples:
        print("No data rows: results remain NOT YET MEASURED.", file=sys.stderr)
        return 2

    overall = summarize_ranging(samples)
    grouped = grouped_summaries(samples)
    payload = {
        "source": str(args.input),
        "overall": overall.to_dict(),
        "groups": {
            dimension: {name: value.to_dict() for name, value in groups.items()}
            for dimension, groups in grouped.items()
        },
    }
    out_dir = args.out_dir or args.input.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "ranging_summary.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    report = [
        "# Object-ranging results",
        "",
        f"Source: `{args.input}`",
        "",
        "The 95% MAE interval is a deterministic non-parametric bootstrap interval.",
        "",
        "## Overall",
        "",
        _markdown_table({"all": overall}),
    ]
    for dimension, groups in grouped.items():
        report.extend(
            ["", f"## By {dimension.replace('_', ' ')}", "", _markdown_table(groups)]
        )
    (out_dir / "ranging_summary.md").write_text(
        "\n".join(report) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload["overall"], indent=2))
    print(
        f"Wrote {out_dir / 'ranging_summary.json'} and {out_dir / 'ranging_summary.md'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
