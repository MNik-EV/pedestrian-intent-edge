#!/usr/bin/env python3
"""Generate publication-oriented HTML report + plots from an experiment folder."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _try_plots(exp: Path, plots_dir: Path) -> list[str]:
    generated: list[str] = []
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return generated

    metrics_csv = exp / "metrics.csv"
    if not metrics_csv.exists():
        return generated

    rows = list(csv.DictReader(metrics_csv.open(encoding="utf-8")))
    if not rows:
        return generated

    # Confidence / pose time series if present
    by_metric: dict[str, list[tuple[float, float]]] = {}
    for r in rows:
        try:
            t = float(r["wall_ns"]) / 1e9
            v = float(r["value"])
        except Exception:
            continue
        by_metric.setdefault(r["metric"], []).append((t, v))

    for metric, series in by_metric.items():
        series = sorted(series)
        xs = [s[0] - series[0][0] for s in series]
        ys = [s[1] for s in series]
        fig, ax = plt.subplots(figsize=(7, 3.5), dpi=140)
        ax.plot(xs, ys, color="#2eb7c9", lw=1.5)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(metric)
        ax.set_title(f"{exp.name} · {metric}")
        ax.grid(True, alpha=0.3)
        for ext in ("png", "pdf", "svg"):
            path = plots_dir / f"{metric}.{ext}"
            fig.savefig(path, bbox_inches="tight")
            generated.append(str(path.name))
        plt.close(fig)
    return generated


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--experiment", required=True)
    args = ap.parse_args()
    exp = Path(args.experiment)
    plots_dir = exp / "plots"
    plots_dir.mkdir(exist_ok=True)
    meta = {}
    meta_path = exp / "metadata.json"
    if meta_path.exists():
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
    eval_summary = {}
    if (exp / "eval_summary.json").exists():
        eval_summary = json.loads((exp / "eval_summary.json").read_text(encoding="utf-8"))

    generated = _try_plots(exp, plots_dir)
    summary = {
        "experiment_id": exp.name,
        "metadata": meta,
        "evaluation": eval_summary or {"status": "NOT YET MEASURED"},
        "plots": generated,
    }
    (exp / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8"/><title>Report {exp.name}</title>
<style>body{{font-family:Georgia,serif;max-width:900px;margin:2rem auto;padding:0 1rem;color:#222}}
code,pre{{font-family:Consolas,monospace;background:#f4f6f8;padding:.2rem .4rem}}
img{{max-width:100%;border:1px solid #ddd}}</style></head><body>
<h1>Experiment Report: {exp.name}</h1>
<p><strong>Status:</strong> Metrics marked NOT YET MEASURED until ground-truth experiments are run.</p>
<h2>Metadata</h2>
<pre>{json.dumps(meta, indent=2)}</pre>
<h2>Evaluation</h2>
<pre>{json.dumps(eval_summary or {"status": "NOT YET MEASURED"}, indent=2)}</pre>
<h2>Plots</h2>
{''.join(f'<p>{p}</p><img src="plots/{p}"/>' for p in generated if p.endswith('.png')) or '<p>No plots generated (install matplotlib or add metrics.csv).</p>'}
</body></html>"""
    (exp / "experiment_report.html").write_text(html, encoding="utf-8")
    # Copy metrics if present
    if (exp / "metrics.csv").exists():
        pass
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
