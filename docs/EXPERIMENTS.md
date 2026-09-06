# Experiments

```bash
python scripts/record_experiment.py --seconds 30 --mode adaptive_fusion --scenario normal
bash scripts/replay_experiment.sh experiments/EXP_...
python evaluation/run_experiment.py --experiment experiments/EXP_...
python evaluation/generate_report.py --experiment experiments/EXP_...
```

For final physical object-ranging and runtime measurements:

```bash
python evaluation/evaluate_ranging.py --input experiments/FINAL_.../ranging_measurements.csv
python evaluation/evaluate_live_run.py --experiment experiments/SHOW_...
```

Use `evaluation/ranging_measurements_template.csv` as the measurement schema. Empty inputs
remain `NOT YET MEASURED`; the evaluators do not synthesize observations.

Experiment folder layout:

```
experiments/EXP_.../
  metadata.yaml / metadata.json
  metrics.csv
  events.jsonl
  telemetry.sqlite
  rosbag/
  screenshots/
  plots/
  README.md
```
