# Experiments

```bash
python scripts/record_experiment.py --seconds 30 --mode adaptive_fusion --scenario normal
bash scripts/replay_experiment.sh experiments/EXP_...
python evaluation/run_experiment.py --experiment experiments/EXP_...
python evaluation/generate_report.py --experiment experiments/EXP_...
```

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
