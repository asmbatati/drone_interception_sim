# Interception benchmark matrix

Cross-method capture benchmark on the standard scenario (scripted circling
target, spawn offset 10 m, capture radius 1.0 m, 120 s timeout). Produced by
`interception_metrics` — every run appends one row to
`/tmp/interception_results.csv`; the method string self-labels the backend
combination, so the table below is the file verbatim.

## Results (2026-06-11)

| method | outcome | time_to_intercept [s] | min_range [m] |
|---|---|---|---|
| pipeline/rendezvous+**gru**+passthrough+px4_setpoint | **capture** | 15.60 | 0.78 |
| pipeline/rendezvous+**gru**+passthrough+px4_setpoint (repeat) | **capture** | 16.10 | 0.87 |
| pipeline/tail_chase+**gru**+passthrough+px4_setpoint | **capture** | 15.30 | 0.83 |
| pipeline/pn_lead+gru+passthrough+px4_setpoint | timeout | 120.00 | 3.91 |
| pipeline/rendezvous+**const_vel**+passthrough+px4_setpoint | **capture** | 12.75 | 0.97 |
| pn (d2dtracker_interception guidance, velocity-PN baseline) | **capture** | 10.70 | 0.75 |

## Reading the table

- **Timing is NOT directly comparable across the two families.** Pipeline rows
  include the FSM head's arm + `AUTO.TAKEOFF` phase (~11 s) before OFFBOARD
  pursuit begins; the PN baseline arms straight into OFFBOARD velocity flight.
  Engagement-only time for the pipeline (OFFBOARD→capture) is ~4–5 s — on the
  engagement itself the full pipeline closes *faster* than the 10.7 s PN run.
- **`pn_lead` as a planner does not converge** (closest approach 3.91 m): PN is
  a velocity/acceleration law; converting its lead point into a *position*
  reference for the setpoint controller maintains a standoff on a circling
  target. PN belongs in velocity space — the dedicated `pn` baseline (velocity
  setpoints) captures cleanly. Use `rendezvous`/`tail_chase` planners with the
  position-control cascade, or PN via `guidance.launch.py`.
- **GRU vs const_vel ablation:** both capture under the `rendezvous` planner
  (it re-plans continuously, so it is forgiving of forecast error), but the GRU
  rows show tighter terminal misses (0.78/0.87 vs 0.97 m, i.e. closer to the
  target when crossing the radius). Standalone prediction accuracy is where the
  GRU dominates: 0.044 vs 0.270 m RMSE @1 s offline, 0.19 m RMSE live (see
  ARCHITECTURE.md §7).
- **Repeatability:** the rendezvous+gru repeat agrees within 0.5 s / 0.09 m.
  Times vary by a few seconds with the target's phase on its circle at the
  moment OFFBOARD engages — single-run deltas of that size are phase luck, not
  method differences.

## Caveats

- n=1 per combination (n=2 for rendezvous+gru) on a single scripted trajectory;
  no evasion. For publishable numbers: repeat ≥10× per combo, randomize the
  target phase, and add the `evade` target mode.
- `min_range` at capture only says where the 10 Hz sampling crossed the 1.0 m
  radius; sub-0.1 m differences are not meaningful.
- `mpc_12state` and the `se3`/`pn_velocity` control backends were not in this
  matrix (passthrough+px4_setpoint everywhere) — they are the next axes to add
  once flight-validated.

## Reproduce

```bash
# sim up (FastRTPS), then one row per run — vary the backend args:
ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=fsm metrics:=true \
    planner:=rendezvous predictor:=gru csv_path:=/tmp/trace_r_gru.csv
# PN baseline:
ros2 launch d2dtracker_interception guidance.launch.py &
ros2 launch drone_interception_sim metrics.launch.py method:=pn csv_path:=/tmp/trace_pn.csv
# table:
column -s, -t /tmp/interception_results.csv
```
Between runs, wait for the interceptor to RTL + disarm (the FSM logs
`Return -> Done`). Per-run 10 Hz range/position traces are the `trace_csv`
column.
