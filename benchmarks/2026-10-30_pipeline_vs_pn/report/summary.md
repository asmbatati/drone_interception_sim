| scenario | method | captures | capture rate (95 % CI) | liftoff-to-capture median [IQR] (s) | t_total median (s) | min range median / max (m) |
|---|---|---|---|---|---|---|
| circle | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 10/10 (+4 infra re-runs) | 100% (72%–100%) | 9.6 [9.0–10.5] | 16.0 | 0.96 / 1.00 |
| circle | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 10/10 (+2 infra re-runs) | 100% (72%–100%) | 9.5 [9.3–10.4] | 16.0 | 0.95 / 0.99 |
| circle | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 10/10 (+13 infra re-runs) | 100% (72%–100%) | 12.6 [10.3–18.7] | 18.5 | 0.99 / 1.00 |
| circle | `pipeline/rendezvous+gru+passthrough+se3` | 6/10 (+7 infra re-runs) | 60% (31%–83%) | 16.1 [13.8–24.0] | 22.6 | 0.99 / 4.22 |
| circle | `bt/rendezvous+gru` | 10/10 (+7 infra re-runs) | 100% (72%–100%) | 4.6 [4.3–5.5] | 7.9 | 0.97 / 1.00 |
| circle | `pn` | 4/10 (+11 infra re-runs) | 40% (17%–69%) | 13.4 [11.1–16.8] | 19.0 | 1.61 / 2.14 |
| circle | `pursuit` | 10/10 (+9 infra re-runs) | 100% (72%–100%) | 11.8 [9.0–13.7] | 18.5 | 0.82 / 0.99 |
| circle | `pn_int` | 9/10 (+3 infra re-runs) | 90% (60%–98%) | 7.7 [7.1–7.8] | 14.0 | 0.87 / 1.06 |
| circle | `rl/model_evasive_40k` | 2/10 (+2 infra re-runs) | 20% (6%–51%) | 28.9 [27.6–30.1] | 34.5 | 3.86 / 18.48 |
| evade | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 8/10 (+5 infra re-runs) | 80% (49%–94%) | 22.1 [10.7–45.2] | 27.6 | 0.99 / 1.06 |
| evade | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 9/10 (+10 infra re-runs) | 90% (60%–98%) | 10.3 [10.0–13.3] | 16.9 | 0.97 / 1.20 |
| evade | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 8/10 (+9 infra re-runs) | 80% (49%–94%) | 28.5 [24.3–40.3] | 33.7 | 0.99 / 1.12 |
| evade | `pipeline/rendezvous+gru+passthrough+se3` | 1/10 (+9 infra re-runs) | 10% (2%–40%) | 19.4 [19.4–19.4] | 26.1 | 3.06 / 4.13 |
| evade | `bt/rendezvous+gru` | 10/10 (+5 infra re-runs) | 100% (72%–100%) | 9.9 [4.5–12.9] | 13.6 | 0.96 / 1.00 |
| evade | `pn` | 1/10 (+8 infra re-runs) | 10% (2%–40%) | 13.5 [13.5–13.5] | 22.3 | 2.59 / 3.57 |
| evade | `pursuit` | 10/10 (+8 infra re-runs) | 100% (72%–100%) | 9.6 [7.7–15.8] | 15.7 | 0.88 / 0.99 |
| evade | `pn_int` | 8/10 | 80% (49%–94%) | 7.7 [6.8–8.3] | 14.1 | 0.84 / 1.83 |
| evade | `rl/model_evasive_40k` | 0/10 (+6 infra re-runs) | 0% (0%–28%) | — | — | 2.39 / 7.46 |
