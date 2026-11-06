| scenario | method | captures | capture rate (95 % CI) | liftoff-to-capture median [IQR] (s) | t_total median (s) | min range median / max (m) |
|---|---|---|---|---|---|---|
| circle | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 8/10 | 80% (49%–94%) | 10.7 [10.3–12.3] | 17.8 | 0.98 / 1.35 |
| circle | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 7/10 | 70% (40%–89%) | 9.8 [9.1–12.0] | 17.1 | 0.98 / 1.72 |
| circle | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 3/10 | 30% (11%–60%) | 14.2 [12.2–14.5] | 19.8 | 1.11 / 1.45 |
| circle | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 8/10 | 80% (49%–94%) | 14.0 [8.5–15.6] | 21.6 | 0.80 / 1.12 |
| circle | `pipeline/rendezvous+gru+passthrough+se3` | 10/10 | 100% (72%–100%) | 10.1 [9.6–13.1] | 17.1 | 0.88 / 0.98 |
| circle | `bt/rendezvous+gru` | 9/10 | 90% (60%–98%) | 5.0 [4.6–13.9] | 9.6 | 0.93 / 1.03 |
| circle | `pn` | 1/10 | 10% (2%–40%) | 11.3 [11.3–11.3] | 20.6 | 1.42 / 2.36 |
| circle | `pursuit` | 8/10 | 80% (49%–94%) | 12.4 [7.3–34.0] | 19.5 | 0.85 / 1.81 |
| circle | `pn_int` | 6/10 (+1 infra re-runs) | 60% (31%–83%) | 7.7 [6.7–8.4] | 15.5 | 0.99 / 1.74 |
| circle | `rl/model_evasive_40k` | 0/10 | 0% (0%–28%) | — | — | 3.33 / 11.86 |
| evade | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 4/10 | 40% (17%–69%) | 21.1 [14.1–32.5] | 27.8 | 1.10 / 5.17 |
| evade | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 5/10 | 50% (24%–76%) | 25.4 [17.8–26.3] | 31.4 | 1.02 / 2.11 |
| evade | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 5/10 | 50% (24%–76%) | 29.1 [27.8–33.2] | 33.6 | 0.99 / 2.07 |
| evade | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 7/10 | 70% (40%–89%) | 11.0 [9.8–17.5] | 18.7 | 0.91 / 1.24 |
| evade | `pipeline/rendezvous+gru+passthrough+se3` | 4/10 | 40% (17%–69%) | 10.5 [10.1–12.5] | 17.6 | 1.16 / 2.36 |
| evade | `bt/rendezvous+gru` | 8/10 (+1 infra re-runs) | 80% (49%–94%) | 28.9 [17.0–36.2] | 35.1 | 0.98 / 1.24 |
| evade | `pn` | 1/10 | 10% (2%–40%) | 26.8 [26.8–26.8] | 33.0 | 1.72 / 2.32 |
| evade | `pursuit` | 10/10 | 100% (72%–100%) | 13.3 [10.9–16.8] | 21.7 | 0.92 / 0.97 |
| evade | `pn_int` | 1/10 | 10% (2%–40%) | 8.9 [8.9–8.9] | 15.8 | 1.29 / 1.77 |
| evade | `rl/model_evasive_40k` | 0/10 (+1 infra re-runs) | 0% (0%–28%) | — | — | 3.52 / 13.86 |
