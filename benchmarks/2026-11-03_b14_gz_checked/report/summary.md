| scenario | method | captures | capture rate (95 % CI) | liftoff-to-capture median [IQR] (s) | t_total median (s) | min range median / max (m) |
|---|---|---|---|---|---|---|
| circle | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 0/10 | 0% (0%–28%) | — | — | 0.97 / 1.00 |
| circle | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 2/10 | 20% (6%–51%) | 17.1 [14.4–19.7] | 23.1 | 0.97 / 1.00 |
| circle | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 2/10 | 20% (6%–51%) | 14.7 [14.2–15.2] | 19.6 | 0.97 / 1.00 |
| circle | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 2/10 | 20% (6%–51%) | 12.3 [10.1–14.6] | 17.8 | 0.88 / 0.98 |
| circle | `pipeline/rendezvous+gru+passthrough+se3` | 2/10 | 20% (6%–51%) | 18.1 [13.9–22.2] | 23.8 | 0.72 / 0.98 |
| circle | `bt/rendezvous+gru` | 1/10 | 10% (2%–40%) | 9.0 [9.0–9.0] | 13.7 | 0.99 / 1.00 |
| circle | `pn` | 1/10 | 10% (2%–40%) | 10.9 [10.9–10.9] | 21.0 | 1.47 / 1.72 |
| circle | `pursuit` | 4/10 | 40% (17%–69%) | 12.5 [12.0–13.1] | 20.1 | 0.89 / 13.14 |
| circle | `pn_int` | 0/10 (+1 infra re-runs) | 0% (0%–28%) | — | — | 0.93 / 1.31 |
| circle | `rl/model_evasive_40k` | 0/10 | 0% (0%–28%) | — | — | 3.38 / 9.69 |
| evade | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 5/10 | 50% (24%–76%) | 25.3 [13.6–26.9] | 31.5 | 0.97 / 1.20 |
| evade | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 1/10 | 10% (2%–40%) | 46.5 [46.5–46.5] | 51.1 | 0.95 / 0.98 |
| evade | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 3/10 | 30% (11%–60%) | 20.3 [17.1–40.4] | 26.6 | 0.98 / 0.99 |
| evade | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 2/10 | 20% (6%–51%) | 20.4 [20.4–20.5] | 27.0 | 0.81 / 0.90 |
| evade | `pipeline/rendezvous+gru+passthrough+se3` | 1/10 | 10% (2%–40%) | 24.6 [24.6–24.6] | 30.1 | 1.13 / 1.58 |
| evade | `bt/rendezvous+gru` | 1/10 | 10% (2%–40%) | 26.7 [26.7–26.7] | 30.6 | 0.98 / 1.04 |
| evade | `pn` | 0/10 (+1 infra re-runs) | 0% (0%–28%) | — | — | 1.77 / 14.29 |
| evade | `pursuit` | 2/10 | 20% (6%–51%) | 11.4 [11.3–11.6] | 18.0 | 0.88 / 0.95 |
| evade | `pn_int` | 1/10 | 10% (2%–40%) | 6.9 [6.9–6.9] | 13.2 | 0.84 / 2.15 |
| evade | `rl/model_evasive_40k` | 0/10 | 0% (0%–28%) | — | — | 4.14 / 10.83 |
