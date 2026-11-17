| scenario | method | captures | capture rate (95 % CI) | liftoff-to-capture median [IQR] (s) | t_total median (s) | min range median / max (m) |
|---|---|---|---|---|---|---|
| circle | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 10/10 | 100% (72%–100%) | 12.7 [12.4–14.2] | 18.4 | 0.97 / 1.00 |
| circle | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 10/10 | 100% (72%–100%) | 9.8 [9.4–12.2] | 16.8 | 0.93 / 1.00 |
| circle | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 10/10 | 100% (72%–100%) | 11.9 [10.9–15.6] | 19.3 | 0.94 / 1.00 |
| circle | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 10/10 | 100% (72%–100%) | 9.6 [9.1–19.9] | 17.8 | 0.80 / 0.93 |
| circle | `pipeline/rendezvous+gru+passthrough+se3` | 8/10 | 80% (49%–94%) | 9.6 [8.6–25.5] | 16.5 | 0.92 / 1.47 |
| circle | `bt/rendezvous+gru` | 10/10 | 100% (72%–100%) | 6.6 [4.5–8.6] | 11.9 | 0.95 / 1.00 |
| circle | `pn` | 2/10 | 20% (6%–51%) | 10.6 [10.6–10.7] | 17.6 | 1.49 / 1.59 |
| circle | `pursuit` | 10/10 | 100% (72%–100%) | 8.2 [7.0–11.4] | 15.5 | 0.90 / 0.98 |
| circle | `pn_int` | 8/10 | 80% (49%–94%) | 8.2 [8.0–8.4] | 15.2 | 0.81 / 1.51 |
| circle | `rl/model_evasive_40k` | 0/10 | 0% (0%–28%) | — | — | 3.93 / 13.91 |
| evade | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 9/10 | 90% (60%–98%) | 19.6 [13.8–24.0] | 26.4 | 0.96 / 1.02 |
| evade | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 10/10 | 100% (72%–100%) | 22.4 [10.7–25.1] | 27.8 | 0.94 / 1.00 |
| evade | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 10/10 | 100% (72%–100%) | 21.0 [17.2–25.0] | 28.5 | 0.97 / 0.99 |
| evade | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 10/10 | 100% (72%–100%) | 11.4 [9.5–14.5] | 18.4 | 0.77 / 0.95 |
| evade | `pipeline/rendezvous+gru+passthrough+se3` | 4/10 | 40% (17%–69%) | 10.6 [10.1–12.7] | 18.1 | 1.03 / 1.31 |
| evade | `bt/rendezvous+gru` | 9/10 | 90% (60%–98%) | 10.0 [4.3–10.3] | 15.0 | 0.97 / 1.13 |
| evade | `pn` | 0/10 | 0% (0%–28%) | — | — | 2.15 / 2.65 |
| evade | `pursuit` | 10/10 | 100% (72%–100%) | 8.9 [7.1–11.7] | 16.7 | 0.93 / 0.99 |
| evade | `pn_int` | 7/10 | 70% (40%–89%) | 8.1 [7.2–8.4] | 15.6 | 0.95 / 1.45 |
| evade | `rl/model_evasive_40k` | 0/10 | 0% (0%–28%) | — | — | 4.07 / 14.31 |
