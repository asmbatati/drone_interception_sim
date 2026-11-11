| scenario | method | captures | capture rate (95 % CI) | liftoff-to-capture median [IQR] (s) | t_total median (s) | min range median / max (m) |
|---|---|---|---|---|---|---|
| circle | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 10/10 | 100% (72%–100%) | 12.1 [10.1–19.2] | 18.8 | 0.96 / 1.00 |
| circle | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 5/10 | 50% (24%–76%) | 13.8 [11.3–14.5] | 20.0 | 1.01 / 1.51 |
| circle | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 9/10 | 90% (60%–98%) | 11.3 [10.2–12.6] | 18.0 | 0.99 / 1.29 |
| circle | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 8/10 | 80% (49%–94%) | 10.1 [9.8–12.0] | 17.5 | 0.94 / 3.08 |
| circle | `pipeline/rendezvous+gru+passthrough+se3` | 4/10 | 40% (17%–69%) | 10.4 [9.2–11.9] | 16.7 | 1.05 / 1.53 |
| circle | `bt/rendezvous+gru` | 10/10 | 100% (72%–100%) | 7.4 [4.5–13.3] | 19.4 | 0.97 / 1.00 |
| evade | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 6/10 | 60% (31%–83%) | 27.7 [16.9–41.5] | 32.8 | 0.97 / 1.59 |
| evade | `pipeline/rendezvous+const_vel+passthrough+px4_setpoint` | 5/10 | 50% (24%–76%) | 17.2 [14.8–39.2] | 23.6 | 1.05 / 1.60 |
| evade | `pipeline/tail_chase+gru+passthrough+px4_setpoint` | 6/10 | 60% (31%–83%) | 29.4 [27.3–31.3] | 36.9 | 1.00 / 2.18 |
| evade | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 10/10 | 100% (72%–100%) | 17.2 [11.1–25.5] | 25.2 | 0.83 / 0.99 |
| evade | `pipeline/rendezvous+gru+passthrough+se3` | 7/10 | 70% (40%–89%) | 11.0 [9.8–17.9] | 18.3 | 0.91 / 1.78 |
| evade | `bt/rendezvous+gru` | 7/10 (+1 infra re-runs) | 70% (40%–89%) | 14.4 [11.2–26.9] | 18.6 | 1.00 / 2.66 |
