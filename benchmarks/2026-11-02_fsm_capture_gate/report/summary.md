| scenario | method | captures | capture rate (95 % CI) | liftoff-to-capture median [IQR] (s) | t_total median (s) | min range median / max (m) |
|---|---|---|---|---|---|---|
| circle | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 10/10 (+17 infra re-runs) | 100% (72%–100%) | 9.6 [8.9–10.2] | 15.1 | 0.95 / 0.99 |
| circle | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 10/10 (+11 infra re-runs) | 100% (72%–100%) | 10.4 [9.2–14.3] | 16.8 | 0.74 / 0.94 |
| evade | `pipeline/rendezvous+gru+passthrough+px4_setpoint` | 10/10 (+7 infra re-runs) | 100% (72%–100%) | 19.8 [16.6–28.9] | 26.5 | 0.97 / 1.00 |
| evade | `pipeline/rendezvous+gru+mpc_12state+px4_setpoint` | 10/10 (+8 infra re-runs) | 100% (72%–100%) | 15.2 [9.8–18.1] | 21.2 | 0.79 / 0.95 |
