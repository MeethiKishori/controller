use ml as conda baseline

![alt text](image.png)

![alt text](image-1.png)


![alt text](image-2.png)## Results

Five controllers, matched to identical closed-loop bandwidth (ωₙ = 3 rad/s, ζ = 0.9) so the comparison reflects controller structure rather than tuning effort.

### Steady-state error [m]

| scenario | PID | LQR | LQI | L1 | MPC |
|---|---|---|---|---|---|
| drop | 0.0904 | 0.3113 | 0.0299 | **0.0129** | 0.4415 |
| climb | 0.1683 | 0.0000 | 0.0755 | 0.0030 | **0.0012** |
| gust | **0.0121** | 0.0201 | 0.0378 | 0.0388 | 0.0536 |
| noise | 0.0005 | 0.0003 | 0.0003 | **0.0001** | 0.0004 |
| ratelimit | 0.3375 | 0.2379 | 0.5481 | **0.2519** | 0.3014 |

### RMS error [m]

| scenario | PID | LQR | LQI | L1 | MPC |
|---|---|---|---|---|---|
| drop | 0.1372 | 0.2233 | 0.0824 | **0.0282** | 0.3222 |
| climb | 0.8643 | 0.8206 | 0.8507 | **0.7918** | 0.8187 |
| gust | 0.1128 | 0.0975 | 0.0897 | **0.0342** | 0.1522 |
| noise | 0.0007 | 0.0006 | 0.0008 | **0.0004** | 0.0006 |
| ratelimit | 0.8991 | 0.8027 | 0.8996 | **0.7968** | 0.8652 |

### Actuator saturation [% of time]

| scenario | PID | LQR | LQI | L1 | MPC |
|---|---|---|---|---|---|
| climb | 4.0 | 3.4 | 5.9 | 4.7 | **2.6** |
| ratelimit | 6.0 | 4.9 | 12.2 | 5.1 | **1.2** |

## What the numbers show

The results suggest that steady-state rejection depends more on integral or estimator action than on controller complexity. LQR and MPC both leave a noticeable offset after the payload drop, around 0.31 m and 0.44 m respectively, because neither of them includes integral action. PID, LQI, and L1 remove that offset much more effectively.

The comparison also shows that estimation is better than pure integration when the disturbance changes over time. L1 performs increasingly well as the disturbance becomes time-varying: its advantage grows from about 2× better than LQI on the constant drop case to roughly 3× better than PID on the sinusoidal gust case. This makes sense because integral action has a fixed phase lag and cannot track a moving disturbance as quickly as the predictor-based estimate does.

MPC only stands out when the actuator hits its limits. It wins on the constrained scenarios, such as ratelimit, where saturation is around 1.2%, compared with 5–12% for the other controllers. On unconstrained scenarios, it behaves much like LQR, which is expected because an unconstrained MPC reduces to the same quadratic optimal regulator structure.

There is also a tradeoff with integral action. LQI is strong on the payload-drop case, but it is the worst performer on the rate-limited scenario, with saturation reaching 12.2%. That penalty comes from the extra integral state, which adds lag and makes the controller more prone to windup under constraint limits.