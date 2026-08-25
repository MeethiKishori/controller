import matplotlib.pyplot as plt
import numpy as np
#gemini version
# Simulation parameters
dt = 0.001  # Time step (seconds)
T = 10.0  # Total time (seconds)
steps = int(T / dt)
time = np.linspace(0, T, steps)

# Physical System (Plant): x_dot = A*x + B*(u + d)
A, B = -1.0, 1.0

# Reference Model / Predictor: x_hat_dot = Am*x_hat + B*(u + sigma_hat)
Am = -2.0  # Desired stable pole

# L1 Filter parameters
omega_c = 10.0  # Low-pass filter cutoff frequency (rad/s)
Gamma = 1000.0  # Adaptation gain (high rate)

# Initialize variables
x = 0.0  # Actual state
x_hat = 0.0  # State predictor
sigma_hat = 0.0  # Estimated disturbance
u_ad = 0.0  # Filtered adaptive control output

# Data logging
x_history, u_ad_history, d_history = [], [], []

for t in time:
    # 1. Simulate an external disturbance 'd' (e.g., sudden wind gust at t=3s)
    d = 3.0 if t >= 3.0 else 0.0

    # Desired reference trajectory
    r = np.sin(0.5 * np.pi * t)

    # Baseline control (proportional)
    u_m = r

    # Total control output fed to system and predictor
    u = u_m + u_ad

    # 2. Physical System step
    x_dot = A * x + B * (u + d)
    x += x_dot * dt

    # 3. State Predictor step
    x_hat_dot = Am * x_hat + (A - Am) * x + B * (u + sigma_hat)
    x_hat += x_hat_dot * dt

    # 4. Adaptive Law (rapid disturbance estimation)
    error = x_hat - x
    sigma_hat_dot = -Gamma * error * B
    sigma_hat += sigma_hat_dot * dt

    # 5. Low-Pass Filter step: C(s) = omega_c / (s + omega_c)
    # Filter equation: u_ad_dot = -omega_c * u_ad - omega_c * sigma_hat
    u_ad_dot = -omega_c * u_ad - omega_c * sigma_hat
    u_ad += u_ad_dot * dt

    # Log data
    x_history.append(x)
    u_ad_history.append(u_ad)
    d_history.append(d)

# Plotting results
plt.figure(figsize=(10, 5))
plt.plot(time, d_history, "r--", label="Actual Disturbance (d)")
plt.plot(time, u_ad_history, "g", label="Filtered L1 Adaptation Control (u_ad)")
plt.plot(time, x_history, "b", label="System State (x)")
plt.title("L1 Adaptive Control Simulation")
plt.xlabel("Time (s)")
plt.ylabel("Magnitude")
plt.legend()
plt.grid(True)
plt.show()