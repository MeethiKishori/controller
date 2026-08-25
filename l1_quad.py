import numpy as np, matplotlib.pyplot as plt

# --- quadrotor roll axis:  Ixx*phi_ddot = tau + d ---
Ixx, wn, ze = 0.01, 8.0, 0.8          # inertia, bandwidth, damping
Gamma, k = 2000.0, 60.0               # adaptation gain, L1 filter bandwidth
P_b = np.array([0.7812, 3.9673])      # (P*b)', P from lyap(Am',I)

dt, T = 1e-4, 6.0
t = np.arange(0, T, dt)

x = np.zeros(2)      # true    [phi, phi_dot]
xh = np.zeros(2)     # predictor
xr = np.zeros(2)     # reference model
sh = 0.0             # sigma_hat  (disturbance estimate)
u = 0.0              # L1 output
log = []

for tk in t:
    r = 0.2                                   # commanded roll [rad]
    d = 0.15 if tk >= 2.0 else 0.0            # unknown gust torque [N*m]

    tau = Ixx*(wn**2*(r - x[0]) - 2*ze*wn*x[1]) + u      # PD + L1
    tau = np.clip(tau, -1.0, 1.0)

    x  += dt*np.array([x[1], (tau + d)/Ixx])            # true plant
    xh += dt*np.array([xh[1], wn**2*(r-xh[0]) - 2*ze*wn*xh[1] + (u+sh)/Ixx])
    xr += dt*np.array([xr[1], wn**2*(r-xr[0]) - 2*ze*wn*xr[1]])

    sh += dt*(-Gamma*np.dot(xh - x, P_b))               # adaptation
    u  += dt*k*(-sh - u)                                # low-pass filter

    log.append([tk, x[0], xr[0], sh])

log = np.array(log)
print(f"sigma_hat = {log[-1,3]:.4f}  (true 0.15)")
print(f"final error = {np.degrees(log[-1,1]-log[-1,2]):.3f} deg")

fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
ax[0].plot(log[:,0], np.degrees(log[:,1]), label='roll')
ax[0].plot(log[:,0], np.degrees(log[:,2]), 'k--', label='reference')
ax[0].set_ylabel('roll [deg]'); ax[0].legend(); ax[0].grid(True)
ax[1].plot(log[:,0], log[:,3], label=r'$\hat\sigma$')
ax[1].axhline(0.15, ls='--', c='k', label='true disturbance')
ax[1].set_ylabel('torque [N m]'); ax[1].set_xlabel('t [s]')
ax[1].legend(); ax[1].grid(True)
plt.tight_layout(); plt.show()

# Try: Gamma = 0   -> L1 off, roll sits 13.4 deg below reference forever
#      k = 30      -> slower recovery (0.42 s instead of 0.05 s)
#      Gamma = 5000-> tracking still fine, but sigma_hat drifts to nonsense