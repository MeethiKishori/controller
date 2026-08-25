"""
L1 adaptive control on a quadrotor roll axis, with four disturbance types
and six diagnostic curves.

Change DIST below and re-run. Each disturbance teaches something different.
"""
import numpy as np, matplotlib.pyplot as plt


DIST = 'impulse'   # step | impulse | sine | uniform | chirp | ramp | pulse | doublet | none
REF  = 'step'      # step | sine | square | ramp | zero

# --- plant: Ixx*phi_ddot = tau + d ---
Ixx, wn, ze = 0.01, 8.0, 0.8
Gamma, k    = 2000.0, 60.0            # adaptation gain 2000, filter bandwidth 60
P_b = np.array([0.7812, 3.9673])      # (P*b)' from lyap(Am', I)

dt, T = 1e-5, 8.0
t = np.arange(0, T, dt)
rng = np.random.default_rng(0)

# ---------------- disturbance signals ----------------
if DIST == 'step':                                   # constant gust
    d_vec = np.where(t >= 2.0, 0.15, 0.0)
elif DIST == 'sine':                                 # slow periodic gust
    d_vec = 0.15*np.sin(2*np.pi*0.5*t)
elif DIST == 'uniform':                              # turbulence, held 20 ms
    hold = int(0.02/dt)
    d_vec = np.repeat(rng.uniform(-0.15, 0.15, len(t)//hold + 1), hold)[:len(t)]
elif DIST == 'chirp':                                # sweep 0.2 -> 20 Hz
    f0, f1 = 0.2, 20.0
    d_vec = 0.10*np.sin(2*np.pi*(f0*t + 0.5*(f1-f0)/T*t**2))
elif DIST == 'impulse':                              # 20 ms hit at t=2
    d_vec = np.where((t >= 2.0) & (t < 2.02), 3.0, 0.0)
elif DIST == 'ramp':                                 # slowly growing
    d_vec = np.clip(0.05*(t - 2.0), 0, 0.3)
elif DIST == 'pulse':                                # on/off square
    d_vec = 0.15*(np.sign(np.sin(2*np.pi*0.4*t)) > 0)
elif DIST == 'doublet':                              # +then- , 0.5 s each
    d_vec = np.where((t >= 2) & (t < 2.5), 0.15,
            np.where((t >= 2.5) & (t < 3.0), -0.15, 0.0))
elif DIST == 'none':
    d_vec = np.zeros_like(t)

# ---------------- reference signals ----------------
if   REF == 'step':   r_vec = np.full_like(t, 0.2)
elif REF == 'sine':   r_vec = 0.2*np.sin(2*np.pi*0.25*t)
elif REF == 'square': r_vec = 0.2*np.sign(np.sin(2*np.pi*0.25*t))
elif REF == 'ramp':   r_vec = np.clip(0.05*t, 0, 0.3)
elif REF == 'zero':   r_vec = np.zeros_like(t)

# ---------------- simulate ----------------
x = np.zeros(2); xh = np.zeros(2); xr = np.zeros(2)
sh = 0.0; u = 0.0
L = np.zeros((len(t), 6))

for i, tk in enumerate(t):
    r, d = r_vec[i], d_vec[i]

    tau = Ixx*(wn**2*(r - x[0]) - 2*ze*wn*x[1]) + u          # PD + L1
    tau = np.clip(tau, -1.0, 1.0)

    x  += dt*np.array([x[1],  (tau + d)/Ixx])                        # truth
    xh += dt*np.array([xh[1], wn**2*(r-xh[0]) - 2*ze*wn*xh[1] + (u+sh)/Ixx])
    xr += dt*np.array([xr[1], wn**2*(r-xr[0]) - 2*ze*wn*xr[1]])      # reference

    sh += dt*(-Gamma*np.dot(xh - x, P_b))                    # adaptation
    u  += dt*k*(-sh - u)                                     # low-pass filter

    L[i] = [x[0], xr[0], sh, d, u, (xh - x)[0]]

roll, ref, sig_h, d_true, u_log, xtil = L.T
m = t > 3.0
print(f"disturbance      : {DIST}")
print(f"tracking rms     : {np.degrees(np.sqrt(np.mean((roll[m]-ref[m])**2))):.3f} deg")
print(f"estimation rms   : {np.sqrt(np.mean((sig_h[m]-d_true[m])**2)):.4f} Nm")
print(f"sigma_hat range  : [{sig_h.min():.3f}, {sig_h.max():.3f}]   (d is +-{np.abs(d_true).max():.2f})")
print(f"control |u| max  : {np.abs(u_log).max():.3f} Nm")

# ---------------- six curves ----------------
fig, ax = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
fig.suptitle(f'L1 adaptive control  -  disturbance: {DIST}   reference: {REF}')

ax[0, 0].plot(t, np.degrees(roll), label='roll')
ax[0, 0].plot(t, np.degrees(ref), 'k--', lw=1, label='reference')
ax[0, 0].set_ylabel('roll [deg]')
ax[0, 0].set_title('1. Does it track?')
ax[0, 0].legend()

ax[0, 1].plot(t, np.degrees(roll - ref), 'C3')
ax[0, 1].axhline(0, c='k', lw=.5)
ax[0, 1].set_ylabel('error [deg]')
ax[0, 1].set_title('2. Tracking error')

ax[1, 0].plot(t, sig_h, label=r'$\hat\sigma$')
ax[1, 0].plot(t, d_true, 'k--', lw=1, label='true d')
ax[1, 0].set_ylabel('torque [N m]')
ax[1, 0].set_title('3. Estimate vs truth')
ax[1, 0].legend()

ax[1, 1].plot(t, sig_h - d_true, 'C3')
ax[1, 1].axhline(0, c='k', lw=.5)
ax[1, 1].set_ylabel('[N m]')
ax[1, 1].set_title(r'4. Estimation error $\hat\sigma-d$')

ax[2, 0].plot(t, u_log, 'C2')
ax[2, 0].plot(t, -sig_h, 'C1', lw=.8, alpha=.7, label=r'$-\hat\sigma$ (unfiltered)')
ax[2, 0].set_ylabel('u [N m]')
ax[2, 0].set_xlabel('t [s]')
ax[2, 0].set_title('5. Control signal - filter smooths the estimate')
ax[2, 0].legend()

ax[2, 1].plot(t, d_true, 'C4', linewidth=2, label='disturbance d(t)')
ax[2, 1].axhline(0, c='k', lw=.5)
ax[2, 1].set_ylabel('disturbance [N m]')
ax[2, 1].set_xlabel('t [s]')
ax[2, 1].set_title('6. Disturbance input')
ax[2, 1].legend()


# ax[2, 1].plot(t, np.degrees(xtil), 'C4')
# ax[2, 1].axhline(0, c='k', lw=.5)
# ax[2, 1].set_ylabel('[deg]')
# ax[2, 1].set_xlabel('t [s]')
# ax[2, 1].set_title(r'6. Prediction error $\hat x - x$')


for a in ax.flat:
    a.grid(True, alpha=.3)

plt.tight_layout()
plt.show()

# ---------------------------------------------------------------------
# WHAT EACH DISTURBANCE TEACHES
#
# 'step'    sigma_hat lands on 0.15 exactly. Panel 4 goes to zero.
#           Panel 3 overshoots ~100% first - the adaptation loop rings.
#
# 'sine'    sigma_hat tracks perfectly (panel 4 ~ 0) yet roll still
#           deviates ~0.5 deg. Estimation is not the bottleneck here;
#           the filter delay between sigma_hat and u is.
#
# 'uniform' THE INTERESTING ONE. sigma_hat swings +-0.53 chasing a +-0.15
#           signal - 3x overshoot, panel 3 looks broken. But panel 5 shows
#           u stays smooth and small. The filter refuses to pass the noise.
#           Tracking is still 0.2 deg. Bad estimate, good control.
#
# 'chirp'   Panel 5: the u envelope shrinks as frequency rises past
#           k = 60 rad/s = 9.5 Hz. That is the filter rolling off, visible.
#           Panel 2: tracking error IMPROVES anyway at high frequency -
#           the airframe's own inertia rejects fast disturbances for free.
#           You do not need the controller up there.
#
# Panel 6 is the one you never normally see. Every correction the
# controller makes traces back to this signal being nonzero.
# ---------------------------------------------------------------------