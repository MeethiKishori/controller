"""
Controller comparison scaffold - quadrotor altitude channel.

One plant, five controllers, five scenarios. Each scenario is designed so a
DIFFERENT controller should win; if one controller wins everything, either
the tuning is unfair or the scenario is not testing what you think.

    python compare.py            -> run the whole 5x5 matrix
    python compare.py drop       -> run one scenario with plots
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are, solve_lyapunov

# ============================ PLANT ============================
g      = 9.81
m_dry  = 1.0
m_pay  = 0.4
m_nom  = m_dry + m_pay
z_ref  = 2.0

T_MAX  = 2.2*m_nom*g          # thrust ceiling - tight enough that MPC matters
T_MIN  = 0.0

dt, T_END = 1e-3, 6.0         # 6 s is plenty; nothing here is slower than ~1.5 s
t = np.arange(0, T_END, dt)

# shared design point - EVERY controller is tuned to this bandwidth
WN, ZE = 3.0, 0.9
A  = np.array([[0.0, 1.0], [0.0, 0.0]])      # [z, zdot], input = accel
B  = np.array([[0.0], [1.0]])

# ============================ SCENARIOS ============================
TAU_DROP = 0.15               # payload release takes 150 ms, not instant

def scenario(name):
    """returns mass(t), disturbance accel d(t), reference z_ref(t), noise_std"""
    m = np.full_like(t, m_nom)
    d = np.zeros_like(t)
    r = np.full_like(t, z_ref)
    noise = 0.0

    if name == 'drop':                        # tests integral / disturbance action
        s = np.clip((t - 2.0)/TAU_DROP, 0, 1)         # smooth release
        s = s*s*(3 - 2*s)                              # smoothstep
        m = m_nom - m_pay*s

    elif name == 'climb':                     # tests constraint handling
        r = np.where(t < 1.0, z_ref, z_ref + 3.0)      # big step into the ceiling

    elif name == 'gust':                      # tests tracking a MOVING disturbance
        d = 2.0*np.sin(2*np.pi*0.30*t)*(t >= 1.0)

    elif name == 'noise':                     # tests effort/noise tradeoff
        noise = 0.02                                   # 2 cm altimeter noise
       # d = 0.5*np.sin(2*np.pi*0.3*t)

    elif name == 'ratelimit':                 # tests planning around a limit
        r = z_ref + 0.8*np.sign(np.sin(2*np.pi*0.35*t))

    return m, d, r, noise

SCENARIOS = ['drop', 'climb', 'gust', 'noise', 'ratelimit']

# ============================ CONTROLLERS ============================
# Every controller returns a COMMANDED ACCELERATION (m/s^2) above hover.
# The plant converts that to thrust, so they are all compared on equal terms.

class PID:
    """baseline. integral action, no model."""
    name = 'PID'
    def __init__(self):
        self.kp, self.kd = WN**2, 2*ZE*WN
        self.ki = 4.0
        self.I = 0.0
    def reset(self): self.I = 0.0
    def __call__(self, z, zd, r, dt):
        e = r - z
        self.I = np.clip(self.I + e*dt, -5, 5)         # anti-windup
        return self.kp*e + self.ki*self.I - self.kd*zd


class LQR:
    """optimal gains, but structurally a PD -> expect steady-state offset."""
    name = 'LQR'
    def __init__(self):
        k1, k2 = WN**2, 2*ZE*WN                 # target the SHARED design point
        Q = np.diag([k1**2, k2**2 - 2*k1]); R = np.array([[1.0]])
        P = solve_continuous_are(A, B, Q, R)
        self.K = (np.linalg.inv(R) @ B.T @ P).ravel()
    def reset(self): pass
    def __call__(self, z, zd, r, dt):
        return -self.K @ np.array([z - r, zd])


class LQI:
    """LQR + integral state. The fair comparison for L1."""
    name = 'LQI'
    def __init__(self):
        Aa = np.block([[A, np.zeros((2,1))], [np.array([[1.,0.]]), np.zeros((1,1))]])
        Ba = np.vstack([B, [[0.0]]])
        k1, k2 = WN**2, 2*ZE*WN
        Q  = np.diag([k1**2, k2**2 - 2*k1, 60.0]); R = np.array([[1.0]])
        P  = solve_continuous_are(Aa, Ba, Q, R)
        self.K = (np.linalg.inv(R) @ Ba.T @ P).ravel()
        self.I = 0.0
    def reset(self): self.I = 0.0
    def __call__(self, z, zd, r, dt):
        self.I = np.clip(self.I + (z - r)*dt, -5, 5)
        return -self.K @ np.array([z - r, zd, self.I])


class L1:
    """PD baseline + disturbance estimator + low-pass filter."""
    name = 'L1'
    def __init__(self, Gamma=400.0, k=25.0):
        self.G, self.k = Gamma, k
        a, c = WN**2, 2*ZE*WN
        Am = np.array([[0.0, 1.0], [-a, -c]])
        P  = solve_lyapunov(Am.T, -np.eye(2))
        self.Pb = (P @ np.array([[0.0],[1.0]])).ravel()
        self.reset()
    def reset(self):
        self.zh = np.array([z_ref, 0.0]); self.sh = 0.0; self.u = 0.0
    def __call__(self, z, zd, r, dt):
        a, c = WN**2, 2*ZE*WN
        base = a*(r - z) - c*zd
        x  = np.array([z, zd])
        self.zh += dt*np.array([self.zh[1],
                                a*(r - self.zh[0]) - c*self.zh[1] + self.u + self.sh])
        self.sh += dt*(-self.G*np.dot(self.zh - x, self.Pb))
        self.u  += dt*self.k*(-self.sh - self.u)
        return base + self.u


class MPC:
    """condensed finite-horizon QP with HARD accel bounds. Only one that plans."""
    name = 'MPC'
    def __init__(self, N=20, dt_mpc=0.05):
        self.N, self.h = N, dt_mpc
        Ad = np.array([[1.0, dt_mpc], [0.0, 1.0]])
        Bd = np.array([[0.5*dt_mpc**2], [dt_mpc]])
        # build prediction matrices  Z = Sx*x0 + Su*U   (position rows only)
        Sx = np.zeros((N, 2)); Su = np.zeros((N, N))
        Ak = np.eye(2)
        for i in range(N):
            Ak = Ak @ Ad if i else Ad
            Sx[i] = Ak[0]
            for j in range(i+1):
                M = np.linalg.matrix_power(Ad, i-j) @ Bd
                Su[i, j] = M[0, 0]
        self.Sx, self.Su = Sx, Su
        self.H = Su.T @ Su + 0.02*np.eye(N)
        self.Hinv = np.linalg.inv(self.H)
        self.amax = T_MAX/m_nom - g
        self.amin = T_MIN/m_nom - g
    def reset(self): pass
    def __call__(self, z, zd, r, dt):
        x0 = np.array([z, zd])
        f  = self.Su.T @ (self.Sx @ x0 - r*np.ones(self.N))
        U  = -self.Hinv @ f
        for _ in range(30):                       # projected gradient on the box
            U = np.clip(U, self.amin, self.amax)
            U = U - 0.6*self.Hinv @ (self.H @ U + f)
        return float(np.clip(U[0], self.amin, self.amax))


CONTROLLERS = [PID, LQR, LQI, L1, MPC]

# ============================ SIMULATION ============================
def run(ctrl, scen, seed=0):
    m_vec, d_vec, r_vec, noise = scenario(scen)
    rng = np.random.default_rng(seed)
    ctrl.reset()
    z, zd = z_ref, 0.0
    log = np.zeros((len(t), 5))
    for i in range(len(t)):
        z_meas = z + (rng.normal(0, noise) if noise else 0.0)
        a_cmd  = ctrl(z_meas, zd, r_vec[i], dt)
        thrust = np.clip(m_nom*(g + a_cmd), T_MIN, T_MAX)      # HARD limit
        zdd    = thrust/m_vec[i] - g + d_vec[i]
        z  += zd*dt
        zd += zdd*dt
        log[i] = [z, r_vec[i], thrust, m_vec[i], d_vec[i]]
    return log


def metrics(log):
    z, r, thr = log[:,0], log[:,1], log[:,2]
    settle = t > (T_END - 1.0)
    return {
        'rms':   np.sqrt(np.mean((z - r)**2)),
        'final': abs(z[settle].mean() - r[settle].mean()),
        'peak':  np.abs(z - r).max(),
        'effort': np.sqrt(np.mean(np.diff(thr)**2))/dt,
        'sat':   100*np.mean((thr >= T_MAX-1e-6) | (thr <= T_MIN+1e-6)),
    }


def matrix():
    print(f"{'':12s}" + "".join(f"{c.name:>12s}" for c in CONTROLLERS))
    for scen in SCENARIOS:
        for label, key, fmt in [('rms err [m]','rms','{:12.4f}'),
                                ('final err[m]','final','{:12.4f}'),
                                #('effort N/s ','effort','{:12.0f}'),
                                ('saturated %','sat','{:12.1f}')]:
            row = f"{scen if label.startswith('rms') else '':12s}"
            for C in CONTROLLERS:
                row += fmt.format(metrics(run(C(), scen))[key])
            print(row + f"   <- {label}")
        print()


def plot_one(scen):
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.suptitle(f'scenario: {scen}')
    for C in CONTROLLERS:
        log = run(C(), scen)
        ax[0].plot(t, log[:,0], lw=1.4, label=C.name)
        ax[1].plot(t, log[:,0]-log[:,1], lw=1.2, label=C.name)
        ax[2].plot(t, log[:,2], lw=1.0, label=C.name)
    log = run(PID(), scen)
    ax[0].plot(t, log[:,1], 'k--', lw=1, label='reference')
    ax[2].axhline(T_MAX, c='r', ls=':', lw=1, label='thrust limit')
    ax[0].set_ylabel('z [m]');   ax[0].set_title('altitude')
    ax[1].set_ylabel('err [m]'); ax[1].set_title('tracking error'); ax[1].axhline(0, c='k', lw=.5)
    ax[2].set_ylabel('T [N]');   ax[2].set_title('thrust'); ax[2].set_xlabel('t [s]')
    for a in ax: a.grid(alpha=.3); a.legend(fontsize=8, ncol=6)
    plt.tight_layout(); plt.show()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] in SCENARIOS:
        plot_one(sys.argv[1])
    else:
        matrix()