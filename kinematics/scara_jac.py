import numpy as np
from scara_kin import fk
from scara_ik import wrap

L1, L2 = 0.200, 0.150

def jacobian(q1, q2, q3, q4):
    """Rows: x, y, z, yaw. Cols: q1, q2, q3, q4 (URDF joint values)."""
    t1, t2 = -q1, -q2
    s1, c1 = np.sin(t1), np.cos(t1)
    s12, c12 = np.sin(t1 + t2), np.cos(t1 + t2)
    return np.array([
        [ L1*s1 + L2*s12,  L2*s12, 0, 0],
        [-(L1*c1 + L2*c12), -L2*c12, 0, 0],
        [ 0, 0, -1, 0],
        [-1, -1,  0, 1],
    ])

def numeric_jacobian(q, h=1e-6):
    q = np.array(q, dtype=float)
    J = np.zeros((4, 4))
    for i in range(4):
        dq = np.zeros(4); dq[i] = h
        a = np.array(fk(*(q + dq)))
        b = np.array(fk(*(q - dq)))
        d = a - b
        d[3] = wrap(d[3])
        J[:, i] = d / (2*h)
    return J

if __name__ == "__main__":
    rng = np.random.default_rng(1)
    N = 5000
    max_jac_err = 0.0
    max_det_err = 0.0
    for _ in range(N):
        q = (rng.uniform(-2.618, 2.618), rng.uniform(-2.531, 2.531),
             rng.uniform(0, 0.1), rng.uniform(-np.pi, np.pi))
        Ja = jacobian(*q)
        Jn = numeric_jacobian(q)
        max_jac_err = max(max_jac_err, np.max(np.abs(Ja - Jn)))
        det_formula = L1*L2*abs(np.sin(q[1]))
        max_det_err = max(max_det_err, abs(abs(np.linalg.det(Ja)) - det_formula))
    print(f"samples: {N}")
    print(f"max |analytic - numeric| Jacobian error: {max_jac_err:.2e}")
    print(f"max |det J| vs L1*L2*|sin q2| error: {max_det_err:.2e}")
    print("det at q2 = 0 (singular):",
          np.linalg.det(jacobian(0.3, 0.0, 0.05, 0.1)))
    print("det at q2 = 1.0:", np.linalg.det(jacobian(0.3, 1.0, 0.05, 0.1)))