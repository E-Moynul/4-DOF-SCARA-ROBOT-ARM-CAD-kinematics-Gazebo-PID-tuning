import numpy as np
from scara_kin import fk

L1, L2 = 0.200, 0.150
Z0 = 0.143   # z when q3 = 0

def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def ik(x, y, z, yaw):
    """Returns list of (name, (q1,q2,q3,q4)) inside joint limits, URDF convention."""
    c2 = (x*x + y*y - L1**2 - L2**2) / (2*L1*L2)
    if abs(c2) > 1 + 1e-9:
        return []                      # out of reach
    c2 = min(1.0, max(-1.0, c2))
    q3 = Z0 - z
    sols = []
    for name, s in (("righty", 1), ("lefty", -1)):
        t2 = s * np.arccos(c2)
        t1 = np.arctan2(y, x) - np.arctan2(L2*np.sin(t2), L1 + L2*np.cos(t2))
        q1, q2 = wrap(-t1), wrap(-t2)
        q4 = wrap(yaw + q1 + q2)
        e = 1e-9
        if (abs(q1) <= 2.618+e and abs(q2) <= 2.531+e
                and -e <= q3 <= 0.1+e and abs(q4) <= np.pi+e):
            sols.append((name, (q1, q2, q3, q4)))
    return sols

def angdiff(a, b):
    return abs(wrap(a - b))

if __name__ == "__main__":
    rng = np.random.default_rng(0)
    N = 10000
    not_found = 0
    max_q_err = 0.0
    max_fk_err = 0.0
    for _ in range(N):
        q = (rng.uniform(-2.618, 2.618), rng.uniform(-2.531, 2.531),
             rng.uniform(0, 0.1), rng.uniform(-np.pi, np.pi))
        pose = fk(*q)
        sols = ik(*pose)
        best = None
        for name, s in sols:
            err = max(angdiff(s[0], q[0]), angdiff(s[1], q[1]),
                      abs(s[2] - q[2]), angdiff(s[3], q[3]))
            best = err if best is None else min(best, err)
            p2 = fk(*s)
            fe = max(abs(p2[0]-pose[0]), abs(p2[1]-pose[1]),
                     abs(p2[2]-pose[2]), angdiff(p2[3], pose[3]))
            max_fk_err = max(max_fk_err, fe)
        if best is None or best > 1e-6:
            not_found += 1
        else:
            max_q_err = max(max_q_err, best)
    print(f"samples: {N}")
    print(f"original q not recovered: {not_found}")
    print(f"max joint error (recovered): {max_q_err:.2e}")
    print(f"max FK(IK) pose error: {max_fk_err:.2e}")
    print("example target (0.25, 0.10, 0.10, 0.3):")
    for name, s in ik(0.25, 0.10, 0.10, 0.3):
        print(" ", name, np.round(s, 4))