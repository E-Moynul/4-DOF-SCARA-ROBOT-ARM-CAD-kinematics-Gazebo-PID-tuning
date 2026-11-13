import numpy as np

def dh(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ],
    ])

def fk(q1, q2, q3, q4):
    """q = URDF joint values (rad, rad, m, rad). Returns x, y, z (m), yaw (rad)."""
    T = (dh(-q1, 0.170, 0.200, 0)
         @ dh(-q2, 0.0, 0.150, np.pi)
         @ dh(0.0, q3 + 0.012, 0.0, 0)
         @ dh(-q4, 0.015, 0.0, 0))
    x, y, z = T[:3, 3]
    yaw = np.arctan2(T[1, 0], T[0, 0])
    return x, y, z, yaw

if __name__ == "__main__":
    tests = [
        ((0, 0, 0, 0),        (0.350,  0.000, 0.143,  0.000)),
        ((0.495, 0, 0, 0),    (0.308, -0.166, 0.143, -0.495)),
        ((0, 0.506, 0, 0),    (0.331, -0.073, 0.143, -0.506)),
        ((0, 0, 0.05, 0),     (0.350,  0.000, 0.093,  0.000)),
        ((0, 0, 0, 0.493),    (0.350,  0.000, 0.143,  0.493)),
    ]
    for q, expected in tests:
        got = fk(*q)
        print("q =", q)
        print("  mine :", np.round(got, 3))
        print("  URDF :", expected)