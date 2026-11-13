import numpy as np
import matplotlib.pyplot as plt
from scara_kin import fk
from scara_ik import ik, wrap

L1, L2 = 0.200, 0.150
target = (0.25, 0.10, 0.10, 0.3)

fig, ax = plt.subplots(1, 2, figsize=(12, 5.2))

# (a) lefty vs righty for the same target
colors = {"lefty": "tab:blue", "righty": "tab:red"}
for name, (q1, q2, q3, q4) in ik(*target):
    t1, t2 = -q1, -q2          # URDF -> math convention (verified earlier)
    ex, ey = L1*np.cos(t1), L1*np.sin(t1)
    tx, ty = ex + L2*np.cos(t1 + t2), ey + L2*np.sin(t1 + t2)
    assert abs(tx - target[0]) < 1e-9 and abs(ty - target[1]) < 1e-9
    ax[0].plot([0, ex, tx], [0, ey, ty], "-o", color=colors[name], lw=3, label=name)
ax[0].plot(target[0], target[1], "k*", ms=14, label="target")
ax[0].plot(0, 0, "s", color="gray", ms=10)
ax[0].set_aspect("equal"); ax[0].grid(alpha=0.3)
ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
ax[0].set_title("Two IK solutions, same target (top view)")
ax[0].legend()

# (b) round-trip error histogram
rng = np.random.default_rng(0)
errs = []
for _ in range(5000):
    q = (rng.uniform(-2.618, 2.618), rng.uniform(-2.531, 2.531),
         rng.uniform(0, 0.1), rng.uniform(-np.pi, np.pi))
    pose = fk(*q)
    for name, s in ik(*pose):
        p2 = fk(*s)
        e = max(abs(p2[0]-pose[0]), abs(p2[1]-pose[1]),
                abs(p2[2]-pose[2]), abs(wrap(p2[3]-pose[3])))
        errs.append(max(e, 1e-17))
errs = np.array(errs)
ax[1].hist(np.log10(errs), bins=40, color="tab:green")
ax[1].axvline(-6, color="r", ls="--", label="1e-6 target")
ax[1].set_xlabel("log10( FK(IK(pose)) pose error )")
ax[1].set_ylabel("count")
ax[1].set_title(f"Round-trip error, {len(errs)} IK solutions")
ax[1].legend()

fig.tight_layout()
fig.savefig("../docs/figures/ik_configs_error.png", dpi=150)
print(f"IK solutions tested: {len(errs)}")
print(f"max pose error: {errs.max():.2e}")
print("saved ../docs/figures/ik_configs_error.png")