import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "..", "docs", "figures")
COL = {"joint_1": 1, "joint_2": 2, "joint3_z": 3, "joint4": 4}

PANELS = [
    ("joint3_z, step 0.03 m", "joint3_z", 0.03, 2.0,
     [("z_p50", "p=50"), ("z_p200", "p=200")]),
    ("joint_1, step 0.3 rad", "joint_1", 0.3, 4.0,
     [("j1_base", "d=0.1"), ("j1_d0.3", "d=0.3"), ("j1_d0.6", "d=0.6")]),
    ("joint_2, step 0.3 rad", "joint_2", 0.3, 4.0,
     [("j2_base", "d=0.05"), ("j2_d0.15", "d=0.15"), ("j2_d0.3", "d=0.3")]),
]

fig, axs = plt.subplots(1, 3, figsize=(15, 4))
for ax, (title, j, delta, tmax, runs) in zip(axs, PANELS):
    for f, lab in runs:
        a = np.loadtxt(os.path.join(HERE, f + ".csv"), delimiter=",", skiprows=1)
        t, y = a[:, 0], a[:, COL[j]]
        ax.plot(t, y - y[0], label=lab)
    ax.axhline(delta, ls="--", c="gray")
    ax.set_xlim(-0.1, tmax)
    ax.set_title(title); ax.set_xlabel("time (s)")
    ax.set_ylabel("displacement"); ax.grid(alpha=0.3); ax.legend()
plt.tight_layout()
plt.savefig(os.path.join(FIG, "tuning_comparison.png"), dpi=150)
print("saved tuning_comparison.png")