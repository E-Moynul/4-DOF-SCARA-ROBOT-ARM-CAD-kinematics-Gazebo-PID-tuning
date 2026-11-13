import numpy as np
import matplotlib.pyplot as plt
from scara_kin import fk

L1, L2 = 0.200, 0.150
Q1, Q2 = 2.618, 2.531

# dense grid over joint limits (x, y depend only on q1, q2)
q1 = np.linspace(-Q1, Q1, 700)
q2 = np.linspace(-Q2, Q2, 700)
A, B = np.meshgrid(q1, q2)
t1, t2 = -A, -B
X = L1*np.cos(t1) + L2*np.cos(t1 + t2)
Y = L1*np.sin(t1) + L2*np.sin(t1 + t2)
M = np.abs(np.sin(B))            # normalized manipulability = |det J| / (L1*L2)

# sanity check against the verified FK
for i, j in [(10, 20), (300, 400), (650, 100)]:
    x, y, _, _ = fk(q1[j], q2[i], 0, 0)
    assert abs(x - X[i, j]) < 1e-9 and abs(y - Y[i, j]) < 1e-9

R = np.hypot(X, Y)
print(f"r_min = {R.min()*1000:.1f} mm, r_max = {R.max()*1000:.1f} mm")
ang = np.degrees(np.arctan2(Y, X))
print(f"angle range = {ang.min():.1f} to {ang.max():.1f} deg")

fig, ax = plt.subplots(1, 2, figsize=(12, 5.5))
ax[0].scatter(X.ravel(), Y.ravel(), s=0.3, c="tab:blue", alpha=0.3)
ax[0].set_title("Reachable workspace (xy)")
sc = ax[1].scatter(X.ravel(), Y.ravel(), s=0.3, c=M.ravel(), cmap="viridis")
ax[1].set_title("|sin q2|  (0 = singular)")
fig.colorbar(sc, ax=ax[1], label="|det J| / (L1*L2)")
for a in ax:
    a.plot(0, 0, "r*", ms=10)
    a.set_aspect("equal")
    a.set_xlabel("x (m)"); a.set_ylabel("y (m)")
    a.grid(alpha=0.3)
fig.tight_layout()
fig.savefig("../docs/figures/workspace_singularity.png", dpi=150)
print("saved ../docs/figures/workspace_singularity.png")