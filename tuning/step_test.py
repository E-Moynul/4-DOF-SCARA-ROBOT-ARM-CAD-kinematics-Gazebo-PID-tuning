"""usage: python3 step_test.py <joint_index 0-3> <delta> <label> [T=6]"""
import sys, os, csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

JOINTS = ["joint_1", "joint_2", "joint3_z", "joint4"]
HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "..", "docs", "figures")


class Step(Node):
    def __init__(self):
        super().__init__("step_test", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/joint_states", self.cb, 100)
        self.q = None
        self.rec = []
        self.t_cmd = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb(self, msg):
        d = dict(zip(msg.name, msg.position))
        self.q = [d[j] for j in JOINTS]
        if self.t_cmd is not None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.rec.append((t - self.t_cmd, *self.q))

    def spin_sim(self, sec):
        end = self.now() + sec
        while self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, q, secs):
        m = JointTrajectory()
        m.joint_names = JOINTS
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        m.points = [p]
        self.pub.publish(m)


def metrics(t, y, y0, delta):
    r = (y - y0) / delta
    final = r[t > t[-1] - 0.5].mean()
    t10 = t[np.argmax(r >= 0.1)] if (r >= 0.1).any() else np.nan
    t90 = t[np.argmax(r >= 0.9)] if (r >= 0.9).any() else np.nan
    out = np.where(np.abs(r - 1) > 0.02)[0]
    settle = t[out[-1]] if len(out) else 0.0
    if len(out) and out[-1] == len(t) - 1:
        settle = np.nan  # never settled
    return dict(rise=t90 - t10, overshoot=max(0.0, (r.max() - 1) * 100),
                settle=settle, sse=(1 - final) * delta)


def main():
    idx, delta, label = int(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
    T = float(sys.argv[4]) if len(sys.argv) > 4 else 6.0
    rclpy.init()
    n = Step()
    while n.q is None or n.pub.get_subscription_count() == 0:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.spin_sim(0.5)
    q0 = list(n.q)
    tgt = list(q0)
    tgt[idx] += delta
    n.rec.clear()
    n.t_cmd = n.now()
    n.send(tgt, 0.02)
    n.spin_sim(T)
    n.t_cmd = None
    n.send(q0, 3.0)          # return home
    n.spin_sim(5.0)

    a = np.array(n.rec)
    t, y = a[:, 0], a[:, 1 + idx]
    m = metrics(t, y, q0[idx], delta)
    os.makedirs(FIG, exist_ok=True)
    np.savetxt(os.path.join(HERE, f"{label}.csv"), a, delimiter=",",
               header="t," + ",".join(JOINTS), comments="")
    plt.figure(figsize=(7, 4))
    plt.plot(t, y, label="measured")
    plt.axhline(q0[idx] + delta, ls="--", c="gray", label="target")
    plt.xlabel("time (s)"); plt.ylabel(f"{JOINTS[idx]} position")
    plt.title(f"{label}: step {delta}")
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig(os.path.join(FIG, f"tuning_{label}.png"), dpi=150)

    row = [label, JOINTS[idx], delta, f"{m['rise']:.3f}", f"{m['overshoot']:.1f}",
           f"{m['settle']:.3f}", f"{m['sse']:.2e}"]
    res = os.path.join(HERE, "results.csv")
    new = not os.path.exists(res)
    with open(res, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["label", "joint", "step", "rise_s", "overshoot_pct",
                        "settle_s", "sse"])
        w.writerow(row)
    print("samples:", len(t))
    print(dict((k, round(float(v), 4)) for k, v in m.items()))
    rclpy.shutdown()


if __name__ == "__main__":
    main()