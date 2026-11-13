import time
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

from scara_kin import fk
from scara_ik import ik, wrap

JOINTS = ["joint_1", "joint_2", "joint3_z", "joint4"]
TARGETS = [  # x, y, z (m), yaw (rad)
    (0.25, 0.10, 0.10, 0.3),
    (0.30, -0.12, 0.05, -1.0),
    (0.20, 0.20, 0.02, 2.0),
    (0.15, -0.05, 0.08, 0.0),
]


class Checker(Node):
    def __init__(self):
        super().__init__("scara_sim_check")
        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self):
        try:
            t = self.buf.lookup_transform("base_link", "tool_flange",
                                          rclpy.time.Time())
        except Exception:
            return None
        p, q = t.transform.translation, t.transform.rotation
        yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        return np.array([p.x, p.y, p.z, yaw])

    def send(self, q, secs=4):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start = Duration(sec=secs)
        msg.points = [pt]
        self.pub.publish(msg)

    def settle(self, first_wait=5.0, timeout=40.0):
        self.spin_for(first_wait)
        prev, end = None, time.time() + timeout
        while time.time() < end:
            self.spin_for(1.0)
            p = self.pose()
            if p is not None and prev is not None \
                    and np.max(np.abs(p[:3] - prev[:3])) < 1e-5:
                return p
            prev = p
        return prev


def main():
    rclpy.init()
    n = Checker()
    while n.pub.get_subscription_count() == 0:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.spin_for(1.0)

    print(f"{'target':<28}{'config':<8}{'pos err (mm)':>14}{'yaw err (rad)':>16}")
    worst_pos, worst_yaw = 0.0, 0.0
    for tgt in TARGETS:
        sols = ik(*tgt)
        if not sols:
            print(tgt, "-> no IK solution inside limits")
            continue
        for name, q in sols:
            expected = np.array(fk(*q))
            n.send(q)
            got = n.settle()
            if got is None:
                print(tgt, name, "-> no tf")
                continue
            pe = np.linalg.norm(got[:3] - expected[:3]) * 1000
            ye = abs(wrap(got[3] - expected[3]))
            worst_pos, worst_yaw = max(worst_pos, pe), max(worst_yaw, ye)
            print(f"{str(tgt):<28}{name:<8}{pe:>14.4f}{ye:>16.2e}")
    print(f"\nworst position error: {worst_pos:.4f} mm, worst yaw error: {worst_yaw:.2e} rad")
    rclpy.shutdown()


if __name__ == "__main__":
    main()