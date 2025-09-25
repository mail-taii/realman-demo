#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import rospy
from datetime import datetime
from embodied_arm_msgs.msg import Joint_Current


class JointCurrentLogger:
    def __init__(self, out_dir, left_topic, right_topic):
        self.out_dir = os.path.abspath(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.left_path = os.path.join(self.out_dir, f"left_joint_current_{ts}.csv")
        self.right_path = os.path.join(self.out_dir, f"right_joint_current_{ts}.csv")

        self.left_file = open(self.left_path, "w", newline="")
        self.right_file = open(self.right_path, "w", newline="")
        self.left_writer = csv.writer(self.left_file)
        self.right_writer = csv.writer(self.right_file)

        # 写表头
        header = [
            "ros_time", "sec", "nsec",
            "j0", "j1", "j2", "j3", "j4", "j5", "j6"
        ]
        self.left_writer.writerow(["side", "left"]) ; self.left_writer.writerow(header)
        self.right_writer.writerow(["side", "right"]) ; self.right_writer.writerow(header)

        self.left_sub = rospy.Subscriber(left_topic, Joint_Current, self._left_cb, queue_size=50)
        self.right_sub = rospy.Subscriber(right_topic, Joint_Current, self._right_cb, queue_size=50)

        rospy.loginfo("JointCurrentLogger started. Left: %s, Right: %s", left_topic, right_topic)
        rospy.loginfo("Writing to: %s and %s", self.left_path, self.right_path)

    def _row_from_msg(self, msg):
        now = rospy.Time.now()
        currents = list(getattr(msg, "joint_current", []))
        # 填充至7关节长度（RM75为7 DOF）
        while len(currents) < 7:
            currents.append(0.0)
        return [now.to_sec(), now.secs, now.nsecs] + currents[:7]

    def _left_cb(self, msg):
        self.left_writer.writerow(self._row_from_msg(msg))

    def _right_cb(self, msg):
        self.right_writer.writerow(self._row_from_msg(msg))

    def close(self):
        try:
            self.left_file.flush(); self.left_file.close()
        except Exception:
            pass
        try:
            self.right_file.flush(); self.right_file.close()
        except Exception:
            pass


def main():
    rospy.init_node("log_joint_current", anonymous=True)

    out_dir = rospy.get_param("~out_dir", "~/ros_logs")
    out_dir = os.path.expanduser(out_dir)

    left_topic = rospy.get_param("~left_topic", "/l_arm/rm_driver/Udp_Joint_Current")
    right_topic = rospy.get_param("~right_topic", "/r_arm/rm_driver/Udp_Joint_Current")

    logger = JointCurrentLogger(out_dir, left_topic, right_topic)
    rospy.on_shutdown(logger.close)
    rospy.spin()


if __name__ == "__main__":
    main()


