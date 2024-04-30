#!/usr/bin/env python3

import rospy
from legolas_biped.msg import joint_angles
# from sensor_msgs.msg import Joy
import time
import csv
import numpy as np

__authors__ = "David Ho"
__license__ = "BSD-3-Clause"


class Gait_Publisher():
    def __init__(self, gait_csv) -> None:
        # rate = rospy.Rate(30)
        rospy.init_node('gait_publisher_node')

        pub_topic = 'joint_angles'
        self.gait_pub = rospy.Publisher(pub_topic, joint_angles, queue_size=1)

        # sub_topic = 'joy_throttled'
        # self.JoySub = rospy.Subscriber(sub_topic, Joy, self.main)

        rate = rospy.Rate(30)

        self.gait_joint_msg = joint_angles()

        self.gait = gait_csv

        self.index = 0

        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

    def publish_gait_joint(self, i):
        self.gait_joint_msg.Left_Joints = self.gait[i][0:5]
        self.gait_joint_msg.Right_Joints = self.gait[i][5:]

        rospy.loginfo(self.gait_joint_msg)
        self.gait_pub.publish(self.gait_joint_msg)

        return

    def main(self):
        self.publish_gait_joint(self.index)
        self.index += 1

        if (self.index == len(self.gait)) or (self.index > len(self.gait)):
            self.index = 0


if __name__ == '__main__':
    try:
        # Assuming 'your_file.csv' is the CSV file containing the data
        file_path = 'gait_joint_angles_test.csv'

        input_gait = []
        with open(file_path) as csvfile:
            # change contents to floats
            reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:  # each row is a list
                input_gait.append(row)

        Gait_Publisher(input_gait)

    except rospy.ROSInterruptException:
        pass
