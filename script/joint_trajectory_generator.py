#!/usr/bin/env python

import sys
import actionlib
import argparse
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class JointTrajectoryGenerator:
    def __init__(self, time=1.0, step=0.01, freq=1.0, joint_num=1, topic='/joint_trajectory', action='/joint_trajectroy_action'):
        self.time = time
        self.step = step
        self.freq = freq
        self.joint_names = ['joint_'+str(n) for n in range(joint_num)]

        self.pub = rospy.Publisher(topic, JointTrajectory, queue_size=10)
        self.client = actionlib.SimpleActionClient(
            action, FollowJointTrajectoryAction)

    def update(self):
        points = []
        tick = 0
        self.phase = np.random.uniform(np.pi)
        while tick <= self.time:
            x = 2*np.pi*self.freq*tick + self.phase
            positions = [np.sin(x)]*len(self.joint_names)
            velocities = [np.cos(x)]*len(self.joint_names)
            accelerations = [-np.sin(x)]*len(self.joint_names)
            points.append(JointTrajectoryPoint(positions=positions,
                                               velocities=velocities,
                                               accelerations=accelerations,
                                               time_from_start=rospy.Time.from_sec(tick)))
            tick += self.step
        msg = JointTrajectory(joint_names=self.joint_names, points=points)
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

        # joint_trajectroy_action
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self.client.send_goal(goal)

if __name__ == "__main__":
    rospy.init_node('joint_trajectory_generator')

    parser = argparse.ArgumentParser()
    parser.add_argument('--time-step', type=float, help='Time step between each trajectory point')
    parser.add_argument('--topic', type=str, default="/joint_trajectory", help='Topic name to be published')
    parser.add_argument('--action', type=str, default="/joint_trajectory_action", help='Action name to be called')
    parser.add_argument('--joint-num', type=int, default=1, help='Number of joints')
    sys.argv = rospy.myargv()
    args = parser.parse_args()

    generator = JointTrajectoryGenerator(joint_num=args.joint_num,
                                         topic=args.topic,
                                         action=args.action)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        generator.update()
        rate.sleep()
