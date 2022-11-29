#!/usr/bin/env python

import sys
import copy
from tokenize import group

from numpy import rate
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints


move_dic = {
    'go ':(0.1, 0, 0),
    'back ':(-0.1, 0, 0),
    'left ':(0, 0.1, 0),
    'right ':(0, -0.1, 0),
    'up ':(0, 0, 0.1),
    'down ':(0, 0, -0.1),
    # 'stop ':(0, 0, 0),
}

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)

path = Path()
poseStamped = PoseStamped()

path_publisher = rospy.Publisher('/path', Path, queue_size=1)


def sound_callback(data):
    cmd = data.data
    rospy.loginfo([cmd, len(cmd)])
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = group.get_planning_frame()
    poseStamped.header.frame_id = group.get_planning_frame()
    poseStamped.header.stamp = rospy.Time.now()
    if cmd in move_dic:
        x, y, z = move_dic[cmd]
        pose = group.get_current_pose().pose
        t = 0.0
        waypoints = []
        while t <= 1.0:
            pose.position.x += x * 0.1
            pose.position.y += y * 0.1
            pose.position.z += z * 0.1

            waypoints.append(copy.deepcopy(pose))
            poseStamped.pose = pose
            path.poses.append(copy.deepcopy(poseStamped))
            t += 0.1
            
        group.set_start_state_to_current_state()
        count = 0
        fraction = 0
        while fraction < 1.0 and count < 100:
            (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
            count += 1
            if fraction == 1:
                group.execute(plan)
                path_publisher.publish(path)
                rate = rospy.Rate(10)
        waypoints = []
        path.poses = []
        rospy.sleep(1)
    



def main():
    

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=1)

    sound_subscriber = rospy.Subscriber('kws_data', String, sound_callback)
    while not rospy.is_shutdown():
        # rate = rospy.Rate(10)
        
        # path_publisher.publish(path)
        rospy.spin()




if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise