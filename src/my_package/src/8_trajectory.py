#!/usr/bin/env python

import sys
import copy
from tokenize import group
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


def generate_path(group):
    waypoints = []
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = group.get_planning_frame()

    poseStamped = PoseStamped()
    poseStamped.header.frame_id = group.get_planning_frame()
    poseStamped.header.stamp = rospy.Time.now()

    t = 0.0
    pose = group.get_current_pose().pose
    # rospy.loginfo("pose1: %s", pose)
    c_x = pose.position.x
    c_y = pose.position.y
    c_z = pose.position.z

    while t < 2 * pi:
        x = c_x
        y = 0.1 - 0.1 * math.cos(t) + c_y
        z = 0.1 * math.sin(t) + c_z
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        # pose.orientation.w = 1.0
        poseStamped.pose = pose
        waypoints.append(copy.deepcopy(pose))

        # poseStamped.pose.orientation.x = 0.0
        # poseStamped.pose.orientation.y = 0.0
        # poseStamped.pose.orientation.z = 0.0
        # poseStamped.pose.orientation.w = 1.0
        path.poses.append(copy.deepcopy(poseStamped))


        t += 0.2

    t = 0
    pose = group.get_current_pose().pose
    # rospy.loginfo("pose2: %s", pose)
    c_x = pose.position.x
    c_y = pose.position.y
    c_z = pose.position.z
    while t < 2 * pi:
        x = c_x
        y = - 0.1 + 0.1 * math.cos(t) + c_y
        z = 0.1 * math.sin(t) + c_z
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        # pose.orientation.w = 1.0
        poseStamped.pose = pose
        waypoints.append(copy.deepcopy(pose))

        # poseStamped.pose.orientation.x = 0.0
        # poseStamped.pose.orientation.y = 0.0
        # poseStamped.pose.orientation.z = 0.0
        # poseStamped.pose.orientation.w = 1.0
        path.poses.append(copy.deepcopy(poseStamped))
        t += 0.2
        


    return waypoints, path



def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
    #                                                queue_size=1)

    path_publisher = rospy.Publisher('/path', Path, queue_size=1)

    waypoints, path = generate_path(group)
    
    # path = Path()
    # path.header.frame_id = robot.get_planning_frame()
    # path.header.stamp = rospy.Time.now()
    # path.poses = waypoints

    group.set_start_state_to_current_state()
    count = 0
    fraction = 0
    while fraction < 1.0 and count < 100:
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        count += 1
        if fraction == 1:
            group.execute(plan)
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                path_publisher.publish(path)
                rate.sleep()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass