#!/usr/bin/env python


from operator import truediv
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
robot = moveit_commander.RobotCommander()

 
scene = moveit_commander.PlanningSceneInterface()


group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=1)

def go_to_pose(robot, group, pose, i):
    rospy.loginfo("Go to pose " + str(i + 1))
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = pose[3]
    pose_goal.orientation.y = pose[4]
    pose_goal.orientation.z = pose[5]
    pose_goal.orientation.w = pose[6]

    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]
    group.set_pose_target(pose_goal)

    group.go(wait = True)
    group.stop()
    group.clear_pose_targets()
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")



pose = [
  [0.30603, 0.017247, 0.64808, 0.59731, 0.52117, -0.4175, -0.44418],
  [0.090837, 0.42689, 0.19629, 0.92343, 0.38265, -0.026938, -0.011112],
  [-0.10, 0.42689, 0.19629, 0.92343, 0.38265, -0.026938, -0.011112],
  [-0.21, 0.30, 0.30, 0.0, 0.51, 0.36, -0.1, 0.0],
  [-0.01, 0.50, 0.10, 0.2, 0.38, 0.13, -0.3, 0.0],
  [-0.41, 0.60, 0.40, 0.1, 0.1, 0.05, -0.1, 0.3]
]


def go_to_6_poses(robot, group):
  for i in range(len(pose)):
    go_to_pose(robot, group, pose[i], i)
    rospy.loginfo("Get pose " + str(i + 1) + "...")
    rospy.sleep(2.0)


go_to_6_poses(robot, group)

rospy.loginfo("Successfully finished!")
rospy.sleep(10)


