#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')
        
        reset = rospy.get_param('~reset', False)
        
        # name
        arm_joints = ['joint1',
                      'joint2',
                      'joint3', 
                      'joint4',
                      'joint5',
                      'joint6']
        
        if reset:
            arm_goal  = [0, 0, 0, 0, 0, 0]

        else:
            arm_goal  = [-0.3, -1.0, 0.5, 0.8, 1.0, -0.7]
    
        # Connect the trajectory action server of the robot arm trajectory planning
        rospy.loginfo('Waiting for arm trajectory controller...')       
        arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        arm_client.wait_for_server()        
        rospy.loginfo('...connected.')  
    
        # Create a track data with the set target position
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        rospy.loginfo('Moving the arm to goal position...')
        
        # Creates an empty object for a track target
        arm_goal = FollowJointTrajectoryGoal()
        
        # Add the previously created trajectory data to the trajectory target object
        arm_goal.trajectory = arm_trajectory
        
        # Set the allowable error 
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the trajectory target to the action server for processing to realize the motion control of the robotic arm
        arm_client.send_goal(arm_goal)

        arm_client.wait_for_result(rospy.Duration(5.0))
        
        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    
