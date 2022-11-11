#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_cartesian_demo', anonymous=True)
        
        # Whether motion planning using Cartesian space is required
        cartesian = rospy.get_param('~cartesian', True)
                        
        # Initialize the arm group in the robotic arm that needs to be controlled by the move group
        arm = MoveGroupCommander('arm')
        
        arm.allow_replanning(True)
        
        arm.set_pose_reference_frame('base_link')
                
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)
        
        end_effector_link = arm.get_end_effector_link()
                                        
        arm.set_named_target('forward')
        arm.go()
        
        start_pose = arm.get_current_pose(end_effector_link).pose
                
        waypoints = []
                
        if cartesian:
            waypoints.append(start_pose)
            
        # Set the second waypoint data and add it to the waypoint list
        # The second waypoint needs to move 0.2 meters backward and 0.2 meters to the right
        wpose = deepcopy(start_pose)
        wpose.position.x -= 0.2
        wpose.position.y -= 0.2

        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
         
        # Set the third waypoint data and add it to the waypoint list
        wpose.position.x += 0.05
        wpose.position.y += 0.15
        wpose.position.z -= 0.15
          
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
        
        # Set the fourth waypoint data, return to the initial position, and add to the waypoint list
        if cartesian:
            waypoints.append(deepcopy(start_pose))
        else:
            arm.set_pose_target(start_pose)
            arm.go()
            rospy.sleep(1)
            
        if cartesian:
            fraction = 0.0  
            maxtries = 100  
            attempts = 0     
            
            # Set the current state of the robot arm as the initial state of motion
            arm.set_start_state_to_current_state()
     
            # Try to plan a path in Cartesian space, passing all the waypoints in sequence
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = arm.compute_cartesian_path (
                                        waypoints,   
                                        0.01,        
                                        0.0,         
                                        True)        # avoid_collisions
                
                attempts += 1
                
                # print
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            # If the path planning is successful (100% coverage), start to control the movement of the robotic arm
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            # If path planning fails, print failure information
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        # Control the robotic arm back to the initial position
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
