#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'grasping_frame'

GRIPPER_OPEN = [0.004]
GRIPPER_CLOSED = [0.01]

REFERENCE_FRAME = 'base_link_of_arm'

class MoveItPickAndPlaceDemo:
    def __init__(self):
        # initialize the move_group        
	moveit_commander.roscpp_initialize(sys.argv)
        
        # initialize the ros
        rospy.init_node('moveit_pick_and_place_demo')
        
        # initialize the scene
        scene = PlanningSceneInterface()
        
        # create a scene pub
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
        
        # create a gripper pose
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
        
        # create a dict to store the color of stuff
        self.colors = dict()
                        
        # initialize move group controlled arm group
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # initialize move group controlled gripper group
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # get the end link name
        end_effector_link = arm.get_end_effector_link()
 
        # set goal position and state
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.1)

        # if failed, replan
        arm.allow_replanning(True)
        
        # set position axel
        arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # set planning time limit：5s
        arm.set_planning_time(5)
        
        # set maximum trying of pick and place
        max_pick_attempts = 5
        max_place_attempts = 5
        rospy.sleep(2)

        # set name in the scene
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        target_id = 'target'
                
        # remove the left objects
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        scene.remove_world_object(target_id)
        scene.remove_world_object('target2')
        
        scene.remove_attached_object(GRIPPER_FRAME, target_id)  
        rospy.sleep(1)
        
        # back to initial
        arm.set_named_target('home')
        arm.go()
        
        # open gripper
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(1)

        # set table height
        table_ground = 0.19
        
        # set size of table box1 box2
        table_size = [0.2, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]
        
        # add object on table
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.35
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = REFERENCE_FRAME
        box1_pose.pose.position.x = 0.31
        box1_pose.pose.position.y = -0.1
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = REFERENCE_FRAME
        box2_pose.pose.position.x = 0.29
        box2_pose.pose.position.y = 0.13
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0   
        scene.add_box(box2_id, box2_pose, box2_size)       
                
        # set table to red，two box to orange
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)
        
        # set size of subject
        target_size = [0.02, 0.01, 0.12]
        
        # set subject in middle
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.32
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        target_pose.pose.orientation.w = 1.0
        
        scene.add_box(target_id, target_pose, target_size)
        
        # set subject yellow
        self.setColor(target_id, 0.9, 0.9, 0, 1.0)
        
        self.sendColors()

        arm.set_support_surface_name(table_id)
        
        # set goal position place
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = 0.32
        place_pose.pose.position.y = -0.2
        place_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        place_pose.pose.orientation.w = 1.0

        # set grasp position
        grasp_pose = target_pose
                
        grasps = self.make_grasps(grasp_pose, [target_id])

        # display in rviz
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)
    
        # set result and attemp time
        result = None
        n_attempts = 0
        
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = arm.pick(target_id, grasps)
            rospy.sleep(0.2)
        
        # if pick successfully，go to place
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            
            # set pose
            places = self.make_places(place_pose)
            
            # try until success of exceed limit
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = arm.place(target_id, place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                rospy.sleep(0.2)
                #-----------------------------------------------------------------------------------------------------
                if n_attempts==2:
                    result=1
                    scene.remove_world_object(target_id)
                    print("Moved successfully")
            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")

        scene.remove_world_object(target_id)
        target_size1 = [0.02, 0.01, 0.12]
        target2_pose = PoseStamped()
        target2_pose.header.frame_id = REFERENCE_FRAME
        target2_pose.pose.position.x = 0.32
        target2_pose.pose.position.y = -0.2
        target2_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        target2_pose.pose.orientation.w = 1.0
        scene.add_box('target2', target2_pose, target_size1)

        self.sendColors()
        # open gripper
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(1)            
        # go back
        arm.set_named_target('home')
        arm.go()

        # get out moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
    # create the data of joint:JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # initialize
        t = JointTrajectory()
        t.joint_names = ['finger_joint1']      
        tp = JointTrajectoryPoint()
        tp.positions = joint_positions
        
        # set force
        tp.effort = [1.0]
        
        # set speed
        tp.time_from_start = rospy.Duration(1.0)
        
        # set target trace
        t.points.append(tp)
        
        # return trace
        return t
    
    # set translation of gripper with given vector
    def make_gripper_translation(self, min_dist, desired, vector):
        g = GripperTranslation()
        
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        g.direction.header.frame_id = GRIPPER_FRAME

        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # create a list of grap pose
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):

        g = Grasp()
        
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
                
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])
        
        g.grasp_pose = initial_pose_stamped
    
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        yaw_vals = [0]

        grasps = []

        for y in yaw_vals:
            for p in pitch_vals:
                # Euler Angle to Quaternion Conversion
                q = quaternion_from_euler(0, p, y)
                
                # Set the grab pose
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                
                # Set the grab id
                g.id = str(len(grasps))
                
                # Set allowed objects
                g.allowed_touch_objects = allowed_touch_objects
                
                # Put this planned crawl into the crawl list
                grasps.append(deepcopy(g))
                
        return grasps
    
    # Create a list of allowed placement poses
    def make_places(self, init_pose):
        # Initialize the position to place the grabbed object
        place = PoseStamped()
        
        # Set the position to place the grabbed object
        place = init_pose
        
        # Defines the offset parameter in the x direction for trying to place the object
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        # Defines the offset parameter in the y direction for trying to place the object
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        pitch_vals = [0]
        
        # Defines the yaw angle parameter for trying to place the object
        yaw_vals = [0]

        # Defines the list of poses to place objects in
        places = []
        
        # Generate grab poses for each angle and offset direction
        for y in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.pose.position.x = init_pose.pose.position.x + x
                        place.pose.position.y = init_pose.pose.position.y + y
                        
                        # Euler Angle to Quaternion Conversion
                        q = quaternion_from_euler(0, p, y)
                        
                        place.pose.orientation.x = q[0]
                        place.pose.orientation.y = q[1]
                        place.pose.orientation.z = q[2]
                        place.pose.orientation.w = q[3]
                        
                        # add to list
                        places.append(deepcopy(place))
        
        # return list
        return places
    
    # Set the color of scene objects
    def setColor(self, name, r, g, b, a = 0.9):
        color = ObjectColor()
        
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # update dict of color
        self.colors[name] = color

    # Send and apply color settings to the moveit scene
    def sendColors(self):
        # Initialize the planning scene object
        p = PlanningScene()

        # difference between the planning scenarios that need to be set    
        p.is_diff = True
        
        # get color set
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # publish
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItPickAndPlaceDemo()

    
