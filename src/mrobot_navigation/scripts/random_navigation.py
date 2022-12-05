#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  

class NavTest():  
    def __init__(self):  
        rospy.init_node('random_navigation', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  
 
        # pause time in each target  
        self.rest_time = rospy.get_param("~rest_time", 2)  

        # set goal state  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
 
        # set destination  
        locations = dict()  

        locations['p1'] = Pose(Point(1.150, 5.461, 0.000), Quaternion(0.000, 0.000, -0.013, 1.000))  
        locations['p2'] = Pose(Point(6.388, 2.66, 0.000), Quaternion(0.000, 0.000, 0.063, 0.998))  
        locations['p3'] = Pose(Point(8.089, -1.657, 0.000), Quaternion(0.000, 0.000, 0.946, -0.324))  
        locations['p4'] = Pose(Point(9.767, 5.171, 0.000), Quaternion(0.000, 0.000, 0.139, 0.990))  
        locations['p5'] = Pose(Point(0.502, 1.270, 0.000), Quaternion(0.000, 0.000, 0.919, -0.392)) 
        locations['p6'] = Pose(Point(4.557, 1.234, 0.000), Quaternion(0.000, 0.000, 0.627, 0.779)) 
 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  
   
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  

        initial_pose = PoseWithCovarianceStamped()  
  
        n_locations = len(locations)  
        n_goals = 0  
        n_successes = 0  
        i = n_locations  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = ""  

        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  

        rospy.loginfo("Starting navigation test")  
 
        while not rospy.is_shutdown():    
  
            if i == n_locations:  
                i = 0  
                sequence = sample(locations, n_locations)  
    
                if sequence[0] == last_location:  
                    i = 1  

            location = sequence[i]  

            if initial_pose.header.stamp == "":  
                distance = sqrt(pow(locations[location].position.x -   
                                    locations[last_location].position.x, 2) +  
                                pow(locations[location].position.y -   
                                    locations[last_location].position.y, 2))  
            else:  
                rospy.loginfo("Updating current pose.")  
                distance = sqrt(pow(locations[location].position.x -   
                                    initial_pose.pose.pose.position.x, 2) +  
                                pow(locations[location].position.y -   
                                    initial_pose.pose.pose.position.y, 2))  
                initial_pose.header.stamp = ""  
 
            last_location = location  

            i += 1  
            n_goals += 1  
 
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = locations[location]  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  
 
            rospy.loginfo("Going to: " + str(location))  
 
            self.move_base.send_goal(self.goal)  

            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   

            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    n_successes += 1  
                    distance_traveled += distance  
                    rospy.loginfo("State:" + str(state))  
                else:  
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
 
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0  

            rospy.loginfo("Success so far: " + str(n_successes) + "/" +   
                          str(n_goals) + " = " +   
                          str(100 * n_successes/n_goals) + "%")  

            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +   
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  

            rospy.sleep(self.rest_time)  

    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  

def trunc(f, n):   
    slen = len('%.*f' % (n, f))  

    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try:  
        NavTest()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Random navigation finished.")
