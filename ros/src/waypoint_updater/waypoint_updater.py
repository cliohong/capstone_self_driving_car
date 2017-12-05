#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf
from std_msgs.msg import Int32
import numpy as np
from copy import deepcopy
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
#states
INIT = 0
MPS = 0.44704

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
#        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.car_index_pub = rospy.Publisher('car_index', Int32, queue_size=1)

        self.count = 0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.pose = None
        self.theta = None
        self.waypoints = None
        self.decelerate = None
        self.state = INIT
        self.timer = 0
        self.limit_speed = 0.0
        self.poly1d = None
        self.red_light_wpt=None

#        self.get_closest_waypoint_idx()
        #start the loop
        self.loop()
        
        
        # TODO: Add other member variables you need below

        rospy.spin()


    def loop(self):
        rate = rospy.Rate(8)
    
        while not rospy.is_shutdown():
            rate.sleep()
            if self.waypoints is None or self.pose is None:
                continue
        def get_closest_waypoint_idx(waypoints, pose):
            ##return index of the closest waypoints in the list
            closest_gap = float('inf')
            closest_index =0
            p0 = pose.position
            
            
            for i,waypoint in enumerate(waypoints):
                #calculate distances
                p1 = waypoint.pose.pose.position
                gap = (p0.x-p1.x)**2 + (p0.y - p1.y)**2
                if gap<closest_gap:
                    closest_gap,closest_index = gap, i
            
            shift_x = waypoints[closest_index].pose.pose.position.x- pose.x
            shift_y = waypoints[closest_index].pose.pose.position.y- pose.y
            
            #do coordinate system transformation
            shift = shift_x*cos(0 - self.theta) - shift_y*sin(0 - self.theta)
            if shift <=0:
                closest_index+=1
                    
            return closest_index
            
            #where car is located at
            car_index = get_closest_waypoint_idx(self.waypoints,self.pose)
            #get subset waypoints ahead
            min_wpts=min(len(self.waypoints),LOOKAHEAD_WPS+car_index)
            ahead_wpts =deepcopy(self.waypoints[car_index : min_wpts])
    
    def pose_cb(self, msg):
        # TODO: Implement
        self.count += 1
        self.pose = msg.pose
        self.orientation = self.pose.orientation
        euler = tf.transformations.euler_from_quaternion([self.orientation.x,
                                                          self.orientation.y,
                                                          self.orientation.z,
                                                          self.orientation.w])
        self.theta = euler[2]
        if self.state == INIT:
            print("INITIALIZING TRAFFIC LIGHT DETECTOR ....")

#    def get_velocity_and_distance(self,  index):
#        self.current_linear_velocity  = msg.twist.linear.x
#        self.current_angular_velocity = msg.twist.angular.z

    def waypoints_cb(self, msg):
        
        # TODO: Implement
        self.waypoints = msg.waypoints

#        if self.waypoints is None:
#            #cut down on resource usage
#            self.sub_waypoints.unregister()

        

            
            
                
                

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

#    def get_waypoint_velocity(self, waypoint):
#        return waypoint.twist.twist.linear.x

#    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
#        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
