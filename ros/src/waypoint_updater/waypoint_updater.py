#!/usr/bin/env python

"""
This node publishes waypoints from the car's current position
to some waypoints ahead with respected velocity
"""

import rospy
import math
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane
import waypoint_helper
import numpy as np
LOOKAHEAD_WPS = 200 #: Number of waypoints will be published


#-------------------------------------------------------------------------------
class WaypointUpdater(object):
    """
    Given the position and map this object publishes
    points with target velocities representing path ahead
    """

    #---------------------------------------------------------------------------
    def __init__(self):
        """Initialize Waypoint Updater"""

        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity',TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.car_index_pub = rospy.Publisher('car_index', Int32, queue_size=1)

        self.base_waypoints = None
        self.pose = None #: Current vehicle location + orientation
        self.frame_id = None
        #self.current_waypoint_ahead = None
        self.current_velocity = 0.
        self.current_waypoint_ahead = None
        #self.previous_car_index = 0 #: Where in base waypoints list the car is
     #   position fo closetst traffic light
        self.traffic_index = 0 #: Where in base waypoints list the traffic light is
        self.traffic_time_received = rospy.get_time() #: When traffic light info was received
        self.stopped_distance = 0.25

        self.loop()

    #---------------------------------------------------------------------------
    def loop(self):
        """
        Publishes car index and subset of waypoints with ideal velocities
        """
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.base_waypoints is None or self.pose is None or self.frame_id is None:
                continue
            
            #rospy.logwarn("tarffic light index:={}".format(self.traffic_index))
            closest_car_index = waypoint_helper.get_closest_waypoint_index(self.pose, self.base_waypoints)
            self.current_waypoint_ahead = closest_car_index
            lookahead_waypoints = self.get_waypoint_indxs(closest_car_index,LOOKAHEAD_WPS)

            # Get subset waypoints ahead
            #lookahead_waypoints = waypoint_helper.get_next_waypoints(self.base_waypoints,closest_car_index, LOOKAHEAD_WPS)

            # Traffic light must be new and near ahead
            time = rospy.get_time() - self.traffic_time_received
            #rospy.logwarn("time we received tl location:={}".format(time))
            #rospy.logwarn("---------------------------")

            is_near = False

            # Set target speeds
            # if is_new and is_near_ahead:
            #wait for initializing......
            closest_gap = self.traffic_index - closest_car_index
            stopped_distance = self.distance(self.base_waypoints, closest_car_index,self.traffic_index)
            #rospy.logwarn("the distance with no tl around:={}".format(stopped_distance))
            #rospy.logwarn("---------------------------")
            if stopped_distance < 30 and stopped_distance > 0 :
                is_near = True
            if is_near:
                # for i , waypoint in enumerate(lookahead_waypoints):
                for i in lookahead_waypoints
                     waypoint.twist.twist.linear.x = self.get_slow_down_speed(closest_car_index+i)
            if self.current_velocity > 11:
                self.current_velocity = 10.5
#rospy.logwarn("the current speed is:={}".format(self.current_velocity))

            # Publish
            lane = waypoint_helper.publish_lane_object(self.frame_id, self.base_waypoints, lookahead_waypoints)
            #lane = waypoint_helper.publish_lane_object(self.frame_id,lookahead_waypoints)

            self.final_waypoints_pub.publish(lane)
            self.car_index_pub.publish(closest_car_index)

    def get_slow_down_speed(self, index):
        """
        Return target speed for slowing down
        """
        dist = self.distance(self.base_waypoints, index, self.traffic_index)
        car_speed = self.base_waypoints[index].twist.twist.linear.x
        speed = 0.0

        if dist > self.stopped_distance:
            speed = car_speed * (1 - 12/dist)

        if speed < 1.76:
            speed = 0.0
        return speed
    
    def get_waypoint_indxs(self,start_index,length):
        """ Computes a cyclic list of waypoints indices
            Args:
            start_index : initial index of the list
            length: default length of waypoint list
            Returns:
            cyclic list of waypoint indices
        """
        length = min(self.base_waypoints, length)
        end_index = start_index + length
        q, r = divmod(end_index, len(self.base_waypoints))
        if q == 0 :
            return range(start_index, r)
        return range(start_index,len(self.base_waypoints))+range(0,r)

    #---------------------------------------------------------------------------
    def pose_cb(self, msg):
        """ Update vehicle location """
        self.pose = msg.pose # store location (x, y)
        self.frame_id = msg.header.frame_id
        if self.base_waypoints is None:
            return None

    #---------------------------------------------------------------------------
    def waypoints_cb(self, msg):
        """ Store the given map """
        self.base_waypoints = msg.waypoints
    
    def velocity_cb(self,msg):
        self.current_velocity = msg.twist.linear.x
    
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
#
#    def get_waypoint_velocity(self, waypoint):
#        """Unwrap the waypoint to return the value of the linear speed."""
#        return waypoint.twist.twist.linear.x
#
#    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
#        """Unwraps the waypoint object to set the value for the linear speed."""
#        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    #---------------------------------------------------------------------------
    def traffic_cb(self, msg):
        """
        Consider traffic light when setting waypoint velocity
        """
        self.traffic_index = msg.data
        self.traffic_time_received = rospy.get_time()



#-------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
