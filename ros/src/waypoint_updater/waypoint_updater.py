#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Author: Mithi Sevilla <mithi.sevilla@gmail.com>
# Author: Maurice Loskyll <maurice@loskyll.de>
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   15.09.2017
#-------------------------------------------------------------------------------

"""
This node publishes waypoints from the car's current position
to some distance ahead with respective target speeds for each waypoint.
"""

import rospy
import math
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane
import waypoint_helper

LOOKAHEAD_WPS = 200 #: Number of waypoints we will publish
STALE_TIME = 2

def euclidean_distance(a, b):
    """The distance between two points"""
    return sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

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
        #self.closest_obs = None
        self.current_waypoint_ahead = None
        self.current_velocity = 0.
        self.previous_car_index = 0 #: Where in base waypoints list the car is
     #   position fo closetst traffic light
        self.traffic_index = 0 #: Where in base waypoints list the traffic light is
        self.traffic_time_received = rospy.get_time() #: When traffic light info was received

#self.slowdown_coefficient = rospy.get_param("~slowdown_coefficient")
        self.stopped_distance = 0.25

        self.loop()

    #---------------------------------------------------------------------------
    def loop(self):
        """
        Publishes car index and subset of waypoints with target velocities
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.base_waypoints is None or self.pose is None or self.frame_id is None:
                continue
            
            rospy.logwarn("tarffic light index:={}".format(self.traffic_index))
            rospy.logwarn("slow-down coeffs:={}".format(self.slowdown_coefficient))
            # Where in base waypoints list the car is
            #car_index = waypoint_helper.get_closest_waypoint_index(self.pose, self.base_waypoints)

            closest_car_index = waypoint_helper.get_closest_waypoint_index(self.pose, self.base_waypoints)

            # Get subset waypoints ahead
            lookahead_waypoints = waypoint_helper.get_next_waypoints(self.base_waypoints,
                                                    closest_car_index, LOOKAHEAD_WPS)

            # Traffic light must be new and near ahead
            time = rospy.get_time() - self.traffic_time_received
            rospy.logwarn("time we received tl location:={}".format(time))
            is_new = time < STALE_TIME
            is_near_ahead = False

#            if (self.traffic_index - car_index) > 0:
#                d = self.distance(self.base_waypoints, start_index, self.traffic_index)
#                car_wp = self.base_waypoints[start_index]
#                if d < car_wp.twist.twist.linear.x ** self.slowdown_coefficient:
#                    is_near_ahead = True

            # Set target speeds
            # if is_new and is_near_ahead:
            #wait for initializing......
            speeds = 10
            closest_gap = self.traffic_index - closest_car_index

            if self.traffic_index == 0  or self.traffic_index is None:
                speeds = 0
                rospy.logwarn("please wait for initializing.....")
            elif self.traffic_index != -1 and closest_gap > 0 :
                if closest_gap> 5 and closest_gap <= 20:
                    speeds=0
                elif closest_gap <=5:
                    self.current_velocity = self.current_velocity+0.5 if self.current_velocity < 10

                elif closest_gap > 20 and closest_gap <= 120:
                    speeds  = self.current_velocity*(1-(16/closest_gap))
                else:
                    speeds = min(10,.15 * (closest_gap - 10))
             
            else:
                #now there is no tl ahead, we could speed up!
                rospy.logwarn("we could speed up now!....")
                if self.current_velocity < 15:
                    speeds = = np.linspace(self.current_velocity, 15, 12)
                else :
                    speeds = 15

            rospy.logwarn("wp_updater: published speed: {}".format(speeds))

#            if self.traffic_index != -1 and is_new:
#                speeds = 10.
#                dist=self.distance(self.base_waypoints, 0, self.traffic_index-closest_car_index)
#                if self.current_velocity >1.:
#                    speeds = min(10, 0.15*(dist-5))
#                else:
#                    speeds = 0.

                # Slow down and stop
            for i, waypoint in enumerate(lookahead_waypoints):
                self.set_waypoint_velocity(self.base_waypoints, i+closest_car_index, speeds)
#                    _, waypoint.twist.twist.linear.x = self.get_distance_speed_tuple(car_index + i)

            # Publish
            lane = waypoint_helper.make_lane_object(self.frame_id, lookahead_waypoints)
            self.final_waypoints_pub.publish(lane)
            self.car_index_pub.publish(closest_car_index)

#    def get_distance_speed_tuple(self, index):
#        """
#        Return tuple of distance from traffic light
#        and target speed for slowing down
#        """
#        d = waypoint_helper.distance(self.base_waypoints, index, self.traffic_index)
#        car_wp = self.base_waypoints[index]
#        car_speed = car_wp.twist.twist.linear.x
#        speed = 0.0
#
#        if d > self.stopped_distance:
#            speed = (d - self.stopped_distance) * (car_speed ** (1-self.slowdown_coefficient))
#
#        if speed < 1.0:
#            speed = 0.0
#        return d, speed

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
    
    def get_waypoint_velocity(self, waypoint):
        """Unwrap the waypoint to return the value of the linear speed."""
        return waypoint.twist.twist.linear.x
    
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        """Unwraps the waypoint object to set the value for the linear speed."""
        waypoints[waypoint].twist.twist.linear.x = velocity

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
#
#    def _get_waypoint_indices(self, start_index, length):
#        """Computes a cyclic list of waypoint indices.
#
#            Args:
#            start_index (int): Initial index of the list
#            length (int): Desired length of resulting list, defaults to length of base points list
#
#            Returns:
#            cyclic list of waypoint indices
#            """
#
#            # making sure that length does not overpass base waypoints length
#            length = min(self.len_base_waypoints, length)
#
#            end_index = start_index + length
#            q, r = divmod(end_index, self.len_base_waypoints)
#
#            # q can either be 0 or 1
#            if q == 0:
#                return range(start_index, r)
#
#            return range(start_index, self.len_base_waypoints) + range(0, r)
#
#    def _closest_waypoint_index(self):
#        """ Computes the index of closest waypoint w.r.t current position."""
#
#            rospy.logdebug("computing closest_waypoint_index for pos %d, %d",
#                           self.current_pose.position.x,
#                           self.current_pose.position.y)
#
#            if self.current_waypoint_ahead is None:
#                possible_waypoint_indices
#                self._get_waypoint_indices(0,self.len_base_waypoints)
#                closest_distance = float('inf')
#
#            else:
#            possible_waypoint_indices=self._get_waypoint_indices(self.current_waypoint_ahead, LOOKAHEAD_WPS)
#            closest_distance=dl(self.base_waypoints[self.current_waypoint_ahead].pose.pose.position,self.current_pose.position)
#
#                prev_index = possible_waypoint_indices.pop(0)
#                closer_point_found = True
#
#                while closer_point_found and len(possible_waypoint_indices) > 0:
#                    index = possible_waypoint_indices.pop(0)
#                    distance = dl(self.base_waypoints[index].pose.pose.position,
#                                                         self.current_pose.position)
#
#                    if distance > closest_distance:
#                         closer_point_found = False
#                    else:
#                         closest_distance = distance
#                         prev_index = index
#
#                while is_waypoint_behind_pose(self.current_pose, self.base_waypoints[prev_index]):
#                         prev_index += 1
#                         prev_index %= self.len_base_waypoints
#
#                self.current_waypoint_ahead = prev_index
#
#                return prev_index
    #---------------------------------------------------------------------------


#-------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
