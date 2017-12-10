"""
Helper functions for waypoints_updater
"""
# pylint: disable=invalid-name

from math import cos, sin, sqrt
from copy import deepcopy

import rospy
import numpy as np
import tf
import math
from styx_msgs.msg import Lane
dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)


def make_lane_object(frame_id, waypoints):
    """Lane object contains the list of final waypoints ahead with velocity"""
    lane = Lane()
    lane.header.frame_id = frame_id
    lane.waypoints = waypoints
    lane.header.stamp = rospy.Time.now()
    return lane


def get_square_gap(a, b):
    """Returns squared euclidean distance between two 2D points"""
    dx = a.x - b.x
    dy = a.y - b.y
    return dx * dx + dy * dy


def is_waypoint_behind(pose, waypoint):
    """Take a waypoint and a pose , do a coordinate system transformation
    setting the origin at the position of the pose object and as x-axis
    the orientation of the z-axis of the pose

    Args:
        pose (object) : A pose object
        waypoints (object) : A waypoint object

    Returns:
        bool : True if the waypoint is behind the car False if in front

    """
    _, _, yaw = tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                          pose.orientation.y,
                                                          pose.orientation.z,
                                                          pose.orientation.w])
    originX = pose.position.x
    originY = pose.position.y

    shift_x = waypoint.pose.pose.position.x - pose.position.x
    shift_y = waypoint.pose.pose.position.y - pose.position.y

    shifted_rotated_x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)

    if shifted_rotated_x > 0:
        return False
    return True


def get_closest_waypoint_index(pose, waypoints):
    """
    pose: geometry_msg.msgs.Pose instance
    waypoints: list of styx_msgs.msg.Waypoint instances
    returns index of the closest waypoint in the list waypoints
    """
    closest_dist =  float('inf')
    closest_index = 0
    my_position = pose.position

    for i, waypoint in enumerate(waypoints):
        dist =dl(pose.position, waypoint.pose.pose.position)
    #get_square_gap(pose.position, waypoint.pose.pose.position)

        if dist < closest_dist:
            closest_index, closest_dist = i, dist

    is_behind = is_waypoint_behind(pose, waypoints[closest_index])
    if is_behind:
        closest_index += 1
        closest_index %= len(waypoints)
    return closest_index


def get_next_waypoints(waypoints, i, n):
    """Returns a list of n waypoints ahead of the vehicle"""
    m = min(len(waypoints), i + n)
    return deepcopy(waypoints[i:m])


def fit_polynomial(waypoints, degree):
    """fits a polynomial for given waypoints"""
    x_coords = [waypoint.pose.pose.position.x for waypoint in waypoints]
    y_coords = [waypoint.pose.pose.position.y for waypoint in waypoints]
    return np.polyfit(x_coords, y_coords, degree)


def calculateRCurve(coeffs, X):
    """calculates the radius of curvature"""
    if coeffs is None:
        return None
    a = coeffs[0]
    b = coeffs[1]
    return (1 + (2 * a * X + b) ** 2) ** 1.5 / np.absolute(2 * a)
