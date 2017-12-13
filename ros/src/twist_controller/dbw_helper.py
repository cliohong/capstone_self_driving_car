#!/usr/bin/env python

import numpy as np
import tf
from math import sqrt, cos, sin

POINTS_TO_FIT = 15

#
#def fit_polynomial(waypoints, degree):
#    """fits a polynomial for given waypoints"""
#    x_coords = [waypoint.pose.pose.position.x for waypoint in waypoints]
#    y_coords = [waypoint.pose.pose.position.y for waypoint in waypoints]
#    return np.polyfit(x_coords, y_coords, degree)


def shift_and_rotate_waypoints(pose, waypoints, points_no=None):
    """
    From a pose object transfrom a series of waypoints so that
    the origin is at the pose position and the orientation matches
    the yaw of the pose

    Args:
        pose (object) : A pose object
        waypoints (list) : A list of waypoint objects
        points_no (int) : How many points takes in account

    Returns:
        x_coords (list): The transformed x-coordinates of waypoints
        y_coords (list): The transformed y-coordinates of waypoints
    """
    x_coords = []
    y_coords = []

    _, _, yaw =tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                     pose.orientation.y,
                                                     pose.orientation.z,
                                                     pose.orientation.w])

    if points_no is None:
        points_no = len(waypoints)

    for i in range(points_no):

        shift_x = waypoints[i].pose.pose.position.x - pose.position.x
        shift_y = waypoints[i].pose.pose.position.y - pose.position.y

        shifted_rotated_x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)
        shifted_rotated_y = shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw)

        x_coords.append(shifted_rotated_x)
        y_coords.append(shifted_rotated_y)

    return x_coords, y_coords

def cte(pose, waypoints):
    """
    From a pose object and a series of waypoints, calculate the cte of the point

    Args:
        pose (object) : A pose object
        waypoints (list) : A list of waypoint objects

    Returns:
        cte (float) : the cross track error (signed)
    """
    x_coords, y_coords = shift_and_rotate_waypoints(
        pose, waypoints, POINTS_TO_FIT)
    coefficients = np.polyfit(x_coords, y_coords, 3)
    cte = np.polyval(coefficients, 5.0)

    return cte
