#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   26.09.2017
#-------------------------------------------------------------------------------

"""
A Traffic Light Detector ROS Node
"""

import rospy
from dummy_detector import DummyDetector
from real_detector import RealDetector

if __name__ == '__main__':
    try:
        detector_type = rospy.get_param("/traffic_light_detector")
        if detector_type.lower() == 'dummy':
            detector = DummyDetector()
        else:
            detector = RealDetector()

        detector.loop()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
