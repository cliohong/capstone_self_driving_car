#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from timeit import default_timer as timer
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
from collections import namedtuple

"""
The model is based on semantic segmentation (FCN). The model will predict the potential traffic lights in image and classfiy what kind of light it is.
If next traffic light is out of range/in range but not Red then -1 will be published
"""

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 200
Point = namedtuple('Point',['x','y'])

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.sub_raw_image = None
        self.theta = None
        self.waypoint_len = 0
        self.lights = []
        self.car_index = None
        self.bridge = CvBridge()
        self.has_image = False
        self.in_range = False
        
#        self.listener = tf.TransformListener()
#        self.field_of_view = math.pi/4.
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
       
        self.state_count = 0
        self.init = False
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        #self.camera_topic = rospy.get_param('~camera_topic')
        #cached positions of stop lines in front of traffic lights
#        self.stop_line_positions=[Point(x,y) for x,y in self.config['stop_line_positions']]
#        self.closest_stop_line_index = None
        self.next_traffic_light_wpt= None
        self.nwp = None
#        self.time_received = rospy.get_time()
#self.stop_light_loc={}
        self.traffic_light_to_wpt_map = []
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        

        self.light_classifier = TLClassifier()
        self.light_classifier.initialize()
        rospy.logwarn('classifier initialized')
                      
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        
        #rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        self.sub_raw_image = rospy.Subscriber('/image_color', Image, self.image_cb)

        rospy.Subscriber('/car_index', Int32,self.car_index_cb, queue_size=1)

#self.camera_info = self.config['camera_info']
                      
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        
        self.loop()

#rospy.spin()
                      
                      
    def loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.init:
                if self.waypoints and self.theta:
                    self.nwp = self.next_traffic_Light_wpt()
                    self.next_traffic_light_wpt = self.getNextLightWaypoint(LOOKAHEAD_WPS)
                    if self.next_traffic_light_wpt is not None and self.sub_raw_image is None:
                            self.sub_raw_image = rospy.Subscriber(self.camera_topic,Image,self.image_cb)
                    elif self.next_traffic_light_wpt is None and self.sub_raw_image is not None:
                            self.sub_raw_image.unregister()
                            self.sub_raw_image = None
                            self.last_wp = -1
                            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            rate.sleep()

#        if self.waypoints is None or self.car_index is None:
#            return
#
#        light_wpoint = self.get_next_light_waypoint()
#        if light_wpoint is None or light_wpoint == -1:
#            continue
#
#        if self.in_range:
#            state = self.get_light_state()
#            if self.state != state:
#                self.state_count = 0
#                self.state = state
#                self.closest_stop_line_index = None
#
#            if self.state_count >= STATE_COUNT_THRESHOLD:
#                if state == TrafficLight.RED:
#                    self.closest_stop_line_index = light_wpoint
#                    self.time_received = rospy.get_time()
#                    self.upcoming_red_light_pub.publish(self.closest_stop_line_index)
#                    rospy.logwarn("red light detected!")
#                else:
#                    self.upcoming_red_light_pub.publish(-1)

    
                      
    def pose_cb(self, msg):
        self.pose = msg.pose
        self.orientation = self.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
                                    self.orientation.x,
                                    self.orientation.y,
                                    self.orientation.z,
                                    self.orientation.w])
        rospy.logwarn("Initialize the traffic light detector....")
        
        self.theta = euler[2]

    def waypoints_cb(self, msg):
        if self.waypoints is None:
            self.waypoints=[]
            for waypoint in msg.waypoints:
                self.waypoints.append(waypoint)
        self.waypoint_len=len(self.waypoints)
        self.base_waypoints.unregister()
        self.base_waypoints= None
        dl = lambda a, b : math.sqrt((a.x - b[0])**2 + (a.y - b[1])**2)

   # List of positions that correspond to the line to stop in front of for a given intersection
   
        #initialize lights to waypoint map
        for x in range(len(self.config['stop_line_positions'])):
            dist = float('inf')
            light_wpt = 0
            for y in range(len(self.waypoints)):
                d1 = dl(self.waypoints[y].pose.pose.position,
                        self.config['stop_line_positions'][x])
                if dist > d1:
                    light_wp = x1
                    dist = d1
            self.traffic_light_to_wpt_map.append(light_wp)
        
#        self.waypoints = [x.pose.pose.position for x in msg.waypoints]
#        for stop_loc in self.stop_line_positions:
#            stop_index =self.get_closest_waypoint_index(stop_loc)
#            self.stop_light_loc[stop_loc]=stop_index


    def traffic_cb(self, msg):
        self.lights = msg.lights

    def car_index_cb(self,msg):
        self.car_index = msg.data
                      
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            slef.state_count = 0
            self.state = state
        elif self.state_count >  STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if state == TrafficLight.GREEN and light_wp is not None:
                light_wp = - light_wp
            elif state == TrafficLight.UNKNOWN:
                light_wp= -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub(Int32(light_wp))
            rospy.logwarn("Red or Yellow light detected:={}".format(state))
        else:
            self.upcoming_red_light_pub(Int32(self.last_wp))
        self.state_count+=1
        if self.init:
            self.init = False
        

    def nextWaypoint(self, pose):
        """Identifies the next path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
            Args:
            pose (Pose): position to match a waypoint to
            
            Returns:
            int: index of the next waypoint in self.waypoints
        """
        #DONE implement
        location = pose.position
        dist = 100000.
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        nwp = 0
        for i in range(len(self.waypoints)):
            d1 = dl(location, self.waypoints[i].pose.pose.position)
            if dist > d1:
                    nwp = i
                    dist = d1
        x = self.waypoints[nwp].pose.pose.position.x
        y = self.waypoints[nwp].pose.pose.position.y
        heading = np.arctan2((y-location.y), (x-location.x))
        angle = np.abs(self.theta-heading)
        if angle > np.pi/4.:
            nwp += 1
            if nwp >= len(self.waypoints):
                nwp = 0
        return nwp

    def getNextLightWaypoint(self, number):
        # find the closest waypoint from our pre-populated waypoint to traffic light map
        tlwp = None
        light = None
        for i in range(len(self.traffic_light_to_wpt_map)):
            # make sure its forward in our direction
            if self.nwp < self.traffic_light_to_wpt_map[i] and tlwp is None:
                tlwp = self.traffic_light_to_wpt_map[i] - self.nwp
                # is it within the given number?
                if tlwp < number-2:
                    # is it within our traffic light tracking distance of 100 meters?
                    if self.distance(self.waypoints, self.nwp, (self.nwp+tlwp)%self.waypoint_len) < 100.:
                        
                    # set the traffic light waypoint target
                    # light = (self.nwp+tlwp)%self.wlen
                    # use relative waypoint ahead of current one instead!
                        light = tlwp
        return light
    
    
#    def get_closest_waypoint_index(self, light_position):
#        """Identifies the closest path waypoint to the given position
#            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
#        Args:
#            light_position: 2d location of light
#            waypoints: a list of waypoint object from self.waypoints
#
#        Returns:
#            int: index of the closest waypoint in self.waypoints
#
#        """
#        #TODO implement
#        def get_squared_distance(a, b):
#        #returns squared distance between two points
#            return (a.x - b.x)**2 + (a.y - b.y)**2
#
#        if len(self.waypoints) == 0:
#            rospy.logwarn("waypoints array is not initialized")
#            return -1
#
#        closest_dist = float('inf')
#        closest_index = 0
#
#        for i ,waypoint in enumerate(self.waypoints):
#            dist =get_squared_distance(light_position,waypoint)
#            if dist < closest_dist:
#                closest_dist, closest_index=dist, i
#
#        return closest_index
#
#    def get_next_light(self):
#        """
#            find index and location of next traffic light which should be in front
#            of the car
#        """
#        if len(self.waypoints) == 0:
#            rospy.logdebug("waypoints array is not set")
#            return -1
#        if self.pose == None:
#            rospy.logdebug("Pose is not set")
#            return -1
#
#        next_light_index =float('Inf')
#        next_light = None
#        for light,light_index in self.stop_light_loc.items():
#            if light_index > self.car_index and light_index < next_light_index:
#              next_light = light
#              next_light_index = light_index
#
#        if next_light is None:
#            return None, None
#        return next_light_index,next_light
#
#    def get_next_light_waypoint(self):
#        """
#            get next traffic light point if it is within the view of camera
#        """
#        light_idx, light = self.get_next_light()
#        if light is None:
#            return None
#
#        if 0 < (light_idx - self.car_index) and (light_idx - self.car_index)<=120:
#
##            rospy.logwarn("Traffic light in range StopLine_WP: {}, Car_WP: {}".format(light_idx, self.car_index))
#            self.in_range =True
#            return light_idx
#        else:
#            self.in_range = False
#            return -1


    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            return TrafficLight.RED
        

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8") #"bgr8"
        #get bounding box of traffic light which has been detected
        bboxes , _ = self.light_classifier.get_detector(cv_image)
        for i ,box in  enumerate(bboxes):
            x1 = box[0][0]
            y1 = box[0][1]
            x2 = box[1][0]
            y2 = box[1][1]
            pred_img = cv_image[y1:y2,x1:x2]
        #skip small images to avoid false positive
        MIN_IMG_HEIGHT = 20
        if pred_img.shape[0]<MIN_IMG_HEIGHT:
            rospy.loginfo("image detected is too small:false positive avoided")
            return TrafficLight.UNKNOWN
        else:

            classification = {TrafficLight.RED: 0 , TrafficLight.YELLOW: 0 ,
            TrafficLight.GREEN : 0, TrafficLight.UNKNOWN : 0}
            light_class = self.light_classifier.get_classification(pred_img)
            classification[light_class]+=1
            light_state = max(classification, key = classification.get)
            return light_state


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
    
        if self.init:
            return -1, TrafficLight.UNKNOWN
        elif self.next_traffic_light_wpt:
            state = self.get_light_state()
            return self.next_traffic_light_wpt, state
        return -1, TrafficLight.UNKNOWN

#        # List of positions that correspond to the line to stop in front of for a given intersection
#        stop_line_positions = self.config['stop_line_positions']
#        if(self.pose):
#            car_position = self.get_closest_waypoint(self.pose.pose)
#
#        #TODO find the closest visible traffic light (if one exists)
#
#        if light:
#            state = self.get_light_state(light)
#            return light_wp, state
#        self.waypoints = None
#        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
