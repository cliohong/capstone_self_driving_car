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
from threading import Lock, Thread, Event

'''
    The model is based on semantic segmentation (FCN). The model will predict the potential traffic lights in image and classfiy what kind of light it is.
   If next traffic light is out of range/in range but not Red then -1 will be published
'''

STATE_COUNT_THRESHOLD = 1
LOOKAHEAD_WPS = 200

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = np.array([])
        self.theta = None
        #self.lights = []
        self.bridge = CvBridge()
        self.has_image = False
        self.camera_image = None

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.state_count = 0
        self.in_range = False
        self.in_last_range = False
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        #cached positions of stop lines in front of traffic lights
        self.stop_line_pos_indxs=[]
            #[Point(x,y) for x,y in self.config['stop_line_positions']]
        #self.closest_stop_line_index = None
        #last detected index of red oncoming traffic light
        self.next_traffic_light_wpt_index= -1
        #last nearest waypoint index of the car
        self.car_index = None

        #self.time_received = rospy.get_time()
        #self.stop_light_loc={}
        
        
        #lock to access to image data between 2 threads at same time
        self.lock = Lock()
        #Detector thread waits for the event
        self.event = Event()
        self.event.clear()
        #create and start a next detector thread that do jobs of detecting/classfying traffic light images
        self.thread = Thread(target = self.detector_thread)
        self.thread.start()
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb,queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb,queue_size=1)
        

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
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

#rospy.Subscriber('/car_index', Int32,self.car_index_cb, queue_size=1)

#self.camera_info = self.config['camera_info']
                      
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        rospy.spin()
        #wait for 5 secs for next detector thread
        self.thread.join(timeout = 5)
                      
                      
#    def loop(self):
#        rate = rospy.Rate(8)
#        while not rospy.is_shutdown():
#            if not self.init:
#                if self.waypoints:
#                    #self.nwp = self.nextWaypoint(self.pose)
#                    self.next_traffic_light_wpt = self.get_next_light_waypoint()
#                    if self.next_traffic_light_wpt is not None and self.sub_raw_image is None:
#                            self.sub_raw_image = rospy.Subscriber('/image_color', Image, self.image_cb)
#                    elif self.next_traffic_light_wpt is None and self.sub_raw_image is not None:
#                            self.sub_raw_image.unregister()
#                            self.sub_raw_image = None
#                            self.last_wp = -1
#                            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
#            elif self.light_classifier is None:
#                self.upcoming_red_light_pub.publish(Int32(-1))
#
#            rate.sleep()


    
                      
    def pose_cb(self, msg):
        self.pose = msg.pose
        self.orientation = self.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
                                    self.orientation.x,
                                    self.orientation.y,
                                    self.orientation.z,
                                    self.orientation.w])
                                    #rospy.logwarn("Initialize the traffic light detector....")
        
        self.theta = euler[2]

    def waypoints_cb(self, msg):
        if self.base_waypoints is None:
            waypoints=np.array([])
            for waypoint in msg.waypoints:
                waypoints= np.append(waypoints, complex(waypoint.pose.pose.position.x,waypoint.pose.pose.position.y))
            waypoints = self.base_waypoints
#

#        for stop_loc in self.stop_line_positions:
#            stop_index =self.get_closest_waypoint_index(stop_loc)
#            self.stop_light_loc[stop_loc]=stop_index


#    def traffic_cb(self, msg):
#        self.lights = msg.lights
#
#    def car_index_cb(self,msg):
#        self.car_index = msg.data

    def image_cb(self, msg):
        """
        Args:
            msg (Image): image from car-mounted camera
        """
        
        self.has_image = True
        self.camera_image = msg
        self.lock.acquire()
        self.event.set()
        self.lock.release()
    
    def get_closest_waypoint_index(self, position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            position: position to match a waypoint to ros type or complex(x,y)

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
#        def get_squared_distance(a, b):
#        #returns squared distance between two points
#            return (a.x - b.x)**2 + (a.y - b.y)**2

        if len(self.base_waypoints) == 0:
            rospy.logwarn("waypoints array is not initialized")
            return -1
        if type(position) is list:
            dist = np.vectorize(lambda waypoint: np.linalg.norm(complex(position[0],position[1]) - waypoint))
        else:
            dist = np.vectorize(lambda waypoint:np.linalg.norm(complex(position.position.x, position.position.y) - waypoint))
#
#        for i ,waypoint in enumerate(self.waypoints):
#            dist =get_squared_distance(light_position,waypoint)
#            if dist < closest_dist:
#                closest_dist, closest_index=dist, i
        closest_index = dist(self.base_waypoints)

        return np.argmin(closest_index)
#
    def get_next_light(self,stop_line_locations):
        """
            find index and location of next traffic light which should be in front
            of the car
            
            Returns:
                int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
        """
        if len(self.base_waypoints) == 0:
            rospy.logdebug("waypoints array is not set")
            return -1
        if self.pose == None:
            rospy.logdebug("Pose is not set")
            return -1
        car_dir = 1
        next_light_index = -1
        if len(self.stop_line_pos_indxs)==0:
            for stop_line_loc in stop_line_locations:
                stop_line_pos_indxs =self.get_closest_waypoint_index(stop_light_loc)
            self.stop_line_pos_indxs.append(stop_line_pos_indxs)
        
        #get car_index
        curr_car_index = self.get_closest_waypoint_index(self.pose.pose)
        if self.car_index is not None:
            if(self.car_index - curr_car_index) <= 0:
                car_dir = 1
            else:
                car_dir= -1
        else:
            car_dir = 1

        self.car_index = curr_car_index
        closest_light_index = []
        self.in_last_range = self.in_range
        #find the closest upcoming stop line waypoint index in front of the car
        for line_index in self.stop_line_pos_indxs:
            if 0 < (car_dir * (line_index - car_index)) <= 100:
                closest_light_index.append(line_index)

        if any(closest_light_index):
            next_light_index  = self.stop_line_pos_indxs[np.argmax(closest_light_index)]
            self.in_range = True
        else:
            self.in_range = False

        if self.in_range and not self.in_last_range:
            rospy.logwarn("Traffic light in range:={}".format(next_light_index))
            self.in_last_range = self.in_range

        return next_light_index
            

    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.has_image:
            return TrafficLight.UNKNOWN

        MIN_IMG_HEIGHT = 50

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8") #"rgb8"
        #get bounding box of traffic light which has been detected
        
        classification = {TrafficLight.RED: 0 , TrafficLight.YELLOW: 0 ,
            TrafficLight.GREEN : 0, TrafficLight.UNKNOWN : 0}
    
        bboxes , _ = self.light_classifier.get_detector(cv_image)
        for i ,box in  enumerate(bboxes):
            x1 = box[0][0]
            y1 = box[0][1]
            x2 = box[1][0]
            y2 = box[1][1]
            pred_img = cv_image[y1:y2,x1:x2]
            #skip small images to avoid false positive
            if pred_img.shape[0] < MIN_IMG_HEIGHT:
                rospy.loginfo("too small to be detected, we would discard and continue...")
                continue

            light_class = self.light_classifier.get_classification(pred_img)
            classification[light_class]+=1

        light_state = max(classification, key = classification.get)
        if light_state == TrafficLight.RED:
            rospy.logwarn("RED light has been detected")
        elif light_state == TrafficLight.YELLOW:
            rospy.logwarn("YELLOW light has been detected")
        elif light_state == TrafficLight.GREEN:
            rospy.logwarn("GREEN light has been detected")
        else:
            rospy.logwarn("the detector is UNKNOWN...")
        return light_state
    
    def detector_thread(self):
        """ Loop runs in seperate thread. Identifies red lights in the incoming camera image and publishes the index of the waypoint closest to the red light's stop line to /traffic_waypoint
        """
        while not rospy.is_shutdown() and self.event.wait():
            self.lock.acquire()
            self.event.clear()
            self.lock.release()
            start = timer()
            traffic_light_index = self.get_next_light(self.config['stop_line_positions'])
    
            if traffic_light_index > -1:
                #run image detection
                light_state = self.get_light_state()
                self.process_and_public_traffic_lights(light_state,traffic_light_index)
                    #rospy.logwarn("detected thread state = {}".format(light_state))
            else:
                self.process_and_public_traffic_lights(TrafficLight.UNKNOWN,-1)


    def process_and_public_traffic_lights(self, state, traffic_light_index):
        """Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
        """
    
        if self.state != state:
            self.state_count =1
            self.state = state
        else:
            self.state_count += 1
        
        if self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if self.state == TrafficLight.RED:
                self.next_traffic_light_wpt_index = traffic_light_index
            else:
                self.next_traffic_light_wpt_index = -1
        self.upcoming_red_light_pub.publish(Int32(self.next_traffic_light_wpt_index))

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

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
