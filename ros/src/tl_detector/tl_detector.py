#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
from collections import namedtuple
import tf

STATE_COUNT_THRESHOLD = 3
Point = namedtuple('Point',['x','y'])

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
#        self.camera_image = None
#        self.lights = []
        #self.traffic_light_to_wpt_map = []
        self.car_index = None
        self.bridge = CvBridge()
        
        self.light_classifier = TLClassifier(rospkg.get_ros_home())
        
        self.listener = tf.TransformListener()
        self.field_of_view = math.pi/4.
        self.state = self.light_classifier.UNKNOWN
#        self.last_state = TrafficLight.UNKNOWN
#        self.last_wp = -1
        self.state_count = 0
        self.stop_line_positions=[Point(x,y) for x,y in config['stop_line_positions']]
        self.closest_stop_line_index = None
        self.time_received = rospy.get_time()
        self.stop_light_loc={}
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        

        if self.light_classifier.need_models():
            rospy.logwarn('Need to download models, it may take a while.')
            self.light_classifier.download_models()
            rospy.logwarn('Finished downloading!')
        self.light_classifier.initialize()
        rospy.logwarn('classifier initialized)
                      
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        rospy.Subscriber('/car_index', Int32,self.car_index_cb, queue_size=1)
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.camera_info = self.config['camera_info']
                      
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        
        self.loop()

        rospy.spin()
                      
                      
    def loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
            if(rospy.get_time()- self.time_received)>2.0:
               continue
            if self.closest_stop_line_index is not None:
                self.upcoming_red_light_pub.publish(self.closest_stop_line_index)
                      
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = [x.pose.pose.position for x in msg.waypoints]
        for stop_loc in self.stop_line_positions:
            stop_index =get_closest_waypoint_index(stop_loc)
            self.stop_light_loc[stop_loc]=stop_index
                  

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
#        self.has_image = True
#        self.camera_image = msg
#        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.waypoints is None or self.car_index is None:
            return
        light_wpoint = self.get_next_light_waypoint()
        if light_wpoint is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg,"rgb8")
        state = self.light_classifier.get_classification(img)
        if self.state != state:
            self.state_count = 0
            self.state = state
            self.closest_stop_line_index = None
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            if state == self.light_classifier.RED:
                self.closest_stop_line_index = light_wpoint
                self.time_received = rospy.get_time()
        
        self.state_count+=1
                      
#            self.upcoming_red_light_pub.publish(Int32(light_wp))
#        else:
#            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
#        self.state_count += 1

    def get_closest_waypoint_index(self, light_position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            light_position: 2d location of light
            waypoints: a list of waypoint object from self.waypoints

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        def get_squared_distance(a, b):
        """returns squared distance between two points"""
            return (a.x - b.x)**2 + (a.y - b.y)**2
                      
        closest_dist = float('inf')
        closest_index = 0
        
        for i ,waypoint in enumerate(self.waypoints):
            dist =get_squared_distance(light_position,waypoint)
            if dist <closest_dist:
                closest_dist, closest_index=dist, i
                
        return closest_index

    def get_next_light(self):
        """
            find index and location of next traffic light which should be in front
            of the car
        """
        next_light_index =float('Inf')
        next_light = None
        for light,light_index in self.stop_light_loc.items():
            if light_index > self.car_index and light_index < next_light_index:
              next_light = light
              next_light_index = light_index
                      
        if next_light is None:
            return None, None
        return next_light_index,next_light
                      
    def get_next_light_waypoint(self):
        """
            get next traffic light point if it is within the view of camera
        """
        light_idx, light = self.get_next_light()
        if light is None:
            return None
        #convert to local coordinate system
        light_pt = PointStamped()
        light_pt.header.stamp = rospy.get_rostime()
        light_pt.header.frame_id = '/world'
        light_pt.point.x=light.x
        light_pt.point.y=light.y
        light_pt.point.z=0.
                      
        try:
            self.listener.waitForTransform("/base_link","/world",
                                           rospy.get_rostime(),
                                           rospy.Duration(0.1))
            light_pt = self.listener.transformPoint("/base_link",light_pt)
        
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("failed to find camera to map transform)
            return None
           
        #check out both dist and angle are in the detective area
        eul_dist = math.sqrt(light_pt.point.x**2 + light_pt.point.y**2)
        if eul_dist > 45 :
            return None
                         
        angle = abs(math.atan2(light_pt.point.y, light_pt.point.x))
        if angle > self.field_of_view:
            return None
        
        return light_index
 
#
#    def get_light_state(self, light):
#        """Determines the current color of the traffic light
#
#        Args:
#            light (TrafficLight): light to classify
#
#        Returns:
#            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
#
#        """
#        if(not self.has_image):
#            self.prev_light_loc = None
#            return False
#
#        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
#
#        #Get classification
#        return self.light_classifier.get_classification(cv_image)
#
#    def process_traffic_lights(self):
#        """Finds closest visible traffic light, if one exists, and determines its
#            location and color
#
#        Returns:
#            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
#            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
#
#        """
#        light = None
#
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
