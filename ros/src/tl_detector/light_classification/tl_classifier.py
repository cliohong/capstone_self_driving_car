from styx_msgs.msg import TrafficLight
import os
import cv2
import tensorflow as tf
import numpy as np
from timeit import default_timer as timer
from scipy.ndimage.measurements import label
import rospy
#os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

DETECTOR_MODEL = 'detector_graph.pb'


def decode_boxes(seg):
    bboxes = []
    labels = label(seg)
    for i in range(1, labels[1]+1):
        nonzero_pix = (labels[0] == i).nonzero()
        #identify both x and y yalues in those pixels
        nonzerox = np.array(nonzero_pix[1])
        nonzeroy = np.array(nonzero_pix[0])
        bbox = ((np.min(nonzerox),np.min(nonzeroy)), (np.max(nonzerox),np.max(nonzeroy)))
        #remove noise from detection
        if (bbox[1][0]- bbox[0][0])<4. or (bbox[1][1]-bbox[0][1])<8. :
            continue
        bboxes.append(bbox)
    return bboxes

class TLClassifier(object):

    def __init__(self):
        #TODO load classifier
        self.detector_graph = DETECTOR_MODEL
        self.session = None
        self.class_input = None
        self.class_prediction = None
        self.class_keep_prob = None
        self.img_size =(288,384)

    def initialize(self):
        """
            Initialize the classifier - starts the TensorFlow session and makes
            sure it allocates all the necessary memory
            """
        config = tf.ConfigProto(log_device_placement=False)
        config.gpu_options.allow_growth = True
        config.gpu_options.per_process_gpu_memory_fraction = 0.9
        self.session = tf.Session(config = config)
        sess=self.session
 
        # Read the detector metagraph
        detector_graph_def = tf.GraphDef()
        with open(self.detector_graph, 'rb') as f:
            serialized = f.read()
            detector_graph_def.ParseFromString(serialized)


        tf.import_graph_def(detector_graph_def, name='detector')
        self.class_input = sess.graph.get_tensor_by_name('detector/data/images:0')
        self.class_prediction = sess.graph.get_tensor_by_name('detector/predictions/prediction_class:0')
        self.class_keep_prob = sess.graph.get_tensor_by_name('detector/keep_prob:0')
        
    
        # test TensorFlow to allocate all the data structs in the right way
        fake_img = np.zeros((288, 384, 3), dtype=np.uint8)
        self.get_detector(fake_img)

    def get_detector(self, img):
        """Detects the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            a list of images which are detected by model

        """
        #TODO implement light color prediction
        sess =self.session
        #detect boxes
        h ,w =img.shape[0],img.shape[1]
        crop_h, crop_w = self.img_size[0],self.img_size[1]
        resized_img = cv2.resize(img,(crop_w,crop_h), interpolation = cv2.INTER_NEAREST)
        
        img_expanded = np.expand_dims(img, axis = 0)
        
        #RUN PREDICTION
        start_time = timer()
        light_class = sess.run([self.class_prediction],
                               feed_dict={self.class_input:[resized_img],
                               self.class_keep_prob:1.})
        light_class = np.array(light_class[0][0,:,:,:],dtype = np.uint8)
        tf_time =int(timer() - start_time)*1000
        #take pics which contain only traffic light
        seg = np.expand_dims(light_class[:,:,1], axis=2)
        bboxes = decode_boxes(seg)
        boxes=[]
        for i ,box in enumerate(bboxes):
            x_min = int(box[0][0]*w / crop_w)
            y_min = int(box[0][1]*h / crop_h)
            x_max = int(box[1][0]*w / crop_w)
            y_max = int(box[1][1]*h / crop_h)
            boxes.append(((x_min,y_min),(x_max,y_max)))
    

        return boxes, tf_time

    def get_classification(self, img):
        """classify the traffic light in color
            
            Args:
            image (cv::Mat): image containing the traffic light
            the area of the image with highest L value corresponds to the section of traffic light which currently lights on
            the classifier classify the traffic light with a series of 3 light bulbs: red at the top, yellow in the middle and green at the bottom
            
            Returns:
            Trafficlight type id corresponds to light color
            
        """
        self.img_h, self.img_w, _ =img.shape
        #take LUV color conversion and extract L channel
        l_channel = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
        self.l_channel = l_channel[int(self.img_h*0.1):int(self.img_h*0.9),
                                       int(self.img_w):int(self.img_w*0.9),0]
        #split the image to three parts
        self.top_red = int(self.l_channel.shape[0]/3)
        self.bottom_green = int(self.l_channel.shape[0] - self.top_red)
        count_ryg = {'RED':0, 'YELLOW':0, 'GREEN':0}
        top, mid, bottom = 0, 0, 0

        for i in range(self.top_red):
            for j in range(self.l_channel.shape[1]):
                top +=self.l_channel[i][j]
        count_ryg['RED'] = top

        for i in range(self.top_red,self.bottom_green):
            for j in range(self.l_channel.shape[1]):
                mid += self.l_channel[i][j]
        count_ryg['YELLOW'] = mid

        for i in range(self.bottom_green):
            for j in range(self.l_channel.shape[1]):
                bottom+=self.l_channel[i][j]
        count_ryg['GREEN'] = bottom

        #get the max value which is the color we predicted
        max_ryg  = max(count_ryg,  key = count_ryg.get)
        if max_ryg == 'RED':
            rospy.logdebug("Red light detected!")
            return TrafficLight.RED
        elif max_ryg == 'YELLOW':
            rospy.logdebug("Yellow light detected!")
            return TrafficLight.YELLOW
        elif max_ryg == 'GREEN':
            rospy.logdebug("Green light detected!")
            return TrafficLight.GREEN
        else:
            rospy.logwarn("ErroR - cannot classify !")
            return TrafficLight.UNKNOWN
