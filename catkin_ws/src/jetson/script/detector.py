#!/usr/bin/env python
import cv2
import rospy

from const.node import DETECTOR_NODE
from const.topic import CAMERA_IMAGE_RAW_TOPIC, DETECTOR_RESULT_TOPIC
from cv_bridge import CvBridge
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy
from jetson.msg import BoundingBox2D
from jetson.msg import Detection2D
from jetson.msg import Detection2DArray
from jetson.msg import ObjectHypothesis
from sensor_msgs.msg import Image

'''
How to use (sample)

$roscore
$roslaunch video_stream_opencv camera.launch video_stream_provider:=/home/jetson/playground/videos/parking.avi visualize:=false loop_videofile:=true
$rosrun jetson detector.py
'''

class Detector:

    def __init__(self):
        self.__net = detectNet('ssd-mobilenet-v2', threshold=0.5)
        self.__bridge = CvBridge()
        self.__pub = rospy.Publisher(DETECTOR_RESULT_TOPIC, Detection2DArray, queue_size=10)
        rospy.init_node(DETECTOR_NODE, anonymous=True)
        rospy.loginfo('%s started', DETECTOR_NODE)

    def subscribe_image(self):
        rospy.Subscriber(CAMERA_IMAGE_RAW_TOPIC, Image, self.__process_image)
        rospy.spin()

    def __process_image(self, image):
        try:
            origin = self.__bridge.imgmsg_to_cv2(image, 'bgr8')

            rgb_img = cv2.cvtColor(origin, cv2.COLOR_BGR2RGB)
            
            cuda_mem = cudaFromNumpy(rgb_img)

            detections = self.__net.Detect(cuda_mem, overlay='box,labels,conf')

            rospy.loginfo('Object Detection | Network %0f FPS | detected %d objects in image' % (self.__net.GetNetworkFPS(), len(detections)))

            detection2d_array = []

            for detection in detections:
                detection2d = self.__convert_detect2d(detection)
                detection2d_array.append(detection2d)

                bbox = detection2d.bbox
                result = detection2d.result
                cv2.rectangle(origin, (bbox.left, bbox.top), (bbox.right, bbox.bottom), (255, 0, 0))
                rospy.loginfo('%s, Confidence=%1f' % (result.name, result.score))

            self.__pub.publish(detection2d_array)

            cv2.imshow('Result', origin)
            cv2.waitKey(1)
            
        except rospy.ROSInterruptException as e:
            rospy.logerr('Exception %s' % (e.args))
    
    def __convert_detect2d(self, detection):
        left, top, right, bottom = int(detection.Left), int(detection.Top), int(detection.Right), int(detection.Bottom)
        name, score = self.__net.GetClassDesc(detection.ClassID), detection.Confidence
        bbox = BoundingBox2D(left, top, right, bottom)
        object_hypothesis = ObjectHypothesis(name, score)
        return Detection2D(bbox=bbox, result=object_hypothesis)

if __name__ == '__main__':
    detector = Detector()
    detector.subscribe_image()