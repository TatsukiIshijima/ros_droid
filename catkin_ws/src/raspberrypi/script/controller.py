#!/usr/bin/env python
import cv2
import rospy

from const.node import CONTROLLER_NODE
from const.topic import ARDUINO_DC_MOTOR_TOPIC
from const.topic import ARDUINO_INFRARED_TOPIC
from const.topic import ARDUINO_LED_TOPIC
from const.topic import ARDUINO_SERVO_TOPIC
from const.topic import DETECTOR_RESULT_TOPIC
from jetson.msg import BoundingBox2D
from jetson.msg import Detection2D
from jetson.msg import Detection2DArray
from jetson.msg import ObjectHypothesis
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import UInt8

class Controller:
    
    def __init__(self):
        self.__led_pub = rospy.Publisher(ARDUINO_LED_TOPIC, Bool, queue_size=10)
        self.__servo_pub = rospy.Publisher(ARDUINO_SERVO_TOPIC, UInt8, queue_size=10)
        self.__dc_motor_pub = rospy.Publisher(ARDUINO_DC_MOTOR_TOPIC, Int8, queue_size=10)
        rospy.init_node(CONTROLLER_NODE, anonymous=True)
        rospy.loginfo('%s started', CONTROLLER_NODE)

    def __subscribe_detection(self):
        rospy.Subscriber(DETECTOR_RESULT_TOPIC, Detection2DArray, self.__process_detection_result)

    def __process_detection_result(self):
        pass

    def __subscribe_infrared(self):
        rospy.Subscriber(ARDUINO_INFRARED_TOPIC, Float32MultiArray, self.__process_infrared)

    def __process_infrared(self):
        pass

    def subsciribe(self):
        self.__subscribe_infrared()
        self.__subscribe_detection()
        rospy.spin()

    def publish_led(self, isOn):
        self.__led_pub.publish(isOn)

    def publish_servo(self, value):
        if value < 0 or value > 180:
            return
        self.__servo_pub.publish(value)

    def publish_dc_motor(self, value):
        if value < -128 or value > 127:
            return
        self.__dc_motor_pub.publish(value)

if __name__ == '__main__':
    controller = Controller()
    controller.subsciribe()