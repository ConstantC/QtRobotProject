#!/usr/bin/env python
import sys
import csv
import rospy
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *
import cv2
import threading
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from qt_nuitrack_app.msg import Faces, FaceInfo


speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)

class image_converter:
    faces = None
    faces_time = None   
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.image_callback)


    def image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def rules():
    try:
        # call a ros service with text message
        rospy.loginfo("Explaining rules !")
        # speechSay("Je vais te montrer des mouvements et tu dois les imiter.")
    except KeyboardInterrupt:
        pass

def end():
    try:
        # call a ros service with text message
        rospy.loginfo("End of the exercise")
        #speechSay("Fin de l'exercice, merci pour tes réponses !")
    except KeyboardInterrupt:
        pass

def gesturesImitation(gesture):
    rospy.loginfo("Gesture imitation launched, waiting for the services to initialized!")
    
    # define the ros service
    rospy.init_node('gesture_imitation_node')
    rospy.loginfo("gesture_imitation_node started!")

    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/emotion/show')
    rospy.loginfo("Services initiated")
    ic = image_converter()

    
    try:
        rospy.loginfo("Gesture: %s", gesture)
        emotionShow('gestures/' + gesture)
                        
        rospy.loginfo("End of the gesture")
    except KeyboardInterrupt:
        pass

gesturesImitation('wave')
rospy.loginfo("finsihed!")
