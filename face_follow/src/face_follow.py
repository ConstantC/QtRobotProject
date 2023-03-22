#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
import threading
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from qt_nuitrack_app.msg import Faces, FaceInfo



class image_converter:
    faces = None
    faces_time = None   
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.image_callback)
        self.face_sub = rospy.Subscriber("/qt_nuitrack_app/faces", Faces, self.face_callback)
        

    def face_callback(self, data):
        # print("face_callback")
        self.lock.acquire()
        self.faces = data.faces
        self.faces_time = rospy.Time.now()
        self.lock.release()

    def image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        self.lock.acquire()
        new_faces = self.faces
        new_faces_time = self.faces_time
        self.lock.release()

        if new_faces and (rospy.Time.now()-new_faces_time) < rospy.Duration(5.0):
            face = new_faces[0]
            rect = face.rectangle
            cv2.rectangle(cv_image, (int(rect[0]*cols),int(rect[1]*rows)),
                                    (int(rect[0]*cols+rect[2]*cols), int(rect[1]*rows+rect[3]*rows)), (0,255,0), 2)
            
        
            global xpos 
            global ypos
            xpos = int(rect[0]*cols)
            ypos = int(rect[1]*rows)
            
            x = int(rect[0]*cols)
            y = int(rect[1]*rows)
            w = int(rect[2]*cols)
            h = int(rect[3]*rows)
            #rospy.loginfo("Rectangle POS: x: %s || y: %s" % (str(int(rect[0]*cols)),str(int(rect[1]*rows))))  
            #cv2.putText(cv_image, "Gender:", (x, y+h+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), lineType=cv2.LINE_AA)
            cv2.putText(cv_image, "Gender: %s" % face.gender, (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            cv2.putText(cv_image, "Age: %d" % face.age_years, (x, y+h+40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)

            # Neutral
            cv2.putText(cv_image, "Neutral:", (x, y+h+60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            cv2.rectangle(cv_image, (x+80,y+h+50),
                                    (x+80+int(face.emotion_neutral*100), y+h+10+50), (0,255,0), cv2.FILLED)
            cv2.rectangle(cv_image, (x+80,y+h+50),
                                    (x+80+100, y+h+10+50), (255,255,255), 1)
            # Angry
            cv2.putText(cv_image, "Angry:", (x, y+h+80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            cv2.rectangle(cv_image, (x+80,y+h+70),
                                    (x+80+int(face.emotion_angry*100), y+h+10+70), (0,255,0), cv2.FILLED)
            cv2.rectangle(cv_image, (x+80,y+h+70),
                                    (x+80+100, y+h+10+70), (255,255,255), 1)

            # Happy
            cv2.putText(cv_image, "Happy:", (x, y+h+100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            cv2.rectangle(cv_image, (x+80,y+h+90),
                                    (x+80+int(face.emotion_happy*100), y+h+10+90), (0,255,0), cv2.FILLED)
            cv2.rectangle(cv_image, (x+80,y+h+90),
                                    (x+80+100, y+h+10+90), (255,255,255), 1)

            cv2.putText(cv_image, "Surprise:", (x, y+h+120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            cv2.rectangle(cv_image, (x+80,y+h+110),
                                    (x+80+int(face.emotion_surprise*100), y+h+10+110), (0,255,0), cv2.FILLED)
            cv2.rectangle(cv_image, (x+80,y+h+110),
                                    (x+80+100, y+h+10+110), (255,255,255), 1)
                                    
            cv2.putText(cv_image, "POS: x: %s || y: %s" % (str(int(rect[0]*cols)),str(int(rect[1]*rows))), (x, y+h+140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('face_follow', anonymous=True)
    head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=1)
    
     # wait for publisher connections
    wtime_begin = rospy.get_time()
    while (head_pub.get_num_connections() == 0) :
        rospy.loginfo("waiting for publisher connections...")
        if rospy.get_time() - wtime_begin > 10.0:
            rospy.logerr("Timeout while waiting for publisher connection!")
            sys.exit()
        rospy.sleep(1)

    # centering the head
    href = Float64MultiArray(data=[0,0])
    head_pub.publish(href)
    rospy.sleep(1)
    xpos = 256
    ypos = 128
    xoffset = 0
    yoffset = 0
    xaxis = 256
    yaxis = 128

    ic = image_converter()
    
    try:
        while not rospy.is_shutdown():
            #rospy.loginfo("Rectangle POS: x: %s || y: %s" % (str(xpos),str(ypos)))  
            relativexpos = (xpos-256)/256
            relativeypos = (ypos-128)/128
            
            rospy.loginfo("Rectangle POS: x: %s || y: %s" % (str(relativexpos),str(relativeypos)))  
            xoffset = xoffset + -30 * relativexpos
            yoffset = yoffset + 5 * relativeypos
            
            #if xpos > 400:
            #    xoffset += -10
            #elif xpos < 100:
            #    xoffset += 10
            #if ypos < 50:
            #    yoffset += -2
            #elif ypos > 200:
            #    yoffset += 2

            if xoffset > 60:
                xoffset = 60
            elif xoffset < -60:
                xoffset = -60
            if yoffset > 10:
                yoffset = 10
            elif yoffset < -10:
                yoffset = -10
            
            rospy.loginfo("Head offset: x: %s || y: %s" % (str(xoffset),str(yoffset)))  
                             
            href.data = [xoffset, yoffset]
            head_pub.publish(href)
            rospy.sleep(2)
        #rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
