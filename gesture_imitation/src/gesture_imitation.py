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


if __name__ == '__main__':


    rospy.loginfo("Simple questions launched, waiting for the services to initialized!")
    
    # define the ros service
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
    rospy.init_node('simple_questions_node')
    rospy.loginfo("simple_questions_node started!")

    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/speech/recognize')
    rospy.loginfo("Services initiated")
    ic = image_converter()
    
    # Fetch the questions 
    path = "/home/qtrobot/catkin_ws/src/QtRobotProject/simple_questions/test.csv"
    rospy.loginfo("Fetching questions from: %s", path)
    questions = []
    answers = []
    
    try:
        with open(path, 'r') as file:
            csvreader = csv.reader(file)
            for row in csvreader:
                first = True
                temp = []
                for column in row:
                    if first:
                        print(column)
                        questions.append(column)
                        first = False
                    else:
                        temp.append(column)
                answers.append(temp)
                print(temp)
               
    except IOError:
        rospy.loginfo("Unable to find or open the file: %s", path)
        sys.exit(1)
    
    if not questions:
        rospy.loginfo("No data found in the file")
        sys.exit(1)
    
    rospy.loginfo("Questions fetched, found: %i", len(questions))
                  
    try:
        # call a ros service with text message
        rospy.loginfo("Explaining rules !")
        # speechSay("Je vais maintenant te demander de répondre au questions que je vais te poser.")
        score = 0
        interaction = 0
        for i in range(len(questions)):
            question = questions[i]
            rospy.loginfo("Reading question: %s", question)
            rospy.loginfo("Possible answers: %s", answers[i])
            # speechSay(question)
            rospy.loginfo("test ######### %s", answers[i])
            resp = recognize("fr_FR", [], 10)
            rospy.loginfo("Words found: %s", resp.transcript)
            
            
            if any(answer in resp.transcript for answer in answers[i]):
                #speechSay("Super c'est la bonne réponse")
                rospy.loginfo("Acceptable answer")
                score += 1
                interaction += 1

            elif any(question in resp.transcript for question in question.split()):
                print(question.split())
                print(any(question in resp.transcript for question in question.split()))
                #speechSay("Il ne faut pas répéter la question mais y répondre.")
                rospy.loginfo("The question has been repeated by the children")
                i = i-1
                interaction += 1

            elif not resp.transcript:
                #speechSay("Je n'ai pas bien entendu ta réponse.")
                rospy.loginfo("No answer detected")
                i = i-1
                
            else:
                #speechSay("Merci pour ta réponse.")
                rospy.loginfo("Answer is not right")
                interaction += 1

        rospy.loginfo("End of the exercise")
        #speechSay("Fin de l'exercice, merci pour tes réponses !")
        finalScore = str(int(score/interaction*100))
        rospy.loginfo("The answers are %s pourcent right !", finalScore)
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")
