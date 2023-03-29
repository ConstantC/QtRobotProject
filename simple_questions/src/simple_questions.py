#!/usr/bin/env python
import sys
import csv
import rospy
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *



def simpleQuestions(questionsAnswers):

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
    
                  
    try:

        # call a ros service with text message
        rospy.loginfo("Explaining rules !")
        speechSay("Je vais maintenant te demander de répondre au questions que je vais te poser.")
        score = 0
        interaction = 0
        for i in range(len(questionsAnswers)):
            question = questionsAnswers[i][0]
            answers = []
            for j in range(1, len(questionsAnswers[i])):
                answers.append(questionsAnswers[i][j])

            rospy.loginfo("Reading question: %s", question)
            rospy.loginfo("Possible answers: %s", answers)
            speechSay(question)
            resp = recognize("fr_FR", [], 10)
            rospy.loginfo("Words found: %s", resp.transcript)
            
            
            if any(answer in resp.transcript for answer in answers):
                speechSay("Super c'est la bonne réponse")
                rospy.loginfo("Acceptable answer")
                score += 1
                interaction += 1

            elif any(question in resp.transcript for question in question.split()):
                print(question.split())
                print(any(question in resp.transcript for question in question.split()))
                speechSay("Il ne faut pas répéter la question mais y répondre.")
                rospy.loginfo("The question has been repeated by the children")
                i = i-1
                interaction += 1

            elif not resp.transcript:
                speechSay("Je n'ai pas bien entendu ta réponse.")
                rospy.loginfo("No answer detected")
                i = i-1
                
            else:
                speechSay("Merci pour ta réponse.")
                rospy.loginfo("Answer is not right")
                interaction += 1

        rospy.loginfo("End of the exercise")
        speechSay("Fin de l'exercice, merci pour tes réponses !")
        finalScore = str(int(score/interaction*100))
        rospy.loginfo("The answers are %s pourcent right !", finalScore)
    except KeyboardInterrupt:
        pass
    
    return finalScore

simpleQuestions([['De quelle couleur est la banane ?', 'jaune','vert'], ['De quelle couleur est la pomme ?', 'rouge','vert']])
rospy.loginfo("finsihed!")
