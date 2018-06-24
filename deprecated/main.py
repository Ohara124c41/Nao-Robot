import argparse
import motion
import time
import almath
import random
import math
import sys
import os
import csv
import numpy as np
import naoqi

from naoqi import ALProxy

robotIP = "192.168.0.117"

def InitRobot():
    nao.InitProxy("192.168.0.117")




###########



def main(robotIP = "192.168.0.117", PORT=9559):

    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    tts = ALProxy ("ALTextToSpeech", robotIP, 9559)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Stand
    postureProxy.goToPosture("StandInit", 0.5)

    #####################
    ## Enable arms control by Motion algorithm
    #####################
    motionProxy.setMoveArmsEnabled(True, True)
    # motionProxy.setMoveArmsEnabled(False, False)

    #####################
    ## FOOT CONTACT PROTECTION
    #####################
    #motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", False]])
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

    #TARGET VELOCITY
    X = 0.5  
    Y = 0.0
    Theta = 0.0
    Frequency =0.0 # low speed
    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("There has been an earthquake. Don't panic!")
    except (Exception, errorMsg):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()

    time.sleep(3)
    #dialog 1
    X = 0.0  
    Y = 0.0
    Theta = 0.0
    Frequency =0.0 # low speed

    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("This is dialog one")
    except (Exception, errorMsg):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()

    #dialog 2






   # userArmsCartesian(motionProxy)

#def dialog1():
#	tts.say("This is dialog one.")
#	time.sleep(3.0)

#   	myWordList = ("Hello")
#   	try:
#		tts.say("Hello") 
#
#
#   		time.sleep(3.0)
#   		nao.Say("Bye")
#
#
#    except (Exception, errorMsg):
#        print (str(errorMsg))
#        print ("This example is not allowed on this robot.")
#        exit()
#
#   	time.sleep(4.0)

    ### TARGET VELOCITY
    X = 0.8
    Y = 0.0
    Theta = 0.1
    Frequency =1.0 # max speed
    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("Let's go this way!")

    except (Exception, errorMsg):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()

    time.sleep(4.0)

def dialog2():
	tts.say("This is dialog two.")
	time.sleep(3.0)

##   	myWordList = ("")
# #  	try:
#		tts.say("This is madness!") 
#
#
#   	time.sleep(3.0)
#   		nao.Say("Oh no")
#
#
#    except (Exception, errorMsg):
#        print (str(errorMsg))
#        print ("This example is not allowed on this robot.")
#        exit()
#
#   	time.sleep(4.0)    

    #TARGET VELOCITY
    X = 0.5
    Y = 0.2
    Theta = 0.1
    Frequency =0.5 # max speed
    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("Maybe over here?")

    except (Exception, errorMsg):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()

    time.sleep(4.0)

    X = 0
    Y = 0
    Theta = 0
    Frequency = 0

def dialog3():
	tts.say("This is dialog three.")
	time.sleep(3.0)    

        #TARGET VELOCITY
    X = 0.6
    Y = -0.3
    Theta = 0.1
    Frequency =0.5 # max speed
    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("Nope!")

    except (Exception, errorMsg):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()

    time.sleep(4.0)
    #TARGET VELOCITY
    X = 0.2
    Y = -0.5
    Theta = 0.2
    Frequency = 1.0

    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("Please send more time with me, so we can all improve.")

    except (Exception, errorMs):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()

    time.sleep(2.0)
#    userArmArticular(motionProxy)
    time.sleep(2.0)

    #####################
    ## End Walk
    #####################
    #TARGET VELOCITY
    X = 0.0
    Y = 0.0
    Theta = 0.0
    motionProxy.moveToward(X, Y, Theta)
    tts.say("Better luck next time.")



    motionProxy.waitUntilMoveIsFinished()

    # Go to rest position
    motionProxy.rest()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.0.117",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
