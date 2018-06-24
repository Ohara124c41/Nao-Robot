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

import behaviour_based_navigation_empty.py as bn
import nao_nocv_2_0.py as nao
import navigation as nav
import kalman_filter as kf
import forehead

from naoqi import ALProxy

robotIP = "192.168.0.117"

def InitRobot():
    nao.InitProxy("192.168.0.117")




###########
# Voice References

# Say the sentence 50% slower than normal speed
# tts.say("\\rspd=50\\hello my friends")

# Insert a pause of 1s
# tts.say("Hello my friends \\pau=1000\\ how are you ?")

# Say the sentence with a volume of 50%
# tts.say("\\vol=50\\Hello my friends")

# Say the sentence with a weak phrase boundary (no silence in speech)
#tts.say(“\\bound=W\\ Hello my friends”)
# Say the sentence with a strong phrase boundary (silence in speech)
#tts.say(“\\bound=S\\ Hello my friends”)

# Nuance only - Emphasis
# tts.say(“Hello my \\emph=0\\ friends”) # reduced
# tts.say(“Hello my \\emph=1\\ friends”) # stressed
# tts.say(“Hello my \\emph=2\\ friends”) # accented

# Nuance only - Break between sentences
# tts.say(“Hello my friends.\\eos=0\\How are you ?”) # no break
# tts.say(“Hello my friends.\\eos=1\\How are you ?”) # break


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
    X = 0.2  
    Y = 0.0
    Theta = 0.0
    Frequency =0.0 # low speed
    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("There has been an \\emph=2\\ earthquake. \\eos=1\\Don't panic!")
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
    time.sleep(5)



    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("This is dialog one")
        time.sleep(3)
        tts.say("I like pineapple")
        time.sleep(3)
        tts.say("Something is missing...")
    except (Exception, errorMsg):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()
    X = 0.4  
    Y = 0.1
    Theta = 0.2
    Frequency =0.2 # low speed
    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("Let's go this way")
    except (Exception, errorMsg):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()

    time.sleep(3)
    #dialog 2
    X = 0.0  
    Y = 0.0
    Theta = 0.0
    Frequency =0.0 # low speed
    time.sleep(5)


    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("This is dialog two")
        time.sleep(3)
        tts.say("I like pizza")
        time.sleep(3)
        tts.say("Russian girls are mean!")
    except (Exception, errorMsg):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()

        #userArmsCartesian(motionProxy)
    X = 0.3
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
    time.sleep(5)

    try:
        motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        tts.say("This is another dialog")

    except (Exception, errorMsg):
        print (str(errorMsg))
        print ("This example is not allowed on this robot.")
        exit()

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
   #userArmArticular(motionProxy)
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
    time.sleep(4)
    forehead_wipe_motion



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
