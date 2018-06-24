import nao_nocv_2_0 as nao
import behaviour_based_navigation_empty as bn
import math

# import efk.py as ekf


#import numpy as np
#from enum import Enum

'''
class States(Enum):
    GREETING = 0
    FACE_DETECT = 1
    WALKING = 2
    OBS_AVOID = 3
    DIALOG = 4
    LANDMARK = 5
    GOAL = 6
'''
robotIP = "192.168.0.117"

def DoInitState():
    robotIP ="192.168.0.117"
    nao.InitProxy(robotIP)
    nao.InitSonar()
    nao.InitLandMark()
    nao.InitTrack()
    nao.InitPose()

    return "navigate"

def compute_markerpos(markerinfo):
    marker_real_height = 0.55 #init = 0.1 #m #dangerous assumption?
    marker_detected_height_angle = markerinfo[0][4]
    
    marker_distance=marker_real_height/math.tan(marker_detected_height_angle) 
    marker_angle=markerinfo[0][1]

    return marker_distance, marker_angle

def compute_facepos(face_info):

    # face_x is the yaw angle with respect to the head orientation in radians
    # use nao.GetYaw() to convert this in a body orientation
    # distance can be estimated from detected head size, currently nao.DetectFace does not support this.

    face_distance=0
    face_angle=0

    return face_distance, face_angle

    

def DoNavigateState(current_state):

    done = False
    marker_distance = 3 #init = 5 #m
    marker_angle = 0 # radians
    face_distance = 0
    face_angle=0
    min_distance = 0.2

    #the navigate state contains its own loop.
    # Would be better to define substates and put the whole thing in separate file (see main loop)
    # but here its just the simple while not done loop, with some if statements
    while not done:
        #sonar
        [sl,sr]=nao.ReadSonar()

        #landmark detect
        landmark_detected, timestamp, markerinfo=nao.DetectLandmark()
        if landmark_detected:
            [marker_distance, marker_angle] = compute_markerpos(markerinfo)
            print('marker_distance')
            nao.say("Landmark has been detected!")
            if marker_distance < min_distance: # the target is reached
                nao.say('We found the exit!')
                current_state = 'exit'
                done=True
                
        else:
            # if no marker detected, do something else
            pass

        #facedetect
        face_detected, timestamp, [face_x, face_y] = nao.DetectFace()
        if face_detected:
            [face_distance, face_angle] = compute_facepos(face_x, face_y)
            nao.say('I see you!')
            #face detected, now what?
            pass
        else:
            # if no face detected, do something else
            pass
        
        #compute turnrate and velocity
        velocity=bn.compute_velocity(sl, sr) #could be anything, but here: slow down for obstacles
        turn_rate=bn.compute_turnrate(marker_distance, marker_angle, sl, sr,) #turn towards target and the landmark is the target 

        # finally, update the robot's speed and turnrate
        nao.Move(velocity,0,turn_rate)
        nao.sleep(0.05) # maximum update speed of the robot is 20 Hz or so.
        
    nao.Move(0.0, 0.0, 0.0) # make sure the robot is not moving when exiting this state!
    return current_state

def DoExitState():
    nao.Crouch()
    return "done"

def main():
    state="init"
    while not state=="done":
        if state=="init":
            state=DoInitState()
        elif state=="navigate":
            state=DoNavigateState(state)
        elif state=="exit":
            state=DoExitState()
            

if __name__=="__main__":
    main()
