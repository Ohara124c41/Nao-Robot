import sys
import argparse
import math
import almath as m # python's wrapping of almath
import time
#import numpy as np


from naoqi import ALProxy
degree = math.pi/180.0 # radians per degree
pi = math.pi 
exp = math.exp
atan2 = math.atan2



def FTarget(target_distance, target_angle):
    # Attractor Behaviors
    
    #fr1 = -sin(psi-Psi1)
    #fr3 = -exp(-dt)*sin(psi-Psi3)
    #frf = fr1+fr3




    Ftar=0
    return Ftar

def FObstacle(obs_distance, obs_angle):
 #   too_far=2.5 #m

    # Repulsive Behaviors
    #fr2 = exp(((-psi-Psi2)^2)/(2(sb^2)))*(psi-Psi2)*exp(-do/b2)
    #do something useful here

  #  if obs_distance < too_far:
        #do something useful here
        #X = 0.2
        #Y = 0.0
        #Theta = 0.0
        #Frequency =0.5 # low speed
        #try:
        #    motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
        #    tts.say("There has been an \\emph=2\\ earthquake. \\eos=1\\Don't panic!")
        #except (Exception, errorMsg):
        
        #print (str(errorMsg))
        #print ("This example is not allowed on this robot.")
        #exit()


   # time.sleep(3)

    #    Fobs=0 # needs replacing !
    #else:
   #     Fobs=0
   # return Fobs


    b1 = 6
    #b1 = 8
    b2 = 18
    #b2 = 15
    
    ##
    z = pi/6
    
    sig0 = dist[0]* atan2(30*pi/180)
    sig1 = dist[1]* atan2(30*pi/180)
    
    lamb1 = b1*exp(-(dist[0]-r0)/b2)
    lamb2 = b1*exp(-(dist[1]-r0)/b2)
    
    fr1 = -(lamb1*z*exp(-(z**2)/(sig0)**2))
    fr2 = -(lamb2*z*exp(-(z**2)/(sig0)**2))
    
    Fobs = fr1 + fr2
    return = Fobs



#def FStochastic():
    #do something useful here
 #   Fstoch=0
  #  return Fstoch

#def FOrienting():
    #do something useful here
 #   Forient=0
  #  return Forient

Q = .00001
S = 999 # sensor at t=0
R = 0.1**2 # var(sensor)

def Kalman(S, Bel):
    
    # from MATLAB code
    # s.x = s.A*s.x + s.B*s.u;
    # s.P = s.A * s.P * s.A' + s.Q;
    # % Compute Kalman gain factor:
    # K = s.P*s.H'*inv(s.H*s.P*s.H'+s.R);
    # % Correction based on observation:
    # s.x = s.x + K*(s.z-s.H*s.x);
    # s.P = s.P - K*s.H*s.P;


    x_prev = S
    x = S
    p_prev = Bel
    p = Bel + Q

    K = p_prev/(p_prev + R)
    
    x = x_prev + K * x_prev
    p = (1 - K) * p_prev

    S = x
    Bel = p

    return S, Bel

def compute_velocity(sonar_distance_left, sonar_distance_right):
    max_velocity = 1.0
    max_distance = 0.5 #m
    min_distance = 0.2 #m

    if sonar_distance_left>max_distance and sonar_distance_right > max_distance:
        velocity = max_velocity
        obs=False
        stop=False
        print "Nominal movements"

    elif sonar_distance_left<min_distance or sonar_distance_right < min_distance:
        velocity = 0.0
        obs=True
        stop=True
        print "Imminent obstacle detected"
    elif sonar_distance_left<sonar_distance_right:
        velocity = max_velocity*sonar_distance_left/max_distance

    else:
        velocity = max_velocity*sonar_distance_right/max_distance
        
    return velocity

def compute_turnrate(target_dist, target_angle, sonar_distance_left, sonar_distance_right):
    max_turnrate = 0.349 #rad/s

    delta_t = 0.01 # may need adjustment!
    sonar_angle_left = 30 * degree
    sonar_angle_right = -30 * degree
    
    Fobs_left = FObstacle(sonar_distance_left, sonar_angle_left)
    Fobs_right = FObstacle(sonar_distance_right, sonar_angle_right)

    FTotal = FTarget(target_dist, target_angle) + Fobs_left + Fobs_right + FOrienting() + FStochastic()
             
    # turnrate: d phi(t) / dt = sum( forces ) 
    turnrate =  Ftotal*delta_t
    
    #normalise turnrate value
    if turnrate>max_turnrate:
        turnrate=1.0
    else:
        turnrate=turnrate/max_turnrate

    return turnrate

def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


def main(robotIP):
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
    except Exception, e:
        print ("Could not create proxy to ALMotion")
        print ("Error was: ")


    # Set NAO in stiffness On
    StiffnessOn(motionProxy)

    #####################
    ## Enable arms control by move algorithm
    #####################
    motionProxy.setWalkArmsEnabled(True, True)
    #~ motionProxy.setWalkArmsEnabled(False, False)

    #####################
    ## FOOT CONTACT PROTECTION
    #####################
    #~ motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

    #####################
    ## get robot position before move
    #####################
    initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

    X = 0.3
    Y = 0.0
    Theta = math.pi/2.0
    motionProxy.post.moveTo(X, Y, Theta)
    # wait is useful because with post moveTo is not blocking function
    motionProxy.waitUntilMoveIsFinished()

    #####################
    ## get robot position after move
    #####################
    endRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

    #####################
    ## compute and print the robot motion
    #####################
    robotMove = m.pose2DInverse(initRobotPosition)*endRobotPosition
    print ("Robot Move :")


if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print ("Usage python motion_moveTo.py robotIP (optional default: 127.0.0.1)")
    else:
        robotIp = sys.argv[1]

    main(robotIp)

