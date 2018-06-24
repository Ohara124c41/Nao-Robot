from naoqi import ALProxy
import nao_nocv_2_0 as nao
import time
import math
import numpy 
import naoqi
import motion_planning as mp
import behaviour_based_navigation_empty.py as bbn



robotIP = "192.168.0.115"

nao.InitProxy(robotIP)
nao.InitLandMark()
nao.InitSonar()


global obstacle
obstacle=False

nao.Move(0.5,0.0,0.0,1.0)

def state(targetNum, step):
    Bel=1
    oldtimestamp = 0
    oldtime = time.time()
    pivotRate = 0
    dt=0

    spoken=False
    while not step:
        [SL, SR] = nao.ReadSonar()

        if SL == 0.0 and SR == 0.0:  
            SL = 999
            SR = 999

        SL, Bel = bbn.Kalman(SL, Bel)
        SR, Bel = bbn.Kalman(SR, Bel)

        velocity, obstacle, right, step = mp.sensor(SL, SR)

        if step == True:
            break


        m = 0
        while m < 5:  
            detected, timestamp, ID = nao.DetectLandMark()
            time.sleep(0.2)
            print "detected", detected
            if detected:
                break
            m = m + 1

        # compute the time interval

        dt = time.time() - oldtime
        print "dt ", dt

        if detected:
            if dt > 1:
                dt = 1

            if ID[0][0] == targetNum:
                oldtime = dt + oldtime
                
                if not spoken:
                    nao.Say("land mark detected")
                    time.sleep(0.5)
                    print "land mark detected", timestamp, ID
                    spoken = True

                if ID[0][3] == 0:
                    pass
                else:
                    w = 0.1
                    d = w / atan(ID[0][3])  
                    print "distance = ", d
                # approaching target    
                if d < 0.6: 

                    step = True
                    break

                pivotRate = dt * ID[0][1]

                if obstacle == True:
                    obavoid = mp.fobs([SL, SR])
                    time.sleep(0.5)
                    pivotRate = dt * obavoid
                    nao.Say("Avoiding obstacle")
                    print "Avoiding obstacle"
                    pivotRate = mp.turnDir(right, pivotRate)

                pivotRate = mp.pivRate(pivotRate)

                nao.Move(velocity, 0, pivotRate, 1)
                if dt < 0.5:
                    time.sleep(0.5 - dt)
            else:
                if not spoken:
                    nao.Say("Landmark" + str(ID[0][0]) + "detected." )
                    print "Incorrect Marker ID"




        else:
            if dt > 1.5:
                dt = 1
                oldtime = dt + oldtime

                if obstacle == False:

                    turnAngle, steps, avoidDirection, gotohelp = mp.search(targetNum)
                    time.sleep(0.5)

                    if gotohelp == True:
                        nao.Crouch()
                        time.sleep(20)
                    else:
                        j = 0
                        while j <= steps:
                            nao.Walk(0, 0, avoidDirection * 0.3, False)
                            time.sleep(0.5)
                            j = j + 1
                    mp.headReset()



                else:
                    obavoid = mp.fobs([SL, SR])
                    time.sleep(0.5)
                    pivotRate = dt * obavoid
                    nao.Say("Avoiding obstacle")
                    print "Avoiding obstacle"
                    time.sleep(0.5)

                    pivotRate = mp.turnDir(right, pivotRate)
                    pivotRate = mp.pivRate(pivotRate)

                    nao.Move(velocity, 0, pivotRate, 1)
                if dt < 0.5:
                    time.sleep(0.5 - dt)
            else:
                pass

    print pivotRate, timestamp, oldtimestamp



    return step

