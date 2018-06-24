import nao_nocv_2_0 as nao
import time

IP="192.168.0.115" #marvin
#IP="192.168.0.118" #bender

# connect to and initialize robot
nao.InitProxy(IP)
nao.InitPose()
nao.InitLandMark()
nao.InitSonar()

dt=0.1 # this value is probably wrong ...
count =0
maxcount=2000 # make sure the loop ends
oldtimestamp=0
oldtime=0
dtheta=0
while count < maxcount:
    count=count+1

    #### Sonar ####
    [SL, SR]=nao.ReadSonar()
    #### put code to deal with sonar data here
    
    # stop if too close to obstacle
    if SL<0.5 and SR<0.5:
        print SL,SR
        #do something here
        break
    elif SL<0.2 or SR<0.2:
        print SL,SR
        #do something here
        break

    #### detect landmark ####
    detected, timestamp, markerinfo =nao.DetectLandMark()
    if detected:
        # each marker has a number, you can use this to identify it
        #if markerinfo[0][0]==80:
        print SL, SR, markerinfo[0][1]
        
#compute the turnrate
        dtheta=dt*markerinfo[0][1] # replace this with decent formula!

# limit the turnrate
        if dtheta>0.2:
            dtheta=0.2
        elif dtheta<-0.2:
            dtheta=-0.2
# move the robot
        nao.Move(1.0,0,dtheta,1) # head to the target
    else:
        # stop the robot if you dont see a landmark after some time!
        pass
    
    time.sleep(0.1) # you should not send too many commands to the robot, so wait 0.1s
    print dtheta, dt, timestamp, oldtimestamp
            
nao.Move(0.0,0,0,0)
nao.Crouch()

