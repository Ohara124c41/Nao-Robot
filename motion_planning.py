from naoqi import ALProxy
import nao_nocv_2_0 as nao
import time
import math
import numpy 
import naoqi


def sensor(SL,SR):
    corr=True
    if SL > 0.4 and SR > 0.4:

        if SL>0.8 and SR >0.8:
            velocity = 1.0 
            obstacle=False
            stop=False
            print "Nominal operations",SL, SR 
                     
        else:
            velocity=0.5
            obstacle=False
            stop=False
            print "Distant obsticle dectected"        
            
    else:

        velocity = 0.5   
        obstacle=True
        stop=False
        print "Imminent obsticle detected",SL, SR, obstacle
        
        print SL, SR        

        if SL<SR :
            corr=True
        else:
            corr=False

    if SL<0.3 and SR <0.3:
        print "too close and stop", SL, SR
        velocity = 0
        obstacle=True
        stop=True 
                
    return velocity,stop,obstacle,corr  

def search(landmark):
    
    turnAngle=0
    direction=[0.3,0.4,-0.4,-0.3,-0.3,-0.4,0.4,0.3]
    n=0
    psi=0

    find,ID=findTar(landmark)
    nao.Move(0, 0.0, 0, 1)   
    while k<100 and not find:
              
        for i in direction:
            nao.MoveHead(i,0,False,True,timeLists=[[1], [1]])
            time.sleep(0.3)
            find,ID=findTar(landmark)
            print "Finding Landmark"
            time.sleep(0.3)
            if find:                
                break

        if find:
            
            break
        else:
            print"Searching for landmark"
            nao.Move(0.2, 0.0, -0.5, 1)
            nao.Move(0.2, 0.0, -0.5, 1)
            nao.Move(0.2, 0.0, -0.5, 1)
            nao.Move(0.2, 0.0, -0.5, 1)
            nao.sleep(0.3)
            
            nao.Move(0, 0.0, 0, 1)
            print"Finding landmark"
            find,ID=findTar(landmark)
            time.sleep(0.3)
            if find:
                break
        k=k+1
        if k==10:
        	reset=True
        	time.sleep(0.5)

        	break
    
    psi=nao.GetYaw()
    turnAngle=psi[0]+ID[0][1]
    steps=round(turnAngle/0.3,0)
    d=0

    if turnAngle >0:
        d=1
    if turnAngle<0:
        d=-1
    return turnAngle,reset, d, steps 



def findTar(landmark):

    print"Landmark detected"
    detected, timestamp, ID = nao.DetectLandMark()   
    if detected==True: 
        if ID[0][0]==landmark:
            return True,ID
        else:
            return False,ID
    else:
        return False,ID




def pivRate(pivotRate):
	if pivotRate > 0.2:
		pivotRate = 0.2
		nao.Say("Pivot right")
		time.sleep(0.5)
	if pivotRate < -0.2:
		pivotRate = -0.2
		nao.Say("Pivot left")
		time.sleep(0.5)
	return pivotRate

def turnDir(corr,pivotRate):
	
	if corr==True: 
		
		if pivotRate>0:
			pivotRate=pivotRate*(-1)
	else:
		           
		if pivotRate<0:
			pivotRate=pivotRate*(-1)
	return pivotRate


	
	
def headReset(headRes):
	headRes=0.5 #duration in s
	nao.MoveHead(0,0,timeLists=[[headRes],[headRes]])
	time.sleep(2)
	return headRes



