import nao_nocv_2_0 as nao

def InitRobot():
    nao.InitProxy("192.168.0.115")
    
def behaviourA():
    #state A
    nao.InitPose()
    nao.RunMovement("sad2.py")
    nao.Say("Ik ben heel heel heel erg verdrietig")
    nao.sleep(5)
    nao.Crouch()

def behaviourB(): 
    #state B

    nao.InitTrack()
    nao.ALTrack(True)
    nao.Say("Kijk me aan ...")
    nao.sleep(10)
    nao.ALTrack(False)
    nao.EndTrack()

    
if __name__=="__main__":
    InitRobot()
    behaviourA()
    behaviourB()
