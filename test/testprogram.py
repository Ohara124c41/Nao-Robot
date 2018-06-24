import nao_nocv_2_0 as nao

# here the program starts
nao.InitProxy("192.168.0.115")
nao.InitPose()
nao.RunMovement("sad2.py")
nao.Say("Ik ben heel heel heel erg verdrietig")
nao.sleep(5)
nao.Crouch()

nao.Say("Dit is de volgende test.")
nao.sleep(2)

nao.InitTrack()
nao.ALTrack(True)
nao.Say("Kijk me aan ...")
nao.sleep(10)
nao.ALTrack(False)
nao.EndTrack()
