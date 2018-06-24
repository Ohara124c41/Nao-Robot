import nao_2_0 as nao

nao.InitProxy("127.0.0.1",[0])
nao.InitPose()
nao.InitVideo(2)
nao.Move(1,0,0)
while True:
    im=nao.GetImage()
    nao.cv.ShowImage("frame",im)
    if nao.cv.WaitKey(1)==ord('q'):
        break
nao.Crouch()
nao.sleep(1)
nao.cv.DestroyAllWindows()
