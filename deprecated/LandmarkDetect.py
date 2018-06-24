from naoqi import ALProxy
import time


robotIP = "192.168.0.115"
PORT = 9559

motionProxy  = ALProxy("ALMotion", robotIP, PORT)


# Create a proxy to ALLandMarkDetection
markProxy = ALProxy("ALLandMarkDetection", robotIP, PORT)
# Subscribe to the ALLandMarkDetection extractor
period = 500
markProxy.subscribe("Test_Mark", period, 0.0 )

# Create a proxy to ALMemory.
memProxy = ALProxy("ALMemory", robotIP, PORT)
# Get data from landmark detection (assuming face detection has been activated).
data = memProxy.getData("LandmarkDetected")
print(MarkID)

for i in range(0, 10):
  time.sleep(0.5)
  val = memoryProxy.getData(memValue, 0)
  print ""
  print "\*****"
  print ""

# Check whether we got a valid output: a list with two fields.
if(val and isinstance(val, list) and len(val) == 2):
  # We detected faces !
  # For each face, we can read its shape info and ID.
  # First Field = TimeStamp.
  timeStamp = val[0]
  # Second Field = array of face_Info's.
  ShapeInfoArray = val[1]

  try:
  # Browse the faceInfoArray to get info on each detected face.
    for MarkInfo in MarkInfoArray:
    # First Field = Shape info.
    markShapeInfo = markInfo[0]
    # Second Field = Extra info (empty for now).
    markExtraInfo = markInfo[1]
    print "  alpha %.3f - beta %.3f" % (markShapeInfo[1], markShapeInfo[2])
    print "  width %.3f - height %.3f" % (markShapeInfo[3], markShapeInfo[4])
  except Exception, e:
    print "mark detected, but it seems getData is invalid. ALValue ="
    print val
    print "Error msg %s" % (str(e))
else:
  print "Error with getData. ALValue = %s" % (str(val))
  # Unsubscribe the module.

markProxy.unsubscribe("Test_Mark")
print "Test terminated successfully."  

#markProxy.unsubscribe("Test_Mark")