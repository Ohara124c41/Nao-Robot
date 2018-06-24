import sys

from naoqi import ALProxy


def main(robotIP = "192.168.0.117", PORT=9559):

    try:
        postureProxy = ALProxy("ALRobotPosture", "192.168.0.117", 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    postureProxy.goToPosture("StandInit", 1.0)

    print postureProxy.getPostureFamily()


if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python alrobotposture.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main("192.168.0.117")
    motionProxy.rest()