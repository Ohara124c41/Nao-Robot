import math
import time
import argparse
from naoqi import ALProxy

def main(robotIP = "192.168.0.117", PORT=9559):
    ''' Use case of breath API
    '''

    motionProxy  = ALProxy("ALMotion","192.168.0.117", 9559)
    postureProxy = ALProxy("ALRobotPosture", "192.168.0.117", 9559)

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Start breathing
    motionProxy.setBreathEnabled('Body', True)

    # Get default breath config
    print 'Current breath config: ' + str(motionProxy.getBreathConfig())

    # Wait for 10 seconds, and let the robot play the breathing animation
    time.sleep(10)

    # Change breathing configuration to a faster one
    print 'Breath faster'
    motionProxy.setBreathConfig([['Bpm', 30], ['Amplitude', 0.0]])

    # Let the robot play the new animation
    time.sleep(10)

    # Stop breathing
    motionProxy.setBreathEnabled('Body', False)

    # Go to rest position
    print 'rest'
    motionProxy.rest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)