import naoqi
import nao_nocv_2_1 as nao
import time
import hello_movement

from naoqi import ALProxy
from naoqi import ALBroker



nao.InitProxy("192.168.0.117")
#broker = ALBroker("pythonBroker", "127.0.0.1", 9998, "192.168.0.117", 9559)
motion = ALProxy("ALMotion", "192.168.0.117", 9559)
#tts = ALProxy("ALTextToSpeech", "192.168.0.117", 9559)


nao.RunMovement("hello_movement.py",True,True)
time.sleep(4)

nao.Say("Hi! My name is Felix. There is an earthquake, do you need directions?")
asr = ALProxy("ALSpeechRecognition", "192.168.0.117", 9559)

asr.setLanguage("English")
vocabulary = ["Yes", "No"]

# Example: Adds "yes", "no" and "please" to the vocabulary
asr.pause(True)
asr.setVocabulary(vocabulary, False)

# Or, if you want to enable word spotting:
#asr.setVocabulary(vocabulary, True)
# Start the speech recognition engine with user Test_ASR
asr.subscribe("Test_ASR")
print 'Speech recognition engine started'

time.sleep(10)

done=False
while not done:
	result = nao.DetectSpeech()
	if len(result)>1:
		print result
		done = True

# def registerCallback(self, onRecogTxt):
# 	global callback
# 	callback = onRecogTxt
# def startRecog(self, vocabulary):
# 	global asr
# 	asr = ALProxy("ALSpeechRecognition")
# 	asr.setLanguage("English")
# 	vocabulary = ["Yes", "No"]
	if vocabulary[0] == "Yes":
 		nao.Say("Great! Follow me!")
	elif vocabulary[0] == "No":
 		nao.Say("Are you sure? I think you are in danger! Will you follow me?")

 		time.sleep(10)
 		done=False
 		while not done:
 			result = nao.DetectSpeech()
 			if len(result)>1:
 				print result
 				done = True

 			if vocabulary[0] == "Yes":
 				nao.Say("Great! Follow me!")
			elif vocabulary[0] == "No":
 				nao.Say("You are in danger! Follow me!")
 			else:
 				print "Nao does nothing"
 	
 	else:
 		print "Nao does nothing"
asr.unsubscribe("Test_ASR")

#nao.Say("Be careful!")
#time.sleep(3.0)
#nao.Say("We made it! You are safe now. Goodbye, and have a good day!")

motionProxy.waitUntilMoveIsFinished()

motionProxy.rest()


#		global Recog
#		Recog = RecogMod("Recog")

#asr.unsubscribe("Test_ASR")

#motion.setStiffnesses("Body", 1.0)
#motion.moveInit()
#motion.post.moveTo(0.5, 0, 0)
#tts.say("I'm walking")

#motion.moveInit()
#motion.moveTo(0.5, 0, 0)
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.0.117",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)