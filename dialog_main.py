import naoqi
from naoqi import ALProxy
import nao_nocv_2_0 as nao
import time
import math

#asr.unsubscribe("MyModule") # sometimes needed when robot crashed unexpectedly
robotIP='192.168.0.115'
nao.InitProxy(robotIP)
nao.InitPose()
asr = ALProxy("ALSpeechRecognition", IP, 9559)

def introduction()

    motion.basicWave()
    nao.Say("Hello, my name is Felix")
    time.sleep(2)
    nao.Say("There appears to be an emergency")
    time.sleep(2)
    nao.Say("It's my task to help people find the emergency exit")
    nao.EyeLED([60,60,60])



### Scenario 1 - Offer to guide the person to the exit
def offerHelp():

    time.sleep(1)
    myWordList = ["Yes","No"]
    myLanguage="English"
    nao.InitSpeech(myWordList, myLanguage)

    nao.Say("Do you need any help?")

    for i in range (0,5):
        time.sleep(3)
        asr.subscribe("MyModule")
        time.sleep(2)
        #print ("1:"+ str(result))
        result = nao.DetectSpeech()
        #print("2:" + str(result))
        time.sleep(2)
     
        if len(result) > 0:
            if result[0] == myWordList[0]:
                print "myWordList", myWordList[0]
                asr.unsubscribe("MyModule")
                nao.Say("Good, just follow me")
                time.sleep(1)
                nao.Say("You can trust me")
                nao.EyeLED([90,90,90])
                break
            elif result[0] == myWordList[1]:
                print "myWordList", myWordList[1]
                asr.unsubscribe("MyModule")
                nao.Say("Are you sure? You're not safe here!")
                time.sleep(1)
                nao.Say("Do you want me to help")
                
        else:
            nao.Say("I'm sorry, I didn't understand you")
            time.sleep(1)
            nao.Say("Yes or no?")
            time.sleep(2)
            print "Error: Response not in wordlist."

    time.sleep(2)
    #nao.Say("Let's Go")


### Scenario 2 - Confirmation of person in tow
def findHuman():

    time.sleep(1)
    myWordList = ["Yes"]
    myLanguage="English"
    nao.InitSpeech(myWordList, myLanguage)

    nao.Say("Are you still with me??")

    for i in range (0,5):
        time.sleep(3)
        asr.subscribe("MyModule")
        time.sleep(2)
        #print ("1:"+ str(result))
        result = nao.DetectSpeech()
        #print("2:" + str(result))
        time.sleep(2)
     
        if len(result) > 0:
            if result[0] == myWordList[0]:
                print "myWordList", myWordList[0]
                asr.unsubscribe("MyModule")
                nao.Say("Good, just follow me!")
                time.sleep(2)
                nao.Say("Just a little while longer, don't worry.")
                break
            elif result[0] !== myWordList[0]:
                print "No response detected"
                nao.Say("Hello, are you there?")
                time.sleep(3)
                nao.Say("Please respond with yes.")
                nao.EyeLED([60,90,60])
                
        else:
            nao.Say("I'm sorry, I didn't understand you.")
            time.sleep(2)
            nao.Say("Was that a yes?")
            time.sleep(2)
            print "Error: Response not in wordlist."

    time.sleep(2)

### Scenario 3 - Busy talk, to keep person distracted from imminent danger
def threeLaws():

    time.sleep(1)
    myWordList = ["Yes","No"]
    myLanguage="English"
    nao.InitSpeech(myWordList, myLanguage)

    nao.Say("Do you know the three laws of robotics?")

    for i in range (0,5):
        time.sleep(3)
        asr.subscribe("MyModule")
        time.sleep(2)
        #print ("1:"+ str(result))
        result = nao.DetectSpeech()
        #print("2:" + str(result))
        time.sleep(2)
     
        if len(result) > 0:
            if result[0] == myWordList[0]:
                print "myWordList", myWordList[0]
                asr.unsubscribe("MyModule")
                nao.Say("Wonderful!")
                time.sleep(2)
                nao.Say("What do you think about them?")
                break
            elif result[0] == myWordList[1]:
                print "myWordList", myWordList[1]
                asr.unsubscribe("MyModule")
                nao.Say("Lucky for you, I know them by heart!")
                time.sleep(2)
                nao.Say("First, I cannot let any human be in harms way.")
                time.sleep(2)
                nao.Say("Second, I must obey human orders.")
                time.sleep(2)
                nao.Say("Third, I cannot self terminate.")
                # Terminator 2 reference ;)
                
        else:
            nao.Say("I'm sorry, I didn't understand you...")
            sleep.time(2)
            nao.Say("Yes or no?")
            time.sleep(2)
            print "Error: Response not in wordlist."

    time.sleep(2)

def goodBye():
    nao.Say("We have arrived at the exit, you're safe now.")
    time.sleep(2)
    nao.Say("Your family will be notified of your safety.")
    time.sleep(2)
    nao.Say("Have a nice day, goodbye!")
    motion.basicWave()
    print "Sequence complete."
