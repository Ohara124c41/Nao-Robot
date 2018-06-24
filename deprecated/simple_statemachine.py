# import statements
import test_lib as mylib

#globals (preferably not)

#function definitions
def DoStateInit():
    """Do initialiazation"""
    
    mylib.InitRobot()
    
    return "My state A"
    
def DoStateA(mystate):
    """Execute state A"""
    
    if mystate=="My state A":
        mylib.behaviourA()
        pass
    elif mystate=="My state B":
        pass
    return "My state B"

def DoStateB():
    """Execute state B"""

    mylib.behaviourB()
    return "Done"

#main
def main():
    state="Init"
    while not state=="Done":
        if state=="My state A":
            state=DoStateA(state)
        elif state=="My state B":
            state=DoStateB()
        elif state=="Init":
            state=DoStateInit()

if __name__=="__main__":
    main()
    

