#! /usr/bin/env python

import json
import time, sys
import random
import qi
import argparse
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from mummer_dialogue.ShopList import ShopList
from rivescript import RiveScript
from mummer_dialogue.State import state
import rospy
from nao_interaction_msgs.msg import PersonDetectedArray
from nao_interaction_msgs.srv import Say, SayRequest
import rospkg
from actionlib import SimpleActionServer, SimpleActionClient
from mummer_dialogue.msg import dialogueAction
from threading import Thread
from pepper_goal_server.msg import GoalServerAction, GoalServerGoal
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue
from pepper_goal_server.msg import RouteDescriptionGoalServerAction, RouteDescriptionGoalServerGoal
import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)

goal = None
some_state = None
memory = None
chatbot = None
nextAction = None
userStoppedTalking = False
actionName = None
shopList = None
ALDialog = None
ALMemory = None
#ALTracker = None
ALSpeechRecognition = None
pplperc = None

rosSubscriber = None
rosMessagetimeStamp = None

stateThread = None

# State attributes
tskC = False
tskF = False
prevA = 0
dist = 1
ctx = ''
turn = False  # 0 = agent, 1 = user
usrEng = True
mode = False  # 0 = task, 1 = chat
timeout = False
usrTerm = False
bye = False
usrEngC = False
lowConf = False

foundperson = False

lastUsrInput = ''

# Action dictionary
actionID = {"taskConsume": 1, "Greet": 2, "Goodbye": 3, "Chat": 4, "giveDirections": 5, "wait": 6, "confirm": 7,
            "requestTask": 8}


#### Action selection function
def action(state):
    bestAction = []
    s = json.dumps(state.__dict__, sort_keys=True)
    print s
    with open(path + '/etc/jsonDump.json') as data_file:
        data = json.load(data_file)

    for v in data.values():
        if json.dumps(v['s']['s'], sort_keys=True) == s:
            maxQ = v['qEntry'][0]['q']  # set the first q as max
            for action in v['qEntry']:
                if action['q'] == maxQ:
                    bestAction.append(action['a'])
                if action['q'] > maxQ:
                    maxQ = action['q']
                    bestAction = [action['a']]
    print "actions", len(bestAction)
    try:
        return random.choice(bestAction)['name']
    except IndexError:
        pass
        # return 'Chat'


class SpeechEventModule(ALModule):
    """ A simple module able to react to facedetection events """

    def __init__(self, name):
        ALModule.__init__(self, name)

    def onSpeechDetected(self, *_args):
        global memory
        memory.unsubscribeToEvent("Dialog/LastInput", "SpeechEvent")
        ALDialog.unsubscribe('my_dialog_example')

        # flipTurn()

        global userStoppedTalking
        userStoppedTalking = True

        # If no task was given or the user did not say goodbye, assume the user is chatting
        # if not str2bool(ALMemory.getData('tskFilled')):
        #     ALMemory.insertData('usrEngChat', 'True')


################ ROS ################
def FaceDetected(data):
    for p in data.person_array:
        if True: #p.id == int(goal.userID):
#            rospy.loginfo("Found person '%s'" % str(p.id))
            global dist
            global rosMessagetimeStamp
            rosMessagetimeStamp = data.header.stamp.to_sec()
            # print rosMessagetimeStamp
            dist = data.person_array[0].person.distance
            dist = 2 if dist >= 2.5 else 1
            # memory.unsubscribeToEvent("PeoplePerception/PeopleDetected", "HumanGreeter")
        
            global foundperson
            if not foundperson:
                foundperson = True
        
        #        ALTracker.registerTarget("People", data.person_array[0].id)
        #        ALTracker.track("People")
                global memory
                global ALDialog
                ALDialog.subscribe('my_dialog_example')
        
                # observeState()
                global stateThread
                stateThread = Thread(target=observeState, args=())
                stateThread.start()
            break
        else:
            rospy.logwarn("Unknown person '%s'" % str(p.id))


def __call_service(srv_name, srv_type, req):
    while not rospy.is_shutdown():
        try:
            s = rospy.ServiceProxy(
                srv_name,
                srv_type
            )
            s.wait_for_service(timeout=1.)
        except rospy.ROSException:
            rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % srv_name)
            rospy.sleep(1.)
        else:
            return s(req)


def say(text):
    __call_service("/naoqi_driver/animated_speech/say", Say, SayRequest(text=text))
    

def clean_up():
    rospy.loginfo("End of interaction ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    client = SimpleActionClient("/goal_server/disengage", GoalServerAction)
    client.wait_for_server()
    client.send_goal(GoalServerGoal())

#    ALTracker.unregisterAllTargets()
#    ALTracker.stopTracker()

    req = KnowledgeUpdateServiceArrayRequest()
    req.update_type = req.ADD_KNOWLEDGE
    k = KnowledgeItem()
    k.knowledge_type = KnowledgeItem.FACT
    k.attribute_name = "engaged"
    k.values = [
        KeyValue(key="i", value="iid_0"),
        KeyValue(key="t", value="hello")
    ]
    req.knowledge.append(k)
    rosSubscriber.unregister()
#    __call_service(
#        __update_srv_name,
#        KnowledgeUpdateServiceArray,
#        req
#    )
    actionServer.set_succeeded()


#####################################

# Read memory variables (state attributes)
def readMemoryState(ALMemory):
    global tskC
    global tskF
    global prevA
    global dist
    global ctx
    global turn
    global usrEng
    global mode
    global timeout
    global usrTerm
    global bye
    global usrEngC
    global lowConf
    global lastUsrInput
    # global distance

    tskC = str2bool(ALMemory.getData("tskCompleted"))
    tskF = str2bool(ALMemory.getData("tskFilled"))
    prevA = ALMemory.getData("prevAct")  # int
    #    dist = getDistance()
    ctx = ALMemory.getData("ctxTask")  # string
    turn = str2bool(ALMemory.getData("turntaking"))
    usrEng = str2bool(ALMemory.getData("usrEngaged"))
    mode = str2bool(ALMemory.getData("mode"))
    timeout = str2bool(ALMemory.getData("timeout"))
    usrTerm = str2bool(ALMemory.getData("usrTermination"))
    bye = str2bool(ALMemory.getData("bye"))
    usrEngC = str2bool(ALMemory.getData("usrEngChat"))
    lowConf = str2bool(ALMemory.getData("lowConf"))
    lastUsrInput = ALMemory.getData("Dialog/LastInput")

    # print "memory", tskC, tskF, prevA, dist, ctx, usrEng, mode, timeout, usrTerm, bye, usrEngC, lowConf, turn, lastUsrInput


# Hardcoded fallback state in case of error (right after the robot greeted the user)
def pushFallbackStateToMemory():
    global ALMemory
    ALMemory.insertData("ctxTask", "")
    ALMemory.insertData("shopName", "")
    ALMemory.insertData("tskCompleted", "False")
    ALMemory.insertData("prevAct", 2)
    ALMemory.insertData("distance", 1)
    ALMemory.insertData("turntaking", "True")
    ALMemory.insertData("usrEngaged", "True")
    ALMemory.insertData("mode", "False")
    ALMemory.insertData("timeout", "False")
    ALMemory.insertData("usrTermination", "False")
    ALMemory.insertData("bye", "False")
    ALMemory.insertData("usrEngChat", "False")
    ALMemory.insertData("lowConf", "False")
    ALMemory.insertData("tskFilled", "False")


def instantiateMemory():
    global ALMemory
    ALMemory.insertData("ctxTask", "")
    ALMemory.insertData("shopName", "")
    ALMemory.insertData("tskCompleted", "False")
    ALMemory.insertData("prevAct", 0)
    # ALMemory.insertData("distance", 1)
    ALMemory.insertData("turntaking", "False")
    ALMemory.insertData("usrEngaged", "True")
    ALMemory.insertData("mode", "False")
    ALMemory.insertData("timeout", "False")
    ALMemory.insertData("usrTermination", "False")
    ALMemory.insertData("bye", "False")
    ALMemory.insertData("usrEngChat", "False")
    ALMemory.insertData("lowConf", "False")
    ALMemory.insertData("tskFilled", "False")
    # ALMemory.insertData("Dialog/LastInput","")

    readMemoryState(ALMemory)


def initChatbot(path):
    # Initialize chatbot
    global chatbot
    chatbot = RiveScript()
    chatbot.load_directory(path)
    chatbot.sort_replies()


#    print chatbot.reply("localuser", "Hello")

def observeState():
    global some_state

    print ALMemory.getData("Dialog/LastInput")
    some_state = generateState()
    some_state.printState()

    global nextAction
    nextAction = action(some_state)
    print "Action selected: ", nextAction

    decodeAction(nextAction)


def decodeAction(nextAction):
    global ALMemory    
    
    try:
        if nextAction == "Greet":
            greet()
        elif nextAction == "Chat":
            chat(lastUsrInput)
        elif nextAction == "wait":
            wait()
        elif nextAction == "taskConsume":
            taskConsume()
        elif nextAction == "giveDirections":
            giveDirections()
        elif nextAction == "Goodbye":
            goodbye()
            return
        elif nextAction == "confirm":
            confirm()
        elif nextAction == "requestTask":
            requestTask()

        if not turn:
            if nextAction == "Chat":
                ALMemory.insertData("mode", "True")
            else:
                ALMemory.insertData("mode", "False")

        # Write action taken to state
        if actionID[nextAction] != 6:
            ALMemory.insertData("prevAct", actionID[nextAction])
    except KeyError:
        print "Fallback to default action"

        if rosMessagetimeStamp + 1. < rospy.Time.now().to_sec():
            goodbye()
            return

        chat('fallback utterance')
        if not turn:
            print "falling back to backup user state:"
            pushFallbackStateToMemory()
            print '\n'
            observeState()

    print '\n'
    flipTurn()
    observeState()


def generateState():
    global some_state
    readMemoryState(ALMemory)
    some_state = state(tskC, tskF, prevA, dist, ctx, usrEng, mode, timeout, usrTerm, bye, usrEngC, lowConf, turn)
    return some_state


def flipTurn():
    #    if nextAction != None:
    global turn
    turn = not turn
    ALMemory.insertData("turntaking", bool2str(turn))


def chat(sentence):
    # print sentence
    global chatbot
    try:
        say(str(chatbot.reply("localuser", str(sentence))))
    except RuntimeError:
        print "error in chatbot"


def greet():
    say("Hi")


def goodbye():
    say("Have a nice day")
    clean_up()


def confirm():
    if lastUsrInput is not None:
        say("Sorry, did you say " + lastUsrInput + "?")
    else:
        say("Sorry, can you repeat that please?")


def giveDirections():
    say("Let me see.")
    client = SimpleActionClient("/route_description_goal_server", RouteDescriptionGoalServerAction)
    client.wait_for_server()
    client.send_goal_and_wait(RouteDescriptionGoalServerGoal(shop_id=shopList.getId(ALMemory.getData("shopName"))))
    print "########################################"
    print "Finished description"
    
#    global shopList
#    print "shopName: ", ALMemory.getData("shopName")
#    say(shopList.getDirections(ALMemory.getData("shopName")))

    ALMemory.insertData("ctxTask", "")
    ALMemory.insertData("tskFilled", "False")
    ALMemory.insertData("tskCompleted", "True")


def taskConsume():
    global ALMemory
    say("Let me see. There are " + str(len(shopList.filteredCategory(ALMemory.getData("ctxTask")))) +
        " " + ALMemory.getData("ctxTask") + " shops nearby")

    # print shopList.filteredCategory(ALMemory.getData("ctxTask")).enumShops()
    say("These are " + shopList.filteredCategory(ALMemory.getData("ctxTask")).enumShops() + ".")

    ALMemory.insertData("ctxTask", "")
    ALMemory.insertData("tskFilled", "False")
    ALMemory.insertData("tskCompleted", "True")


def requestTask():
    say("Is there anything I can help you with?")


def wait():
    global memory
    global userStoppedTalking
    global timeout
    tStart = time.time()  # Start time counter for timeout

    # Assume that the user is chatting unless recognized otherwise
    ALMemory.insertData('usrEngChat', 'True')

    ALMemory.insertData("tskCompleted", "False")
    ALMemory.insertData("timeout", "False")
    # memory.subscribeToEvent("EngagementZones/PersonMovedAway", "EngagementZone", "onMoveAway")
    # memory.subscribeToEvent("EngagementZones/PersonApproached", "EngagementZone", "onMoveCloser")
    memory.subscribeToEvent("Dialog/LastInput", "SpeechEvent", "onSpeechDetected")
    ALDialog.subscribe('my_dialog_example')
    while True:
        time.sleep(1)
        if userStoppedTalking:
            # print "user stopped talking"
            userStoppedTalking = False
            break

        # if time.time() > tStart + 10:
        #     ALMemory.insertData("timeout", "True")
        #     print "timeout"
        #     break

        if dist == 2:
            print "walked away"
            break


def str2bool(s):
    if s == 'True':
        return True
    elif s == 'False':
        return False
    else:
        raise ValueError("Cannot convert {} to a bool".format(s))


def bool2str(s):
    if s == 1:
        return 'True'
    elif s == 0:
        return 'False'
    else:
        raise ValueError("Cannot convert {} to a bool".format(s))


def getDistance(data):
    distance = None
    try:
        if data[1][0][1] <= 1:
            distance = 1
        if 1 < data[1][0][1] <= 2.5:
            distance = 1
        elif data[1][0][1] > 2.5:
            distance = 2
            #            ALMemory.insertData("timeout","False")

    except TypeError:
        distance = 1

    return distance


def entryPoint():
    global goal
    goal = actionServer.accept_new_goal()
    rospy.loginfo("Start of interaction ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    # Populate the shop list from file
    global shopList
#    global ALTracker
    global ALSpeechRecognition
    global ALMemory
    global ALDialog
#    global ALEngagementZones
#    global ALMotion
    shopList = ShopList(path + '/etc/shop_list.txt')

    global foundperson
    foundperson = False

    ALSpeechRecognition = session.service("ALSpeechRecognition")
    ALMemory = session.service("ALMemory")
    ALDialog = session.service("ALDialog")
#    ALTracker = session.service("ALTracker")
    # ALEngagementZones = session.service("ALEngagementZones")
    # ALEngagementZones.setSecondLimitDistance(3.5)
    # ALEngagementZones.setFirstLimitDistance(2.5)
#    ALMotion = session.service("ALMotion")

    topic_content = ('topic: ~example_topic_content()\n'
                     'language: enu\n'
                     'concept:(bye) [bye goodbye cheers farewell "see you later"]\n'
                     'concept:(coffee) [coffee cappuccino latte espresso americano]\n'
                     'concept:(shop) [starbucks costa public "hardware electronics" tesco primark "phone heaven"]\n'
                     'concept:(electronics) [iPhone Samsung case adapter television TV charger mobile phone]\n'
                     'concept:(clothing) [shoes jacket "t-shirt" belt jeans trousers shirt suit coat underwear clothing]\n'
                     # 'u: ([e:FrontTactilTouched e:MiddleTactilTouched e:RearTactilTouched]) $tskFilled = True $ctxTask = directions $shopName = costa\n'
                     'u: (Open the Pod bay doors) I am sorry Dave, I am afraid I can not do that.\n'
                     'u: (* ~coffee) $tskFilled=True $ctxTask=coffee $usrEngChat=False\n'
                     'u: (* ~electronics) $tskFilled=True $ctxTask=electronics $usrEngChat=False\n'
                     'u: (* ~clothing) $tskFilled=True $ctxTask=clothing $usrEngChat=False\n'
                     'u: (* ~bye) $bye=True $usrEngChat=False \n'
                     #'u: (e:Dialog/NotUnderstood) $usrEngChat=True \n'
                     'u: (_*) $Dialog/LastInput=$1 \n'
                     'u: (e:Dialog/NotSpeaking5) $timeout=True $usrEngChat=False \n'
                     'u: (* _~shop) $tskFilled=True $ctxTask=directions $shopName=$1 $usrEngChat=False \n')

    # Loading the topics directly as text strings
    topic_name = ALDialog.loadTopicContent(topic_content)

    # Activating the loaded topics
    ALDialog.activateTopic(topic_name)

    ALDialog.setLanguage("English")
#    ALMotion.setBreathEnabled('Arms', True)
    instantiateMemory()

    initChatbot(path + '/etc/chatbot/eg/brain')

    # Subscribe to the speech and face detection events:
    global memory
    global pplperc
    memory = ALProxy("ALMemory")
    pplperc = ALProxy("ALPeoplePerception")

    global rosSubscriber
    rosSubscriber = rospy.Subscriber('/naoqi_driver_node/people_detected', PersonDetectedArray, FaceDetected)


rospy.init_node('DialogueStart', anonymous=True)
actionServer = SimpleActionServer('dialogue_start', dialogueAction, auto_start=False)
actionServer.register_goal_callback(entryPoint)

r = rospkg.RosPack()
path = r.get_path('mummer_dialogue')

parser = argparse.ArgumentParser()
parser.add_argument("--ip", type=str, default="137.195.108.20",
                    help="Robot's IP address. If on a robot or a local Naoqi")
parser.add_argument("--port", type=int, default=9559,
                    help="port number, the default value is OK in most cases")
parser.add_argument("--topic-path", type=str, required=False,
                    help="absolute path of the dialog topic file (on the robot)")

args = parser.parse_args()
session = qi.Session()

try:
    session.connect("tcp://{}:{}".format(args.ip, args.port))
    myBroker = ALBroker("myBroker",
                        "0.0.0.0",  # listen to anyone
                        0,  # find a free port and use it
                        args.ip,  # parent broker IP
                        args.port)  # parent broker port
    print "Success!"
except RuntimeError:
    print ("\nCan't connect to Naoqi at IP {} (port {}).\nPlease check your script's arguments."
           " Run with -h option for help.\n".format(args.ip, args.port))
    sys.exit(1)

# some_state = state(tskC, tskF, prevA, dist, ctx, usrEng, mode, timeout, usrTerm, bye, usrEngC, lowConf, turn)
# some_state = state(False, False, 2, 1, '', True, False, False, False, False, False, False, False)
# print action(some_state)

# ALDialog.subscribe('my_dialog_example')

# print "shopName", ALMemory.getData("shopName"), '\n'

# Reset some state variables in ALMemory
# resetAttributes()

global SpeechEvent
SpeechEvent = SpeechEventModule("SpeechEvent")

__update_srv_name = rospy.get_param("~update_srv_name","/kcl_rosplan/update_knowledge_base_array")

actionServer.start()
rospy.spin()


# try:
#    while True:
#        time.sleep(1)
# finally:
#    print
#    print "Interrupted by user, shutting down..."
#    ALTracker.stopTracker()
#    ALTracker.unregisterAllTargets()
#    myBroker.shutdown()
#    ALMotion.setBreathEnabled('Body', False)
#    sys.exit(0)
