#! /usr/bin/env python

import qi
import json
import time, sys
import random
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
from dialogue_task_actions.msg import RouteDescriptionGoalServerAction, RouteDescriptionGoalServerGoal
import signal
from dialogue_task_actions.msg import GiveVoucherAction, GiveVoucherGoal
from dialogue_task_actions.msg import EmptyAction, EmptyGoal
from pymongo import MongoClient
import flask, requests
from mummer_dialogue.msg import DialogueText
from dynamic_reconfigure.server import Server as DynServer
from mummer_dialogue.cfg import MummerDialogueConfig


signal.signal(signal.SIGINT, signal.SIG_DFL)

goal = None
some_state = None
fallback_state = None
memory = None
chatbot = None
nextAction = None
userStoppedTalking = False
actionName = None
shopList = None
ALDialog = None
ALMemory = None
topic_name = None
# ALTracker = None
ALSpeechRecognition = None
pplperc = None

rosSubscriber = None
rosMessagetimeStamp = None

stateThread = None

logfile = None
startTime = None

chat_enabled = True

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
asrconf = []

# Action dictionary
actionID = {"taskConsume": 1, "Greet": 2, "Goodbye": 3, "Chat": 4, "giveDirections": 5, "wait": 6, "confirm": 7,
            "requestTask": 8, "requestShop": 9}


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

class WordModule(ALModule):
    def __init__(self, name):
        ALModule.__init__(self, name)
        
    def onWordRecognized(self, value, identifier):
        global asrconf
        asrconf = identifier
        
        global memory
        try:
#            memory.unsubscribeToEvent("Dialog/LastInput", "SpeechEvent")
            memory.unsubscribeToEvent("WordRecognized", "WordEvent")
        except Exception:
            pass

def publish_dialogue_text(actor, text, utterances):
    d = DialogueText()
    d.header.stamp = rospy.Time.now()
    d.actor = actor
    d.used_utterance = text
    d.utterances = utterances[0::2]
    d.confidences = utterances[1::2]
    dialoguePublisher.publish(d)
    
def log(actor, text, utterances):
    try:
        publish_dialogue_text(str(actor), str(text), utterances)
        if logging:
            logfile.write(str(rospy.Time.now().to_sec())+"\t"+actor+"\t" + text + 
            "\t" + str(utterances) + "\t" + str(nextAction) + '\n')
    except ValueError as e:
        rospy.logwarn(e)
        
def log_human(text, utterances):
    log("Human", text, utterances)
    
def log_robot(text):
    log("Robot", text, [text,1.0])
    

class SpeechEventModule(ALModule):
    """ A simple module able to react to facedetection events """

    def __init__(self, name):
        ALModule.__init__(self, name)

    def onSpeechDetected(self, *_args):
        global memory
        memory.unsubscribeToEvent("Dialog/LastInput", "SpeechEvent")
#        memory.unsubscribeToEvent("WordRecognized", "WordEvent")
        ALDialog.unsubscribe('my_dialog_example')
        log_human(ALMemory.getData("Dialog/LastInput"), asrconf)
        # flipTurn()

        global userStoppedTalking
        userStoppedTalking = True
        

################ ROS ################
def FaceDetected(data):
    for p in data.person_array:
        if True:  # p.id == int(goal.userID):
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
#                ALDialog.subscribe('my_dialog_example')

                # observeState()
                global stateThread
                stateThread = Thread(target=observeState, args=())
                stateThread.start()
            break
        else:
            rospy.logwarn("Unknown person '%s'" % str(p.id))
            

def dyn_callback(config, level):
    global chat_enabled
    chat_enabled = config["enable_chat"]
    return config


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
    log_robot(text)        
    __call_service("/naoqi_driver/animated_speech/say", Say, SayRequest(text=text))


def clean_up():
    try:    
        memory.unsubscribeToEvent("WordRecognized", "WordEvent")
    except RuntimeError:
        pass
    
    instantiateMemory()
    
    if logging:
        logfile.close()

    if actionServer.is_active():
        rospy.loginfo("End of interaction ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        client = SimpleActionClient("/goal_server/disengage", GoalServerAction)
        client.wait_for_server()
        client.send_goal(GoalServerGoal())

        rosSubscriber.unregister()
        actionServer.set_succeeded()
    else:
        rospy.logerr("Tried to disengage while server was inactive")
        
def shut_down():
    if ALDialog != None:
        # stopping the dialog engine
        try:
            ALDialog.unsubscribe('my_dialog_example')
        except RuntimeError:
            pass
    
        # Deactivating all topics
        ALDialog.deactivateTopic(topic_name)
    
        # now that the dialog engine is stopped and there are no more activated topics,
        # we can unload all topics and free the associated memory
        ALDialog.unloadTopic(topic_name)

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
    # ALMemory.insertData("slotMissing", "False")

    ALMemory.insertData("Dialog/LastInput", "")


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
    ALMemory.insertData("slotMissing", "False")

    ALMemory.insertData("Dialog/LastInput", "")
    
    global nextAction
    global asrconf
    nextAction = None
    asrconf = []
    
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
    global ALMemory
    global nextAction
    global loop_exit
    loop_exit = False
    restore = False
    
    while not loop_exit:
        print "user utterance: ", ALMemory.getData("Dialog/LastInput")
        print asrconf
        if not restore:
            some_state = generateState()
            some_state.printState()
        else:
            some_state = fallback_state
            some_state.printState()
            restore = False
            
        nextAction = action(some_state)
        print "Action selected: ", nextAction
        
        # If the action was not to requestShop, reset the slotMissing variable
        if actionID[nextAction] != 6:
            if actionID[nextAction] != 9:
                ALMemory.insertData("slotMissing", "False")
                
        print "slotMissing: ", ALMemory.getData("slotMissing")
        

    
    #    decodeAction(nextAction)
    #    print "observe state: end"    
    #    
    #def decodeAction(nextAction):
       
        try:
            if nextAction == "Greet":
                greet()
            elif nextAction == "Chat":
                chat(lastUsrInput, not chat_enabled) # Remove "True" or set to False to enable chatbot
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
            elif nextAction == "requestShop":
                requestShop()
    
            if not turn:
                if nextAction == "Chat":
                    ALMemory.insertData("mode", "True")
                else:
                    ALMemory.insertData("mode", "False")
    
            # Write action taken to state
            if actionID[nextAction] != 6:
                ALMemory.insertData("prevAct", actionID[nextAction])
                
                
        except KeyError as e:
            print "Error", e
#            print "Fallback to default action"
    
            if rosMessagetimeStamp + 1. < rospy.Time.now().to_sec():
                print "fucktard left"
                goodbye()
                return
    
            chat('fallback utterance')
            if not turn:
                print "falling back to backup user state:"
                restore = True
#                pushFallbackStateToMemory()
                print '\n'
                continue
            

        
    
        # Save current ROBOT state as fallback state in case next turn produces an error.
        # 
        if turn:
            global fallback_state
            fallback_state = some_state
        
        print
        flipTurn()

    
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


def chat(sentence, disable = False):
    global chatbot

    # print sentence
#    print lastUsrInput
    try:
        if not disable:
            try:
    #            say(str(chatbot.reply("localuser", str(sentence))))
                p = {'question': sentence, 'sessionid': '123'}
                r = requests.get('http://127.0.0.1:5000/api/v1.0/ask', params=p)
                say(r.json().get('response').get('answer'))
            except Exception:
                pass
        else:
            say("I am afraid I do not understand.")
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

    ALMemory.insertData("ctxTask", "")
    ALMemory.insertData("tskFilled", "False")
    ALMemory.insertData("tskCompleted", "True")


def requestShop():
    say("There are " + str(len(shopList.filteredSales())) + " shops that have sales nearby. These are " 
    + shopList.filteredSales().enumShops())
#    say("These are " + shopList.filteredSales().enumShops())
    # say("Which shop would you like to get a voucher for?")
    ALMemory.insertData("ctxTask", "")
    ALMemory.insertData("tskFilled", "False")
    ALMemory.insertData("tskCompleted", "True")


def taskConsume():
    global ALMemory
    print ALMemory.getData("shopName")
    print shopList.filteredSales().getShops()
    if ALMemory.getData("ctxTask") == "voucherANDshop":
        if ALMemory.getData("shopName") in shopList.filteredSales().getShops():
            client = SimpleActionClient("/give_voucher", GiveVoucherAction)
            client.wait_for_server()
            client.send_goal_and_wait(GiveVoucherGoal(shop_id=shopList.getId(ALMemory.getData("shopName"))))
            log_robot("<voucher to " + ALMemory.getData("shopName") + ">")
            ALMemory.insertData("slotMissing", "False")
        else:
            say("I am sorry, this shop does not have give any vouchers this period")
    elif ALMemory.getData("ctxTask") == "selfie":
        client = SimpleActionClient("/take_picture", EmptyAction)
        client.wait_for_server()
        client.send_goal_and_wait(EmptyGoal())
        log_robot("<selfie app>")
    else:
        say("Let me see. There are " + str(len(shopList.filteredCategory(ALMemory.getData("ctxTask")))) +
            " " + ALMemory.getData("ctxTask") + " shops nearby. " +
            "These are " + shopList.filteredCategory(ALMemory.getData("ctxTask")).enumShops() + ".")

        # print shopList.filteredCategory(ALMemory.getData("ctxTask")).enumShops()
#        say("These are " + shopList.filteredCategory(ALMemory.getData("ctxTask")).enumShops() + ".")

    ALMemory.insertData("ctxTask", "")
    ALMemory.insertData("tskFilled", "False")
    ALMemory.insertData("tskCompleted", "True")
    print "Shop Name: ", ALMemory.getData("shopName")

def requestTask():
    say("Is there anything I can help you with?")


def wait():
    global memory
    global userStoppedTalking
    global timeout

    # Assume that the user is chatting unless recognized otherwise
    ALMemory.insertData('usrEngChat', 'True')

    ALMemory.insertData("tskCompleted", "False")
    ALMemory.insertData("timeout", "False")
    memory.subscribeToEvent("Dialog/LastInput", "SpeechEvent", "onSpeechDetected")
    memory.subscribeToEvent("WordRecognized", "WordEvent", "onWordRecognized")
    ALDialog.subscribe('my_dialog_example')
    while True:
        time.sleep(1)
        if userStoppedTalking:
            # print "user stopped talking"
            userStoppedTalking = False
            break


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

    except TypeError:
        distance = 1

    return distance


def entryPoint():
    global goal
    goal = actionServer.accept_new_goal()
    rospy.loginfo("Start of interaction ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    # Populate the shop list from file
    global ALSpeechRecognition
    global ALMemory
    global ALDialog
    global foundperson
    foundperson = False
    
    if logging:
        global logfile
        logfile = open(logpath + "/" + str(time.time())+ ("_w_chatbot" if chat_enabled else "_wo_chatbot") + ".log", "w")

    ALSpeechRecognition = session.service("ALSpeechRecognition")
    ALMemory = session.service("ALMemory")
    ALDialog = session.service("ALDialog")

    topic_content = ('topic: ~example_topic_content()\n'
                     'language: enu\n'
                     'concept:(bye) ["bye bye" Goodbye farewell "see you later" "see you" ]\n'
                     'concept:(coffee) [coffee cappuccino latte espresso americano]\n'
                     'concept:(shop) [starbucks costa public "hardware electronics" tesco primark "phone heaven"]\n'
                     'concept:(electronics) [iPhone Samsung case adapter television TV charger mobile phone]\n'
                     'concept:(clothing) [shoes jacket "t-shirt" belt jeans trousers shirt underwear clothing]\n'
                     'concept:(selfie) [selfie picture photo photograph]\n'
                     'concept:(voucher) [voucher sale sales bargain bargains "special offers" vulture]\n'
                     'u: (_*~voucher{*}_~shop{*}) $tskFilled=True $ctxTask=voucherANDshop $shopName=$2 $usrEngChat=False $slotMissing=False\n'
                     'u: (_*~voucher{*}) $tskFilled=True $ctxTask=voucher $usrEngChat=False $slotMissing=True\n'
                     #'  u1: (* _~shop) $tskFilled=True $ctxTask=voucherANDshop $usrEngChat=False $slotMissing=False $shopName=$1 $slotMissing==True\n'
                     'u: (_*~selfie{*}) $tskFilled=True $ctxTask=selfie $usrEngChat=False\n'
                     #'u: ([e:FrontTactilTouched e:MiddleTactilTouched e:RearTactilTouched]) $ctxTask=voucherANDshop $slotMissing=False  testing $slotMissing==True\n'
                     'u: (_*~coffee{*}) $tskFilled=True $ctxTask=coffee $usrEngChat=False\n'
                     'u: (_*~electronics{*}) $tskFilled=True $ctxTask=electronics $usrEngChat=False\n'
                     'u: (_*~clothing{*}) $tskFilled=True $ctxTask=clothing $usrEngChat=False\n'
                     'u: (_{*}~bye{*}) $bye=True $usrEngChat=False \n'
                     #'u: (e:Dialog/NotUnderstood) $usrEngChat=True \n'
                     'u: (_*) $Dialog/LastInput=$1 \n'
                     'u: (e:Dialog/NotSpeaking10) $timeout=True $usrEngChat=False \n'
                     'u: (_*_~shop{*}) ["$slotMissing==False $tskFilled=True $ctxTask=directions $shopName=$2 $usrEngChat=False" "$slotMissing==True $tskFilled=True $ctxTask=voucherANDshop $usrEngChat=False $shopName=$2"]\n'
                     ) 
                    

    # Loading the topics directly as text strings
    global topic_name
    topic_name = ALDialog.loadTopicContent(topic_content)

    # Activating the loaded topics
    ALDialog.activateTopic(topic_name)

    ALDialog.setLanguage("English")
    #    ALMotion.setBreathEnabled('Arms', True)
    instantiateMemory()

#    initChatbot(path + '/etc/chatbot/eg/brain')

    # Subscribe to the speech and face detection events:
    global memory
    global pplperc
    memory = ALProxy("ALMemory")
    pplperc = ALProxy("ALPeoplePerception")

    global rosSubscriber
    rosSubscriber = rospy.Subscriber('/naoqi_driver_node/people_detected', PersonDetectedArray, FaceDetected, queue_size=1)
    
    global startTime
    startTime = time.time()
      
    
rospy.init_node('DialogueStart', anonymous=True)
actionServer = SimpleActionServer('dialogue_start', dialogueAction, auto_start=False)
actionServer.register_goal_callback(entryPoint)
DynServer(MummerDialogueConfig, dyn_callback)

r = rospkg.RosPack()
path = r.get_path('mummer_dialogue')

logging = rospy.get_param("~logging", False)
logpath = rospy.get_param("~log_path", "/tmp")

parser = argparse.ArgumentParser()
parser.add_argument("--ip", type=str, default="pepper",
                    help="Robot's IP address. If on a robot or a local Naoqi")
parser.add_argument("--port", type=int, default=9559,
                    help="port number, the default value is OK in most cases")
parser.add_argument("--topic-path", type=str, required=False,
                    help="absolute path of the dialog topic file (on the robot)")

#args = parser.parse_args()
args, unknown = parser.parse_known_args()
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

client = MongoClient(
    rospy.get_param("~db_host", "localhost"),
    int(rospy.get_param("~db_port", 62345))
)
db_name = rospy.get_param("~db_name", "semantic_map")
db = client[db_name]
collection_name = rospy.get_param("~collection_name", "idea_park")
semantic_map_name = rospy.get_param("~semantic_map_name")

shopList = ShopList(db[collection_name].find({"semantic_map_name": semantic_map_name}))

# some_state = state(tskC, tskF, prevA, dist, ctx, usrEng, mode, timeout, usrTerm, bye, usrEngC, lowConf, turn)
#some_state = state(False, True, 9, 1, "directions", True, False, False, False, False, False, False, False)
#print action(some_state)

# ALDialog.subscribe('my_dialog_example')

# print "shopName", ALMemory.getData("shopName"), '\n'

# Reset some state variables in ALMemory
# resetAttributes()

global SpeechEvent
SpeechEvent = SpeechEventModule("SpeechEvent")

global WordEvent
WordEvent = WordModule("WordEvent")

__update_srv_name = rospy.get_param("~update_srv_name", "/kcl_rosplan/update_knowledge_base_array")

dialoguePublisher = rospy.Publisher('~dialogue_text', DialogueText, queue_size=10)

rospy.on_shutdown(shut_down)
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
