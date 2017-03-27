#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionServer
from dialogue_task_actions.msg import EmptyAction
from nao_interaction_msgs.srv import BehaviorManagerControl, BehaviorManagerControlRequest
from nao_interaction_msgs.srv import BehaviorManagerInfo, BehaviorManagerInfoRequest


class QuizGame(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._as = SimpleActionServer(name, EmptyAction, self.execute_cb, auto_start=False)
        self.quiz_app = rospy.get_param("~quiz_app", "quiz-9dde66/behavior_1")
        self._as.start()
        rospy.loginfo("... done")
        
    def __call_service(self, srv_name, srv_type, req):
         while not rospy.is_shutdown():
            try:
                s = rospy.ServiceProxy(
                    srv_name,
                    srv_type
                )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException, rospy.ServiceException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % srv_name)
                rospy.sleep(1.)
            else:
                return s(req)
                
    def execute_cb(self, goal):
        self.__enable_behaviour(self.quiz_app)
        while self.__is_behaviour_running(self.quiz_app) and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self._as.set_succeeded()
        
    def __enable_behaviour(self, name, flag=True):
        try:
            self.__call_service(
                "/naoqi_driver/behaviour_manager/start_behaviour" if flag else "/naoqi_driver/behaviour_manager/stop_behaviour", 
                BehaviorManagerControl, 
                BehaviorManagerControlRequest(name=name)
            )
        except:
            rospy.logwarn("Could not start logo app. Service responded with error.")
            
    def __is_behaviour_running(self, name):
        return name in self.__call_service(
            "/naoqi_driver/behaviour_manager/get_running_behaviors", 
            BehaviorManagerInfo, 
            BehaviorManagerInfoRequest()
        ).behaviors


if __name__ == "__main__":
    rospy.init_node("quiz_game")
    QuizGame(rospy.get_name())
    rospy.spin()

