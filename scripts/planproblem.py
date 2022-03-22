#! /usr/bin/env python

## @package exprob_assignment2
#
#  \file planproblem.py
#  \brief this file implements the calls to the planning server
#
#  \author Zoe Betta
#  \version 1.0
#  \date 21/02/2022
#  \details
#  
#  Subscribes to: <BR>
#	 
#
#  Publishes to: <BR>
#	 
#
#  Services: <BR>
#    
#  Action Services: <BR>
#
#  Client Services: <BR>
#  /rosplan_problem_interface/problem_generation_server
#  /rosplan_planner_interface/planning_server
#  /rosplan_parsing_interface/parse_plan
#  /rosplan_plan_dispatcher/dispatch_plan
#  /rosplan_knowledge_base/update
#    
#
#  Description: <BR>
#  This file implements the logic to generate the plan to control the robot.
#  It reads the feedback from the plan before and keeps generating new plans until
#  all the action of one are  successful. It also updates the knowedge base 
#  depending on a ros parameter to customize the behaviour of the robot.



import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import time
import actionlib
import exprob_assignment2.msg
import random

# global variables
prev=4

##
#	\brief This function initializes all the server used for the planning part
#	\param : 
#	\return : None
# 	
#	This function initializes all the servers and waits for all of them to be 
#   running before moving forward
def initialization():
    global problem_generation, planning, parsing, dispatch, update
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning = rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parsing = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    dispatch = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan',DispatchService)	
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    update= rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    print(rospy.get_param('random_hint'))
    print('all servers loaded')

##
#	\brief This function deletes the hint_taken parameter from the knowledge base for one waypoint
#	\param name: it is the waypoint I want to modify 
#	\return : None
# 	
#	This function calls the knowledge base server to delete the predicate (hint_taken)
#   for one waypoint, the one recognized by the parameter name
def update_waypoint(name):
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'hint_taken'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', name))	
    result=update(req)
    
##
#	\brief This function deletes the (hypothesis_complete) fact
#	\param :
#	\return : None
# 	
#	This function calls the knowledge base server to delete the predicate
#   (hypothesis_complete)
def update_complete():
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'hypothesis_complete'
    result=update(req)	

##
#	\brief This function updates the knowledge base
#	\param :
#	\return : None
# 	
#	This function looks at the parameter in the ros param server random.
#   if random is set to true it finds a random waypoint ( different from the previously
#   calculated one) and it proceeds to delete the (hint_taken) for that waypoint.
#   In case the random parameter is set to false it deletes the fact (hint_taken) for
#   all waypoints. After that it deletes the (hypothesis_complete) fact.
def know_update():
    global prev
    # calculate a random number
    n=random.randint(1,4)
    # if the random param is true
    if (rospy.get_param('random_hint')) == True :
		# calculate a random number until it is different from the previous one
        while (n==prev):
            n=random.randint(1,4)
        # update the previous number
        prev=n
        # delete the randomly chosen waypoint
        if n==1:
	        update_waypoint('wp1')
        elif n==2: 
	        update_waypoint('wp2')
        elif n==3:
            update_waypoint('wp3')
        elif n==4:
            update_waypoint('wp4')
    # if the random param is false
    else:
        update_waypoint('wp1')
        update_waypoint('wp2')
        update_waypoint('wp3')
        update_waypoint('wp4')
    # delete the (hypothesis_complete) fact
    update_complete()

##
#	\brief This function is called when the node is started
#	\param :
#	\return : None
# 	
#	This function initializes all of the needed services, then it calculates a new plan
#   until the robot is able to fuòòy complete one. 
def main():
    global pub_, active_, act_s
    rospy.init_node('planproblem')
    initialization()
    success=False
    goal=False
    # until the feedback from the planner is not true
    while ( success== False or goal == False):
		# generate the problem
        response_pg=problem_generation()
        print('problem generates')
        time.sleep(1)
        # generate the plan
        response_pl=planning()
        print('plan generates')
        time.sleep(1)
        # parse the plan 
        response_pars=parsing()
        print('parse generates')
        time.sleep(1)
        # read the feedback
        response_dis=dispatch()
        print(response_dis.success)
        print(response_dis.goal_achieved)
        success= response_dis.success
        goal= response_dis.goal_achieved
        # update the knowledge base
        know_update()
        time.sleep(1)
    print ( 'SUCCESS')

if __name__ == '__main__':
    main()
