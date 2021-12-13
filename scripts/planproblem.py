#! /usr/bin/env python

## @package exprob_assignment2
#
#  \file planning.py
#  \brief 
#
#  \author Zoe Betta
#  \version 1.0
#  \date 13/12/2021
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
#
#  Description: <BR>



import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from diagnostic_msgs.msg import KeyValue
import time

def main():
    global pub_, active_, act_s
    #I initialize the goal
    rospy.init_node('planproblem')
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning = rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parsing = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    dispatch = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan',DispatchService)
    print('all servers loaded')
    response_pg=problem_generation()
    print('problem generates')
    time.sleep(5)
    response_pl=planning()
    print('plan generates')
    time.sleep(5)
    response_pars=parsing()
    print('parse generates')
    time.sleep(5)
    response_dis=dispatch()
    print('dispatch generates')
    time.sleep(5)
    
    rospy.spin()

if __name__ == '__main__':
    main()
