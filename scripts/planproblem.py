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
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import time
import actionlib
import exprob_assignment2.msg


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
    print('all servers loaded')

def update_waypoint(name):
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'hint_taken'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', name))	
    result=update(req)
    
def update_complete():
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'hypothesis_complete'
    result=update(req)	

def know_update():
    update_waypoint('wp1')
    update_waypoint('wp2')
    update_waypoint('wp3')
    update_waypoint('wp4')
    update_complete()

def main():
    global pub_, active_, act_s
    #I initialize the goal
    rospy.init_node('planproblem')
    initialization()
    success=False
    goal=False
    client = actionlib.SimpleActionClient('/go_to_point', exprob_assignment2.msg.GoingAction)
    client.wait_for_server()
    goal=exprob_assignment2.msg.GoingGoal()
    goal.target_pose.pose.position.x=1
    goal.target_pose.pose.position.y=1
    goal.target_pose.pose.orientation.z=1
    client.send_goal(goal)
    client.wait_for_result()
    while ( success== False or goal == False):
        response_pg=problem_generation()
        print('problem generates')
        time.sleep(1)
        response_pl=planning()
        print('plan generates')
        time.sleep(1)
        response_pars=parsing()
        print('parse generates')
        time.sleep(1)
        response_dis=dispatch()
        print(response_dis.success)
        print(response_dis.goal_achieved)
        success= response_dis.success
        goal= response_dis.goal_achieved
        know_update()
        time.sleep(1)
    print ( 'SUCCESS')

if __name__ == '__main__':
    main()
