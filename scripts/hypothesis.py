#! usr/bin/env python2

## @package exprob_assignment2
#
#  \file hypothesis.py
#  \brief This program elaborates the hints received
#
#  \author Zoe Betta
#  \version 1.0
#  \date 29/10/2021
#  \details
#  
#  Subscribes to: <BR>
#       /hint
#
#  Publishes to: <BR>
#	    /hypothesis
#
#  Services: <BR>
#       None
#
#  Client Services: <BR>
#       armor_interface_srv
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#    This node receives the hints published on the topic /hint. Then it is 
#    able to elaborate them: it check if the person, location or weapon is 
#    already present in the ontology. In case it is not in the ontology yet
#    it adds it. After that it checks if the hint has already been saved in
#    the ontology, if it is the first time the hint has been received it 
#    checks if the hypothesy that correspond to that hint ID is complete 
#    and not inconsistent. In that case it checks if the hypothesy has 
#    already been sent, if not it publish the hipothesy on the topic /hypothesis
#    that is of type Hypothesis.msg. 

import copy
import math
import sys
import time
import geometry_msgs.msg
import numpy as np
import rospy
from std_msgs.msg import String
from exprob_assignment2.srv import Oracle
from exprob_assignment2.srv import Hypothesis
from exprob_assignment2.srv import Results

#global variables
checkcorr=[]
complete=[]

##
#	\brief This function implements the ros node
#	\param : None
#	\return : None
# 	
#   When the node is first initialized it loads all of the publishers, 
#   subscribers and servers. It also loads the ontology file needed to 
#   reason on the hints.
#	
def main():
  global  armor_service, pub
  rospy.init_node('hypothesis')
  service=rospy.Service('/correcthypothesis', Hypothesis, correcthypothesis)
  pub = rospy.Subscriber('/complete', String, callbackComplete)

  print('inizializzato tutto')
  rospy.spin() 


def callbackComplete(msg):
	#save all the complete ID
	complete.append(str(msg.data))
	print(complete)
	
def correcthypothesis(req):
    # call the server and control if ok
    global checkcorr
    print(req.start)
    idtocheck=-1
    oraclecall=rospy.ServiceProxy('/oracle_solution', Oracle)
    resultscall = rospy.ServiceProxy('/results', Results)
    for i in range(len(complete)):
        found=0
        for j in range(len(checkcorr)):
            if complete[i]==checkcorr[j]:
                found=1
        print(found)
        if found==0:
            idtocheck=complete[i]
            checkcorr.append(idtocheck)
            break
    del complete[:]
    if idtocheck!=-1:
        resp=oraclecall()
        print(resp.ID)
        winid=resp.ID
        if str(resp.ID)==idtocheck:
            respcall=resultscall(winid)
            print( " %s with the %s in the %s" % (respcall.who , respcall.what ,respcall.where))
            return True
        else:
            return False
    else :
	    return False
            
    	
if __name__ == '__main__':
  main()        
        
   
