#! usr/bin/env python2

## @package exprob_assignment2
#
#  \file hypothesis.py
#  \brief This program checks if an hypothesis is correct
#
#  \author Zoe Betta
#  \version 1.0
#  \date 17/03/2022
#  \details
#  
#  Subscribes to: <BR>
#       /complete
#
#  Publishes to: <BR>
#
#  Services: <BR>
#       /correcthypothesis
#
#  Client Services: <BR>
#       /oracle_solution
#       /results
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#  


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
#   subscribers and servers. 
#	
def main():
  global  armor_service, pub
  rospy.init_node('hypothesis')
  service=rospy.Service('/correcthypothesis', Hypothesis, correcthypothesis)
  pub = rospy.Subscriber('/complete', String, callbackComplete)
  rospy.spin() 


##
#	\brief callback on the topic /complete
#	\param : msg
#	\return : None
# 	
#   This function receives all of the IDs of the complete hypothesis available
#   it then saves them in a global variable.
#	
def callbackComplete(msg):
	#save all the complete ID
	complete.append(str(msg.data))
	#print(complete)
	
##
#	\brief callback for the service /correcthypothesis
#	\param : req
#	\return : Bool to see if we asked for the winning hypothesis
# 	
#   When this server is called the function checks if there is a complete hypotheisis
#   that has not been checked before, in that case it calls the oracle server
#   and compares the results, if the winning ID and the ID to be checked are the same it 
#   prints the correct hypothesis and it returns true, otherwise it returns false.
#	
def correcthypothesis(req):
    global checkcorr
    #print(req.start)
    # set the ID to be checked to -1
    idtocheck=-1
    # initialize the client on the topic /oracle_solution
    oraclecall=rospy.ServiceProxy('/oracle_solution', Oracle)
    # initialize the client on the topic /results
    resultscall = rospy.ServiceProxy('/results', Results)
    # for every element of the global variable complete
    for i in range(len(complete)):
		# initialize found to zero
        found=0
        # for every element of the global variable that stores the already asked hypothesis
        for j in range(len(checkcorr)):
			# if the two elements are the same
            if complete[i]==checkcorr[j]:
				# set found to 1, I have already checked that element
                found=1
        #print(found)
        # if found remains zero, I haven't checked that ID yet
        if found==0:
			# set the variable storing the ID to be checked to the element of the complete array
            idtocheck=complete[i]
            # adding the value to the list of the already checked IDs
            checkcorr.append(idtocheck)
            break
    # deleting the content of the list storing all of the complete hypothesis
    del complete[:]
    # if I found an ID to check
    if idtocheck!=-1:
		# call the server client
        resp=oraclecall()
        #print(resp.ID)
        #save the value returned from the server
        winid=resp.ID
        # if the value returned is the same value I want to check
        if str(resp.ID)==idtocheck:
			# call the server to retrieve the field of the winning hypothesis
            respcall=resultscall(winid)
            # print the winning hypothesis
            print( " %s with the %s in the %s" % (respcall.who , respcall.what ,respcall.where))
            return True
        # if the ID are  different
        else:
            return False
    # if I don't have another ID to check
    else :
	    return False
            
    	
if __name__ == '__main__':
  main()        
        
   
