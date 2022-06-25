# Brief Introduction
This project implements the game of Cluedo played by a robot that has to explore an environment searching for hints. The robot should move in a single room looking for hints that are placed in known locations. When the robot is read to make an hypothesis it should return to the center of the room and ask the oracle if the hypothesis is correct. The logic behind the behaviour of the robot is based on a planninng problem and not a state machine.
# Software Architecture
![state diagram for the program](https://github.com/ZoeBetta/exprob_assignment2/blob/main/Images/Architecture_Exprob_2.pdf)
This architecture is composed of ten nodes. Of these ten nodes six of them implements the action related to the planning problem. There is one node for each action to be implemented.
The node that starts the planning server and generates the problem is the planproblem.py node that controls and implements all the server needed to implement a planning problem. It sends the requests to the ROSPlan server and receives the feedbacks, based on which it then updates the knowledge_base if a replan is needed. In particular if one action fails ( only the check_complete action and the check_hypothesis action can fail) the node deletes from the knowledge base at least one hint_taken predicate and also sets to false the predicate hypothesis_complete in order to look for more hints.
The node FromHomeAction.cpp implements the action defined in the domain as move_from_home. This action implements the motion of the robot from the home position to a predefined waypoint. We can see the same behaviour from the nodes ToHomeAction.cpp (for the action go_home) and MoveAction.cpp ( for the action goto_waypoint). These three nodes implements the exact same behavior but they are associated to three different action in the domain since there is the need to recognize the home position with a predicate that needs to be set to true and false when the robot reaches the home position or moves from the home position. All three nodes call the action server go_to_point that is implemented in the node go_to_point.py.
The node go_to_point.py implements the motion of the robot as an action server; it receives a desired position and orientation and it sends the required velocities to the robot. The motion is divided in three phases: 
1. the robot aligns itself in the direction of the goal
2. the robot moves in a straight line towards the goal
3. the robot rotates to obtain the desired orientation
The node Hint.cpp implements the action take_hint. When it is dispached the robot waits for an hint to be received.  We know that the hints can be found at the waypoint either in a low position or a high position. If when the robot reaches the waypoint no hint is received then the height of the end effector is the wrong one and the robot must move the arm to reach the other level. When the hint is retrieved it is send as a request to the server /hint that is implemented in the node hint.py and has the goal of checking if the hint is correctly formulated and saving it in the ontology.
The node CompleteAction.cpp implements the action check_complete. This node calls the server /checkcomplete that is implemented in the node hint.py. It sends a request and receives a boolean that is true if there is at least one hypothesis that is complete and not consistent and false if no hypothesis is both complete and consistent.
The node CorrectAction.cpp implements the check_hypothesis action. The node sends a request to the server /correcthypothesis that is implemented in the node hypothesis.py. The server returns true if the inquired hypothesis is the correct one and false otherwise.
The node hypothesis.py implements the server /correcthypothesis. It subscribes to the topic /complete where it receives the list of id of complete hypothesis. From the list it then picks one ID that has not been checked yet and it makes a request to the server /oracle_solution to retrieve the winning ID, it then compares the two IDs. If the two IDs are the same, meaning the hypothesis is correct, the node makes a request to the server /results passing as argument the winning ID to retrieve the fields of the hypotheis (who,what, where) and printing them on the screen.
The node hint.py as previously stated implements the three servers /hint, /checkcomplete, /results. All of these servers are based on the knowledge in the ontology. The server /hint checks if the hint is correctly formed, if all fields are filled and if what is written in the fields is meaningful. In case everything is correct the hint is saved in the ontology. The server /checkcomplete retrieves from the ontology all of the hypothesis, identified by a unique ID, that are complete and all the hypothesis that are not consistent. It then checks if there is at least one hypothesis that is in the complete list but not in the inconsistent list. All of the hypothesis that are both complete and consistent are then published on the topic /complete, if at least one hypothesis has been retrieved the server returns true, false otherwise. The server /results receives an ID and has to retrieve from the ontology all of the fields of that specific hypothesis; the fields are then returned as response of the server.
The node simulation.cpp implements the oracle, and so it implements the hint generation algorithm but also the server that returns the correct hypothesis. This node publishes the hint on the topic /oracle_hint whenever the cluedo link reaches some points in the room. The hint that is sent can be malformed or empty.

# Installation
In order to be able to successfully run this code some packages are needed: the armor server is needed in order to work with the ontology. The ontology is needed to save and reason on the hint retrieved during the game. 
The ROSPlan package is needed in order to run the planning server that given a domain file, a problem file is able to plan the action to reach the goal, and in case some action fails to replan. 
In order to obtain them I suggest downloading the docker that can be obtained at the following link: 

Once downloaded the docker in order to run the package it is suggested to clone the exprob_assignment2 github repository in the path /ros_ws/src. From that the file Robot.urdf needs to be moved from inside the package to outside in the path: /ros_ws/src. Also the file _____ that is inside the include folder must be moved  to the path /ros_ws/devel/include/exprob_assignment2. (CONTROLLARE!)
To build the package run:
catkin_make --only-pkg-with-deps exprob_assignment2

# Running Code
to run the code in a terminal run:

roslaunch exprob_assignment2 mylaunch.launch random:= true

random is a ros parameter needed to customize the behaviour of the robot. if random is set to true then one a replan is needed only one waypoint is set as not visited so the robot will explore only that waypoint before looking if there is a complete hypothesis. 
If random is set to false then everytime there is a replan all the waypoints are set as not visited and the robot will have to go to every waypoint before checking if there is at least one hypothesis that is both complete and consistent.

# Working Hypothesis
In order to implement this game I started from some hypothesis on the nature of the game and the actual capabilities of the robot. The main goal while deciding how to develop this application has been to develop a flexible architecture that would allow for easy integration and improvements. For this reason there are a lot of nodes, one for each action. In this way if we want to modify the domain file by removing or adding an action it will be a matter of only changing one file.
## System's Features

## System's Limitations

## Possible technical improvements

## Authors
Zoe Betta
s5063114@studenti.unige.it
zoe.betta@gmail.com
