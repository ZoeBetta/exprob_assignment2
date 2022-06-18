# exprob_assignment2
to run the code in a terminal run:

roslaunch exprob_assignment2 mylaunch.launch random:= true

if random== true then it cancels one random hint and search that one only, if random is false (default value) then every time the plan is not ocmpleted correctly it will delete all the hints

remember to add the file in the include folder!!

catkin_make --only-pkg-with-deps exprob_assignment2 

move the Robot.urdf outside the package folder in the path: /root/ros_ws/src/Robot.urdf

run the following command ./gradlew deployApp in the armor folder
