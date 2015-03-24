# TurtlebotMapping
Mapping things using a turtlebot  

The turtlebot can be ssh'ed to with `ssh turtlebot@192.168.17.211` from within the network.
To run a gmapping demo:
Start up the turtlebot software:  
`roslaunch turtlebot_bringup minimal.launch`  
Launch keyboard teleop to move the robot around:  
`roslaunch turtlebot_teleop keyboard_teleop.launch`    
Run the gmapping demo:  
`roslaunch turtlebot_navigation gmapping_demo.launch`  
And view the data in rviz:  
`roslaunch turtlebot_rviz_launchers view_navigation.launch`  

