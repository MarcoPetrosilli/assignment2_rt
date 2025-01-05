In this part of the assignment two ROS nodes have been developed to work together on a simulation of a mobile robot made up of various nodes and software components that were already provided at the outset.
The first node is an action client that sends a goal to the action server, consisting of the x and y coordinates that the mobile robot must reach. Upon starting the node, the robot begins moving, as the node reads initial target coordinates from the launch file and sends them to the action server. 
During the robot's movement, the user can cancel the goal by typing 'c' in the terminal (any other character will have no effect on the robot). If the goal is canceled, two messages appear on the terminal and is asked to the user to type new coordinates. 
These new coordinates are then sent to the action server, so the robot can resume its movement to the updated target.
Every time the user sets a new goal position, that is published on a new topic named /last_target, this is needed to let the last_target node reach its aim, as is explained later.
Furthermore, the action client node pubish the position and velocity information on a new topic named /pos_vel by using a customized message, named pos_vel too.
The second node that has been developed is a service node that when is called must give as output the last target position which as been set by the user of from the launch file.
To reach this aim, a new service has been developed which doesn't accept parameters for the request and it will give in the response the two last published coordinates.
The service is stored in the appropriate folder srv under the name of target_coord.srv.
The node that implement this service is subscribed to the /last_target topic and update two global variables everytime the subscribtion callback is executed, then when the /target_coord service il called by the user, the service callback whill return the actual values of those variables

Istruction to install and run the software:

Get into ROS workspace, in the src folder, and clone the repo with the given url:

	cd ~/<workspace_folder>/src
	git clone <repo-url> assignment2_rt_part1

Now get back to the workspace folder:

	cd ..

Then, compile the workspace with te following command:

	catkin_make

If all is gone well, is possibile to run the software by opening a new tab and run the following command:

	roslaunch src/assignment2_rt_part1/launch/ass.launch
	
This command will launch de whole software, in that launch file will be called:
	- another launch file named assignment1.launch, where are defined the target default coordinates and are launched 
	all the software components of the simulation
	- the action client node
	- the serive node
	
