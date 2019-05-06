# vrep_quad_exploration
A vrep simulation of a quadcopter performing autonmous exploration is an unknown environment. Uses vrep, python and ROS

## Instructions
### How to run the position publisher
* Open the scene in this folder in verp
* Run roscore in one terminal
* Run `python position_publisher.py` in another terminal
### How to add obstacles to the obstacel space
* Add a non respondable object of the desired width and length inside the vrep scene in this repository.
* Append the name of the newly added object to the `objectsList` list in the file `position_publisher.py`
* When the vrep scene is open, the node will automatically read the objects and create an obstacle space with the size as read from the length of the topWall and the leftWall

## Notes
* The axis is defined as the x axis to the right and y axis up with the origin at the bottom left corner, as viewed from the top.(As represented by a Origin dummy object in the vrep scene in this repository)

