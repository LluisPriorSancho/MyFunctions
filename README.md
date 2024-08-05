# MyFunctions
Personal set of functions to use in future projects. Currently in development.

## Python branch:

### Controllers.py
This file include 2 classes of controllers:
-  ON/OFF controller: returns True or False depending on the state of the controller. Allows the inclusion of a hysteresis and the limitmitation of an angle.
-  PID controller: returns the value of the controller considering a PID approach.

### ros_functions.py
This file contains ready-to-use functions in the ROS2 environment.
-  vector3_to_point: Change of variable type from geometry_msgs.Vector3 to geometry_msgs.Point.
-  point_to_vector3: Change of variable type from geometry_msgs.Point to geometry_msgs.Vector3.
-  get_direction: Given 2 points, get the direction vector.
-  get_distance: Given 2 points, get the distance between them.
-  quaternion_to_euler: Return a quaternion in euler angles.
-  euler_to_quaternion: Return a quaternion given a rotation in euler angles (ZYX).
-  radians_to_degrees: Transform a value of list of them from radians to degrees.
-  degrees_to_radians: Transform a value of list of them from degrees to radians.

### my_ros2_cheatsheet.py
This file is a cheatsheet of the basic functionalities of ROS2. Contains 3 multithreaded nodes:
-  MyCheatsheet_Pub_Sub_Timer: This node publishes an empty message every second and prints a message when it is received by the subscriber.
-  MyCheatsheet_Service_Server: This node offers a service to add 2 numbers.
-  MyCheatsheet_Service_Client: This node cleates an async client for the previous service.

## C++ branch:
Currently in development.