# Task 1
- Create a ros package with `catkin_create_pkg package_name [depend1] [depend2] ...`
- Write all the node files in `src` folder for cpp
- For rospy create a scripts folder and write node files there
- Source the right workspace
- Run `roscore` for running master node
- Run the nodes with
```bash
rosrun trial subs.py
rosrun task2 talk.py   # in a different terminal window
```
- Make sure to run `catkin_make` or `catkin build` while being in the `catkin_ws` folder which should contain the `src` folder
- Also to rosrun python scripts, it is necessary for us to do the necessary changes in the CMake file, so that the python scripts become executables.
- Make sure to run catkin_make after every change in the source script.
- msg are used to store custom messages

## Problems faced
- `catkin build` did not work properly
- Changing the CMake files took some effort.

# Task 2
- To see the contents of a bag file, we need to subscribe to the topics that were recorded by it
- Use the `np.random.normal` method for sampling at random from a normal distribution
```
rosrun add_noise add_noise.py
```
- Make sure to set the model and source the setup file.

# Course 1

## Ros Master
- ROS master-roscore 

## Nodes
- Run a node with "rosrun package_name node_name"
- Active Nodes - "rosnode list"
- Information about a node -"rosnode info node_name"


## Topics
- List active topics  
```
rostopic list
```
- Print the contents of a topic  
```
rostopic echo /topic
```
- Information about a topic 
```
rostopic info /topic
```

## ROS Messages
- Defined in .msg files
- See the type with
```
rostopic type /topic
```
- Publish a message to a topic 
```
rostopic pub /topic type data
```

## ROS Workspace Environment
- Default workspace: 
```
source /opt/ros/noetic/setup.bash
```
- Sourcing your catkin workspace
```bash
cd ~/catkin_ws
source ./devel/setup.bash # or source ./devel/setup.zsh
```
- We can gedit bash.rc and edit it so as to source the detup automatically everytime

## catkin build system
- Use `catkin build` instead of `catkin_make`
```
catkin build package_name
```
- Update environment every time you build a new package 
- Work is done in `src` folder
- To clean the build:
```
catkin clean
```
- Check catkin workspace with:
```
catkin config
```

## ROS launch
- Launch multiple nodes 
- Launches launch files which are written in XML as \*.launch
```
roslaunch file_name.launch
```
or 
```
roslaunch package_name file_name.launch
```

---
# Course 2

- ROS packages can contain source code, launch files, configuration files, message definitions, data, and documentation
- Good practice to separate message definitions package from other packages
- A package contains:
	- **config** folder (Parameter files (YAML))
	- **include/package_name** folder (C++ include headers)
	- **launch** folder (\*.launch files)
	- **src** folder (Source files)
	- **test** folder (Unit/ROS tests)
	- CMakeLists.txt (CMake build file)
	- package.xml (Package information)

##### package.xml
- defines the properties of the package
	- Package name
	- Version number
	- Authors
	- Dependencies on other packages
	- ...

##### CMakeLists.xml
1. Required CMake Version (`cmake_minimum_required()`)
2. Package Name (`project()`)
3. Configure C++ standard and compile features
4. Find other CMake/Catkin packages needed for build (`find_package()`)
5. Message/Service/Action Generators (`add_message_files()`, `add_service_files()`, `add_action_files()`)
6. Invoke message/service/action generation (`generate_messages()`)
7. Specify package build info export (`catkin_package()`)
8. Libraries/Executables to build (`add_library()` / `add_executable()`/`target_link_libraries()`)
9. Tests to build (`catkin_add_gtest()`)
10. Install rules(`install()`)
---
-  ROS main header file include
```cpp
#include <ros/ros.h>
```
- `ros::init(...)` has  to be called before other ROS functions
- `NodeHandle` is the access point for communications with ROS systems
- `ros::Rate` is a helper class to run loops at a desired frequency
- `ros::ok()` checks of a node should continue running
- `ROS_INFO()` logs messages to the filesystem
- `ros::spinOnce()` processes incoming messages via callbacks

#### Logging
- Instead of `std::cout`, use `ROS_INFO`
```cpp
ROS_INFO("Result: %d", result); // printf style
ROS_INFO_STREAM("Result: " << result);
```
- For python use rospy.loginfo instead.

#### Subscriber
- When a message is received, callback function is called with  the contents of the message as argument
- Start listening to a topic by calling the method `subscribe()` of the node handle
```python
subs = rospy.Subscriber(topic, queue_size, callback_function)
```
- `rospy.spin()` processes callbacks and will not return until the node has been shutdown

#### Publisher
- Create a publisher with the help of the node handle
```python
pub = rospy.Publisher(topic,message type, queue_size);
```
- Create the message contents
- Publish the contents with
```python
pub.publish(message);
```

#### ROS parameter Server
- Nodes use the parameter server to store and retrieve parameters at runtime
- Best used for static data such as configuration parameters
- Parameters can be defined in launch files or separate YAML files
- List all parameters with
```
rosparam list
```
- Get the value of parameters with
```
rosparam get parameter_name
```
- Set the value of a parameter with
```
rosparam set parameter_name value
```

## RViz
- 3D visualisation tool for ROS
- Subscribes to topics and visualizes the message contents
- Interactive tools to publish user information
- Run RViz with
```
 rviz
```
