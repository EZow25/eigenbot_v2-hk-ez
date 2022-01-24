# eigenbot
Eigenbot module automatic model building

Contact: Julian Whitman
<jwhitman@cmu.edu>, Shuo
<shuoyang@andrew.cmu.edu> (for how to use eigenbot in ROS environment)

Each module will know its own urdf, and will provide its indentification number/name, which modules are attached to it as children, which ports children are on, and which orientation children are affixed.

Module's urdf.xacro should have their stl, inertial properties these are standard urdf parts.
They will also need additional non-standard properties about the ports and mounts. The ports are places where children modules can be attached, and the mounts are the different orientations that a child module can be affixed to a port. There may also include some other properties, like whether it can communicate wirelessly, or have a battery, for future-proofing.
This framework should allow the addition of arbitrary robots as "modules" to the system: if you take an existing urdf, and add a wrapper which defines the ports, then it can act as a module.

## Quickstart ##
A script that will subscribe to eigenbot/topology, listen for the next message of that type, then parse it, convert it to urdf, and open rviz, where rviz publishes joint states, is:
cd to folder eigenbot/eigenbot/script
chmod +x description_listener.py (only need to do this once)

rosrun eigenbot description_listener.py 

(you must have roscore running in the background)

## Eigenbot package ##

The program is been wrapped up by a ROS package "eigenbot" in order to better leverage tools in ROS to process urdf and visualize the robot. 

To use this eigenbot package, first create a ROS workspace and then put package "eigenbot" under the src of the workspace. Use catkin\_make to compile the package (although this package does not contain any C/C++ files, compliation in ROS workspace allows ROS to search for this package. Please refer to [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) for more information.

There are follows files and folders under eigenbot:

1. CMakeLists.txt   package.xml  | ROS indexing files. Do not need to modify. 

2. description  |  fold contains xml and text files to describe our robot modules

3. include     src   | folders contain C/C++ source code. Empty currently.    

4. launch | folder contains launch files to display robot in rviz and gazebo 

5. meshes | folder contains stl files  
 
6. script | folder contains python scripts. Now the main script for generate robot is in this folder

7. urdf  | folder contains script generated xacro and urdf files

8. urdf.rviz  | an rviz config file to display robot in rviz. It will be automatically read and used by launch/rviz\_test.launch

After package "eigenbot" is correctly configured(especially after the "source YOUR\_ROS\_WORKSPACE/devel/setup.bash" is executed). Following commands below can be used to start rviz, MoveIt, and gazebo.

There will be a program, run when the robot is turned on, which takes all the module arragement info, and copies the urdf onboard each module, and assembles them into a single full robot urdf which can be then dropped into Gazebo, RViz, etc.
The current xacro writer is in folder script description_assembler.py generates autoXACRO.xacro along with a variety of other config files.


## MoveIt Config package ##

Some MoveIt configuration files get written automatically by description_assembler.py, and will appear in eigenbot/eigenbot_moveit_config/config


### Rviz Testing Step ###

(Make sure your system has ros installed)

1. cd to folder eigenbot/eigenbot/script

2. python description_assembler.py (Generate a xacro file)

3. roslaunch eigenbot rviz_test.launch


### Gazebo Testing Step ###

(Make sure your system has ros and gazebo both installed)

1. cd to folder eigenbot/eigenbot/script

2. python description_assembler.py (this will include a catkin_make call)

3. roslaunch gazebo_ros empty_world.launch (open gazebo using roslaunch)

4. rosrun gazebo_ros spawn_model -file autoXACRO.urdf -urdf -x 0 -y 0 -z 1 -model autoXACRO (put urdf model into gazebo)


### MoveIt Testing Step ###

1. cd to folder eigenbot/eigenbot/script

2. python description_assembler.py (this will include a catkin_make call)

3. roslaunch eigenbot_moveit_config demo.launch


### Old stuff ###

rosrun xacro xacro --inorder -o autoXACRO.urdf autoXACRO.xacro (convert xacro to urdf)
- This is now done within description_assembler.py

 roslaunch urdf_tutorial display.launch model:=autoXACRO.xacro  rvizconfig:=urdf.rviz  (open Rviz using roslaunch, load the model)
 - replaced with new rviz launch file

roslaunch gazebo_ros empty_world.launch
roslaunch eigenbot gazebo_add_model.launch model:=autoModel1
the argument "model:=autoModel1" can be omitted if you just want to spawn one model. It should be modified to other names if you want to spawn multiple models. 
- replaced with better launch file
