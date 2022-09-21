# Notes on making custom message
package.xml
1. Add <build_depend>message_generation</build_depend>
1. Add <exec_depend>message_runtime</exec_depend>

CMakeLists.txt
1. find_package(catkin REQUIRED COMPONENTS
  ...
  message_generation 
  ...
  )
1. add_message_files(
  FILES
  ...
  *your_message.msg*
  ...
  )
1. generate_messages(
  DEPENDENCIES
  ...
  *your_dependencies*
  ...
  )
1. catkin_package(
  CATKIN_DEPENDS ... message_runtime ...
  )
1. ...
   - add_executable(*your_node* *your_node_path*)
   - add_dependencies(*your_node* ${${PROJECT_NAME}_EXPORTED_TARGETS})
   - target_link_libraries (*your_node* ${catkin_LIBRARIES})

# Note on network configurations
## Robot machine configuration - roscore
- export ROS_MASTER_URI=http://localhost:11311
- export ROS_HOSTNAME=*robot_machine_ip*
- export ROS_IP=*robot_machine_ip*
- echo "ROS_MASTER_URI: "$ROS_MASTER_URI
- echo "ROS_HOSTNAME: "$ROS_HOSTNAME
- echo "ROS_IP: "$ROS_IP

## Workstation configuration - rosrun
- export ROS_MASTER_URI=http://*robot_machine_ip*:11311
- export ROS_HOSTNAME=*workstation_ip*
- export ROS_IP=*workstation_ip*
- echo "ROS_MASTER_URI: "$ROS_MASTER_URI
- echo "ROS_HOSTNAME: "$ROS_HOSTNAME
- echo "ROS_IP: "$ROS_IP

# Note on turtlebot configuration
## Alias section
- alias burger='export TURTLEBOT3_MODEL=burger'
- alias waffle='export TURTLEBOT3_MODEL=waffle'
- alias tb3fake='roslaunch turtlebot3_fake turtlebot3_fake.launch'
- alias tb3teleop='roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'
- alias tb3='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
- alias tb3maze='roslaunch turtlebot3_gazebo turtlebot3_world.launch'
- alias tb3house='roslaunch turtlebot3_gazebo turtlebot3_house.launch'

## End of the file
- export TURTLEBOT3_MODEL=waffle
- export SVGA_VGPU10=0

# Note on OpenCV
## Installation
- sudo apt-get install ros-noetic-vision-opencv
- sudo apt-get install ros-noetic-usb-cam
- sudo apt-get install ros-noetic-image-view

## Webcam setting
- Install extension pack (make sure updated version)
- USB 3.0 controller
- Connect webcam via USB
- sudo usermod -a -G vboxusers *<your_username>* (Can be checked via whoami)