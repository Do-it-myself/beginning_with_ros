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
