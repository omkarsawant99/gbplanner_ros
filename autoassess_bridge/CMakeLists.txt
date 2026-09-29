cmake_minimum_required(VERSION 3.0.2)
project(autoassess_bridge)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

catkin_python_setup()

add_message_files(FILES
  Finding.msg
  InspectionPlan.msg
  InspectionTask.msg
  UploadStatus.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

catkin_install_python(PROGRAMS scripts/autoassess_bridge_node scripts/worker_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
