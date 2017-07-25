cmake_minimum_required(VERSION 2.8.3)
project(wam_dmp_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  hardware_interface
  control_msgs
  control_toolbox
  realtime_tools
  urdf
  std_msgs
  usc_utilities
)

find_package(Boost REQUIRED COMPONENTS thread signals system filesystem)
add_definitions(${Boost_DEFINITIONS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    control_toolbox
    realtime_tools
    urdf
	usc_utilities
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
  src/joint_position_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

