cmake_minimum_required(VERSION 3.5)
project(messages_88)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs)
find_package(mavros_msgs)
find_package(nav_msgs)
find_package(sensor_msgs)
find_package(std_msgs)



set(msg_files
  msg/Battery.msg
  msg/Frontier.msg
  msg/FrontierList.msg
  msg/SpeciesImageSegment.msg
  msg/TaskStatus.msg
)

set(srv_files
  srv/Emergency.srv
  srv/Geopoint.srv
  srv/GetFrontiers.srv
  srv/GetMapData.srv
  srv/GetOceanScore.srv
  srv/InitDroneState.srv
  srv/RecordVideo.srv
  srv/RequestGoal.srv
  srv/RequestPath.srv
  srv/Save.srv
  srv/SpeciesCsv.srv
  srv/SpeciesHeatmap.srv
)

set(action_files
  action/Explore.action
  action/FindPath.action
  action/NavToPoint.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
  geographic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

ament_package()
