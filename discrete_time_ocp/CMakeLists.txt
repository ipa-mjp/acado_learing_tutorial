cmake_minimum_required(VERSION 2.8.3)
project(discrete_time_ocp)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules cob_control_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs pluginlib roscpp roslint sensor_msgs std_msgs tf tf_conversions trajectory_msgs urdf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Ceres REQUIRED)
find_package(ACADO REQUIRED)
find_package(CASADI REQUIRED)

message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")

catkin_package(
  CATKIN_DEPENDS cob_control_msgs cob_srvs eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs pluginlib roscpp sensor_msgs std_msgs tf tf_conversions urdf visualization_msgs
  DEPENDS Boost CERES ACADO
  INCLUDE_DIRS include
  LIBRARIES
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${ACADO_INCLUDE_DIRS} )
FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)


## add executable
add_executable(discrete_time_ocp discrete_time_ocp_node.cpp)
add_dependencies(discrete_time_ocp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(discrete_time_ocp ${ACADO_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${libacado})
