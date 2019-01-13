cmake_minimum_required(VERSION 2.8.3)
project(donkey_actuator)

find_package(catkin REQUIRED COMPONENTS
    donkey_msgs
    geometry_msgs
)

catkin_python_setup()

catkin_package()

install(PROGRAMS bin/${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
