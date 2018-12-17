cmake_minimum_required(VERSION 2.8.3)
project(donkey_joy)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(PROGRAMS bin/${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
