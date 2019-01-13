cmake_minimum_required(VERSION 2.8.3)
project(donkey_keras)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  bin/${PROJECT_NAME}_drive_node
  bin/${PROJECT_NAME}_train_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
