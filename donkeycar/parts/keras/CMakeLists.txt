cmake_minimum_required(VERSION 2.8.3)
project(ros_donkey_keras)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS bin/${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
