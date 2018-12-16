cmake_minimum_required(VERSION 2.8.3)
project(donkey_actuator)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
)

catkin_python_setup()

add_message_files(FILES Servo.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

install(DIRECTORY launch msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS bin/${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
