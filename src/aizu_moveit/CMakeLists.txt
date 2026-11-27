cmake_minimum_required(VERSION 3.22)
project(aizu_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/multi_joint_teleop.py
  scripts/controller_switcher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()