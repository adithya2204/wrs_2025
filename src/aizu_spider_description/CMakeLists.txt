cmake_minimum_required(VERSION 3.22)
project(aizu_spider_description)

find_package(ament_cmake REQUIRED)

# Install robot assets
install(
  DIRECTORY
    urdf
    meshes
    collision
    launch
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

# Install license
install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)

# Install Python nodes
install(PROGRAMS
  scripts/front_cloud_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
