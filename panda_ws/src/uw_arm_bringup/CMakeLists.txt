cmake_minimum_required(VERSION 3.8)
project(uw_arm_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds materials
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/joint_sweep.py
    scripts/demo_sequence.py
    scripts/record_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
