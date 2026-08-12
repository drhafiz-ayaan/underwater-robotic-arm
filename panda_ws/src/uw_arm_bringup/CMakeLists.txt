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
    scripts/pick_and_place.py
    scripts/object_tf_publisher.py
    scripts/grasp_manager.py
    scripts/kill_sim.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
