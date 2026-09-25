cmake_minimum_required(VERSION 3.8)
project(create3_lidar_bringup LANGUAGES NONE)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(create3_coverage)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_target(source_manifest ALL
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/source_manifest.py
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/source_manifest.json)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/source_manifest.json DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/tf_relay.py scripts/safe_cmd_vel_relay.py scripts/keyboard_teleop.py
  scripts/coverage_supervisor.py scripts/coverage_meter.py tools/preview_coverage.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(coverage_core test/test_core.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 180)
  ament_add_pytest_test(coverage_revision test/test_revision.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 180)
  ament_add_pytest_test(coverage_ros test/test_ros.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 180)
endif()
ament_package()
