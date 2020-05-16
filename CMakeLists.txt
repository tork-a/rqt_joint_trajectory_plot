cmake_minimum_required(VERSION 3.0.2)
project(rqt_joint_trajectory_plot)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(
  FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.ui"
)

install(
  PROGRAMS script/joint_trajectory_generator.py script/rqt_joint_trajectory_plot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.launch"
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_PYTHON_OPTS "--max-line-length=180")
  roslint_python(src/rqt_joint_trajectory_plot/plot_widget.py)
  add_dependencies(tests roslint)

  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(STATUS "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()
