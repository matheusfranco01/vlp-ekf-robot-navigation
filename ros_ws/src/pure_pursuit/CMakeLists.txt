cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs std_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/pure_pursuit_node.py
  scripts/publish_test_path.py
  scripts/publish_interlagos.py
  scripts/validate_simulation.py
  scripts/plot_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/test_pure_pursuit.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/pure_pursuit.test)
endif()
