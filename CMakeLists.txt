cmake_minimum_required(VERSION 3.0.2)
project(panda_block_planner)

find_package(catkin REQUIRED)

# Makes `from panda_block_planner import domain` work from the scripts.
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/plan_and_stack.py
  scripts/setup_scenario.py
  scripts/plan_only.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Not a node - a one-off measuring tool, run by hand against a live move_group
# when the reach envelope needs re-measuring. Installed so `rosrun` finds it.
catkin_install_python(PROGRAMS
  tools/measure_reach.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # The domain, planner and geometry tests need neither ROS nor Gazebo. They
  # also run standalone, which is how you will actually use them:
  #   PYTHONPATH=src python3 -m unittest discover -s test
  catkin_add_nosetests(test)
endif()
