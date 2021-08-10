cmake_minimum_required(VERSION 2.8.3)
project(gen3_odometry)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package(
  CATKIN_DEPENDS)

roslaunch_add_file_check(launch)

catkin_install_python(PROGRAMS scripts/states.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

