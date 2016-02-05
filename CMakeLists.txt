# Copyright (c) 2014-2015 CNRS
# Author: Mylene Campana
#
# This file is part of ur_description.
# ur_description is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# ur_description is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# ur_description.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME ur_description)
SET(PROJECT_DESCRIPTION "Data specific to ur5 robot for hpp-corbaserver")
SET(PROJECT_URL "")

SETUP_PROJECT()

FINDPYTHON()
ADD_REQUIRED_DEPENDENCY ("hpp-corbaserver >= 3")

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  urdf/ur5_robot.urdf
  urdf/ur5_with_gripper.urdf
  urdf/wall.urdf
  urdf/table.urdf
  urdf/obstacles.urdf
  urdf/obstacle_sphere.urdf
  urdf/box.urdf
  urdf/carton.urdf
  urdf/large_table.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )

install(FILES
  srdf/ur5_robot.srdf
  srdf/ur5_with_gripper.srdf 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install (FILES
  src/hpp/corbaserver/ur5_robot/robot.py
  src/hpp/corbaserver/ur5_robot/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/ur5_robot)

install (FILES
  src/hpp/corbaserver/ur5_with_gripper/robot.py
  src/hpp/corbaserver/ur5_with_gripper/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/ur5_with_gripper)
 
SETUP_PROJECT_FINALIZE()
