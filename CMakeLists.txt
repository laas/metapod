# Copyright 2011, 2012,
#
# Maxime Reis (JRL/LAAS, CNRS/AIST)
# Antonio El Khoury (JRL/LAAS, CNRS/AIST)
# Sébastien Barthélémy (Aldebaran Robotics)
#
# This file is part of metapod.
# metapod is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# metapod is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with metapod.  If not, see <http://www.gnu.org/licenses/>.
#
# Creation: 14/06/2011

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmakeutils.cmake)

SET(PROJECT_NAME metapod)
SET(PROJECT_DESCRIPTION "Implementation of a robot dynamics library")
SET(PROJECT_URL "https://github.com/laas/metapod")

SET(CXX_DISABLE_WERROR True)

IF(UNIX OR MINGW)
  ADD_DEFINITIONS("-fvisibility=hidden")
ENDIF()

# Options
CMAKE_POLICY(SET CMP0012 NEW)
OPTION(BENCHMARK "Build the benchmarking tool" OFF)
OPTION(WITH_SIMPLE_HUMANOID "Build the simple humanoid example model" ON)
OPTION(WITH_SIMPLE_ARM "Build the simple arm example model" ON)

SETUP_PROJECT()

# Search for Boost.
# Boost.Test is used by the test suite.
# Boost program_options is used by the embedfile utility, which is used
# by metapod_robotbuilder
# Boost filesystem and regex are used by metapod_robotbuilder.
# Boost filesystem depends on Boost system.
SET(BOOST_COMPONENTS
  filesystem system unit_test_framework program_options regex)
SEARCH_FOR_BOOST()
# If Boost is recent enough, we look for Boost timer which can be used by
# by metapod_timer, which is in turn used by the benchmark.
# Boost timer depends on Boost chrono and system.
IF(BENCHMARK AND NOT Boost_VERSION LESS 104800)
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} timer chrono system)
  SEARCH_FOR_BOOST()
ENDIF()

# Search for Eigen.
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.0")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

# Set header files.
SET(${PROJECT_NAME}_ROOT_HEADERS
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/config.hh
  include/${PROJECT_NAME}/macro.hh
  )
SET(${PROJECT_NAME}_ALGO_HEADERS
  include/${PROJECT_NAME}/algos/rnea.hh
  include/${PROJECT_NAME}/algos/crba.hh
  include/${PROJECT_NAME}/algos/jac_point_relative.hh
  include/${PROJECT_NAME}/algos/jac_point_chain.hh
  )
SET(${PROJECT_NAME}_TOOLS_HEADERS
  include/${PROJECT_NAME}/tools/backward.hh
  include/${PROJECT_NAME}/tools/backward_traversal.hh
  include/${PROJECT_NAME}/tools/backward_traversal_prev.hh
  include/${PROJECT_NAME}/tools/bcalc.hh
  include/${PROJECT_NAME}/tools/bodymacros.hh
  include/${PROJECT_NAME}/tools/common.hh
  include/${PROJECT_NAME}/tools/deepest_common_ancestor.hh
  include/${PROJECT_NAME}/tools/depth_first_traversal.hh
  include/${PROJECT_NAME}/tools/fwd.hh
  include/${PROJECT_NAME}/tools/has_parent.hh
  include/${PROJECT_NAME}/tools/initconf.hh
  include/${PROJECT_NAME}/tools/is_ancestor.hh
  include/${PROJECT_NAME}/tools/jac_point_robot.hh
  include/${PROJECT_NAME}/tools/jcalc.hh
  include/${PROJECT_NAME}/tools/jointmacros.hh
  include/${PROJECT_NAME}/tools/print.hh
  include/${PROJECT_NAME}/tools/spatial.hh
  include/${PROJECT_NAME}/tools/static_assert.hh
  )
SET(${PROJECT_NAME}_SPATIAL_HEADERS
  include/${PROJECT_NAME}/tools/spatial/cm-anyaxis.hh
  include/${PROJECT_NAME}/tools/spatial/cm-freeflyer.hh
  include/${PROJECT_NAME}/tools/spatial/cm-oneaxis.hh
  include/${PROJECT_NAME}/tools/spatial/constraintmotion.hh
  include/${PROJECT_NAME}/tools/spatial/force.hh
  include/${PROJECT_NAME}/tools/spatial/inertia.hh
  include/${PROJECT_NAME}/tools/spatial/lti.hh
  include/${PROJECT_NAME}/tools/spatial/motion.hh
  include/${PROJECT_NAME}/tools/spatial/rm-aboutxaxis.hh
  include/${PROJECT_NAME}/tools/spatial/rm-aboutyaxis.hh
  include/${PROJECT_NAME}/tools/spatial/rm-aboutzaxis.hh
  include/${PROJECT_NAME}/tools/spatial/rm-chgaxis.hh
  include/${PROJECT_NAME}/tools/spatial/rm-general.hh
  include/${PROJECT_NAME}/tools/spatial/rm-identity.hh
  include/${PROJECT_NAME}/tools/spatial/rotation-matrix.hh
  include/${PROJECT_NAME}/tools/spatial/transform.hh
  )
INSTALL(FILES ${${PROJECT_NAME}_ROOT_HEADERS}
  DESTINATION include/${PROJECT_NAME}
  )
INSTALL(FILES ${${PROJECT_NAME}_ALGOS_HEADERS}
  DESTINATION include/${PROJECT_NAME}/algos
  )
INSTALL(FILES ${${PROJECT_NAME}_TOOLS_HEADERS}
  DESTINATION include/${PROJECT_NAME}/tools
  )
INSTALL(FILES ${${PROJECT_NAME}_SPATIAL_HEADERS}
  DESTINATION include/${PROJECT_NAME}/tools/spatial
  )

INCLUDE_DIRECTORIES("include" "${CMAKE_CURRENT_BINARY_DIR}/include")

IF(${WITH_SIMPLE_HUMANOID})
  ADD_SUBDIRECTORY(simple_humanoid)
ENDIF()
IF(${WITH_SIMPLE_ARM})
  ADD_SUBDIRECTORY(simple_arm)
ENDIF()
ADD_SUBDIRECTORY(tests)

ADD_SUBDIRECTORY(embedfile)
ADD_SUBDIRECTORY(robotbuilder)
IF(${BENCHMARK})
  ADD_SUBDIRECTORY(timer)
  ADD_SUBDIRECTORY(benchmark)
ENDIF(${BENCHMARK})

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
