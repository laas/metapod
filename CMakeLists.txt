# Copyright 2011, 2012, 
#
# Maxime Reis,
# Antonio El Khoury
#
# JRL/LAAS, CNRS/AIST
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

SET(PROJECT_NAME metapod)
SET(PROJECT_DESCRIPTION "Implementation of a robot dynamics library")
SET(PROJECT_URL "https://github.com/laas/metapod")

SET(CXX_DISABLE_WERROR True)

# Options
CMAKE_POLICY(SET CMP0012 NEW)
OPTION(BENCHMARK "Build the benchmarking tool" OFF)

SETUP_PROJECT()

# Search for Boost.
# Boost.Test is used by the test suite.
SEARCH_FOR_BOOST()

# Search for Eigen.
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.0")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

# Set header files.
SET(${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS
  include/${PROJECT_NAME}/models/simple-humanoid/body.hh
  include/${PROJECT_NAME}/models/simple-humanoid/joint.hh
  include/${PROJECT_NAME}/models/simple-humanoid/robot.hh
  )

SET(${PROJECT_NAME}_SIMPLE_ARM_MODEL_HEADERS
  include/${PROJECT_NAME}/models/simple-arm/body.hh
  include/${PROJECT_NAME}/models/simple-arm/joint.hh
  include/${PROJECT_NAME}/models/simple-arm/robot.hh
  )

SET(${PROJECT_NAME}_ALGOS_HEADERS
  include/${PROJECT_NAME}/algos/rnea.hh
  include/${PROJECT_NAME}/algos/crba.hh
  )

SET(${PROJECT_NAME}_TOOLS_HEADERS
  include/${PROJECT_NAME}/tools/fwd.hh
  include/${PROJECT_NAME}/tools/common.hh
  include/${PROJECT_NAME}/tools/spatial.hh
  include/${PROJECT_NAME}/tools/jointmacros.hh
  include/${PROJECT_NAME}/tools/bodymacros.hh
  include/${PROJECT_NAME}/tools/initconf.hh
  include/${PROJECT_NAME}/tools/print.hh
  include/${PROJECT_NAME}/tools/jcalc.hh
  include/${PROJECT_NAME}/tools/bcalc.hh
  )

# Set header files installation prefix.
INSTALL(FILES ${${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS}
  DESTINATION include/${PROJECT_NAME}/models/simple-humanoid
  )

INSTALL(FILES ${${PROJECT_NAME}_SIMPLE_ARM_MODEL_HEADERS}
  DESTINATION include/${PROJECT_NAME}/models/simple-arm
  )

INSTALL(FILES ${${PROJECT_NAME}_ALGOS_HEADERS}
  DESTINATION include/${PROJECT_NAME}/algos
  )

INSTALL(FILES ${${PROJECT_NAME}_TOOLS_HEADERS}
  DESTINATION include/${PROJECT_NAME}/tools
  )

ADD_SUBDIRECTORY(tests)

IF(${BENCHMARK})
  ADD_SUBDIRECTORY(benchmark)
ENDIF(${BENCHMARK})

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
