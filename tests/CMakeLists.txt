# Copyright 2011, 2012, 
#
# Maxime Reis
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

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Specify the tests data files directory.
ADD_DEFINITIONS(-DTEST_DIRECTORY="${CMAKE_SOURCE_DIR}/tests/data")

# Set METAPOD_PERF_TEST flag if required
IF(${PERF_TEST})
  ADD_DEFINITIONS(-DMETAPOD_PERF_TEST)
ENDIF(${PERF_TEST})

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  # Link against Boost.
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES})
  PKG_CONFIG_USE_DEPENDENCY(${NAME} ginac)
ENDMACRO(ADD_TESTCASE)

# Generated test.
ADD_TESTCASE(test_ginac)
