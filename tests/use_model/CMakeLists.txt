# Copyright 2012, 2013
#
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

# let tests find the model model headers
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/${CURRENT_MODEL}/include)

ADD_DEFINITIONS(" -DCURRENT_MODEL_ROBOT=${CURRENT_MODEL}")
ADD_DEFINITIONS(
   -DTEST_DIRECTORY="${CMAKE_SOURCE_DIR}/tests/data/${CURRENT_MODEL}")
STRING(TOUPPER "${CURRENT_MODEL}" CURRENT_MODEL_UPPER)
ADD_DEFINITIONS(" -DCURRENT_MODEL_IS_${CURRENT_MODEL_UPPER}=1")

# ADD_MODEL_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'_`CURRENT_MODEL'.
#
# This macro will
#
# * create a test from `NAME'.cc and `common.hh'
# * link it against th model lib
#
# The test should #include common.hh
#
MACRO(ADD_MODEL_TESTCASE NAME)
  SET(NEW_NAME ${NAME}_${CURRENT_MODEL})
  ADD_TESTCASE(${NEW_NAME} ${NAME}.cc common.hh)
  TARGET_LINK_LIBRARIES(${NEW_NAME} metapod_${CURRENT_MODEL})
ENDMACRO(ADD_MODEL_TESTCASE)

ADD_MODEL_TESTCASE(test_rnea)
ADD_MODEL_TESTCASE(test_crba)
ADD_MODEL_TESTCASE(test_bcalc)
ADD_MODEL_TESTCASE(test_jac)
ADD_MODEL_TESTCASE(test_jac_point)
IF(${CURRENT_MODEL} STREQUAL "simple_humanoid")
  ADD_MODEL_TESTCASE(test_jac_point_chain)
ENDIF()
ADD_MODEL_TESTCASE(test_printstate)
ADD_MODEL_TESTCASE(test_initconf)
ADD_MODEL_TESTCASE(test_depth_first_traversal)
ADD_MODEL_TESTCASE(test_backward_traversal)
ADD_MODEL_TESTCASE(test_backward_traversal_prev)
