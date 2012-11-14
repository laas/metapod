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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/benchmark)

# Returns the list of source files for sample_${dof}_dof model
FUNCTION(GET_SAMPLE_SOURCE_FILES output dof)
    SET("${output}"
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/sample_${dof}_dof.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/sample_${dof}_dof.cc
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/body.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/joint.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/robot.hh
        PARENT_SCOPE)
ENDFUNCTION()

# list of the sample models
SET(dof_list 3 7 15 31 63)

# put together a list of all the generated files
FOREACH(dof IN LISTS dof_list)
    GET_SAMPLE_SOURCE_FILES(_sources ${dof})
    LIST(APPEND _generated_files "${_sources}")
ENDFOREACH()

# build the generator
ADD_EXECUTABLE(make_model make_model.cc) #TODO: add missing sources
TARGET_LINK_LIBRARIES(make_model ${Boost_LIBRARIES})

# declare the generated files
ADD_CUSTOM_COMMAND(
  OUTPUT ${_generated_files}
  COMMAND make_model
  DEPENDS make_model
)

# build the sample models libraries
FOREACH(dof IN LISTS dof_list)
    GET_SAMPLE_SOURCE_FILES(_sources ${dof})
    SET(_target sample_${dof}_dof)
    ADD_LIBRARY(sample_${dof}_dof SHARED ${_sources})
    LIST(APPEND _generated_libs "${_target}")
ENDFOREACH()

# and finally, the benchmark
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
ADD_EXECUTABLE(benchmark benchmark.cc)
TARGET_LINK_LIBRARIES(benchmark ${_generated_libs} simple_humanoid)
