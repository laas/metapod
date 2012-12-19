# Copyright 2011, 2012, 2013
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

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/timer/include)
INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/timer)

# Returns the list of source files for sample_${dof}_dof model
FUNCTION(GET_SAMPLE_SOURCE_FILES output dof include_config_hh)
    SET(_libname "sample_${dof}_dof")
    SET(_output
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.cc
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/body.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/joint.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/robot.hh)
    IF (include_config_hh)
        SET(_config_hh_path
            ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/config.hh)
        GENERATE_CONFIG_HEADER(${_config_hh_path} ${_libname})
        LIST(APPEND _output ${_config_hh_path})
    ENDIF()
    SET("${output}" ${_output} PARENT_SCOPE)
ENDFUNCTION()

# list of the sample models
SET(dof_list 3 7 15 31 63)

# put together a list of all the generated files
FOREACH(dof IN LISTS dof_list)
    GET_SAMPLE_SOURCE_FILES(_sources ${dof} False)
    LIST(APPEND _generated_files "${_sources}")
ENDFOREACH()

# build the generator
ADD_EXECUTABLE(make_model make_model.cc make_model.hh) #TODO: add missing sources
TARGET_LINK_LIBRARIES(make_model ${Boost_LIBRARIES})

# declare the generated files
IF(dof_list)
    ADD_CUSTOM_COMMAND(
      OUTPUT ${_generated_files}
      COMMAND make_model
    )
ENDIF()

# build the sample models libraries
FOREACH(dof IN LISTS dof_list)
    GET_SAMPLE_SOURCE_FILES(_sources ${dof} True)
    SET(_target sample_${dof}_dof)
    ADD_LIBRARY(sample_${dof}_dof SHARED ${_sources})
    LIST(APPEND _model_libs "${_target}")
ENDFOREACH()

# add simple humanoid model
IF(WITH_SIMPLE_HUMANOID)
    SET_SOURCE_FILES_PROPERTIES("benchmark.cc" PROPERTIES
        COMPILE_FLAGS " -DWITH_SIMPLE_HUMANOID")
    LIST(APPEND _model_libs "metapod_simple_humanoid")
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/simple_humanoid/include)
    INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/simple_humanoid)
ENDIF()
# and finally, the benchmark
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
ADD_EXECUTABLE(benchmark benchmark.cc benchmark.hh)
TARGET_LINK_LIBRARIES(benchmark metapod_timer ${_model_libs})
