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
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/robotbuilder/include)
INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/robotbuilder/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/timer/include)
INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/timer/include)

# list of the sample models
SET(depth_list 1 2 3 4 )
#SET(depth_list 1 )

# build the generator
ADD_EXECUTABLE(make_model make_model.cc)
TARGET_LINK_LIBRARIES(make_model metapod_robotbuilder)

# build the sample models libraries
FOREACH(depth IN LISTS depth_list)
    SET(_libname "sample_${depth}")
    SET(_sources
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.cc
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/config.hh)
    ADD_CUSTOM_COMMAND(
      OUTPUT ${_sources}
      COMMAND make_model ${depth}
      DEPENDS make_model
    )
    ADD_LIBRARY(${_libname} SHARED ${_sources})
    LIST(APPEND _model_libs "${_libname}")
ENDFOREACH()

# add simple humanoid model
IF(WITH_SIMPLE_HUMANOID)
    SET_SOURCE_FILES_PROPERTIES("benchmark.cc" PROPERTIES
        COMPILE_FLAGS " -DWITH_SIMPLE_HUMANOID")
    LIST(APPEND _model_libs "metapod_simple_humanoid")
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/simple_humanoid/include)
    INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/simple_humanoid/include)
ENDIF()

# and finally, the benchmark
ADD_EXECUTABLE(benchmark benchmark.cc benchmark.hh)
TARGET_LINK_LIBRARIES(benchmark metapod_timer ${_model_libs})

ADD_EXECUTABLE(benchmark_1 benchmark_1.cc benchmark.hh)
TARGET_LINK_LIBRARIES(benchmark_1 metapod_timer ${_model_libs})

ADD_EXECUTABLE(benchmark_2 benchmark_2.cc benchmark.hh)
TARGET_LINK_LIBRARIES(benchmark_2 metapod_timer ${_model_libs})

ADD_EXECUTABLE(benchmark_3 benchmark_3.cc benchmark.hh)
TARGET_LINK_LIBRARIES(benchmark_3 metapod_timer ${_model_libs})

ADD_EXECUTABLE(benchmark_4 benchmark_4.cc benchmark.hh)
TARGET_LINK_LIBRARIES(benchmark_4 metapod_timer ${_model_libs})
