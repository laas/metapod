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

SET(benchmark_cc_definitions "")

IF(WITH_METAPOD_BINARYTREEMODEL)
    SET(depth_list 1 2 3 4)
    FOREACH(depth IN LISTS depth_list)
        SET(_libname "sample_${depth}")
        # build the sample models libraries
        SET(_sources
            ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.hh
            ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.cc
            ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/config.hh)
        ADD_CUSTOM_COMMAND(
            OUTPUT ${_sources}
            COMMAND ${METAPOD_BINARYTREEMODEL_EXECUTABLE} ${depth}
            DEPENDS ${METAPOD_BINARYTREEMODEL_EXECUTABLE})
        ADD_LIBRARY(${_libname} SHARED ${_sources})

        # add the library to a list in order to bench it together with the
        # others
        LIST(APPEND _model_libs "${_libname}")

        # build a benchmark which tests only this model
        IF(BUILD_SINGLE_MODEL_BENCHMARKS)
            SET(_benchname "benchmark_${_libname}")
            CONFIGURE_FILE(
                ${CMAKE_CURRENT_SOURCE_DIR}/benchmark.in.cc
                ${CMAKE_CURRENT_BINARY_DIR}/${_benchname}.cc
                @ONLY)
            ADD_EXECUTABLE(benchmark_${_libname}
                ${CMAKE_CURRENT_BINARY_DIR}/${_benchname}.cc
                benchmark.hh)
            TARGET_LINK_LIBRARIES(${_benchname} metapod_timer ${_libname})
         ENDIF()
    ENDFOREACH()
    SET(benchmark_cc_definitions
        "${benchmark_cc_definitions} -DWITH_METAPOD_BINARYTREEMODEL")
ENDIF()

# add simple humanoid model
IF(WITH_SIMPLE_HUMANOID)
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/simple_humanoid/include)
    INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/simple_humanoid/include)
    SET(_libname "metapod_simple_humanoid")
    # add the library to a list in order to bench it together with the other
    SET(benchmark_cc_definitions
        "${benchmark_cc_definitions} -DWITH_SIMPLE_HUMANOID")
    LIST(APPEND _model_libs "${_libname}")
    # benchmark it alone
    IF(BUILD_SINGLE_MODEL_BENCHMARKS)
        SET(_benchname "benchmark_${_libname}")
        ADD_EXECUTABLE(benchmark_${_libname}
            ${_benchname}.cc
            benchmark.hh)
        TARGET_LINK_LIBRARIES(${_benchname} metapod_timer ${_libname})
    ENDIF()
ENDIF()

# and finally, a benchmark which runs all models at once
IF(BUILD_MULTI_MODEL_BENCHMARK AND _model_libs)
    SET_SOURCE_FILES_PROPERTIES("benchmark.cc" PROPERTIES
        COMPILE_FLAGS "${benchmark_cc_definitions}")
    ADD_EXECUTABLE(benchmark benchmark.cc benchmark.hh)
    TARGET_LINK_LIBRARIES(benchmark metapod_timer ${_model_libs})
ENDIF()
