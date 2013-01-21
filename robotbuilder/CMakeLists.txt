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

# helper function
# EMBED_TEMPLATE(OUTPUT TEMPLATE NAME)
#
# TEMPLATE: filename of the template to be embedded
# NAME:     name of the template, will be used for the generated filename and
#           the variable holding the
# OUTPUT:   name of a variable in the caller scope, the absolute path of the
#           generated file will be appended to it.
FUNCTION(EMBED_TEMPLATE OUTPUT TEMPLATE NAME)
  ADD_CUSTOM_COMMAND(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.c
    COMMAND embedfile ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.c
            ${CMAKE_CURRENT_SOURCE_DIR}/tpl/${TEMPLATE} ${NAME}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tpl/${TEMPLATE} embedfile
    )
  SET(${OUTPUT} "${${OUTPUT}};${CMAKE_CURRENT_BINARY_DIR}/${NAME}.c" PARENT_SCOPE)
ENDFUNCTION()

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/include)

GENERATE_CONFIG_HEADER(
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/robotbuilder/config.hh
  metapod_robotbuilder
  )
SET(METAPOD_ROBOTBUILDER_HEADERS
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/robotbuilder/config.hh
  include/metapod/robotbuilder/robotbuilder.hh
  )
EMBED_TEMPLATE(EMBEDDED_FILES "config.in.hh" "config_hh")
EMBED_TEMPLATE(EMBEDDED_FILES "body.in.hh" "body_hh")
EMBED_TEMPLATE(EMBEDDED_FILES "joint.in.hh" "joint_hh")
EMBED_TEMPLATE(EMBEDDED_FILES "robot.in.hh" "robot_hh")
EMBED_TEMPLATE(EMBEDDED_FILES "init.in.hh" "init_hh")
EMBED_TEMPLATE(EMBEDDED_FILES "init.in.cc" "init_cc")

INSTALL(FILES ${METAPOD_ROBOTBUILDER_MODEL_HEADERS}
  DESTINATION include/metapod/robotbuilder
  )
ADD_LIBRARY(metapod_robotbuilder SHARED
  src/robotbuilder.cc
  src/robotbuilder_p.hh
  src/robotbuilder_p.cc
  ${EMBEDDED_FILES}
  ${METAPOD_ROBOTBUILDER_HEADERS}
  )
TARGET_LINK_LIBRARIES(metapod_robotbuilder ${Boost_LIBRARIES})
