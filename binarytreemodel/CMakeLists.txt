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

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/robotbuilder/include
    ${PROJECT_BINARY_DIR}/robotbuilder/include)

SET(METAPODFROMURDF_EXECUTABLE ${METAPODFROMURDF_EXECUTABLE} CACHE
  STRING "Set by cmake if metapodfromurdf is built, by the user otherwise")

# build the generator
ADD_EXECUTABLE(metapod_binarytreemodel src/binarytreemodel.cc)
TARGET_LINK_LIBRARIES(metapod_binarytreemodel metapod_robotbuilder)

SET(METAPOD_BINARYTREEMODEL_EXECUTABLE metapod_binarytreemodel
  CACHE STRING
  "Set by cmake if metapod_binarytreemodel is built, by the user otherwise")

INSTALL(PROGRAMS
  ${PROJECT_BINARY_DIR}/metapod_binarytreemodel/metapod_binarytreemodel
  DESTINATION bin)
