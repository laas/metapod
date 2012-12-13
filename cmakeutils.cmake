# Copyright 2012,
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
# Copyright 2010, 2012,


# GENERATE_CONFIG_HEADER
#
# Generates a configuration header for DLL API import/export
#
# OUTPUT: path (including filename) of the generated file. Usually the filename
#         is config.hh
# LIBRARY_NAME: the name of the library. It will be normalized.
FUNCTION(GENERATE_CONFIG_HEADER OUTPUT LIBRARY_NAME)
  STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_" LIBRARY_NAME "${LIBRARY_NAME}")
  STRING(TOLOWER "${LIBRARY_NAME}" "LIBRARY_NAME_LOWER")
  SET(EXPORT_SYMBOL "${LIBRARY_NAME_LOWER}_EXPORTS")
  STRING(TOUPPER "${LIBRARY_NAME}" "LIBRARY_NAME")
  # create the directory (and its parents)
  GET_FILENAME_COMPONENT(OUTPUT_DIR "${OUTPUT}" PATH)
  FILE(MAKE_DIRECTORY "${OUTPUT_DIR}")
  # Generate the header. The following variables are used in the template
  # LIBRARY_NAME: CPP symbol prefix, should match the compiled library name,
  #               usually in upper case
  # EXPORT_SYMBOL: what symbol controls the switch between symbol
  #                import/export, usually libname_EXPORTS, with libname in
  #                lower case.
  # PROJECT_VERSION: the project version
  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/config.hh.cmake
    ${OUTPUT}
    @ONLY)
ENDFUNCTION(GENERATE_CONFIG_HEADER)
