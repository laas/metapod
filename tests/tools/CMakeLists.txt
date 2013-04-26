# Copyright 2012, 2013
#
# Olivier STASSE (LAAS/CNRS)
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
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/timer/include)
INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/timer/include)

ADD_TESTCASE(test_timer test_timer.cc)
TARGET_LINK_LIBRARIES(test_timer metapod_timer)

ADD_TESTCASE(test_eigenalign test_eigenalign.cc)
