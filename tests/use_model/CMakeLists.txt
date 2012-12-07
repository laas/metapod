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

ADD_DEFINITIONS(" -DCURRENT_MODEL_NAMESPACE=metapod::${CURRENT_MODEL}")
ADD_DEFINITIONS(
   -DTEST_DIRECTORY="${CMAKE_SOURCE_DIR}/tests/data/${CURRENT_MODEL}")
STRING(TOUPPER "${CURRENT_MODEL}" CURRENT_MODEL_UPPER)
ADD_DEFINITIONS(" -DCURRENT_MODEL_IS_${CURRENT_MODEL_UPPER}=1")

ADD_TESTCASE(test_rnea_${CURRENT_MODEL} test_rnea.cc)
ADD_TESTCASE(test_crba_${CURRENT_MODEL} test_crba.cc)
ADD_TESTCASE(test_bcalc_${CURRENT_MODEL} test_bcalc.cc)
ADD_TESTCASE(test_jac_point_${CURRENT_MODEL} test_jac_point.cc)
ADD_TESTCASE(test_jac_point_chain_${CURRENT_MODEL} test_jac_point_chain.cc)
ADD_TESTCASE(test_printstate_${CURRENT_MODEL} test_printstate.cc)
ADD_TESTCASE(test_initconf_${CURRENT_MODEL} test_initconf.cc)
