// Copyright 2013
//
// Sébastien Barthélémy (Aldebaran Robotics)
//
// This file is part of metapod.
// metapod is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// metapod is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.

/*
 * This test creates a model using the new operator.
 *
 * The test may abort (in debug mode) because of eigen  asserts:
 *
 * http://eigen.tuxfamily.org/dox/TopicUnalignedArrayAssert.html
 *
 */

#include "common.hh"
#include <boost/scoped_ptr.hpp>

using namespace metapod;

BOOST_AUTO_TEST_CASE (test_operator_new)
{
  boost::scoped_ptr<CURRENT_MODEL_ROBOT> robot(new CURRENT_MODEL_ROBOT);
  BOOST_CHECK(robot);
}
