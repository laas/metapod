// Copyright 2012,
//
// Maxime Reis
//
// JRL/LAAS, CNRS/AIST
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
 * Implementation of the body calculation routine.
 * It updates the body global transform (iX0) for the current set of joint
 * transforms.
 */

#ifndef METAPOD_BCALC_HH
# define METAPOD_BCALC_HH

# include "metapod/tools/common.hh"

namespace metapod
{
  template< typename Tree, typename confVector, bool hasParent = false >
  struct bcalc_internal;

  template< typename Robot > struct bcalc
  {
    typedef Eigen::Matrix< FloatType, Robot::NBDOF, 1 > confVector;

    static void run(const confVector & q)
    {
      bcalc_internal< typename Robot::Tree, confVector >::run(q);
    }
  };

  template< typename Tree, typename confVector >
  struct bcalc_internal< Tree, confVector, false >
  {
    typedef Tree Node;

    static void run(const confVector & q)
    {
      Node::Body::iX0 = Node::Joint::sXp;
      bcalc_internal< typename Node::Child1, confVector, true >::run(q);
      bcalc_internal< typename Node::Child2, confVector, true >::run(q);
      bcalc_internal< typename Node::Child3, confVector, true >::run(q);
    }
  };

  template< typename Tree, typename confVector >
  struct bcalc_internal< Tree, confVector, true >
  {
    typedef Tree Node;

    static void run(const confVector & q)
    {
      Node::Body::iX0 = Node::Joint::sXp * Node::Body::Parent::iX0;
      bcalc_internal< typename Node::Child1, confVector, true >::run(q);
      bcalc_internal< typename Node::Child2, confVector, true >::run(q);
      bcalc_internal< typename Node::Child3, confVector, true >::run(q);
    }
  };

  template< typename confVector > struct bcalc_internal< NC, confVector, false >
  {
    static void run(const confVector &) {}
  };

  template< typename confVector > struct bcalc_internal< NC, confVector, true >
  {
    static void run(const confVector &) {}
  };
} // end of namespace metapod

# endif
