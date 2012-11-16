// Copyright 2012,
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
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
 * Implementation of the joint calculation routine, as described in
 * Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_JCALC_HH
# define METAPOD_JCALC_HH

# include "metapod/tools/common.hh"

namespace metapod
{
  template< typename Tree, typename confVector > struct jcalc_internal;

  template< typename Robot > struct jcalc
  {
    static void run(const typename Robot::confVector & q, const typename Robot::confVector & dq)
    {
      jcalc_internal< typename Robot::Tree, typename Robot::confVector >::run(q, dq);
    }
  };

  template< typename Tree, typename confVector > struct jcalc_internal
  {
    typedef Tree Node;

    static void run(const confVector & q, const confVector & dq)
    {
      Node::Joint::jcalc(
        q.template segment< Node::Joint::NBDOF >(Node::Joint::positionInConf),
        dq.template segment< Node::Joint::NBDOF >(Node::Joint::positionInConf)
      );

      jcalc_internal< typename Node::Child0, confVector >::run(q, dq);
      jcalc_internal< typename Node::Child1, confVector >::run(q, dq);
      jcalc_internal< typename Node::Child2, confVector >::run(q, dq);
      jcalc_internal< typename Node::Child3, confVector >::run(q, dq);
      jcalc_internal< typename Node::Child4, confVector >::run(q, dq);
    }
  };

  template< typename confVector > struct jcalc_internal< NC, confVector >
  {
    static void run(const confVector &, const confVector &) {}
  };

} // end of namespace metapod

# endif
