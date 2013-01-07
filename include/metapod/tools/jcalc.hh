// Copyright 2012, 2013
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

# include <metapod/tools/depth_first_traversal.hh>

namespace metapod {
namespace internal {

template< typename Robot, int node_id > struct JcalcVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void discover(Robot& robot,
                       const typename Robot::confVector& q,
                       const typename Robot::confVector& dq)
  {
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    node.joint.jcalc(
      q.template segment< Node::Joint::NBDOF >(Node::q_idx),
      dq.template segment< Node::Joint::NBDOF >(Node::q_idx));
    node.sXp = node.joint.Xj * node.Xt;
  }
  static void finish(Robot&,
                     const typename Robot::confVector&,
                     const typename Robot::confVector&)
  {}
};

} // end of namespace metapod::internal

template< typename Robot > struct jcalc
{
  static void run(Robot& robot,
                  const typename Robot::confVector& q,
                  const typename Robot::confVector& dq)
  {
    depth_first_traversal<internal::JcalcVisitor, Robot>::run(robot, q, dq);
  }
};

} // end of namespace metapod

# endif
