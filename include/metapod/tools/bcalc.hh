// Copyright 2012, 2013
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Antonio El Khoury (JRL/LAAS, CNRS/AIST)
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
 * Implementation of the body calculation routine.
 * It updates the body global transform (iX0) for the current set of joint
 * transforms.
 */

#ifndef METAPOD_BCALC_HH
# define METAPOD_BCALC_HH

#include <metapod/tools/depth_first_traversal.hh>
#include <metapod/tools/has_parent.hh>

namespace metapod {
namespace internal {

template< typename Robot, int node_id, bool has_parent=true >
struct UpdateBodyAbsolutePose
{
  typedef typename Nodes<Robot, node_id>::type Node;
  typedef typename Nodes<Robot, Node::parent_id>::type Parent;
  static void run(Robot& robot)
  {
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    Parent& parent = boost::fusion::at_c<Node::parent_id>(robot.nodes);
    node.body.iX0 = node.sXp * parent.body.iX0;
  }
};

// specialization for root nodes
template< typename Robot, int node_id>
struct UpdateBodyAbsolutePose<Robot, node_id, false>
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void run(Robot& robot)
  {
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    node.body.iX0 = node.sXp;
  }
};

template< typename Robot, int node_id >
struct BcalcVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void discover(Robot& robot, const typename Robot::confVector& q)
  {
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    node.joint.bcalc(q.template segment< Node::Joint::NBDOF >(Node::q_idx));
    node.sXp = node.joint.Xj * node.Xt;
    UpdateBodyAbsolutePose<Robot, node_id, has_parent<Robot, node_id>::value>::run(robot);
  }
  static void finish(Robot&, const typename Robot::confVector&)
  {}
};
} // end of namespace metapod::internal

template< typename Robot > struct bcalc
{
  static void run(Robot& robot, const typename Robot::confVector& q)
  {
    depth_first_traversal<internal::BcalcVisitor, Robot>::run(robot, q);
  }
};
} // end of namespace metapod

# endif
