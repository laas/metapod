// Copyright 2011, 2012, 2013
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
 * This file contains print methods.
 */

#ifndef METAPOD_PRINT_HH
# define METAPOD_PRINT_HH

#include <metapod/tools/depth_first_traversal.hh>

namespace metapod {

// Print state of the robot in a stream.
namespace internal {
template < typename Robot, int node_id > struct PrintStateVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void discover(const Robot & robot, std::ostream & os)
  {
    const Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    os << Node::body_name << " :\n"
       << "sXp :\n" << node.sXp << "\n"
       << "Xt :\n" << Node::Xt << "\n"
       << "Xj :\n" << node.joint.Xj << "\n"
       << "S :\n" << node.joint.S.S() << "\n"
       << "iX0 :\n" << node.body.iX0 << "\n"
       << "vi :\n" << node.body.vi << "\n"
       << "ai :\n" << node.body.ai << "\n"
       << "I :\n" << robot.inertias[node_id] << "\n"
       << "f :\n" << node.joint.f << "\n"
       << "τ :\n" << node.joint.torque << "\n"
       << std::endl;
  }
  static void finish(const Robot &, std::ostream & ) {}
};
} // end of namespace metapod::internal.

template< typename Robot >
void printState(const Robot & robot, std::ostream & os)
{
  depth_first_traversal<internal::PrintStateVisitor, Robot>::run(robot, os);
}

// Print conf vector of the robot in a stream.
namespace internal {
template < typename Robot, int node_id > struct PrintConfVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void discover(const VectorN& q, std::ostream& os)
  {
    VectorN qi = q.segment<Node::Joint::NBDOF>(Node::q_idx);
    os << Node::joint_name << "\n" << qi << "\n";
  }
  static void finish(const VectorN &, std::ostream & ) {}
};
} // end of namespace internal.

template< typename Robot >
void printConf(const VectorN & q, std::ostream & os)
{
  depth_first_traversal<internal::PrintConfVisitor, Robot>::run(q, os);
}

// Print Transforms of the robot bodies in a stream.
namespace internal {
template < typename Robot, int node_id> struct PrintTransformsVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void discover(const Robot& robot, std::ostream & os)
  {
    const Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    os << Node::body_name << "\n"
       << node.body.iX0.E() << "\n"
       << node.body.iX0.r().transpose() << "\n"
       << std::endl;
  }
  static void finish(const Robot&, std::ostream & ) {}
};
} // end of namespace metapod::internal.

template< typename Robot >
void printTransforms(const Robot& robot, std::ostream & os)
{
  depth_first_traversal<internal::PrintTransformsVisitor, Robot>::run(robot, os);
}

// Print Torques of the robot in a stream.
namespace internal {
template < typename Robot, int node_id> struct PrintTorquesVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;

  static void discover(const Robot& robot, std::ostream& os)
  {
    const Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    os << Node::joint_name << "\n"
       << node.joint.torque << "\n"
       << std::endl;
  }
  static void finish(const Robot&, std::ostream & ) {}
};
} // end of namespace metapod::internal.

template< typename Robot >
void printTorques(Robot& robot, std::ostream & os)
{
  depth_first_traversal<internal::PrintTorquesVisitor, Robot>::run(robot, os);
}

// save the Torques of the robot in a vector.
namespace internal {
template < typename Robot, int node_id> struct GetTorquesVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;

  static void discover(const Robot& robot, typename Robot::confVector& v)
  {
    const Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    v.template
    segment<Node::Joint::NBDOF>(Node::q_idx) = node.joint.torque;
  }
  static void finish(const Robot&, typename Robot::confVector&) {}
};
} // end of namespace metapod::internal.

template< typename Robot >
void getTorques(Robot& robot, typename Robot::confVector& v)
{
  depth_first_traversal<internal::GetTorquesVisitor, Robot>::run(robot, v);
}
} // end of namespace metapod

#endif
