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
 * Implementation of the Composite Rigid Body Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_CRBA_HH
# define METAPOD_CRBA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/jcalc.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/backward_traversal_prev.hh"

namespace metapod {
namespace internal {

// helper function: update Parent inertia with the contribution of child Node
template < typename Robot, int parent_id, int node_id >
struct crba_update_parent_inertia
{
  typedef typename Nodes<Robot, parent_id>::type Parent;
  typedef typename Nodes<Robot, node_id>::type Node;
  static void run(Robot& robot)
  {
    Parent& parent = boost::fusion::at_c<parent_id>(robot.nodes);
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    parent.body.Iic = parent.body.Iic + node.sXp.applyInv(node.body.Iic);
  }
};
// Do nothing if parent_id is NO_PARENT
template < typename Robot, int node_id >
struct crba_update_parent_inertia<Robot, NO_PARENT, node_id>
{
  static void run(Robot&) {}
};

} // end of namespace metapod::internal

// frontend
template< typename Robot, bool jcalc = true > struct crba {};
template< typename Robot > struct crba<Robot, false>
{
  template <typename AnyRobot, int node_id >
  struct DftVisitor
  {
    typedef typename Nodes<Robot, node_id>::type NI;
    typedef NI Node;
    // Update NJ with data from PrevNJ
    template< typename AnyyRobot, int nj_id, int prev_nj_id >
    struct BwdtVisitor
    {
      typedef typename Nodes<AnyyRobot, nj_id>::type NJ;
      typedef typename Nodes<AnyyRobot, prev_nj_id>::type PrevNJ;
      static void discover(AnyyRobot& robot)
      {
        NI& ni = boost::fusion::at_c<node_id>(robot.nodes);
        NJ& nj = boost::fusion::at_c<nj_id>(robot.nodes);
        PrevNJ& prev_nj = boost::fusion::at_c<prev_nj_id>(robot.nodes);
        ni.joint_F = prev_nj.sXp.mulMatrixTransposeBy(ni.joint_F);
        robot.H.template
          block< NI::Joint::NBDOF, NJ::Joint::NBDOF >
               ( NI::q_idx, NJ::q_idx )
          = ni.joint_F.transpose() * nj.joint.S.S();
        robot.H.template
          block< NJ::Joint::NBDOF, NI::Joint::NBDOF >
               ( NJ::q_idx, NI::q_idx )
          = robot.H.template
              block< NI::Joint::NBDOF, NJ::Joint::NBDOF >
                   ( NI::q_idx, NJ::q_idx ).transpose();
      }

      static void finish(AnyyRobot&) {}
    };

    // forward propagation
    static void discover(AnyRobot& robot)
    {
      NI& ni = boost::fusion::at_c<node_id>(robot.nodes);
      ni.body.Iic = robot.inertias[node_id];
    }

    static void finish(AnyRobot& robot)
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      internal::crba_update_parent_inertia<AnyRobot, Node::parent_id, node_id>::run(robot);
      node.joint_F = node.body.Iic * node.joint.S;

      robot.H.template block<Node::Joint::NBDOF, Node::Joint::NBDOF>(
              Node::q_idx, Node::q_idx)
                       = node.joint.S.transpose() * node.joint_F;
      backward_traversal_prev< BwdtVisitor, Robot, node_id >::run(robot);
    }
  };

  static void run(Robot& robot, const typename Robot::confVector& )
  {
    depth_first_traversal< DftVisitor, Robot >::run(robot);
  }
  static void run(Robot& robot)
  {
    depth_first_traversal< DftVisitor, Robot >::run(robot);
  }
};

// frontend
template< typename Robot > struct crba< Robot, true >
{
  static void run(Robot& robot, const typename Robot::confVector& q)
  {
    jcalc< Robot >::run(robot, q, Robot::confVector::Zero());
    crba< Robot, false >::run(robot);
  }
};

} // end of namespace metapod

#endif
