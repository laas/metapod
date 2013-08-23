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

// Implementation of the Recursive Newton Euler Algorithm,
// based on Featherstone's Rigid Body Dynamics Algorithms.

#ifndef METAPOD_RNEA_HH
# define METAPOD_RNEA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/jcalc.hh"

namespace metapod {

/// Templated Recursive Newton-Euler Algorithm.
/// Takes the multibody tree type as template parameter,
/// and recursively proceeds on the Nodes.

template< typename Robot, bool jcalc = true > struct rnea{};

template< typename Robot > struct rnea< Robot, false >
{
  typedef typename Robot::confVector confVector;

  // update body kinematics using data from parent body and joint
  template< int node_id, int parent_id >
  struct update_kinematics
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    typedef typename Nodes<Robot, parent_id>::type Parent;

    static void run(
        Robot & robot,
        const Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > & ddqi)
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      Parent& parent = boost::fusion::at_c<parent_id>(robot.nodes);
      // iX0 = iXλ(i) * λ(i)X0
      // vi = iXλ(i) * vλ(i) + vj
      // ai = iXλ(i) * aλ(i) + Si * ddqi + cj + vi x vj
      node.body.iX0 = node.sXp * parent.body.iX0;
      node.body.vi = node.sXp * parent.body.vi + node.joint.vj;
      node.body.ai = sum(node.sXp * parent.body.ai,
                         Spatial::Motion(node.joint.S.S() * ddqi),
                         node.joint.cj,
                         (node.body.vi^node.joint.vj));
    }
  };

  // specialization when parent_id == NO_PARENT
  template< int node_id >
  struct update_kinematics<node_id, NO_PARENT>
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    static void run(
        Robot & robot,
        const Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > & ddqi)
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      // iX0 = iXλ(i)
      // vi = vj
      // ai = iXλ(i) * aλ(i) + Si * ddqi + cj + vi x vj
      // (with aλ(i) = a0 = -g, cf. Rigid Body Dynamics Algorithms for a
      // detailed explanation of how the gravity force is applied)
      node.body.iX0 = node.sXp;
      node.body.vi = node.joint.vj;
      node.body.ai = sum((node.body.iX0 * minus_g),
                          Spatial::Motion(node.joint.S.S() * ddqi),
                          node.joint.cj,
                          (node.body.vi^node.joint.vj));
    }

  };
  // update parent force
  template< int node_id, int parent_id >
  struct update_force
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    typedef typename Nodes<Robot, parent_id>::type Parent;

    static void run(Robot & robot)
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      Parent& parent = boost::fusion::at_c<parent_id>(robot.nodes);
      // fλ(i) = fλ(i) + λ(i)Xi* * fi
      parent.joint.f = parent.joint.f + node.sXp.applyInv(node.joint.f);
    }
  };

  // specialization when parent_id == NO_PARENT
  template< int node_id >
  struct update_force<node_id, NO_PARENT>
  {
    static void run(Robot&) {}
  };

  template <typename AnyRobot, int node_id> struct DftVisitor
  {
    typedef typename Nodes<AnyRobot, node_id>::type Node;

    METAPOD_HOT
    static void discover(AnyRobot & robot,
                         const confVector & ,
                         const confVector & ,
                         const confVector & ddq)
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      // Extract subvector corresponding to current Node
      const Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > ddqi =
        ddq.template segment<Node::Joint::NBDOF>(Node::q_idx);

      // delegate the actual computation, because the computation is
      // different when the node has a parent and when it does not.
      update_kinematics<node_id, Node::parent_id>::run(robot, ddqi);

      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      Spatial::Inertia &I = robot.inertias[node_id];
      node.joint.f = sum((I * node.body.ai),
                         (node.body.vi^( I * node.body.vi )),
                         (node.body.iX0 * -node.body.Fext ));
    }

    METAPOD_HOT
    static void finish(AnyRobot & robot,
                       const confVector & ,
                       const confVector & ,
                       const confVector & )
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      // backward computations follow
      // τi = SiT * fi
      node.joint.torque = node.joint.S.S().transpose()
                          * node.joint.f.toVector();
      update_force<node_id, Node::parent_id>::run(robot);
    }

  };

  static void run(Robot & robot,
                  const typename Robot::confVector & q,
                  const typename Robot::confVector & dq,
                  const typename Robot::confVector & ddq)
  {
    depth_first_traversal<DftVisitor, Robot>::run(robot, q, dq, ddq);
  }
};

template< typename Robot > struct rnea< Robot, true >
{
  static void run(Robot & robot,
                  const typename Robot::confVector & q,
                  const typename Robot::confVector & dq,
                  const typename Robot::confVector & ddq)
  {
    jcalc< Robot >::run(robot, q, dq);
    rnea< Robot, false >::run(robot, q, dq, ddq);
  }
};

} // end of namespace metapod

#endif
