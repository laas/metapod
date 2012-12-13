// Copyright 2011, 2012,
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
 * Implementation of the Recursive Newton Euler Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_RNEA_HH
# define METAPOD_RNEA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/depth_first_traversal.hh"

namespace metapod
{

  /** Templated Recursive Newton-Euler Algorithm.
    * Takes the multibody tree type as template parameter,
    * and recursively proceeds on the Nodes.
    */
  template< typename Tree, typename confVector, bool HasParent = false >
  struct rnea_internal {};

  template< typename Robot, bool jcalc = true > struct rnea{};

  template< typename Robot > struct rnea< Robot, false >
  {
    // update body kinematics from parent body and joint
    template< typename Node, typename ParentBody >
    struct update_kinematics
    {
      static void run(
          const Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > & ddqi)
      {
        // iX0 = iXλ(i) * λ(i)X0
        // vi = iXλ(i) * vλ(i) + vj
        // ai = iXλ(i) * aλ(i) + Si * ddqi + cj + vi x vj
        Node::Body::iX0 = Node::Joint::sXp * ParentBody::iX0;
        Node::Body::vi = Node::Joint::sXp * ParentBody::vi
                         + Node::Joint::vj;
        Node::Body::ai = sum(Node::Joint::sXp * ParentBody::ai,
                             Spatial::Motion(Node::Joint::S.S() * ddqi),
                             Node::Joint::cj,
                             (Node::Body::vi^Node::Joint::vj));
      }
    };

    // specialization when Parent == NP
    template< typename Node >
    struct update_kinematics<Node, NP>
    {
      static void run(
          const Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > & ddqi)
      {
        // iX0 = iXλ(i)
        // vi = vj
        // ai = iXλ(i) * aλ(i) + Si * ddqi + cj + vi x vj
        // (with aλ(i) = a0 = -g, cf. Rigid Body Dynamics Algorithms for a
        // detailed explanation of how the gravity force is applied)
        Node::Body::iX0 = Node::Joint::sXp;
        Node::Body::vi = Node::Joint::vj;
        Node::Body::ai = sum((Node::Body::iX0 * minus_g),
                             Spatial::Motion(Node::Joint::S.S() * ddqi),
                             Node::Joint::cj,
                             (Node::Body::vi^Node::Joint::vj));
      }

    };
    // update parent force
    template< typename Node, typename Parent>
    struct update_force
    {
      static void run()
      {
        // fλ(i) = fλ(i) + λ(i)Xi* * fi
        Parent::Joint::f = Parent::Joint::f
                         + Node::Joint::sXp.applyInv(Node::Joint::f);
      }
    };

    // specialization when Parent == NP
    template< typename Node >
    struct update_force<Node, NP>
    {
      static void run() {}
    };

    template <typename Node> struct DftVisitor
    {
      typedef typename Robot::confVector confVector;

      METAPOD_HOT
      static void discover(const confVector & ,
                           const confVector & ,
                           const confVector & ddq)
      {
        // Extract subvector corresponding to current Node
        const Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > ddqi =
          ddq.template segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);

        // delegate the actual computation, because the computation is
        // different when the node has a parent and when it does not.
        update_kinematics<Node, typename Node::Body::Parent>::run(ddqi);

        // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
        Node::Joint::f = sum((Node::Body::I * Node::Body::ai),
                             (Node::Body::vi^( Node::Body::I * Node::Body::vi )),
                             (Node::Body::iX0 * -Node::Body::Fext ));
      }

      METAPOD_HOT
      static void finish(const confVector & ,
                         const confVector & ,
                         const confVector & )
      {
        // backward computations follow
        // τi = SiT * fi
        Node::Joint::torque = Node::Joint::S.S().transpose()
                              * Node::Joint::f.toVector();
        update_force<Node, typename Node::Body::Parent>::run();
      }

    };
    static void run(const typename Robot::confVector & q,
                    const typename Robot::confVector & dq,
                    const typename Robot::confVector & ddq)
    {
      depth_first_traversal<DftVisitor, Robot>::run(q, dq, ddq);
    }
  };

  template< typename Robot > struct rnea< Robot, true >
  {
    static void run(const typename Robot::confVector & q,
                    const typename Robot::confVector & dq,
                    const typename Robot::confVector & ddq)
    {
      jcalc< Robot >::run(q, dq);
      rnea< Robot, false >::run(q, dq, ddq);
    }
  };
} // end of namespace metapod.

#endif
