// Copyright 2011, 2012,
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
 * Implementation of the Recursive Newton Euler Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_RNEA_HH
# define METAPOD_RNEA_HH

# include "metapod/tools/common.hh"

namespace metapod
{

  /** Templated Recursive Newton-Euler Algorithm.
    * Takes the multibody tree type as template parameter,
    * and recursively proceeds on the Nodes.
    */
  template< typename Tree, typename confVector, bool HasParent = false >
  struct rnea_internal {};

  template< typename Robot, bool jcalc = true > struct rnea{};

  template< typename Robot > struct rnea< Robot, true >
  {
    typedef Eigen::Matrix< FloatType, Robot::NBDOF, 1 > confVector;

    static void run(const confVector & q,
                    const confVector & dq,
                    const confVector & ddq)
    {
      jcalc< Robot >::run(q, dq);
      rnea_internal< typename Robot::Tree, confVector >::run(q, dq, ddq);
    }
  };

  template< typename Robot > struct rnea< Robot, false >
  {
    typedef Eigen::Matrix< FloatType, Robot::NBDOF, 1 > confVector;

    static void run(const confVector & q,
                    const confVector & dq,
                    const confVector & ddq)
    {
      rnea_internal< typename Robot::Tree, confVector >::run(q, dq, ddq);
    }
  };

  template< typename Tree, typename confVector >
  struct rnea_internal< Tree, confVector, true >
  {
    typedef Tree Node;

    static void run(const confVector & q,
                    const confVector & dq,
                    const confVector & ddq) __attribute__ ((hot))
    {
      // Extract subvector corresponding to current Node
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > ddqi =
        ddq.template segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);

      // iX0 = iXλ(i) * λ(i)X0
      // vi = iXλ(i) * vλ(i) + vj
      // ai = iXλ(i) * aλ(i) + Si * ddqi + cj + vi x vj
      Node::Body::iX0 = Node::Joint::sXp*Node::Body::Parent::iX0;
      Node::Body::vi = Node::Joint::sXp*Node::Body::Parent::vi
                     + Node::Joint::vj;
      Node::Body::ai = sum(Node::Joint::sXp*Node::Body::Parent::ai,
                           Spatial::Motion(Node::Joint::S * ddqi),
                           Node::Joint::cj,
                           (Node::Body::vi^Node::Joint::vj));

      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      Node::Joint::f = sum((Node::Body::I * Node::Body::ai),
                           (Node::Body::vi^( Node::Body::I * Node::Body::vi )),
                           (Node::Body::iX0 * -Node::Body::Fext ));

      // recursion on children
      rnea_internal< typename Node::Child0, confVector, true >::run(q, dq, ddq);
      rnea_internal< typename Node::Child1, confVector, true >::run(q, dq, ddq);
      rnea_internal< typename Node::Child2, confVector, true >::run(q, dq, ddq);
      rnea_internal< typename Node::Child3, confVector, true >::run(q, dq, ddq);
      rnea_internal< typename Node::Child4, confVector, true >::run(q, dq, ddq);

      // backward computations follow
      // τi = SiT * fi
      Node::Joint::torque = Node::Joint::S.transpose()*Node::Joint::f.toVector();
      // fλ(i) = fλ(i) + λ(i)Xi* * fi
      Node::Body::Parent::Joint::f = Node::Body::Parent::Joint::f
                                   + Node::Joint::sXp.applyInv(Node::Joint::f);
    }
  };

  template< typename Tree, typename confVector >
  struct rnea_internal< Tree, confVector, false >
  {
    typedef Tree Node;

    static void run(const confVector & q,
                    const confVector & dq,
                    const confVector & ddq)
    {
      // Extract subvector corresponding to current Node
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > ddqi =
        ddq.template segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);

      // iX0 = iXλ(i)
      // vi = vj
      // ai = Si * ddqi + cj + vi x vj (with a0 = -g, cf. Rigid Body Dynamics 
      // Algorithms for a detailed explanation of how the gravity force is
      // applied)
      Node::Body::iX0 = Node::Joint::sXp;
      Node::Body::vi = Node::Joint::vj;
      Node::Body::ai = sum(Spatial::Motion(Node::Joint::S * ddqi),
                           Node::Joint::cj,
                           (Node::Body::vi^Node::Joint::vj),
                           (Node::Body::iX0*minus_g));

      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      Node::Joint::f = sum((Node::Body::I * Node::Body::ai),
                           (Node::Body::vi^( Node::Body::I * Node::Body::vi )),
                           (Node::Body::iX0 * -Node::Body::Fext));


      // recursion on children
      rnea_internal< typename Node::Child0, confVector, true >::run(q, dq, ddq);
      rnea_internal< typename Node::Child1, confVector, true >::run(q, dq, ddq);
      rnea_internal< typename Node::Child2, confVector, true >::run(q, dq, ddq);
      rnea_internal< typename Node::Child3, confVector, true >::run(q, dq, ddq);
      rnea_internal< typename Node::Child4, confVector, true >::run(q, dq, ddq);

      // backward computations follow
      // τi = SiT * fi
      Node::Joint::torque = Node::Joint::S.transpose()
                          * Node::Joint::f.toVector();
    }
  };

  /**
    \brief  Specialization, to stop recursion on leaves of the Tree
  */
  template< typename confVector > struct rnea_internal< NC, confVector, true >
  {
    static void run(const confVector &,
                    const confVector &,
                    const confVector &) {}
  };

} // end of namespace metapod.

#endif
