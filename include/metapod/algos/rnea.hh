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
  template< typename Tree, bool HasParent = false > struct rnea {};

  template< typename Tree > struct rnea< Tree, true >
  {
    typedef Tree Node;

    static void run(const vectorN & q,
                    const vectorN & dq,
                    const vectorN & ddq) __attribute__ ((hot))
    {
      // Extract subvector corresponding to current Node
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > qi =
        q.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > dqi =
        dq.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > ddqi =
        ddq.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);

      /* forward computations follow */
      // Jcalc: update sXp, S, dotS, cj, vj
      Node::Joint::jcalc(qi, dqi);

      // iX0 = iXλ(i) * λ(i)X0
      // vi = iXλ(i) * vλ(i) + vj
      // ai = iXλ(i) * aλ(i) + Si * ddqi + cj + vi x vj
      Node::Body::iX0 = Node::Joint::sXp*Node::Body::Parent::iX0;
      Node::Body::vi = Node::Joint::sXp*Node::Body::Parent::vi
                     + Node::Joint::vj;
      Node::Body::ai = sum(Node::Joint::sXp*Node::Body::Parent::ai,
                           Motion(Node::Joint::S * ddqi),
                           Node::Joint::cj,
                           (Node::Body::vi^Node::Joint::vj));

      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      vector3d global_CoM = Node::Body::iX0*Node::Body::CoM;

      vector3d gravity_force = vector3d(0, 0, GRAVITY_CST);
      vector3d gravity_torque = global_CoM.cross(gravity_force);

      Force Fext = Force(gravity_torque,gravity_force);

      Node::Joint::f = sum((Node::Body::I * Node::Body::ai),
                           (Node::Body::vi^( Node::Body::I * Node::Body::vi )),
                           (Node::Body::iX0 * ( Node::Body::mass * Fext
                                              - Node::Body::Fext )));

      // recursion on children
      rnea< typename Node::Child1, true >::run(q, dq, ddq);
      rnea< typename Node::Child2, true >::run(q, dq, ddq);
      rnea< typename Node::Child3, true >::run(q, dq, ddq);

      // backward computations follow
      // τi = SiT * fi
      Node::Joint::torque = Node::Joint::S.transpose()*Node::Joint::f.toVector();
      // fλ(i) = fλ(i) + λ(i)Xi* * fi
      Node::Body::Parent::Joint::f = Node::Body::Parent::Joint::f
                                   + Node::Joint::sXp.applyInv(Node::Joint::f);
    }
  };

  template< typename Tree > struct rnea< Tree, false >
  {
    typedef Tree Node;

    static void run(const vectorN & q,
                    const vectorN & dq,
                    const vectorN & ddq)
    {
      // Extract subvector corresponding to current Node
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > qi =
        q.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > dqi =
        dq.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > ddqi =
        ddq.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);

      /* forward computations follow */
      // Jcalc: update sXp, S, dotS, cj, vj
      Node::Joint::jcalc(qi, dqi);


      // iX0 = iXλ(i)
      // vi = vj
      // ai = Si * ddqi + cj + vi x vj
      Node::Body::iX0 = Node::Joint::sXp;
      Node::Body::vi = Node::Joint::vj;
      Node::Body::ai = sum(Motion(Node::Joint::S * ddqi),
                           Node::Joint::cj,
                           (Node::Body::vi^Node::Joint::vj));

      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      vector3d global_CoM = Node::Body::iX0*Node::Body::CoM;

      vector3d gravity_force = vector3d(0, 0, GRAVITY_CST);
      vector3d gravity_torque = global_CoM.cross(gravity_force);

      Force Fext = Force(gravity_torque,gravity_force);

      Node::Joint::f = sum((Node::Body::I * Node::Body::ai),
                           (Node::Body::vi^( Node::Body::I * Node::Body::vi )),
                           (Node::Body::iX0 * ( Node::Body::mass * Fext
                                              - Node::Body::Fext )));

      // recursion on children
      rnea< typename Node::Child1, true >::run(q, dq, ddq);
      rnea< typename Node::Child2, true >::run(q, dq, ddq);
      rnea< typename Node::Child3, true >::run(q, dq, ddq);

      // backward computations follow
      // τi = SiT * fi
      Node::Joint::torque = Node::Joint::S.transpose()
                          * Node::Joint::f.toVector();
    }
  };

  /**
    \brief  Specialization, to stop recursion on leaves of the Tree
  */
  template<> struct rnea< NC, true >
  {
    static void run(const vectorN &,
                    const vectorN &,
                    const vectorN &) {}
  };

} // end of namespace metapod.

#endif
