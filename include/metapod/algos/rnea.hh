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
 * Implementation of the Newton Euler Algorithm, based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef metapod_RNEA_HH
# define metapod_RNEA_HH

# include "metapod/tools/common.hh"

namespace metapod
{

  /** Templated Recursive Newton-Euler Algorithm. Takes the multibody tree type as template parameter, and recursively proceeds on the Nodes. */
  template< typename Tree >
  void rnea(const vectorN & q, const vectorN & dq, const vectorN & ddq)
  {
    typedef Tree Node;
  
    // Extract subvector corresponding to current Node
    // TODO: See if fixed-size vector can be extracted using q.segment<SIZE>(INDEX) without -Wcast-qual flag giving off warning
    vectorN qi = q.segment(Node::Joint::positionInConf,Node::Joint::NBDOF);
    vectorN dqi = dq.segment(Node::Joint::positionInConf,Node::Joint::NBDOF);
    vectorN ddqi = ddq.segment(Node::Joint::positionInConf,Node::Joint::NBDOF);
  
    /* forward computations follow */
    // Jcalc: update sXp, S, dotS, cj, vj
    Node::Joint::jcalc(qi, dqi);
  
    if(Node::Body::HAS_PARENT)
    {
      // iX0 = iXλ(i) * λ(i)X0
      // vi = iXλ(i) * vλ(i) + vj
      // ai = iXλ(i) * aλ(i) + Si * ddqi + cj + vi x vj
      Node::Body::iX0 = Node::Joint::sXp * Node::Body::Parent::iX0;
      Node::Body::vi = Node::Joint::sXp * Node::Body::Parent::vi + Node::Joint::vj;
      {
        vector6d tmp1 = Node::Joint::S * ddqi;
        vector6d tmp2 = Node::Body::vi^Node::Joint::vj;
        tmp1 = tmp1 + Node::Joint::cj + tmp2;
        Node::Body::ai = Node::Joint::sXp * Node::Body::Parent::ai + tmp1;
      }
    }
    else
    {
      // iX0 = iXλ(i)
      // vi = vj
      // ai = Si * ddqi + cj + vi x vj
      Node::Body::iX0 = Node::Joint::sXp;
      Node::Body::vi = Node::Joint::vj;
      {
        vector6d tmp1 = Node::Joint::S * ddqi;
        vector6d tmp2 = Node::Body::vi^Node::Joint::vj;
        tmp1 = tmp1 + Node::Joint::cj + tmp2;
        Node::Body::ai = tmp1;
      }
    }
  
    {
      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      vector3d global_CoM = Node::Body::iX0.R().transpose()*Node::Body::CoM + Node::Body::iX0.p();
    
      vector3d gravity_force, gravity_torque;
      gravity_force[0] = 0;
      gravity_force[1] = 0;
      gravity_force[2] = GRAVITY_CST;
      gravity_torque[0] = global_CoM[1]*gravity_force[2] - global_CoM[2]*gravity_force[1];
      gravity_torque[1] = global_CoM[2]*gravity_force[0] - global_CoM[0]*gravity_force[2];
      gravity_torque[2] = global_CoM[0]*gravity_force[1] + global_CoM[1]*gravity_force[0];
    
      Spatial::Force Fext = Force(gravity_force,gravity_torque);
    
      Momentum tmp1 = Node::Body::I * Node::Body::vi;
      Force tmp2 = Node::Body::vi ^ tmp1;
      Force tmp3 = Node::Body::I * Node::Body::ai + tmp2;
      tmp2 = Fext * Node::Body::mass - Node::Body::Fext;
      Force tmp4 = Node::Body::iX0 * tmp2;
      Node::Joint::f = tmp3 + tmp4;
    }
  
    // recursion on children
    if(Node::Child1::isNode)
    {
      rnea<typename Node::Child1>(q, dq, ddq);
      if(Node::Child2::isNode)
      {
        rnea<typename Node::Child2>(q, dq, ddq);
        if(Node::Child3::isNode)
          rnea<typename Node::Child3>(q, dq, ddq);
      }
    }
  
    // backward computations follow
    // τi = SiT * fi
    {
      vector3d fi = Node::Joint::f.f();
      vector3d ni = Node::Joint::f.n0();
      vector6d tmp;
      for(int k=0; k<3; k++)
      {
        tmp(k) = fi(k);
        tmp(k+3) = ni(k);
      }
        Node::Joint::torque = Node::Joint::S.transpose() * tmp;
    }
    // fλ(i) = fλ(i) + λ(i)Xi* * fi
    {
      PluckerTransformTranspose tmp1 = PluckerTransformTranspose(Node::Joint::sXp);
      Force tmp2 = tmp1 * Node::Joint::f;
      if(Node::Body::HAS_PARENT)
        Node::Body::Parent::Joint::f = Node::Body::Parent::Joint::f + tmp2;
    }
  }
  
  /**
    \brief  Specialization, to stop recursion on leaves of the Tree
  */
  template<>
  inline void rnea<NC>(const vectorN &, const vectorN &, const vectorN &){}
  
} // end of namespace metapod.

#endif
