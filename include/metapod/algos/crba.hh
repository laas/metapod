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
 * Implementation of the Composite Rigid Body Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_CRBA_HH
# define METAPOD_CRBA_HH

# include "metapod/tools/common.hh"

namespace metapod
{
  template< typename Robot, typename Tree > struct crba_forward_propagation;

  template< typename Robot, typename BI, typename BJ, typename Parent > 
  struct crba_backward_propagation;

  template< typename Robot, bool jcalc = true > struct crba {};

  template< typename Robot > struct crba< Robot, false >
  {
    typedef Eigen::Matrix< FloatType, Robot::NBDOF, 1 > confVector;

    static void run(const confVector & q)
    {
      crba_forward_propagation< Robot, typename Robot::Tree >::run();
    }
  };

  template< typename Robot > struct crba< Robot, true >
  {
    typedef Eigen::Matrix< FloatType, Robot::NBDOF, 1 > confVector;

    static void run(const confVector & q)
    {
      jcalc< Robot >::run(q, confVector::Zero());
      crba_forward_propagation< Robot, typename Robot::Tree >::run();
    }
  };

  template < typename Robot, typename Tree >
  struct crba_forward_propagation
  {
    typedef Tree Node;
    typedef typename Node::Body BI;

    static void run()
    {
      Node::Body::Iic = Node::Body::I;

      crba_forward_propagation< Robot, typename Node::Child0 >::run();
      crba_forward_propagation< Robot, typename Node::Child1 >::run();
      crba_forward_propagation< Robot, typename Node::Child2 >::run();
      crba_forward_propagation< Robot, typename Node::Child3 >::run();
      crba_forward_propagation< Robot, typename Node::Child4 >::run();

      if(Node::Body::HAS_PARENT)
        Node::Body::Parent::Iic = Node::Body::Parent::Iic
                                + Node::Joint::sXp.applyInv(Node::Body::Iic);

      Node::Body::Joint::F = Node::Body::Iic.toMatrix() * Node::Joint::S;

      Robot::H.template block<Node::Joint::NBDOF, Node::Joint::NBDOF>(
              Node::Joint::positionInConf, Node::Joint::positionInConf)
                       = Node::Joint::S.transpose() * Node::Body::Joint::F;

      crba_backward_propagation< Robot, BI, BI, typename BI::Parent >::run();
    }
  };

  template< typename Robot >
  struct crba_forward_propagation< Robot, NC >
  {
    static void run() {}
  };

  template< typename Robot, typename BI, typename BJ, typename Parent >
  struct crba_backward_propagation
  {
    typedef BI Body_i;
    typedef BJ Body_j;
    typedef typename Body_i::Joint Joint_i;
    typedef typename Body_j::Joint Joint_j;

    static void run()
    {
      Joint_i::F = Joint_j::sXp.toMatrixTranspose() * Joint_i::F;

      Robot::H.template
        block< Joint_i::NBDOF, Parent::Joint::NBDOF >
             ( Joint_i::positionInConf, Parent::Joint::positionInConf )
        = Joint_i::F.transpose() * Parent::Joint::S;
      Robot::H.template
        block< Parent::Joint::NBDOF, Joint_i::NBDOF >
             ( Parent::Joint::positionInConf, Joint_i::positionInConf )
        = Robot::H.block( Joint_i::positionInConf,
                          Parent::Joint::positionInConf,
                          Joint_i::NBDOF,
                          Parent::Joint::NBDOF ).transpose();
      crba_backward_propagation< Robot,
                                 BI,
                                 Parent,
                                 typename Parent::Parent >::run();
    }
  };

  template< typename Robot, typename BI, typename BJ >
  struct crba_backward_propagation< Robot, BI, BJ, NP >
  {
    static void run() {}
  };

} // end of namespace metapod

#endif
