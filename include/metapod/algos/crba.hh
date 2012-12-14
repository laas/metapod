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
 * Implementation of the Composite Rigid Body Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_CRBA_HH
# define METAPOD_CRBA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/backward_traversal_prev.hh"

namespace metapod
{
  // helper function: update Parent inertia with the contribution of child Node
  template < typename Parent, typename Node >
  struct update_parent_inertia
  {
    static void run()
    {
      Parent::Iic = Parent::Iic + Node::Joint::sXp.applyInv(Node::Body::Iic);
    }
  };
  // Do nothin if parent is NP
  template < typename Node >
  struct update_parent_inertia<NP, Node>
  {
    static void run() {};
  };

  // frontend
  template< typename Robot, bool jcalc = true > struct crba {};
  template< typename Robot > struct crba<Robot, false>
  {
    template <typename Node >
    struct DftVisitor
    {
      typedef typename Node::Body BI;
      typedef Node NI;

      // Update NJ with data from PrevNJ
      template< typename NJ, typename PrevNJ >
      struct BwdtVisitor
      {
        static void discover()
        {
          NI::Joint::F = PrevNJ::Joint::sXp.mulMatrixTransposeBy(NI::Joint::F);
          Robot::H.template
            block< NI::Joint::NBDOF, NJ::Joint::NBDOF >
                 ( NI::Joint::positionInConf, NJ::Joint::positionInConf )
            = NI::Joint::F.transpose() * NJ::Joint::S.S();
          Robot::H.template
            block< NJ::Joint::NBDOF, NI::Joint::NBDOF >
                 ( NJ::Joint::positionInConf, NI::Joint::positionInConf )
            = Robot::H.template
                block< NI::Joint::NBDOF, NJ::Joint::NBDOF >
                     ( NI::Joint::positionInConf, NJ::Joint::positionInConf ).transpose();
        }

        static void finish() {}
      };

      // forward propagation
      static void discover()
      {
        Node::Body::Iic = Node::Body::I;
      }

      static void finish()
      {
        update_parent_inertia<typename Node::Body::Parent, Node>::run();
        Node::Body::Joint::F = Node::Body::Iic * Node::Joint::S;

        Robot::H.template block<Node::Joint::NBDOF, Node::Joint::NBDOF>(
                Node::Joint::positionInConf, Node::Joint::positionInConf)
                         = Node::Joint::S.transpose() * Node::Body::Joint::F;
        backward_traversal_prev< BwdtVisitor, Robot, BI >::run();
      }
    };

    static void run(const typename Robot::confVector & )
    {
      depth_first_traversal< DftVisitor, Robot >::run();
    }
    static void run()
    {
      depth_first_traversal< DftVisitor, Robot >::run();
    }
  };

  // frontend
  template< typename Robot > struct crba< Robot, true >
  {
    static void run(const typename Robot::confVector & q)
    {
      jcalc< Robot >::run(q, Robot::confVector::Zero());
      crba< Robot, false >::run();
    }
  };

} // end of namespace metapod

#endif
