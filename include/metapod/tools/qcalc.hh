// Copyright 2012, 2013
//
// Nuno Guedelha (JRL/LAAS, CNRS/AIST)
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
 * Implementation of the permutation matrix Q, as described in
 * Featherstone's Rigid Body Dynamics Algorithms.
 * Each line of Q gives the position index of the next joint in the "fd" mode.
 * Explore every joint of the system tree :
 * - for each "fd" joint, add a line in the FD matrix with its respective position
 * - for each "id" joint, add a line in the ID matrix with its respective position.
 * cocatenate matrix Q:
 *   => Q = |FD|
 *          |ID|
 */

#ifndef METAPOD_QCALC_HH
# define METAPOD_QCALC_HH

# include <metapod/tools/depth_first_traversal.hh>
# include <cassert>

namespace metapod {

namespace internal {
  
  typedef enum
  {
    ADD = 0,
    BUILD
  } Qoperation;
  
  static int fdNodesFirstFillIndex = 0;
  static int idNodesFillIndex = 0;
  
  template< typename Robot, int q_idx, int nbdof, Qoperation operation, bool fwdDyn >
  struct HandleJointToQmatrix {};
  
  template< typename Robot, int q_idx, int nbdof >
  struct HandleJointToQmatrix<Robot, q_idx, nbdof, ADD, true>
  {
    static void run()
    {
      // Add q_idx index to FD vector in case of a "fd" mode joint
      Robot::fdNodesFirst(0, fdNodesFirstFillIndex) = q_idx;
      fdNodesFirstFillIndex++;
      HandleJointToQmatrix<Robot, q_idx+1, nbdof-1, ADD, true>::run();
    }
  };
  
  template< typename Robot, int q_idx, int nbdof >
  struct HandleJointToQmatrix<Robot, q_idx, nbdof, ADD, false>
  {
    static void run()
    {
      // Add q_idx index to ID vector in case of an "id" mode joint
      Robot::idNodes(0, idNodesFillIndex) = q_idx;
      idNodesFillIndex++;
      HandleJointToQmatrix<Robot, q_idx+1, nbdof-1, ADD, false>::run();
    }
  };
    
  template< typename Robot, int q_idx >
  struct HandleJointToQmatrix<Robot, q_idx, 0, ADD, false>
  {
    static void run() {}
  };
  
  template< typename Robot, int q_idx >
  struct HandleJointToQmatrix<Robot, q_idx, 0, ADD, true>
  {
    static void run() {}
  };
  
  template< typename Robot, int q_idx, int nbdof, bool fwdDyn >
  struct HandleJointToQmatrix<Robot, q_idx, nbdof, BUILD, fwdDyn>
  {
    static void run()
    {
      // concatenate both lists
      Robot::fdNodesFirst.tail(idNodesFillIndex) = Robot::idNodes.head(idNodesFillIndex);
      // get permutation matrix Q from q_idx list
      Robot::Qt = Eigen::PermutationMatrix<Robot::NBDOF, Robot::NBDOF, typename Robot::RobotFloatType>(Robot::fdNodesFirst);
      Robot::Q = Robot::Qt.transpose();
    }
  };
  
  
  template< typename Robot, int node_id > struct QcalcVisitor
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    
    static void discover()
    {
      typedef typename Nodes<Robot, node_id>::type Node;
      // add q_idx to FD or ID permutation matrix depending on the joint mode
      HandleJointToQmatrix<Robot, Node::q_idx, Node::Joint::NBDOF, ADD, Node::jointFwdDyn>::run();
    }
    static void finish()
    {}
  };

} // end of namespace metapod::internal

  template< typename Robot > struct qcalc
  {
    static void run()
    {
      depth_first_traversal<internal::QcalcVisitor, Robot>::run();
      internal::HandleJointToQmatrix<Robot, 0, 0, internal::BUILD, true>::run();
    }
  };

} // end of namespace metapod

# endif
