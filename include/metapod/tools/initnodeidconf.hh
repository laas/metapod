// Copyright 2014
//
// Nuno Guedelha (CNRS/LAAS)
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

// this file implements 2 structures to be used only for testing. "initNodeIdConf" extracts, into a configuration vector, 
// the node IDs as per node id to q_idx mapping from the current robot model. "initNodeIdConfReordRef" extracts the node IDs,
// but sorting first the node IDs directly supported by a joint in forward dynamics (jointFwdDyn = true). Sorting is done as 
// per kinematic tree depth first traversal order.

#ifndef METAPOD_INITNODEIDCONF_HH
# define METAPOD_INITNODEIDCONF_HH

# include <metapod/tools/depth_first_traversal.hh>

namespace metapod {
namespace internal {

template <typename Robot, int node_id>
struct InitNodeIdConfVisitor
{
  static void discover(typename Robot::confVector &v)
  {
    typedef typename boost::fusion::result_of::value_at_c<typename Robot::NodeVector, node_id>::type Node;
    const typename Robot::RobotFloatType nodeid_jointFwdDyn = node_id + (Node::jointFwdDyn? 0.1 : 0);
    for(int i=0; i<Node::Joint::NBDOF; ++i)
      v[Node::q_idx+i] = nodeid_jointFwdDyn;
  }
  static void finish(typename Robot::confVector &) {}
};

template <typename Robot, int node_id>
struct InitNodeIdConfReordRefVisitor
{
  static void discover(typename Robot::confVector &v, bool jointFwdDyn, int &v_idx)
  {
    typedef typename boost::fusion::result_of::value_at_c<typename Robot::NodeVector, node_id>::type Node;
    if(Node::jointFwdDyn == jointFwdDyn)
    {
      const typename Robot::RobotFloatType nodeid_jointFwdDyn = node_id + (Node::jointFwdDyn? 0.1 : 0);
      for(int i=0; i<Node::Joint::NBDOF; ++i)
        v[v_idx+i] = nodeid_jointFwdDyn;
      v_idx += Node::Joint::NBDOF;
    }
  }
  static void finish(typename Robot::confVector &, bool, int &) {}
};

} // end of namespace metapod::internal

/// \brief Extracts the robot model node IDs into a configuration vector, as per node id to q_idx mapping.
/// Each element of the output column vector "v" is formatted as follows:
/// (RobotFloatType) v[v_idx] = <node_id>.1 if jointFwdDyn=true
///                  v[v_idx] = <node_id>   if jointFwdDyn=false
/// the choice of RobotFloatType type is for compatibility with the permutation matrix Q type.
template< typename Robot > struct initNodeIdConf
{
  static void run(typename Robot::confVector & v)
  {
    depth_first_traversal< internal::InitNodeIdConfVisitor, Robot >::run(v);
  }
};

/// \brief Extracts the robot model node IDs into a configuration vector. It sorts first the node IDs directly 
/// supported by a joint in forward dynamics (jointFwdDyn = true). Sorting is done as per kinematic tree depth 
/// first traversal order. Non forward dynamics joint nodes are sorted last.
template< typename Robot > struct initNodeIdConfReordRef
{
  static void run(typename Robot::confVector & v)
  {
    int v_idx = 0;
    bool jointFwdDyn = true;
    depth_first_traversal< internal::InitNodeIdConfReordRefVisitor, Robot >::run(v, jointFwdDyn, v_idx);
    jointFwdDyn = false;
    depth_first_traversal< internal::InitNodeIdConfReordRefVisitor, Robot >::run(v, jointFwdDyn, v_idx);
  }
};


} // end of namespace metapod

#endif
