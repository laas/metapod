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


#ifndef METAPOD_INITCONF_HH
# define METAPOD_INITCONF_HH

# include <metapod/tools/depth_first_traversal.hh>

namespace metapod {
namespace internal {

// Look for a string in a std::ofstream
void findString(std::string s_, std::ifstream & is)
{
  is.clear(); is.seekg(0);
  std::string s;
  while(is >> s)
  {
    if(!s.compare(s_))
      break;
  }
}

template <typename Robot, int node_id>
struct InitConfHybridParamsVisitor
{
  typedef typename boost::fusion::result_of::value_at_c<typename Robot::NodeVector, node_id>::type Node;
  static const int NB_DOF = Node::Joint::NBDOF;

  static void discover(std::ifstream &log, typename Robot::confVector &v, const bool &isTorque)
  {
    findString(Node::joint_name, log);
    if( (isTorque && Node::jointFwdDyn) || (!isTorque && !Node::jointFwdDyn) )
    {
      for(int i=0; i<NB_DOF; ++i)
	log >> v[Node::q_idx+i];
    }
    else
    {
      // copy "invalid" configuration to output conf Vector (v[q_idx] to v[q_idx+NB_DOF-1])
      v.template segment<NB_DOF>(Node::q_idx) = Eigen::Matrix<typename Robot::RobotFloatType, NB_DOF, 1>::Constant(-1.1111111);
    }
  }
  static void discover(typename Robot::confVector &log, typename Robot::confVector &v, const bool &isTorque)
  {
    if( (isTorque && Node::jointFwdDyn) || (!isTorque && !Node::jointFwdDyn) )
    {
      // copy log[q_idx..q_idx+NB_DOF-1] to output conf Vector segment v[q_idx..q_idx+NB_DOF-1]
      v.template segment<NB_DOF>(Node::q_idx) = log.template segment<NB_DOF>(Node::q_idx);
    }
    else
    {
      // copy "invalid" configuration to output conf Vector segment v[q_idx..q_idx+NB_DOF-1]
      v.template segment<NB_DOF>(Node::q_idx) = Eigen::Matrix<typename Robot::RobotFloatType, NB_DOF, 1>::Constant(-1.1111111);
    }
  }

  static void finish(std::ifstream &, typename Robot::confVector &, const bool &) {}
  static void finish(typename Robot::confVector &, typename Robot::confVector &, const bool &) {}
};

template <typename Robot, int node_id>
struct InitConfVisitor
{
  static void discover(std::ifstream &log, typename Robot::confVector &v)
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    findString(Node::joint_name, log);
    const int NB_DOF = boost::fusion::result_of::
      value_at_c<typename Robot::NodeVector, node_id>::type::
      Joint::NBDOF;
    for(int i=0; i<NB_DOF; ++i)
      log >> v[Node::q_idx+i];
  }
  static void discover(typename Robot::confVector &log, typename Robot::confVector &v)
  {
    v = log;
  }

  static void finish(std::ifstream &, typename Robot::confVector &) {}
  static void finish(typename Robot::confVector &, typename Robot::confVector &) {}
};

} // end of namespace metapod::internal

typedef enum
{
  NOT_HYBRID = 0,
  HYBRID_DDQ = 1,
  HYBRID_TORQUES = 2
} ConfType;

/// init a configuration vector with values from text file, formatted
/// as printed by the printConf routine.
template< typename Robot, ConfType confVectorType=NOT_HYBRID, typename ConfTypeStreamOrVector=std::ifstream > struct initConf {};

template< typename Robot, typename ConfTypeStreamOrVector > struct initConf<Robot, NOT_HYBRID, ConfTypeStreamOrVector>
{
  static void run(ConfTypeStreamOrVector & log, typename Robot::confVector & v)
  {
    depth_first_traversal< internal::InitConfVisitor, Robot >::run(log, v);
  }
};

template< typename Robot, typename ConfTypeStreamOrVector > struct initConf<Robot, HYBRID_DDQ, ConfTypeStreamOrVector>
{
  static void run(ConfTypeStreamOrVector & log, typename Robot::confVector & v)
  {
    const bool isTorque = false;
    depth_first_traversal< internal::InitConfHybridParamsVisitor, Robot >::run(log, v, isTorque);
  }
};

template< typename Robot, typename ConfTypeStreamOrVector > struct initConf<Robot, HYBRID_TORQUES, ConfTypeStreamOrVector>
{
  static void run(ConfTypeStreamOrVector & log, typename Robot::confVector & v)
  {
    const bool isTorque = true;
    depth_first_traversal< internal::InitConfHybridParamsVisitor, Robot >::run(log, v, isTorque);
  }
};

} // end of namespace metapod

#endif
