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
struct InitConfVisitor
{
  static void discover(std::ifstream & log, typename Robot::confVector &v)
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    findString(Node::joint_name, log);
    const int NB_DOF = boost::fusion::result_of::value_at_c<typename Robot::NodeVector, node_id>::type::Joint::NBDOF;
    for(int i=0; i<NB_DOF; ++i)
      log >> v[Node::q_idx+i];
  }
  static void finish(std::ifstream &, typename Robot::confVector &) {}
};

} // end of namespace metapod::internal

/// init a configuration vector with values from text file, formatted
/// as printed by the printConf routine.
template< typename Robot > struct initConf
{
  static void run(std::ifstream & log, typename Robot::confVector & v)
  {
    depth_first_traversal< internal::InitConfVisitor, Robot >::run(log, v);
  }
};

} // end of namespace metapod

#endif
