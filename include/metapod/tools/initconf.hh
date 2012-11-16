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
 * This file contains a method setting a configuration vector from a file.
 */

#ifndef METAPOD_INITCONF_HH
# define METAPOD_INITCONF_HH

# include "common.hh"

namespace metapod
{

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

  // Return vector constructed from log file,
  // as printed by the printConf method.
  template< typename Tree, typename confVector > struct initConf_internal;

  template< typename Robot > struct initConf
  {
    static void run(std::ifstream & log, typename Robot::confVector & v)
    {
      initConf_internal< typename Robot::Tree, typename Robot::confVector >::run(log, v);
    }
  };

  template< typename Tree, typename confVector > struct initConf_internal
  {
    static void run(std::ifstream & log, confVector & v)
    {
      typedef Tree Node;
      findString(Node::Joint::name, log);
      for(int i=0; i<Node::Joint::NBDOF; i++)
        log >> v[Node::Joint::positionInConf+i];
      initConf_internal<typename Node::Child0, confVector >::run(log,v);
      initConf_internal<typename Node::Child1, confVector >::run(log,v);
      initConf_internal<typename Node::Child2, confVector >::run(log,v);
      initConf_internal<typename Node::Child3, confVector >::run(log,v);
      initConf_internal<typename Node::Child4, confVector >::run(log,v);
    }
  };

  template< typename confVector > struct initConf_internal< NC, confVector >
  {
    static void run(std::ifstream &, confVector &){}
  };

} // end of namespace metapod.

#endif
