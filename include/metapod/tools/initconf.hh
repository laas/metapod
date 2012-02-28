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
 * This file contains a method setting a configuration vector from a file.
 */

#ifndef metapod_INITCONF_HH
# define metapod_INITCONF_HH

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
  
  // Return vector constructed from log file (as printed by the printConf method)
  template< typename Tree >
  void initConf(std::ifstream & log, vectorN & v)
  {
    typedef Tree Node;
    findString(Node::Joint::name, log);
    for(int i=0; i<Node::Joint::NBDOF; i++)
      log >> v[Node::Joint::positionInConf+i];
    if(Node::Child1::isNode)
    {
      initConf<typename Node::Child1>(log,v);
      if(Node::Child2::isNode)
      {
        initConf<typename Node::Child2>(log,v);
        if(Node::Child3::isNode)
          initConf<typename Node::Child3>(log,v);
      }
    }
  }
  
  template<>
  void initConf<NC>(std::ifstream &, vectorN &){}

} // end of namespace metapod.

#endif
