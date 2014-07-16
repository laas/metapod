// Copyright 2014
//
// Nuno Guedelha (LAAS, CNRS)
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


#ifndef METAPOD_INITNUFWDDYN_HH
# define METAPOD_INITNUFWDDYN_HH

# include <metapod/tools/constants.hh>

namespace metapod {
namespace internal {

  // helper function: updates nu(fd) of current node's base joint depending on parent node's base joint. If node_id parent is part of nu(fd) set, then node_id is also part of nu(fd).

  // common template for valid parent_id
  template <typename Robot, typename Node, int parent_id>
  struct updateNuOfFwdDynFromParentOrLocal
  {
    typedef typename Nodes<Robot, parent_id>::type Parent;
    static const bool value = Node::jointFwdDyn || Parent::jointNuOfFwdDyn;
  };
  
  // If parent_id is NO_PARENT, just init nu(fd) = fd. nu(fd) is not impacted by the parent.
  template <typename Robot, typename Node>
  struct updateNuOfFwdDynFromParentOrLocal<Robot, Node, NO_PARENT>
  {
    static const bool value = Node::jointFwdDyn;
  };

} // end of namespace metapod::internal

  /// init the "nuFwdDyn" parameter for Node's joint
  template< typename Robot, typename Node > struct initNuFwdDyn
  {
    static const bool value = internal::updateNuOfFwdDynFromParentOrLocal<Robot, Node, Node::parent_id>::value;
  };

} // end of namespace metapod

#endif
