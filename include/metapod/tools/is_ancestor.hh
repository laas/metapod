// Copyright 2012,
//
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

/// Implementation of an algorithm that checks whether a node is an ancestor
/// of another one or not.

#ifndef METAPOD_IS_ANCESTOR_HH
# define METAPOD_IS_ANCESTOR_HH

# include <metapod/tools/common.hh>

namespace metapod
{

  /// \brief  value member variable is true if NodeA is an ancestor of NodeB,
  /// false otherwise. A node is its own ancestor (NP also is its own ancestor).
  template < typename Robot, int node_a_id, int node_b_id >
  struct is_ancestor
  {
    typedef typename Nodes<Robot, node_b_id>::type NodeB;
    static const bool value =
        is_ancestor< Robot, node_a_id, NodeB::parent_id >::value;
  };

  // end of recursion: a node is its own ancestor
  template < typename Robot, int node_a_id >
  struct is_ancestor< Robot, node_a_id, node_a_id >
  {
    static const bool value = true;
  };

  // end of recursion: NO_PARENT has no ancestor (except itself)
  template < typename Robot, int node_a_id >
  struct is_ancestor< Robot, node_a_id, NO_PARENT >
  {
    static const bool value = false;
  };

  // end of recursion: NO_PARENT is its own ancestor
  template < typename Robot >
  struct is_ancestor< Robot, NO_PARENT, NO_PARENT >
  {
    static const bool value = true;
  };
} // end of namespace metapod.

#endif
