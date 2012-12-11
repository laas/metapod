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
  template < typename NodeA, typename NodeB>
  struct is_ancestor
  {
    static const bool value =
        is_ancestor< NodeA, typename NodeB::Parent>::value;
  };

  // end of recursion: a node is its own ancestor
  template < typename NodeA>
  struct is_ancestor< NodeA, NodeA>
  {
    static const bool value = true;
  };

  // end of recursion: NP has no ancestor (except itself)
  template < typename NodeA>
  struct is_ancestor< NodeA, NP>
  {
    static const bool value = false;
  };

  // end of recursion: NP is its own ancestor
  template <>
  struct is_ancestor< NP, NP>
  {
    static const bool value = true;
  };
} // end of namespace metapod.

#endif
