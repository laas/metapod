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

/// Implementation of an algorithm that computes the deepest common
/// ancestor of two nodes.

#ifndef METAPOD_DEEPEST_COMMON_ANCESTOR_HH
# define METAPOD_DEEPEST_COMMON_ANCESTOR_HH

# include <metapod/tools/common.hh>
# include <metapod/tools/is_ancestor.hh>

namespace metapod
{
  /// \brief Deepest common ancestor node of two nodes A and B. The member
  //  typedef "type" is the Body type of the deepest common ancestor.
  ///
  /// \tparam NodeA Body type
  /// \tparam NodeB Body type
  template < typename NodeA, typename NodeB>
  struct deepest_common_ancestor;

  // implementation used when A_is_ancestor_of_B == false
  template < typename NodeA, typename NodeB, bool A_is_ancestor_of_B >
  struct deepest_common_ancestor_internal
  {
    // if NodeA is not the deepest_common_ancestor
    // deepest_common_ancestor<NodeA, NodeB>
    // == deepest_common_ancestor<NodeA::Parent, NodeB>
    typedef typename
        deepest_common_ancestor<typename NodeA::Parent, NodeB>::type type;
  };

  template < typename NodeA, typename NodeB>
  struct deepest_common_ancestor_internal<NodeA, NodeB, true>
  {
    typedef NodeA type;
  };

  template < typename NodeA, typename NodeB>
  struct deepest_common_ancestor
  {
    typedef typename deepest_common_ancestor_internal<NodeA, NodeB,
        is_ancestor<NodeA, NodeB>::value >::type type;
  };
} // end of namespace metapod.

#endif
