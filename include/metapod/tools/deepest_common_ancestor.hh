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

namespace metapod {

template <typename Robot, int node_a_id, int node_b_id>
struct deepest_common_ancestor;

namespace internal {

// implementation used when a_is_ancestor_of_b == false
template <typename Robot, int node_a_id, int node_b_id,
          bool a_is_ancestor_of_b>
struct deepest_common_ancestor_internal {
  // if node A is not an ancestor of b, then
  // deepest_common_ancestor<NodeA, NodeB>
  // == deepest_common_ancestor<NodeA::Parent, NodeB>
  typedef typename Nodes<Robot, node_a_id>::type NodeA;
  static const int value =
      deepest_common_ancestor<Robot, NodeA::parent_id, node_b_id>::value;
};

template <typename Robot, int node_a_id, int node_b_id>
struct deepest_common_ancestor_internal<Robot, node_a_id, node_b_id, true> {
  static const int value = node_a_id;
};
} // end of namespace metapod::internal

/// \brief Deepest common ancestor node of two nodes A and B. The member
//  typedef "type" is the Body type of the deepest common ancestor.
///
/// \tparam Robot
/// \tparam node A id
/// \tparam node B id
template <typename Robot, int node_a_id, int node_b_id>
struct deepest_common_ancestor {
  static const int value = internal::deepest_common_ancestor_internal<
      Robot, node_a_id, node_b_id,
      is_ancestor<Robot, node_a_id, node_b_id>::value
      >::value;
};
} // end of namespace metapod

#endif
