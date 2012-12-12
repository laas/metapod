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

/// Implementation of two algorithms that check whether a node is NP (no
/// parent) or not and if it has a parent or not.

#ifndef METAPOD_HAS_PARENT_HH
# define METAPOD_HAS_PARENT_HH

# include <metapod/tools/common.hh>

namespace metapod
{

  /// \brief  value member variable is true if Node is NP (no parent), and
  /// false otherwise.
  template < typename Node >
  struct is_NP
  {
    static const bool value = false;
  };

  template <>
  struct is_NP<NP>
  {
    static const bool value = true;
  };

  /// \brief value member variable is false if Node has no parent. That is, if
  /// it's parent is NP (no parent) and true otherwise.
  template < typename Node>
  struct has_parent
  {
    static const bool value = !is_NP<typename Node::Body::Parent>::value;
  };
} // end of namespace metapod.

#endif

