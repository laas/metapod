// Copyright 2012,
//
// Antonio El Khoury
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

/// Implementation of an algorithm that computes the deepest common
/// body label of two chains linking the root body to start and end
/// bodies respectively.

#ifndef METAPOD_DEEPEST_COMMON_BODY_HH
# define METAPOD_DEEPEST_COMMON_BODY_HH

# include <metapod/tools/common.hh>

namespace metapod
{

  /// \addtogroup tools Tools Algorithms
  ///
  /// This module groups tool algorithms that are used by other
  /// algorithms and that can be used independently.
  ///
  /// \{

  template< typename StartBody, typename EndBody >
  struct deepest_common_body_internal_start;

  template< typename StartBody, typename EndBody >
  struct deepest_common_body_internal_end;

  /// \brief Deepest common body of two sub-chains.
  ///
  /// \tparam StartBody Body type that defines first sub-chain
  /// starting from root body.
  /// \tparam EndBody Body type that defines second sub-chain
  /// starting from root body.
  template< typename StartBody, typename EndBody >
  struct deepest_common_body
  {
    /// \brief Return deepest common body label in tree of the two
    /// chains linking the root to the start and end body
    /// respectively.
    ///
    /// \retval Returned body label.
    static void run(int & label)
    {
      deepest_common_body_internal_start< StartBody, EndBody >::run(label);
    }
  };

  /// \}

  template< typename BI, typename BJ >
  struct deepest_common_body_internal_start
  {
    static void run(int & label)
    {
      // FIXME: Stop condition: avoid if condition and use template
      // specialization instead.
      if (deepest_common_body_internal_end< BI, BJ >::run(label))
        return;

      deepest_common_body_internal_start< typename BI::Parent, BJ >::run(label);
    }
  };

  /// \brief Specialization of deepest_common_body_internal_start.
  template< typename BJ >
  struct deepest_common_body_internal_start< NP, BJ >
  {
    static void run(int){}
  };

  template< typename BI, typename BJ >
  struct deepest_common_body_internal_end
  {
    static bool run(int & label)
    {
      // FIXME: Stop condition: avoid if condition and use template
      // specialization instead.
      if (BI::label == BJ::label)
        {
          label = BI::label;
          return true;
        }

      return deepest_common_body_internal_end< BI,
        typename BJ::Parent >::run(label);
    }
  };

  /// \brief Specialization of deepest_common_body_internal_end: stop
  /// recursion when root of end body chain is reached.
  template< typename BI >
  struct deepest_common_body_internal_end< BI, NP >
  {
    static bool run(int)
    {
      return false;
    }
  };

} // end of namespace metapod.

#endif
