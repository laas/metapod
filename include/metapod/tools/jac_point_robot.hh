// Copyright 2012,
//
// Antonio El Khoury (JRL/LAAS, CNRS/AIST)
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

#ifndef METAPOD_JAC_POINT_ROBOT_HH
# define METAPOD_JAC_POINT_ROBOT_HH

# include "metapod/tools/common.hh"
# include "metapod/algos/jac_point_relative.hh"

namespace metapod
{
  /// \addtogroup jac_point_robot Point Articular Jacobian Test Algorithm
  ///
  /// Compute point articular jacobian for all robot bodies by calling
  /// recursively jac_point. This algorithm is not optimal and is not
  /// meant to be used except for testing purposes.
  ///
  /// \{

  template< typename Robot, typename Tree, bool bcalc = true >
  struct jac_point_robot_internal;

  /// \brief Point articular jacobian algorithm.
  ///
  /// \tparam Robot Robot type for which jacobian for all bodies is
  /// computed.
  ///
  /// \tparam bcalc Boolean type to specify whether all body
  /// transforms need to be updated with respect to the robot
  /// configuration.
  template< typename Robot, bool bcalc = true >
  struct jac_point_robot
  {
    typedef Eigen::Matrix< FloatType, 6*Robot::NBBODIES, Robot::NBDOF,
                           Eigen::RowMajor >
    jacobian_t;

    static void run(const typename Robot::confVector & q,
                    jacobian_t & J)
    {
      J.setZero ();

      // Call internal jacobian routine.
      jac_point_robot_internal< Robot, typename Robot::Tree, bcalc >::run(q, J);
    }
  };

  /// \}
  ///

  /// \brief Internal point articular jacobian algorithm routine.
  ///
  /// \sa jac_point_robot
  template< typename Robot, typename Tree, bool bcalc >
  struct jac_point_robot_internal
  {
    typedef typename jac_point_robot< Robot, bcalc>::jacobian_t
        robotJacobian_t;
    typedef jac_point_relative< Robot, NP, typename Tree::Body,
            0, bcalc> solver;
    typedef typename solver::jacobian_t bodyJacobian_t;

    typedef Tree Node;

    static void run(const typename Robot::confVector & q,
                    robotJacobian_t & J)
    {
      // Compute jacobian sub-block.
      bodyJacobian_t subJ = bodyJacobian_t::Zero();
      solver::run(q, Vector3d(0,0,0), subJ);
      J.template block<6,Robot::NBDOF>(6*Node::Body::label, 0) = subJ;

      // recursion on children
      jac_point_robot_internal< Robot,
        typename Node::Child0, bcalc >::run(q, J);
      jac_point_robot_internal< Robot,
        typename Node::Child1, bcalc >::run(q, J);
      jac_point_robot_internal< Robot,
        typename Node::Child2, bcalc >::run(q, J);
      jac_point_robot_internal< Robot,
        typename Node::Child3, bcalc >::run(q, J);
      jac_point_robot_internal< Robot,
        typename Node::Child4, bcalc >::run(q, J);
    }
  };

  /// \brief Specialization of jac_point_robot: Stop recursion on tree
  /// leaves.
  template< typename Robot, bool bcalc >
  struct jac_point_robot_internal< Robot, NC, bcalc>
  {
    typedef typename jac_point_robot< Robot, bcalc>::jacobian_t
        robotJacobian_t;

    static void run(const typename Robot::confVector &,
                    robotJacobian_t &) {}
  };

} // end of namespace metapod.

#endif
