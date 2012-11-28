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

#ifndef METAPOD_JAC_POINT_HH
# define METAPOD_JAC_POINT_HH

# include "metapod/tools/common.hh"

namespace metapod
{

  /// \addtogroup jac_point Point Articular Jacobian Algorithm
  ///
  /// Compute articular jacobian J of a point (in body coordinates)
  /// attached to a body. This jacobian is such that v = J*dq is the
  /// point frame spatial motion vector in world coordinates.
  ///
  /// \{

  template< typename Robot, typename Body >
  struct jac_point_internal;

  /// \brief Point articular jacobian algorithm.
  ///
  /// \tparam Robot Robot type for which jacobian is computed.
  /// \tparam Body Body type for which jacobian is computed.
  /// \tparam bcalc Boolean type to specify whether all body
  /// transforms need to be updated with respect to the robot
  /// configuration.
  ///
  /// \note Use specializations jac_point< Robot, Body, true > and
  /// jac_point< Robot, Body, false >.
  template< typename Robot, typename Body, bool bcalc = true >
  struct jac_point {};

  /// \brief Specialization of jac_point: Update all body transforms
  /// with respect to configuration vector.
  template< typename Robot, typename Body >
  struct jac_point< Robot, Body, true >
  {
    typedef Eigen::Matrix< FloatType, 6, Robot::NBDOF > jacobian_t;

    /// \brief Compute the articular jacobian J.
    ///
    /// \param q Configuration vector: it is used to update all body
    /// spatial transforms if bcalc is equal to true.
    ///
    /// \param b_p Coordinates of point in Body coordinates.
    /// \retval J Computed jacobian of size 6xRobot::NBDOF.
    static void run(const typename Robot::confVector & q,
                    const vector3d & b_p,
                    jacobian_t & J)
    {
      // Reset jacobian.
      J.setZero ();

      // Update body transformations.
      bcalc< Robot >::run(q);

      // Compute point coordinates in world frame.
      vector3d p = Body::iX0.applyInv(b_p);

      // Call internal jacobian routine.
      jac_point_internal< Robot, Body >::run(p, J);
    }
  };

  /// \brief Specialization of jac_point: Do not update body
  /// transforms with respect to configuration vector.
  template< typename Robot, typename Body >
  struct jac_point< Robot, Body, false >
  {
    typedef Eigen::Matrix< FloatType, 6, Robot::NBDOF >
    jacobian_t;

    /// \brief Compute the articular jacobian J.
    ///
    /// \sa jac_point< Robot, Body, true >::run().
    static void run(const typename Robot::confVector & q,
                    const vector3d & b_p,
                    jacobian_t & J)
    {
      // Reset jacobian.
      J.setZero ();

      // Compute point coordinates in world frame.
      vector3d p = Body::iX0.applyInv(b_p);

      // Call internal jacobian routine.
      jac_point_internal< Robot, Body >::run(p, J);
    }
  };

  /// \}

  /// \brief Internal point articular jacobian algorithm routine.
  ///
  /// \sa jac_point
  template< typename Robot, typename Body >
  struct jac_point_internal
  {
    typedef typename Body::Joint Joint;
    typedef typename jac_point< Robot, Body >::jacobian_t
    jacobian_t;

    static void run(const vector3d & p,
                    jacobian_t & J) __attribute__ ((hot))
    {
      // Compute jacobian block for current node. Formula is given by:
      // Ji = pX0 * (iX0)^(-1) * Si,
      // where pX0 is the word transform in the point frame,
      // iX0 is the world transform in the ith body frame,
      // Si is the ith joint motion subspace matrix.
      J.template
        block<6,Joint::NBDOF>(0,Joint::positionInConf) =
        Joint::applyToS(Body::iX0.inverse ().toPointFrame (p));

      // Recurse over body parent.
      jac_point_internal< Robot, typename Body::Parent >::run(p, J);
    }
  };

  /// \brief Specialization of jac_point_internal: stop recursion at
  /// root of the Tree.
  template< typename Robot >
  struct jac_point_internal< Robot, NP >
  {
    typedef typename jac_point< Robot, NP >::jacobian_t
    jacobian_t;

    static void run(const vector3d &,
                    jacobian_t &) {}
  };

  /// \addtogroup jac_point_robot Point Articular Jacobian Test Algorithm
  ///
  /// Compute point articular jacobian for all robot bodies by calling
  /// recusively jac_point. This algorithm is not optimal and is not
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
    typedef typename jac_point< Robot, typename Robot::Tree::Body,
                                bcalc>::jacobian_t
    bodyJacobian_t;

    typedef Tree Node;

    static void run(const typename Robot::confVector & q,
                    robotJacobian_t & J)
    {
      // Compute jacobian sub-block.
      bodyJacobian_t subJ = bodyJacobian_t::Zero();
      jac_point< Robot, typename Node::Body, bcalc >
        ::run(q, vector3d(0,0,0), subJ);
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
