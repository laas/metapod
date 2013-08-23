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

/// Implementation of articular jacobian of point given an influencing
/// chain of joints.

#ifndef METAPOD_JAC_POINT_CHAIN_HH
# define METAPOD_JAC_POINT_CHAIN_HH

# include <metapod/tools/common.hh>
# include <metapod/algos/jac_point_relative.hh>

namespace metapod {
/// \addtogroup jac_point_chain Point in Chain Articular Jacobian Algorithm
///
/// Compute articular jacobian J of a point (in body coordinates)
/// attached to a body in a specific sub-chain of the robot
/// kinematic tree. This jacobian is such that v = J*dq is the point
/// frame spatial motion vector in world coordinates, under the sole
/// influence of joints in the sub-chain and (optionally) a fictive
/// free-flyer joint at the base of the sub-chain.
///
/// Such a jacobian can be used to control the relative pose of a body
/// with respect to another body.
///
/// Note that the 6 first degrees of freedom of the jacobian are always wiped
/// out: they are either excluded from the jacobian or overwritten by the
/// fictive free-flyer contribution. As a consequence, this function is
/// probably only useful for systems with a root free flyer joint.
/// It will not compile if the robot has less than 6 degrees of freedom.
/// It is included for compatibility with the jrl-dynamics library.
/// \{

namespace internal {
template <typename Robot, int start_node_id, int end_node_id,
          int offset = 0, bool includeFreeFlyer = true>
  struct jac_point_chain_internal_freeflyer;
}

/// \brief Point in Chain Articular Jacobian Algorithm.
///
/// \tparam Robot Robot type for which jacobian is computed.
/// \tparam StartBody Start body type in sub-chain for which
/// jacobian is computed.
/// \tparam EndBody End body type in sub-chain for which
/// jacobian is computed.
/// \tparam offset Rank of first non zero jacobian column.
/// \tparam includeFreeFlyer Boolean type to specify the
/// contribution of a fictive free-flyer joint superposed with the
/// joint of StartBody.
/// \tparam call_bcalc Boolean type to specify whether all body
/// transforms need to be updated with respect to the robot
/// configuration.
template <typename Robot, int start_node_id, int end_node_id,
          int offset = 0, bool includeFreeFlyer = true,
          bool call_bcalc = true>
struct jac_point_chain {
  METAPOD_STATIC_ASSERT((Robot::NBDOF >= 6),
      "jac_point_chain does not support robots with less than 6 DoFs");
  typedef jac_point_relative<Robot, start_node_id, end_node_id,
      offset - 6*(1-includeFreeFlyer), call_bcalc> solver;
  typedef typename solver::Jacobian Jacobian;

  /// \brief Compute the articular jacobian J.
  ///
  /// \param q Configuration vector: it is used to update all body
  /// spatial transforms if bcalc is equal to true.
  ///
  /// \param e_p Coordinates of point in EndBody coordinates.
  /// \retval J Computed jacobian of size 6x(NBDOF+offset) if
  /// free-flyer is included, 6x(NBDOF-6+offset) otherwise.
  static void run(Robot& robot,
                  const typename Robot::confVector& q,
                  const Vector3d& e_p,
                  Jacobian& J) {
    // compute the jacobian
    solver::run(robot, q, e_p, J);
    // handle the fictive free flyer
    internal::jac_point_chain_internal_freeflyer<Robot,
        start_node_id, end_node_id, offset, includeFreeFlyer>::run(robot, e_p, J);
  }
};
/// \}

namespace internal {
/// \brief Internal point in chain articular jacobian algorithm
/// routine.
/// If includeFreeFlyer is true, compute the contribution of a fictive
/// free-flyer joint, and overwrite the jacobian columns corresponding to
/// the 6 first dofs with it (that is: J.block<6, 6>(0, offset))
///
/// \sa jac_point_chain
template <typename Robot, int start_node_id, int end_node_id, int offset>
struct jac_point_chain_internal_freeflyer<Robot, start_node_id, end_node_id,
    offset, true> {

  typedef typename jac_point_chain<Robot, start_node_id, end_node_id,
                                   offset, true >::Jacobian Jacobian;
  typedef typename Nodes<Robot, start_node_id>::type StartNode;
  typedef typename Nodes<Robot, end_node_id>::type EndNode;

  static void run(Robot& robot, const Vector3d& e_p, Jacobian& J) {
    StartNode& start_node = boost::fusion::at_c<start_node_id>(robot.nodes);
    EndNode& end_node = boost::fusion::at_c<end_node_id>(robot.nodes);
    // Compute point coordinates in world frame.
    Vector3d p =  end_node.body.iX0.applyInv(e_p);
    // Compute jacobian block for freeflyer. Formula is given by:
    // Ji = pX0 * (sX0)^(-1) * Sff,
    // where pX0 is the word transform in the point frame,
    // sX0 is the world transform in the start body frame,
    // Sff is the motion subspace Matrix of a fictive free-flyer
    // located at the start body joint.
    J.template block<3,3>(0,3+offset) = Matrix3d::Identity ();
    J.template block<3,3>(3,offset) = Matrix3d::Identity ();
    Spatial::Transform pXs = start_node.body.iX0.inverse ().toPointFrame (p);
    J.template block<3,3>(3,3+offset) = Spatial::skew (- (pXs.E() * pXs.r()));
  }
};

/// \brief Specialization of jac_point_chain_internal_free-flyer: do
/// nothing if freeflyer joint is not included in jacobian.
template <typename Robot, int start_node_id, int end_node_id, int offset>
struct jac_point_chain_internal_freeflyer< Robot, start_node_id, end_node_id,
    offset, false> {

  typedef typename jac_point_chain<Robot, start_node_id, end_node_id,
                                   offset, false >::Jacobian Jacobian;

  static void run(Robot&, const Vector3d&, Jacobian&) {}
};
} // end of namespace metapod::internal
} // end of namespace metapod

#endif
