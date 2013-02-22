// Copyright 2012, 2013
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

# include <metapod/tools/common.hh>
# include <metapod/algos/jac_point_relative.hh>
# include <metapod/tools/depth_first_traversal.hh>


namespace metapod {

template< typename Robot, bool call_bcalc = true >
struct jac_point_robot;

/// \brief Point articular jacobian algorithm.
///
/// \tparam Robot Robot type for which jacobian for all bodies is
/// computed.
///
/// \tparam call_bcalc Boolean type to specify whether all body
/// transforms need to be updated with respect to the robot
/// configuration.
/// \addtogroup jac_point_robot Point Articular Jacobian Test Algorithm
///
/// Compute point articular jacobian for all robot bodies by calling
/// recursively jac_point. This algorithm is not optimal and is not
/// meant to be used except for testing purposes.
///
/// \{
template <typename Robot, bool call_bcalc>
struct jac_point_robot {

  typedef Eigen::Matrix<FloatType, 6*Robot::NBBODIES, Robot::NBDOF,
                        Eigen::RowMajor> RobotJacobian;

  template <typename AnyRobot, int node_id>
  struct DftVisitor {

    typedef jac_point_relative<AnyRobot, NO_PARENT, node_id, 0, call_bcalc> solver;
    typedef typename solver::Jacobian BodyJacobian;
    typedef typename Nodes<AnyRobot, node_id>::type Node;

    static void discover(AnyRobot& robot,
                         const typename Robot::confVector& q,
                         RobotJacobian& J) {
      // Compute jacobian sub-block.
      BodyJacobian subJ = BodyJacobian::Zero();
      solver::run(robot, q, Vector3d(0,0,0), subJ);
      J.template block<6, Robot::NBDOF>(6*Node::id, 0) = subJ;
    }

    static void finish(AnyRobot&,
                       const typename Robot::confVector&,
                       RobotJacobian&) {}
  };

  static void run(Robot& robot,
                  const typename Robot::confVector& q,
                  RobotJacobian & J) {
    depth_first_traversal<DftVisitor, Robot>::run(robot, q, J);
  }
};

/// \}
///

} // end of namespace metapod

#endif
