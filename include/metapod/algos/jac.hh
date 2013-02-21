// Copyright 2012, 2013
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

/**
 * Compute the kinematic jacobian. i.e. the matrix J such that
 * J*dq is the vertical concatenation of each body spatial motion vector
 *(in the moving body frame)
 */

#ifndef METAPOD_JAC_HH
# define METAPOD_JAC_HH

# include <metapod/tools/common.hh>
# include <metapod/tools/backward_traversal.hh>

namespace metapod {

template< typename Robot >
struct jac {
  typedef Eigen::Matrix<FloatType,
                        6 * Robot::NBBODIES,
                        Robot::NBDOF> Jacobian;
  template <typename AnyRobot, int node_id>
  struct DftVisitor {
    typedef typename Nodes<Robot, node_id>::type Node;

    template <typename AnyAnyRobot, int ancestor_id>
    struct BwdtVisitor {
      typedef typename Nodes<Robot, Node::parent_id>::type Parent;
      typedef typename Nodes<AnyAnyRobot, ancestor_id>::type Ancestor;

      static void discover(Jacobian& J,Matrix6d& sXp_matrix) {
        // contribution of the ancestor joint to the current body velocity,
        // copied from the parent body jacobian (which we just computed)
        J.template
            block<6, Ancestor::Joint::NBDOF>(6*Node::id, Ancestor::q_idx) =
            sXp_matrix * J.template
            block<6, Ancestor::Joint::NBDOF>(6*Parent::id, Ancestor::q_idx);
    }

      static void finish(Jacobian&, Matrix6d&) {}
    };

    static void discover(AnyRobot& robot, Jacobian& J) {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      // contribution of the current joint to the current body velocity
      J.template block<6, Node::Joint::NBDOF>(6*Node::id, Node::q_idx)
          = node.joint.S.S();
      // contribution of the ancestor joints to the current body velocity,
      // copied from the parent body jacobian.
      // We pass node.sXp.toMatrix() to avoid toMatrix() to
      // be called again at each step.
      Matrix6d sXp_matrix = node.sXp.toMatrix();
      backward_traversal< BwdtVisitor, Robot, Node::parent_id >::run(
          J, sXp_matrix);
    }

    static void finish(AnyRobot&, Jacobian&) {}
  };

  static void run(Robot& robot, Jacobian& J) {
    depth_first_traversal<DftVisitor, Robot>::run(robot, J);
  }
};

} // end of namespace metapod

#endif
