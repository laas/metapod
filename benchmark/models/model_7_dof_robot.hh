# include "metapod/tools/common.hh"
# include "model_7_dof_joint.hh"
# include "model_7_dof_body.hh"

namespace model_7_dof
{
  class Robot
  {
    public:
      enum { NBDOF = 7 };
      static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;
      typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;
      typedef Node< B0,
                    J0,
                    Node< B1,
                          J1,
                          Node< B2, J2>,
                          Node< B3, J3>
                        >,
                    Node< B4,
                          J4,
                          Node< B5, J5>,
                          Node< B6, J6>
                        >
                  > Tree;
  };
  Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;
} // end of namespace model_7_dof