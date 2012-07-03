# include "metapod/tools/common.hh"
# include "model_3_dof_joint.hh"
# include "model_3_dof_body.hh"

namespace model_3_dof
{
  class Robot
  {
    public:
      enum { NBDOF = 3 };
      static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;
      typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;
      typedef Node< B0,
                    J0,
                    Node< B1, J1>,
                    Node< B2, J2>
                  > Tree;
  };
  Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;
} // end of namespace model_3_dof