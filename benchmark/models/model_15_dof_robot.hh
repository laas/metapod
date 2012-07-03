# include "metapod/tools/common.hh"
# include "model_15_dof_joint.hh"
# include "model_15_dof_body.hh"

namespace model_15_dof
{
  class Robot
  {
    public:
      enum { NBDOF = 15 };
      static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;
      typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;
      typedef Node< B0,
                    J0,
                    Node< B1,
                          J1,
                          Node< B2,
                                J2,
                                Node< B3, J3>,
                                Node< B4, J4>
                              >,
                          Node< B5,
                                J5,
                                Node< B6, J6>,
                                Node< B7, J7>
                              >
                        >,
                    Node< B8,
                          J8,
                          Node< B9,
                                J9,
                                Node< B10, J10>,
                                Node< B11, J11>
                              >,
                          Node< B12,
                                J12,
                                Node< B13, J13>,
                                Node< B14, J14>
                              >
                        >
                  > Tree;
  };
  Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;
} // end of namespace model_15_dof