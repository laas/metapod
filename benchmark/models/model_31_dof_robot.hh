# include "metapod/tools/common.hh"
# include "model_31_dof_joint.hh"
# include "model_31_dof_body.hh"

namespace model_31_dof
{
  class Robot
  {
    public:
      enum { NBDOF = 31 };
      static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;
      typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;
      typedef Node< B0,
                    J0,
                    Node< B1,
                          J1,
                          Node< B2,
                                J2,
                                Node< B3,
                                      J3,
                                      Node< B4, J4>,
                                      Node< B5, J5>
                                    >,
                                Node< B6,
                                      J6,
                                      Node< B7, J7>,
                                      Node< B8, J8>
                                    >
                              >,
                          Node< B9,
                                J9,
                                Node< B10,
                                      J10,
                                      Node< B11, J11>,
                                      Node< B12, J12>
                                    >,
                                Node< B13,
                                      J13,
                                      Node< B14, J14>,
                                      Node< B15, J15>
                                    >
                              >
                        >,
                    Node< B16,
                          J16,
                          Node< B17,
                                J17,
                                Node< B18,
                                      J18,
                                      Node< B19, J19>,
                                      Node< B20, J20>
                                    >,
                                Node< B21,
                                      J21,
                                      Node< B22, J22>,
                                      Node< B23, J23>
                                    >
                              >,
                          Node< B24,
                                J24,
                                Node< B25,
                                      J25,
                                      Node< B26, J26>,
                                      Node< B27, J27>
                                    >,
                                Node< B28,
                                      J28,
                                      Node< B29, J29>,
                                      Node< B30, J30>
                                    >
                              >
                        >
                  > Tree;
  };
  Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;
} // end of namespace model_31_dof