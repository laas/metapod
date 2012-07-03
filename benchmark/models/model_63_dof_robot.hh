# include "metapod/tools/common.hh"
# include "model_63_dof_joint.hh"
# include "model_63_dof_body.hh"

namespace model_63_dof
{
  class Robot
  {
    public:
      enum { NBDOF = 63 };
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
                                      Node< B4,
                                            J4,
                                            Node< B5, J5>,
                                            Node< B6, J6>
                                          >,
                                      Node< B7,
                                            J7,
                                            Node< B8, J8>,
                                            Node< B9, J9>
                                          >
                                    >,
                                Node< B10,
                                      J10,
                                      Node< B11,
                                            J11,
                                            Node< B12, J12>,
                                            Node< B13, J13>
                                          >,
                                      Node< B14,
                                            J14,
                                            Node< B15, J15>,
                                            Node< B16, J16>
                                          >
                                    >
                              >,
                          Node< B17,
                                J17,
                                Node< B18,
                                      J18,
                                      Node< B19,
                                            J19,
                                            Node< B20, J20>,
                                            Node< B21, J21>
                                          >,
                                      Node< B22,
                                            J22,
                                            Node< B23, J23>,
                                            Node< B24, J24>
                                          >
                                    >,
                                Node< B25,
                                      J25,
                                      Node< B26,
                                            J26,
                                            Node< B27, J27>,
                                            Node< B28, J28>
                                          >,
                                      Node< B29,
                                            J29,
                                            Node< B30, J30>,
                                            Node< B31, J31>
                                          >
                                    >
                              >
                        >,
                    Node< B32,
                          J32,
                          Node< B33,
                                J33,
                                Node< B34,
                                      J34,
                                      Node< B35,
                                            J35,
                                            Node< B36, J36>,
                                            Node< B37, J37>
                                          >,
                                      Node< B38,
                                            J38,
                                            Node< B39, J39>,
                                            Node< B40, J40>
                                          >
                                    >,
                                Node< B41,
                                      J41,
                                      Node< B42,
                                            J42,
                                            Node< B43, J43>,
                                            Node< B44, J44>
                                          >,
                                      Node< B45,
                                            J45,
                                            Node< B46, J46>,
                                            Node< B47, J47>
                                          >
                                    >
                              >,
                          Node< B48,
                                J48,
                                Node< B49,
                                      J49,
                                      Node< B50,
                                            J50,
                                            Node< B51, J51>,
                                            Node< B52, J52>
                                          >,
                                      Node< B53,
                                            J53,
                                            Node< B54, J54>,
                                            Node< B55, J55>
                                          >
                                    >,
                                Node< B56,
                                      J56,
                                      Node< B57,
                                            J57,
                                            Node< B58, J58>,
                                            Node< B59, J59>
                                          >,
                                      Node< B60,
                                            J60,
                                            Node< B61, J61>,
                                            Node< B62, J62>
                                          >
                                    >
                              >
                        >
                  > Tree;
  };
  Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;
} // end of namespace model_63_dof