# include "metapod/tools/jointmacros.hh"

namespace model_3_dof
{
using namespace metapod;
using namespace metapod::Spatial;
  JOINT_REVOLUTE(J0);
  const std::string J0::name = "J0";
  const int J0::label = 0;
  const int J0::positionInConf = 0;
  const Transform J0::Xt = Transform(
    matrix3dMaker(
      0.59688, 0.823295, -0.604897,
      -0.329554, 0.536459, -0.444451,
      0.10794, -0.0452059, 0.257742),
    vector3d(
      0.680375, -0.211234, 0.566198));

  JOINT_REVOLUTE(J1);
  const std::string J1::name = "J1";
  const int J1::label = 1;
  const int J1::positionInConf = 1;
  const Transform J1::Xt = Transform(
    matrix3dMaker(
      0.542715, 0.05349, 0.539828,
      -0.199543, 0.783059, -0.433371,
      -0.295083, 0.615449, 0.838053),
    vector3d(
      -0.012834, 0.94555, -0.414966));

  JOINT_REVOLUTE(J2);
  const std::string J2::name = "J2";
  const int J2::label = 2;
  const int J2::positionInConf = 2;
  const Transform J2::Xt = Transform(
    matrix3dMaker(
      0.70184, -0.466669, 0.0795207,
      -0.249586, 0.520497, 0.0250707,
      0.335448, 0.0632129, -0.921439),
    vector3d(
      -0.52344, 0.941268, 0.804416));

} // end of namespace model_3_dof