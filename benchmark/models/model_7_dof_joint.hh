# include "metapod/tools/jointmacros.hh"

namespace model_7_dof
{
using namespace metapod;
using namespace metapod::Spatial;
  JOINT_REVOLUTE(J0);
  const std::string J0::name = "J0";
  const int J0::label = 0;
  const int J0::positionInConf = 0;
  const Transform J0::Xt = Transform(
    matrix3dMaker(
      0.441905, -0.431413, 0.477069,
      0.279958, -0.291903, 0.375723,
      -0.668052, -0.119791, 0.76015),
    vector3d(
      -0.124725, 0.86367, 0.86162));

  JOINT_REVOLUTE(J1);
  const std::string J1::name = "J1";
  const int J1::label = 1;
  const int J1::positionInConf = 1;
  const Transform J1::Xt = Transform(
    matrix3dMaker(
      -0.385084, -0.105933, -0.547787,
      -0.624934, -0.447531, 0.112888,
      -0.166997, -0.660786, 0.813608),
    vector3d(
      0.239193, -0.437881, 0.572004));

  JOINT_REVOLUTE(J2);
  const std::string J2::name = "J2";
  const int J2::label = 2;
  const int J2::positionInConf = 2;
  const Transform J2::Xt = Transform(
    matrix3dMaker(
      0.464297, -0.74905, 0.586941,
      -0.671796, 0.490143, -0.85094,
      0.900208, -0.894941, 0.0431268),
    vector3d(
      0.168977, -0.511175, -0.69522));

  JOINT_REVOLUTE(J3);
  const std::string J3::name = "J3";
  const int J3::label = 3;
  const int J3::positionInConf = 3;
  const Transform J3::Xt = Transform(
    matrix3dMaker(
      0.639355, 0.146637, 0.511162,
      -0.896122, -0.684386, 0.999987,
      -0.591343, 0.779911, -0.749063),
    vector3d(
      -0.860187, -0.59069, -0.0771591));

  JOINT_REVOLUTE(J4);
  const std::string J4::name = "J4";
  const int J4::label = 4;
  const int J4::positionInConf = 4;
  const Transform J4::Xt = Transform(
    matrix3dMaker(
      -0.0948483, 0.374775, -0.80072,
      0.061616, 0.514588, -0.39141,
      0.984457, 0.153942, 0.755228),
    vector3d(
      -0.281809, 0.10497, 0.15886));

  JOINT_REVOLUTE(J5);
  const std::string J5::name = "J5";
  const int J5::label = 5;
  const int J5::positionInConf = 5;
  const Transform J5::Xt = Transform(
    matrix3dMaker(
      0.660024, 0.777898, -0.846011,
      0.299414, -0.503912, 0.258959,
      -0.541726, 0.40124, -0.366266),
    vector3d(
      0.333761, -0.00548296, -0.672064));

  JOINT_REVOLUTE(J6);
  const std::string J6::name = "J6";
  const int J6::label = 6;
  const int J6::positionInConf = 6;
  const Transform J6::Xt = Transform(
    matrix3dMaker(
      -0.0981649, -0.327298, 0.695369,
      -0.130973, -0.993537, -0.310114,
      0.196963, 0.666487, -0.532217),
    vector3d(
      -0.0570331, 0.18508, 0.888636));

} // end of namespace model_7_dof