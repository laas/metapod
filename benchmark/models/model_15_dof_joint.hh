# include "metapod/tools/jointmacros.hh"

namespace model_15_dof
{
using namespace metapod;
using namespace metapod::Spatial;
  JOINT_REVOLUTE(J0);
  const std::string J0::name = "J0";
  const int J0::label = 0;
  const int J0::positionInConf = 0;
  const Transform J0::Xt = Transform(
    matrix3dMaker(
      -0.390089, 0.424175, -0.634888,
      0.243646, -0.918271, -0.172033,
      0.391968, 0.347873, 0.27528),
    vector3d(
      0.350952, -0.0340994, -0.0361284));

  JOINT_REVOLUTE(J1);
  const std::string J1::name = "J1";
  const int J1::label = 1;
  const int J1::positionInConf = 1;
  const Transform J1::Xt = Transform(
    matrix3dMaker(
      -0.620498, 0.968726, -0.992843,
      0.654782, -0.337042, -0.623598,
      -0.127006, 0.917274, 0.837861),
    vector3d(
      0.551534, -0.423241, -0.340716));

  JOINT_REVOLUTE(J2);
  const std::string J2::name = "J2";
  const int J2::label = 2;
  const int J2::positionInConf = 2;
  const Transform J2::Xt = Transform(
    matrix3dMaker(
      0.747958, -0.00371215, 0.152399,
      -0.674487, -0.452178, 0.729158,
      -0.0152024, -0.0726757, 0.697884),
    vector3d(
      -0.405423, 0.809865, 0.819286));

  JOINT_REVOLUTE(J3);
  const std::string J3::name = "J3";
  const int J3::label = 3;
  const int J3::positionInConf = 3;
  const Transform J3::Xt = Transform(
    matrix3dMaker(
      -0.279115, -0.350386, 0.863791,
      0.816969, 0.244191, 0.673656,
      0.636255, -0.00785116, -0.330057),
    vector3d(
      -0.00294904, -0.757482, -0.723523));

  JOINT_REVOLUTE(J4);
  const std::string J4::name = "J4";
  const int J4::label = 4;
  const int J4::positionInConf = 4;
  const Transform J4::Xt = Transform(
    matrix3dMaker(
      -0.469077, 0.317493, 0.523556,
      -0.0251463, -0.685456, 0.766073,
      0.251331, 0.0354294, -0.584313),
    vector3d(
      -0.145345, 0.868989, 0.167141));

  JOINT_REVOLUTE(J5);
  const std::string J5::name = "J5";
  const int J5::label = 5;
  const int J5::positionInConf = 5;
  const Transform J5::Xt = Transform(
    matrix3dMaker(
      0.548772, -0.412644, -0.770664,
      0.73107, 0.442012, -0.901675,
      -0.10179, 0.972934, 0.415818),
    vector3d(
      -0.178424, -0.989183, 0.566564));

  JOINT_REVOLUTE(J6);
  const std::string J6::name = "J6";
  const int J6::label = 6;
  const int J6::positionInConf = 6;
  const Transform J6::Xt = Transform(
    matrix3dMaker(
      -0.11488, -0.761777, 0.179273,
      0.15727, 0.0597984, 0.190091,
      -0.276166, -0.391429, 0.777447),
    vector3d(
      -0.0845688, 0.561737, 0.384153));

  JOINT_REVOLUTE(J7);
  const std::string J7::name = "J7";
  const int J7::label = 7;
  const int J7::positionInConf = 7;
  const Transform J7::Xt = Transform(
    matrix3dMaker(
      -0.418343, -0.285614, 0.756555,
      -0.311498, 0.629817, 0.318292,
      -0.927345, -0.485062, 0.556515),
    vector3d(
      -0.0365333, -0.549018, 0.653539));

  JOINT_REVOLUTE(J8);
  const std::string J8::name = "J8";
  const int J8::label = 8;
  const int J8::positionInConf = 8;
  const Transform J8::Xt = Transform(
    matrix3dMaker(
      -0.228504, 0.065692, -0.690552,
      0.110795, -0.970841, -0.239571,
      -0.235666, -0.389184, 0.474815),
    vector3d(
      -0.368684, -0.131983, -0.538008));

  JOINT_REVOLUTE(J9);
  const std::string J9::name = "J9";
  const int J9::label = 9;
  const int J9::positionInConf = 9;
  const Transform J9::Xt = Transform(
    matrix3dMaker(
      -0.198582, -0.275691, 0.437734,
      0.603793, 0.355625, -0.694248,
      -0.934215, -0.872879, 0.371444),
    vector3d(
      0.236894, 0.285385, 0.0370298));

  JOINT_REVOLUTE(J10);
  const std::string J10::name = "J10";
  const int J10::label = 10;
  const int J10::positionInConf = 10;
  const Transform J10::Xt = Transform(
    matrix3dMaker(
      -0.373541, 0.770028, -0.62747,
      -0.685722, 0.00692197, 0.657915,
      0.351308, 0.80834, -0.617777),
    vector3d(
      0.334681, 0.758019, 0.30661));

  JOINT_REVOLUTE(J11);
  const std::string J11::name = "J11";
  const int J11::label = 11;
  const int J11::positionInConf = 11;
  const Transform J11::Xt = Transform(
    matrix3dMaker(
      0.188995, 0.314402, 0.9906,
      0.871703, -0.350917, 0.748619,
      0.178313, 0.275542, 0.518647),
    vector3d(
      0.878258, 0.599291, 0.628277));

  JOINT_REVOLUTE(J12);
  const std::string J12::name = "J12";
  const int J12::label = 12;
  const int J12::positionInConf = 12;
  const Transform J12::Xt = Transform(
    matrix3dMaker(
      -0.703316, 0.158043, -0.934073,
      0.401821, 0.0363013, 0.665218,
      0.0300982, -0.774704, -0.02038),
    vector3d(
      0.193798, 0.291203, 0.077113));

  JOINT_REVOLUTE(J13);
  const std::string J13::name = "J13";
  const int J13::label = 13;
  const int J13::positionInConf = 13;
  const Transform J13::Xt = Transform(
    matrix3dMaker(
      -0.106515, -0.0452772, 0.99033,
      -0.882554, -0.851479, 0.281533,
      0.19456, -0.554795, -0.560424),
    vector3d(
      0.147442, 0.625894, 0.165365));

  JOINT_REVOLUTE(J14);
  const std::string J14::name = "J14";
  const int J14::label = 14;
  const int J14::positionInConf = 14;
  const Transform J14::Xt = Transform(
    matrix3dMaker(
      -0.0676628, 0.768636, 0.934554,
      -0.632469, -0.083922, 0.560448,
      0.532895, 0.809563, -0.484829),
    vector3d(
      -0.561728, -0.0448975, 0.899641));

} // end of namespace model_15_dof