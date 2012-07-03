# include "metapod/tools/jointmacros.hh"

namespace model_31_dof
{
using namespace metapod;
using namespace metapod::Spatial;
  JOINT_REVOLUTE(J0);
  const std::string J0::name = "J0";
  const int J0::label = 0;
  const int J0::positionInConf = 0;
  const Transform J0::Xt = Transform(
    matrix3dMaker(
      -0.195242, 0.121569, 0.108896,
      0.244333, -0.617945, -0.0440782,
      -0.27979, 0.30776, 0.833045),
    vector3d(
      0.523225, 0.92701, -0.336308));

  JOINT_REVOLUTE(J1);
  const std::string J1::name = "J1";
  const int J1::label = 1;
  const int J1::positionInConf = 1;
  const Transform J1::Xt = Transform(
    matrix3dMaker(
      0.931363, -0.689327, -0.414222,
      0.764409, -0.267944, 0.798863,
      0.495276, -0.0483887, -0.454026),
    vector3d(
      -0.0245586, 0.590463, 0.278018));

  JOINT_REVOLUTE(J2);
  const std::string J2::name = "J2";
  const int J2::label = 2;
  const int J2::positionInConf = 2;
  const Transform J2::Xt = Transform(
    matrix3dMaker(
      -0.972267, -0.678351, -0.760021,
      -0.0763044, 0.29709, 0.830441,
      -0.798286, 0.228453, -0.858886),
    vector3d(
      -0.746662, -0.544896, 0.893849));

  JOINT_REVOLUTE(J3);
  const std::string J3::name = "J3";
  const int J3::label = 3;
  const int J3::positionInConf = 3;
  const Transform J3::Xt = Transform(
    matrix3dMaker(
      0.0470968, -0.598106, 0.323583,
      0.399574, -0.344768, 0.778687,
      0.293424, -0.317035, -0.899664),
    vector3d(
      -0.287234, 0.99594, -0.928667));

  JOINT_REVOLUTE(J4);
  const std::string J4::name = "J4";
  const int J4::label = 4;
  const int J4::positionInConf = 4;
  const Transform J4::Xt = Transform(
    matrix3dMaker(
      -0.727659, 0.178237, -0.883895,
      0.779106, 0.891003, -0.887956,
      0.850439, -0.0619, -0.486062),
    vector3d(
      -0.797092, -0.68545, -0.511703));

  JOINT_REVOLUTE(J5);
  const std::string J5::name = "J5";
  const int J5::label = 5;
  const int J5::positionInConf = 5;
  const Transform J5::Xt = Transform(
    matrix3dMaker(
      -0.304908, -0.588572, 0.0452573,
      -0.198029, -0.385663, 0.359808,
      0.290268, -0.113322, -0.461955),
    vector3d(
      0.0763506, -0.966699, 0.863036));

  JOINT_REVOLUTE(J6);
  const std::string J6::name = "J6";
  const int J6::label = 6;
  const int J6::positionInConf = 6;
  const Transform J6::Xt = Transform(
    matrix3dMaker(
      -0.487544, -0.853529, 0.0287644,
      0.779625, 0.222822, 0.0620655,
      0.642662, 0.917914, 0.473494),
    vector3d(
      0.0536604, 0.696, -0.499579));

  JOINT_REVOLUTE(J7);
  const std::string J7::name = "J7";
  const int J7::label = 7;
  const int J7::positionInConf = 7;
  const Transform J7::Xt = Transform(
    matrix3dMaker(
      -0.913211, -0.831178, -0.510283,
      0.42271, 0.222482, -0.814283,
      0.92313, 0.734938, -0.667812),
    vector3d(
      -0.618033, -0.0327033, 0.846762));

  JOINT_REVOLUTE(J8);
  const std::string J8::name = "J8";
  const int J8::label = 8;
  const int J8::positionInConf = 8;
  const Transform J8::Xt = Transform(
    matrix3dMaker(
      -0.809915, 0.493461, -0.445573,
      -0.653399, 0.875428, 0.521724,
      -0.806637, 0.962217, 0.690546),
    vector3d(
      -0.450207, -0.521358, 0.619486));

  JOINT_REVOLUTE(J9);
  const std::string J9::name = "J9";
  const int J9::label = 9;
  const int J9::positionInConf = 9;
  const Transform J9::Xt = Transform(
    matrix3dMaker(
      -0.578274, 0.336651, 0.0577697,
      -0.375313, 0.886444, 0.536411,
      -0.755828, -0.92347, 0.0298719),
    vector3d(
      -0.771082, 0.761366, 0.164899));

  JOINT_REVOLUTE(J10);
  const std::string J10::name = "J10";
  const int J10::label = 10;
  const int J10::positionInConf = 10;
  const Transform J10::Xt = Transform(
    matrix3dMaker(
      0.734395, -0.874652, -0.905635,
      0.0541499, -0.645734, 0.855732,
      -0.780951, -0.224008, 0.192383),
    vector3d(
      0.571865, 0.369354, 0.820453));

  JOINT_REVOLUTE(J11);
  const std::string J11::name = "J11";
  const int J11::label = 11;
  const int J11::positionInConf = 11;
  const Transform J11::Xt = Transform(
    matrix3dMaker(
      0.868362, 0.698543, 0.662924,
      0.428863, 0.270408, 0.0322771,
      0.249316, 0.00480253, 0.157625),
    vector3d(
      0.615919, -0.225355, 0.494554));

  JOINT_REVOLUTE(J12);
  const std::string J12::name = "J12";
  const int J12::label = 12;
  const int J12::positionInConf = 12;
  const Transform J12::Xt = Transform(
    matrix3dMaker(
      -0.795533, -0.0404457, -0.459359,
      -0.600552, -0.424527, 0.315286,
      0.894002, -0.556165, 0.0138293),
    vector3d(
      -0.374021, -0.788847, 0.776866));

  JOINT_REVOLUTE(J13);
  const std::string J13::name = "J13";
  const int J13::label = 13;
  const int J13::positionInConf = 13;
  const Transform J13::Xt = Transform(
    matrix3dMaker(
      0.235164, 0.527528, 0.548865,
      -0.431421, -0.846493, 0.760017,
      -0.654556, -0.642027, -0.280428),
    vector3d(
      0.546602, -0.943095, 0.180814));

  JOINT_REVOLUTE(J14);
  const std::string J14::name = "J14";
  const int J14::label = 14;
  const int J14::positionInConf = 14;
  const Transform J14::Xt = Transform(
    matrix3dMaker(
      -0.86353, 0.803097, -0.135602,
      0.762463, 0.349699, -0.0786967,
      -0.0567224, -0.415137, -0.551169),
    vector3d(
      -0.211522, -0.455621, 0.199289));

  JOINT_REVOLUTE(J15);
  const std::string J15::name = "J15";
  const int J15::label = 15;
  const int J15::positionInConf = 15;
  const Transform J15::Xt = Transform(
    matrix3dMaker(
      0.617234, 0.300981, 0.375054,
      -0.649174, -0.910541, 0.919433,
      0.550115, -0.774071, 0.72253),
    vector3d(
      -0.994344, 0.801544, -0.422497));

  JOINT_REVOLUTE(J16);
  const std::string J16::name = "J16";
  const int J16::label = 16;
  const int J16::positionInConf = 16;
  const Transform J16::Xt = Transform(
    matrix3dMaker(
      0.0650283, -0.0855207, 0.280736,
      0.434183, -0.0798651, 0.0822792,
      -0.988314, -0.462632, -0.616739),
    vector3d(
      0.347296, -0.352297, -0.305607));

  JOINT_REVOLUTE(J17);
  const std::string J17::name = "J17";
  const int J17::label = 17;
  const int J17::positionInConf = 17;
  const Transform J17::Xt = Transform(
    matrix3dMaker(
      -0.43144, -0.639727, -0.281507,
      -0.122019, 0.707569, 0.366197,
      0.572373, -0.227403, -0.719324),
    vector3d(
      0.758025, 0.973287, 0.0425216));

  JOINT_REVOLUTE(J18);
  const std::string J18::name = "J18";
  const int J18::label = 18;
  const int J18::positionInConf = 18;
  const Transform J18::Xt = Transform(
    matrix3dMaker(
      -0.822943, -0.958983, -0.893853,
      0.795766, 0.799042, -0.920566,
      -0.161712, -0.632398, -0.560293),
    vector3d(
      0.490017, 0.940084, 0.917507));

  JOINT_REVOLUTE(J19);
  const std::string J19::name = "J19";
  const int J19::label = 19;
  const int J19::positionInConf = 19;
  const Transform J19::Xt = Transform(
    matrix3dMaker(
      -0.230982, -0.283529, 0.861709,
      0.833702, -0.793513, 0.801793,
      0.751209, -0.616456, 0.842809),
    vector3d(
      -0.857356, 0.925102, -0.0496723));

  JOINT_REVOLUTE(J20);
  const std::string J20::name = "J20";
  const int J20::label = 20;
  const int J20::positionInConf = 20;
  const Transform J20::Xt = Transform(
    matrix3dMaker(
      -0.429702, 0.105475, 0.35051,
      0.915419, 0.248119, 0.275612,
      -0.134254, -0.982863, 0.992083),
    vector3d(
      0.443645, -0.298989, 0.744057));

  JOINT_REVOLUTE(J21);
  const std::string J21::name = "J21";
  const int J21::label = 21;
  const int J21::positionInConf = 21;
  const Transform J21::Xt = Transform(
    matrix3dMaker(
      0.278397, -0.703539, 0.850757,
      0.351389, 0.740105, -0.448232,
      0.0954457, -0.689596, 0.657243),
    vector3d(
      -0.976639, 0.772689, -0.646599));

  JOINT_REVOLUTE(J22);
  const std::string J22::name = "J22";
  const int J22::label = 22;
  const int J22::positionInConf = 22;
  const Transform J22::Xt = Transform(
    matrix3dMaker(
      0.25576, -0.265763, 0.230982,
      0.0347815, -0.242402, 0.00367067,
      0.388182, -0.964005, 0.300131),
    vector3d(
      -0.505664, -0.279671, 0.834789));

  JOINT_REVOLUTE(J23);
  const std::string J23::name = "J23";
  const int J23::label = 23;
  const int J23::positionInConf = 23;
  const Transform J23::Xt = Transform(
    matrix3dMaker(
      -0.950849, -0.723773, -0.164673,
      -0.575463, -0.229437, 0.555655,
      -0.740673, -0.973677, -0.710108),
    vector3d(
      0.275812, -0.635994, -0.512945));

  JOINT_REVOLUTE(J24);
  const std::string J24::name = "J24";
  const int J24::label = 24;
  const int J24::positionInConf = 24;
  const Transform J24::Xt = Transform(
    matrix3dMaker(
      0.76145, 0.280197, -0.46716,
      -0.570518, -0.443991, -0.103154,
      -0.0834629, -0.39484, 0.173073),
    vector3d(
      -0.815013, -0.073116, 0.882486));

  JOINT_REVOLUTE(J25);
  const std::string J25::name = "J25";
  const int J25::label = 25;
  const int J25::positionInConf = 25;
  const Transform J25::Xt = Transform(
    matrix3dMaker(
      -0.800741, 0.901771, -0.35249,
      0.152959, -0.913242, 0.574394,
      0.0354445, 0.848208, -0.145409),
    vector3d(
      0.728389, 0.0674233, 0.168576));

  JOINT_REVOLUTE(J26);
  const std::string J26::name = "J26";
  const int J26::label = 26;
  const int J26::positionInConf = 26;
  const Transform J26::Xt = Transform(
    matrix3dMaker(
      -0.417137, -0.504075, -0.612872,
      -0.936787, -0.775687, 0.454551,
      0.231789, -0.576428, 0.356322),
    vector3d(
      0.551799, 0.851022, 0.113815));

  JOINT_REVOLUTE(J27);
  const std::string J27::name = "J27";
  const int J27::label = 27;
  const int J27::positionInConf = 27;
  const Transform J27::Xt = Transform(
    matrix3dMaker(
      -0.219899, 0.865432, 0.962907,
      0.114583, 0.417231, 0.813929,
      -0.771602, -0.999906, -0.690147),
    vector3d(
      0.734643, -0.716407, 0.709296));

  JOINT_REVOLUTE(J28);
  const std::string J28::name = "J28";
  const int J28::label = 28;
  const int J28::positionInConf = 28;
  const Transform J28::Xt = Transform(
    matrix3dMaker(
      0.729746, -0.344748, 0.971893,
      -0.507049, -0.610105, -0.744515,
      -0.797753, 0.169996, -0.879082),
    vector3d(
      -0.839232, -0.0636308, 0.326503));

  JOINT_REVOLUTE(J29);
  const std::string J29::name = "J29";
  const int J29::label = 29;
  const int J29::positionInConf = 29;
  const Transform J29::Xt = Transform(
    matrix3dMaker(
      -0.684891, 0.494461, -0.300875,
      0.461353, 0.655229, 0.635494,
      -0.212144, 0.384975, -0.709254),
    vector3d(
      0.780841, -0.375019, -0.928961));

  JOINT_REVOLUTE(J30);
  const std::string J30::name = "J30";
  const int J30::label = 30;
  const int J30::positionInConf = 30;
  const Transform J30::Xt = Transform(
    matrix3dMaker(
      0.814093, 0.336449, -0.0247224,
      -0.289263, 0.11729, 0.600259,
      -0.218224, 0.432399, 0.0947199),
    vector3d(
      -0.787456, -0.486653, 0.776696));

} // end of namespace model_31_dof