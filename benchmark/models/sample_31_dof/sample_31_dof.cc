// Copyright 2011, 2012,
//
// Maxime Reis
//
// JRL/LAAS, CNRS/AIST
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

/* 
 * 31 dof sample model, used for benchmarking.
 */

# include "sample_31_dof.hh"

template struct metapod::crba< metapod::sample_31_dof::Robot , true >;
template struct metapod::rnea< metapod::sample_31_dof::Robot , true >;
template struct metapod::crba< metapod::sample_31_dof::Robot , false >;
template struct metapod::rnea< metapod::sample_31_dof::Robot , false >;

namespace metapod
{
  namespace sample_31_dof
  {
    Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;

    INITIALIZE_JOINT_REVOLUTE(J0);
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
  
    INITIALIZE_JOINT_REVOLUTE(J1);
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
  
    INITIALIZE_JOINT_REVOLUTE(J2);
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
  
    INITIALIZE_JOINT_REVOLUTE(J3);
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
  
    INITIALIZE_JOINT_REVOLUTE(J4);
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
  
    INITIALIZE_JOINT_REVOLUTE(J5);
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
  
    INITIALIZE_JOINT_REVOLUTE(J6);
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
  
    INITIALIZE_JOINT_REVOLUTE(J7);
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
  
    INITIALIZE_JOINT_REVOLUTE(J8);
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
  
    INITIALIZE_JOINT_REVOLUTE(J9);
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
  
    INITIALIZE_JOINT_REVOLUTE(J10);
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
  
    INITIALIZE_JOINT_REVOLUTE(J11);
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
  
    INITIALIZE_JOINT_REVOLUTE(J12);
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
  
    INITIALIZE_JOINT_REVOLUTE(J13);
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
  
    INITIALIZE_JOINT_REVOLUTE(J14);
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
  
    INITIALIZE_JOINT_REVOLUTE(J15);
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
  
    INITIALIZE_JOINT_REVOLUTE(J16);
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
  
    INITIALIZE_JOINT_REVOLUTE(J17);
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
  
    INITIALIZE_JOINT_REVOLUTE(J18);
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
  
    INITIALIZE_JOINT_REVOLUTE(J19);
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
  
    INITIALIZE_JOINT_REVOLUTE(J20);
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
  
    INITIALIZE_JOINT_REVOLUTE(J21);
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
  
    INITIALIZE_JOINT_REVOLUTE(J22);
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
  
    INITIALIZE_JOINT_REVOLUTE(J23);
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
  
    INITIALIZE_JOINT_REVOLUTE(J24);
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
  
    INITIALIZE_JOINT_REVOLUTE(J25);
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
  
    INITIALIZE_JOINT_REVOLUTE(J26);
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
  
    INITIALIZE_JOINT_REVOLUTE(J27);
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
  
    INITIALIZE_JOINT_REVOLUTE(J28);
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
  
    INITIALIZE_JOINT_REVOLUTE(J29);
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
  
    INITIALIZE_JOINT_REVOLUTE(J30);
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

    INITIALIZE_BODY(B0);
    const std::string B0::name = "B0";
    const int B0::label = 0;
    const FloatType B0::mass = 1;
    const vector3d B0::CoM = vector3d(  0.192681,   -0.882279,   0.121744);
    const matrix3d B0::inertie = matrix3dMaker(
    -0.578617, 0.213084, 0.213084,
    -0.780445, -0.252888, -0.252888,
    0.29304, 0.185384, 0.185384);
    Inertia B0::I = spatialInertiaMaker(  B0::mass,
    B0::CoM,
    B0::inertie);
  
    INITIALIZE_BODY(B1);
    const std::string B1::name = "B1";
    const int B1::label = 1;
    const FloatType B1::mass = 1;
    const vector3d B1::CoM = vector3d(  0.866545,   -0.653871,   -0.104037);
    const matrix3d B1::inertie = matrix3dMaker(
    0.127235, -0.514748, -0.514748,
    -0.312317, -0.981853, -0.981853,
    0.202853, 0.541372, 0.541372);
    Inertia B1::I = spatialInertiaMaker(  B1::mass,
    B1::CoM,
    B1::inertie);
  
    INITIALIZE_BODY(B2);
    const std::string B2::name = "B2";
    const int B2::label = 2;
    const FloatType B2::mass = 1;
    const vector3d B2::CoM = vector3d(  -0.594046,   0.25225,   -0.647522);
    const matrix3d B2::inertie = matrix3dMaker(
    0.893281, -0.755348, -0.755348,
    0.246389, 0.437333, 0.437333,
    -0.631868, -0.435432, -0.435432);
    Inertia B2::I = spatialInertiaMaker(  B2::mass,
    B2::CoM,
    B2::inertie);
  
    INITIALIZE_BODY(B3);
    const std::string B3::name = "B3";
    const int B3::label = 3;
    const FloatType B3::mass = 1;
    const vector3d B3::CoM = vector3d(  -0.136313,   0.627807,   0.506765);
    const matrix3d B3::inertie = matrix3dMaker(
    -0.212508, -0.00713807, -0.00713807,
    -0.413645, -0.511863, -0.511863,
    0.132329, -0.618582, -0.618582);
    Inertia B3::I = spatialInertiaMaker(  B3::mass,
    B3::CoM,
    B3::inertie);
  
    INITIALIZE_BODY(B4);
    const std::string B4::name = "B4";
    const int B4::label = 4;
    const FloatType B4::mass = 1;
    const vector3d B4::CoM = vector3d(  -0.621873,   0.182221,   -0.893121);
    const matrix3d B4::inertie = matrix3dMaker(
    0.533402, 0.60666, 0.60666,
    0.363843, 0.808374, 0.808374,
    0.504957, -0.404134, -0.404134);
    Inertia B4::I = spatialInertiaMaker(  B4::mass,
    B4::CoM,
    B4::inertie);
  
    INITIALIZE_BODY(B5);
    const std::string B5::name = "B5";
    const int B5::label = 5;
    const FloatType B5::mass = 1;
    const vector3d B5::CoM = vector3d(  -0.549529,   -0.471656,   0.26717);
    const matrix3d B5::inertie = matrix3dMaker(
    0.174022, -0.662326, -0.662326,
    -0.0472907, 0.631098, 0.631098,
    0.0530451, 0.1645, 0.1645);
    Inertia B5::I = spatialInertiaMaker(  B5::mass,
    B5::CoM,
    B5::inertie);
  
    INITIALIZE_BODY(B6);
    const std::string B6::name = "B6";
    const int B6::label = 6;
    const FloatType B6::mass = 1;
    const vector3d B6::CoM = vector3d(  0.201525,   -0.357045,   0.33592);
    const matrix3d B6::inertie = matrix3dMaker(
    0.406372, -0.334216, -0.334216,
    0.518417, -0.483777, -0.483777,
    -0.967645, 0.690246, 0.690246);
    Inertia B6::I = spatialInertiaMaker(  B6::mass,
    B6::CoM,
    B6::inertie);
  
    INITIALIZE_BODY(B7);
    const std::string B7::name = "B7";
    const int B7::label = 7;
    const FloatType B7::mass = 1;
    const vector3d B7::CoM = vector3d(  0.0148191,   -0.0650583,   -0.843484);
    const matrix3d B7::inertie = matrix3dMaker(
    -0.312081, -0.280116, -0.280116,
    -0.952274, -0.989848, -0.989848,
    -0.414229, 0.416525, 0.416525);
    Inertia B7::I = spatialInertiaMaker(  B7::mass,
    B7::CoM,
    B7::inertie);
  
    INITIALIZE_BODY(B8);
    const std::string B8::name = "B8";
    const int B8::label = 8;
    const FloatType B8::mass = 1;
    const vector3d B8::CoM = vector3d(  0.575285,   0.89287,   -0.797039);
    const matrix3d B8::inertie = matrix3dMaker(
    -0.0481053, 0.514564, 0.514564,
    -0.98604, 0.157225, 0.157225,
    0.487454, 0.845144, 0.845144);
    Inertia B8::I = spatialInertiaMaker(  B8::mass,
    B8::CoM,
    B8::inertie);
  
    INITIALIZE_BODY(B9);
    const std::string B9::name = "B9";
    const int B9::label = 9;
    const FloatType B9::mass = 1;
    const vector3d B9::CoM = vector3d(  -0.244006,   -0.726088,   0.319755);
    const matrix3d B9::inertie = matrix3dMaker(
    -0.316921, 0.384927, 0.384927,
    -0.131204, 0.308057, 0.308057,
    0.200984, -0.740048, -0.740048);
    Inertia B9::I = spatialInertiaMaker(  B9::mass,
    B9::CoM,
    B9::inertie);
  
    INITIALIZE_BODY(B10);
    const std::string B10::name = "B10";
    const int B10::label = 10;
    const FloatType B10::mass = 1;
    const vector3d B10::CoM = vector3d(  -0.0761019,   -0.831631,   0.560501);
    const matrix3d B10::inertie = matrix3dMaker(
    -0.2014, -0.576869, -0.576869,
    -0.679676, -0.383506, -0.383506,
    -0.98913, 0.299573, 0.299573);
    Inertia B10::I = spatialInertiaMaker(  B10::mass,
    B10::CoM,
    B10::inertie);
  
    INITIALIZE_BODY(B11);
    const std::string B11::name = "B11";
    const int B11::label = 11;
    const FloatType B11::mass = 1;
    const vector3d B11::CoM = vector3d(  -0.251598,   -0.236225,   -0.805019);
    const matrix3d B11::inertie = matrix3dMaker(
    0.276819, 0.400679, 0.400679,
    -0.18677, 0.644852, 0.644852,
    0.843102, -0.556548, -0.556548);
    Inertia B11::I = spatialInertiaMaker(  B11::mass,
    B11::CoM,
    B11::inertie);
  
    INITIALIZE_BODY(B12);
    const std::string B12::name = "B12";
    const int B12::label = 12;
    const FloatType B12::mass = 1;
    const vector3d B12::CoM = vector3d(  0.470623,   -0.631949,   0.333414);
    const matrix3d B12::inertie = matrix3dMaker(
    0.343682, -0.941048, -0.941048,
    0.199413, -0.721998, -0.721998,
    -0.608204, 0.55482, 0.55482);
    Inertia B12::I = spatialInertiaMaker(  B12::mass,
    B12::CoM,
    B12::inertie);
  
    INITIALIZE_BODY(B13);
    const std::string B13::name = "B13";
    const int B13::label = 13;
    const FloatType B13::mass = 1;
    const vector3d B13::CoM = vector3d(  0.258719,   0.665109,   0.625994);
    const matrix3d B13::inertie = matrix3dMaker(
    0.556925, 0.872698, 0.872698,
    -0.410797, 0.122015, 0.122015,
    0.746828, -0.534304, -0.534304);
    Inertia B13::I = spatialInertiaMaker(  B13::mass,
    B13::CoM,
    B13::inertie);
  
    INITIALIZE_BODY(B14);
    const std::string B14::name = "B14";
    const int B14::label = 14;
    const FloatType B14::mass = 1;
    const vector3d B14::CoM = vector3d(  0.499438,   -0.202449,   -0.266407);
    const matrix3d B14::inertie = matrix3dMaker(
    -0.113915, -0.242579, -0.242579,
    -0.798629, -0.348577, -0.348577,
    0.215201, -0.791652, -0.791652);
    Inertia B14::I = spatialInertiaMaker(  B14::mass,
    B14::CoM,
    B14::inertie);
  
    INITIALIZE_BODY(B15);
    const std::string B15::name = "B15";
    const int B15::label = 15;
    const FloatType B15::mass = 1;
    const vector3d B15::CoM = vector3d(  -0.733224,   -0.413657,   -0.630845);
    const matrix3d B15::inertie = matrix3dMaker(
    -0.507858, 0.153442, 0.153442,
    -0.74784, 0.498886, 0.498886,
    -0.0282685, -0.615028, -0.615028);
    Inertia B15::I = spatialInertiaMaker(  B15::mass,
    B15::CoM,
    B15::inertie);
  
    INITIALIZE_BODY(B16);
    const std::string B16::name = "B16";
    const int B16::label = 16;
    const FloatType B16::mass = 1;
    const vector3d B16::CoM = vector3d(  -0.613015,   0.675972,   -0.690579);
    const matrix3d B16::inertie = matrix3dMaker(
    -0.585487, 0.988392, 0.988392,
    0.335817, -0.0683305, -0.0683305,
    0.784648, 0.423812, 0.423812);
    Inertia B16::I = spatialInertiaMaker(  B16::mass,
    B16::CoM,
    B16::inertie);
  
    INITIALIZE_BODY(B17);
    const std::string B17::name = "B17";
    const int B17::label = 17;
    const FloatType B17::mass = 1;
    const vector3d B17::CoM = vector3d(  -0.899067,   -0.81136,   0.61871);
    const matrix3d B17::inertie = matrix3dMaker(
    0.386741, -0.111805, -0.111805,
    0.306173, -0.56169, -0.56169,
    0.0287034, -0.147177, -0.147177);
    Inertia B17::I = spatialInertiaMaker(  B17::mass,
    B17::CoM,
    B17::inertie);
  
    INITIALIZE_BODY(B18);
    const std::string B18::name = "B18";
    const int B18::label = 18;
    const FloatType B18::mass = 1;
    const vector3d B18::CoM = vector3d(  -0.208632,   -0.0886192,   0.0646838);
    const matrix3d B18::inertie = matrix3dMaker(
    -0.146891, -0.79322, -0.79322,
    0.935388, -0.781533, -0.781533,
    -0.681351, 0.605207, 0.605207);
    Inertia B18::I = spatialInertiaMaker(  B18::mass,
    B18::CoM,
    B18::inertie);
  
    INITIALIZE_BODY(B19);
    const std::string B19::name = "B19";
    const int B19::label = 19;
    const FloatType B19::mass = 1;
    const vector3d B19::CoM = vector3d(  -0.595535,   0.606453,   0.34512);
    const matrix3d B19::inertie = matrix3dMaker(
    0.556781, 0.245583, 0.245583,
    -0.0770222, -0.182044, -0.182044,
    0.203654, 0.671065, 0.671065);
    Inertia B19::I = spatialInertiaMaker(  B19::mass,
    B19::CoM,
    B19::inertie);
  
    INITIALIZE_BODY(B20);
    const std::string B20::name = "B20";
    const int B20::label = 20;
    const FloatType B20::mass = 1;
    const vector3d B20::CoM = vector3d(  -0.476228,   0.497357,   -0.927008);
    const matrix3d B20::inertie = matrix3dMaker(
    0.857355, -0.82069, -0.82069,
    0.936789, 0.0175978, 0.0175978,
    -0.623504, -0.425621, -0.425621);
    Inertia B20::I = spatialInertiaMaker(  B20::mass,
    B20::CoM,
    B20::inertie);
  
    INITIALIZE_BODY(B21);
    const std::string B21::name = "B21";
    const int B21::label = 21;
    const FloatType B21::mass = 1;
    const vector3d B21::CoM = vector3d(  -0.986092,   0.396193,   0.779022);
    const matrix3d B21::inertie = matrix3dMaker(
    -0.272545, 0.850839, 0.850839,
    -0.470752, 0.602048, 0.602048,
    -0.627943, 0.459403, 0.459403);
    Inertia B21::I = spatialInertiaMaker(  B21::mass,
    B21::CoM,
    B21::inertie);
  
    INITIALIZE_BODY(B22);
    const std::string B22::name = "B22";
    const int B22::label = 22;
    const FloatType B22::mass = 1;
    const vector3d B22::CoM = vector3d(  0.912222,   -0.651729,   -0.624614);
    const matrix3d B22::inertie = matrix3dMaker(
    -0.554044, -0.774178, -0.774178,
    0.721568, 0.0915689, 0.0915689,
    0.713652, 0.819024, 0.819024);
    Inertia B22::I = spatialInertiaMaker(  B22::mass,
    B22::CoM,
    B22::inertie);
  
    INITIALIZE_BODY(B23);
    const std::string B23::name = "B23";
    const int B23::label = 23;
    const FloatType B23::mass = 1;
    const vector3d B23::CoM = vector3d(  -0.646687,   -0.349646,   -0.331968);
    const matrix3d B23::inertie = matrix3dMaker(
    0.23894, 0.387384, 0.387384,
    0.790707, -0.51717, -0.51717,
    0.447951, -0.0712142, -0.0712142);
    Inertia B23::I = spatialInertiaMaker(  B23::mass,
    B23::CoM,
    B23::inertie);
  
    INITIALIZE_BODY(B24);
    const std::string B24::name = "B24";
    const int B24::label = 24;
    const FloatType B24::mass = 1;
    const vector3d B24::CoM = vector3d(  -0.165653,   0.478933,   -0.0463);
    const matrix3d B24::inertie = matrix3dMaker(
    0.490309, 0.0611044, 0.0611044,
    -0.506021, -0.550713, -0.550713,
    0.794111, 0.688226, 0.688226);
    Inertia B24::I = spatialInertiaMaker(  B24::mass,
    B24::CoM,
    B24::inertie);
  
    INITIALIZE_BODY(B25);
    const std::string B25::name = "B25";
    const int B25::label = 25;
    const FloatType B25::mass = 1;
    const vector3d B25::CoM = vector3d(  -0.355098,   0.273313,   0.48035);
    const matrix3d B25::inertie = matrix3dMaker(
    0.751864, 0.0296976, 0.0296976,
    0.307519, 0.289024, 0.289024,
    0.597412, -0.220667, -0.220667);
    Inertia B25::I = spatialInertiaMaker(  B25::mass,
    B25::CoM,
    B25::inertie);
  
    INITIALIZE_BODY(B26);
    const std::string B26::name = "B26";
    const int B26::label = 26;
    const FloatType B26::mass = 1;
    const vector3d B26::CoM = vector3d(  0.58184,   -0.74639,   -0.665518);
    const matrix3d B26::inertie = matrix3dMaker(
    0.568284, -0.72231, -0.72231,
    -0.53487, 0.194227, 0.194227,
    0.638203, -0.053909, -0.053909);
    Inertia B26::I = spatialInertiaMaker(  B26::mass,
    B26::CoM,
    B26::inertie);
  
    INITIALIZE_BODY(B27);
    const std::string B27::name = "B27";
    const int B27::label = 27;
    const FloatType B27::mass = 1;
    const vector3d B27::CoM = vector3d(  0.718883,   -0.35461,   -0.236795);
    const matrix3d B27::inertie = matrix3dMaker(
    0.879299, 0.576531, 0.576531,
    0.453692, -0.388025, -0.388025,
    -0.691717, -0.819741, -0.819741);
    Inertia B27::I = spatialInertiaMaker(  B27::mass,
    B27::CoM,
    B27::inertie);
  
    INITIALIZE_BODY(B28);
    const std::string B28::name = "B28";
    const int B28::label = 28;
    const FloatType B28::mass = 1;
    const vector3d B28::CoM = vector3d(  -0.13052,   -0.371914,   0.146244);
    const matrix3d B28::inertie = matrix3dMaker(
    -0.384474, -0.936694, -0.936694,
    -0.929922, 0.295095, 0.295095,
    0.4264, 0.174394, 0.174394);
    Inertia B28::I = spatialInertiaMaker(  B28::mass,
    B28::CoM,
    B28::inertie);
  
    INITIALIZE_BODY(B29);
    const std::string B29::name = "B29";
    const int B29::label = 29;
    const FloatType B29::mass = 1;
    const vector3d B29::CoM = vector3d(  -0.176898,   0.198581,   -0.103355);
    const matrix3d B29::inertie = matrix3dMaker(
    -0.834846, -0.715421, -0.715421,
    0.979083, -0.487023, -0.487023,
    -0.711064, 0.128503, 0.128503);
    Inertia B29::I = spatialInertiaMaker(  B29::mass,
    B29::CoM,
    B29::inertie);
  
    INITIALIZE_BODY(B30);
    const std::string B30::name = "B30";
    const int B30::label = 30;
    const FloatType B30::mass = 1;
    const vector3d B30::CoM = vector3d(  -0.325699,   -0.775589,   -0.351807);
    const matrix3d B30::inertie = matrix3dMaker(
    -0.240251, 0.877927, 0.877927,
    0.0152343, -0.919826, -0.919826,
    0.136152, -0.754672, -0.754672);
    Inertia B30::I = spatialInertiaMaker(  B30::mass,
    B30::CoM,
    B30::inertie);
  } // end of namespace sample_31_dof
} // end of namespace metapod
