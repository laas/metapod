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
 * 15 dof sample model, used for benchmarking.
 */

# include "sample_15_dof.hh"

template struct metapod::crba< metapod::sample_15_dof::Robot , true >;
template struct metapod::rnea< metapod::sample_15_dof::Robot , true >;
template struct metapod::crba< metapod::sample_15_dof::Robot , false >;
template struct metapod::rnea< metapod::sample_15_dof::Robot , false >;

namespace metapod
{
  namespace sample_15_dof
  {
    Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;

    INITIALIZE_JOINT_REVOLUTE(J0);
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
  
    INITIALIZE_JOINT_REVOLUTE(J1);
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
  
    INITIALIZE_JOINT_REVOLUTE(J2);
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
  
    INITIALIZE_JOINT_REVOLUTE(J3);
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
  
    INITIALIZE_JOINT_REVOLUTE(J4);
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
  
    INITIALIZE_JOINT_REVOLUTE(J5);
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
  
    INITIALIZE_JOINT_REVOLUTE(J6);
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
  
    INITIALIZE_JOINT_REVOLUTE(J7);
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
  
    INITIALIZE_JOINT_REVOLUTE(J8);
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
  
    INITIALIZE_JOINT_REVOLUTE(J9);
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
  
    INITIALIZE_JOINT_REVOLUTE(J10);
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
  
    INITIALIZE_JOINT_REVOLUTE(J11);
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
  
    INITIALIZE_JOINT_REVOLUTE(J12);
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
  
    INITIALIZE_JOINT_REVOLUTE(J13);
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
  
    INITIALIZE_JOINT_REVOLUTE(J14);
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

    INITIALIZE_BODY(B0);
    const std::string B0::name = "B0";
    const int B0::label = 0;
    const FloatType B0::mass = 1;
    const vector3d B0::CoM = vector3d(  0.369513,   0.306261,   -0.485469);
    const matrix3d B0::inertie = matrix3dMaker(
    -0.305768, -0.630755, -0.630755,
    0.254316, 0.461459, 0.461459,
    0.480877, -0.595574, -0.595574);
    Inertia B0::I = spatialInertiaMaker(  B0::mass,
    B0::CoM,
    B0::inertie);
  
    INITIALIZE_BODY(B1);
    const std::string B1::name = "B1";
    const int B1::label = 1;
    const FloatType B1::mass = 1;
    const vector3d B1::CoM = vector3d(  0.186423,   0.333113,   -0.422444);
    const matrix3d B1::inertie = matrix3dMaker(
    0.0648819, -0.824713, -0.824713,
    0.754768, 0.37225, 0.37225,
    -0.777449, -0.276798, -0.276798);
    Inertia B1::I = spatialInertiaMaker(  B1::mass,
    B1::CoM,
    B1::inertie);
  
    INITIALIZE_BODY(B2);
    const std::string B2::name = "B2";
    const int B2::label = 2;
    const FloatType B2::mass = 1;
    const vector3d B2::CoM = vector3d(  -0.592904,   0.587314,   0.0960841);
    const matrix3d B2::inertie = matrix3dMaker(
    0.529743, 0.398151, 0.398151,
    0.371572, -0.232336, -0.232336,
    0.886103, 0.832546, 0.832546);
    Inertia B2::I = spatialInertiaMaker(  B2::mass,
    B2::CoM,
    B2::inertie);
  
    INITIALIZE_BODY(B3);
    const std::string B3::name = "B3";
    const int B3::label = 3;
    const FloatType B3::mass = 1;
    const vector3d B3::CoM = vector3d(  0.448504,   -0.643585,   -0.556069);
    const matrix3d B3::inertie = matrix3dMaker(
    -0.00804521, -0.417893, -0.417893,
    0.368357, 0.455101, 0.455101,
    0.206218, -0.0151566, -0.0151566);
    Inertia B3::I = spatialInertiaMaker(  B3::mass,
    B3::CoM,
    B3::inertie);
  
    INITIALIZE_BODY(B4);
    const std::string B4::name = "B4";
    const int B4::label = 4;
    const FloatType B4::mass = 1;
    const vector3d B4::CoM = vector3d(  -0.423461,   -0.337228,   -0.817703);
    const matrix3d B4::inertie = matrix3dMaker(
    -0.211346, 0.317662, 0.317662,
    -0.482188, -0.69754, -0.69754,
    -0.784303, 0.294415, 0.294415);
    Inertia B4::I = spatialInertiaMaker(  B4::mass,
    B4::CoM,
    B4::inertie);
  
    INITIALIZE_BODY(B5);
    const std::string B5::name = "B5";
    const int B5::label = 5;
    const FloatType B5::mass = 1;
    const vector3d B5::CoM = vector3d(  -0.323514,   0.795121,   -0.727851);
    const matrix3d B5::inertie = matrix3dMaker(
    0.115121, -0.147601, -0.147601,
    -0.211223, -0.511346, -0.511346,
    0.45872, 0.277308, 0.277308);
    Inertia B5::I = spatialInertiaMaker(  B5::mass,
    B5::CoM,
    B5::inertie);
  
    INITIALIZE_BODY(B6);
    const std::string B6::name = "B6";
    const int B6::label = 6;
    const FloatType B6::mass = 1;
    const vector3d B6::CoM = vector3d(  -0.736596,   -0.896983,   -0.893155);
    const matrix3d B6::inertie = matrix3dMaker(
    -0.578234, -0.052212, -0.052212,
    -0.812161, -0.800881, -0.800881,
    -0.396474, 0.31424, 0.31424);
    Inertia B6::I = spatialInertiaMaker(  B6::mass,
    B6::CoM,
    B6::inertie);
  
    INITIALIZE_BODY(B7);
    const std::string B7::name = "B7";
    const int B7::label = 7;
    const FloatType B7::mass = 1;
    const vector3d B7::CoM = vector3d(  -0.802325,   0.847456,   -0.660701);
    const matrix3d B7::inertie = matrix3dMaker(
    -0.0468307, -0.660359, -0.660359,
    0.0514942, 0.237851, 0.237851,
    -0.532688, 0.659617, 0.659617);
    Inertia B7::I = spatialInertiaMaker(  B7::mass,
    B7::CoM,
    B7::inertie);
  
    INITIALIZE_BODY(B8);
    const std::string B8::name = "B8";
    const int B8::label = 8;
    const FloatType B8::mass = 1;
    const vector3d B8::CoM = vector3d(  0.438924,   -0.599295,   -0.197625);
    const matrix3d B8::inertie = matrix3dMaker(
    0.251928, 0.672207, 0.672207,
    -0.557981, -0.603959, -0.603959,
    -0.780535, 0.34921, 0.34921);
    Inertia B8::I = spatialInertiaMaker(  B8::mass,
    B8::CoM,
    B8::inertie);
  
    INITIALIZE_BODY(B9);
    const std::string B9::name = "B9";
    const int B9::label = 9;
    const FloatType B9::mass = 1;
    const vector3d B9::CoM = vector3d(  -0.98799,   0.0659197,   0.687819);
    const matrix3d B9::inertie = matrix3dMaker(
    -0.47911, 0.299318, 0.299318,
    0.839182, 0.371973, 0.371973,
    0.395696, -0.376099, -0.376099);
    Inertia B9::I = spatialInertiaMaker(  B9::mass,
    B9::CoM,
    B9::inertie);
  
    INITIALIZE_BODY(B10);
    const std::string B10::name = "B10";
    const int B10::label = 10;
    const FloatType B10::mass = 1;
    const vector3d B10::CoM = vector3d(  0.71511,   -0.637678,   -0.317291);
    const matrix3d B10::inertie = matrix3dMaker(
    -0.624767, 0.237917, 0.237917,
    0.135662, -0.997749, -0.997749,
    -0.389522, -0.476859, -0.476859);
    Inertia B10::I = spatialInertiaMaker(  B10::mass,
    B10::CoM,
    B10::inertie);
  
    INITIALIZE_BODY(B11);
    const std::string B11::name = "B11";
    const int B11::label = 11;
    const FloatType B11::mass = 1;
    const vector3d B11::CoM = vector3d(  0.86684,   -0.0111862,   0.105137);
    const matrix3d B11::inertie = matrix3dMaker(
    -0.210957, 0.412133, 0.412133,
    0.0947942, 0.477919, 0.477919,
    -0.533762, 0.853152, 0.853152);
    Inertia B11::I = spatialInertiaMaker(  B11::mass,
    B11::CoM,
    B11::inertie);
  
    INITIALIZE_BODY(B12);
    const std::string B12::name = "B12";
    const int B12::label = 12;
    const FloatType B12::mass = 1;
    const vector3d B12::CoM = vector3d(  0.328829,   -0.175035,   0.223961);
    const matrix3d B12::inertie = matrix3dMaker(
    0.550843, 0.58982, 0.58982,
    0.208758, -0.0588716, -0.0588716,
    0.590981, 0.730171, 0.730171);
    Inertia B12::I = spatialInertiaMaker(  B12::mass,
    B12::CoM,
    B12::inertie);
  
    INITIALIZE_BODY(B13);
    const std::string B13::name = "B13";
    const int B13::label = 13;
    const FloatType B13::mass = 1;
    const vector3d B13::CoM = vector3d(  -0.186467,   -0.965087,   0.435193);
    const matrix3d B13::inertie = matrix3dMaker(
    0.0206981, -0.903001, -0.903001,
    -0.230683, 0.275313, 0.275313,
    -0.712036, -0.173844, -0.173844);
    Inertia B13::I = spatialInertiaMaker(  B13::mass,
    B13::CoM,
    B13::inertie);
  
    INITIALIZE_BODY(B14);
    const std::string B14::name = "B14";
    const int B14::label = 14;
    const FloatType B14::mass = 1;
    const vector3d B14::CoM = vector3d(  0.534027,   -0.332862,   0.073485);
    const matrix3d B14::inertie = matrix3dMaker(
    0.260486, 0.847025, 0.847025,
    -0.0742955, -0.122876, -0.122876,
    0.905325, 0.897822, 0.897822);
    Inertia B14::I = spatialInertiaMaker(  B14::mass,
    B14::CoM,
    B14::inertie);
  } // end of namespace sample_15_dof
} // end of namespace metapod
