# include "metapod/tools/bodymacros.hh"

namespace model_15_dof
{
  using namespace metapod;
  using namespace metapod::Spatial;
  // Declaration of B0 class
  CREATE_BODY(  B0,   0,   NP,   J0);
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

  // Declaration of B1 class
  CREATE_BODY(  B1,   1,   B0,   J1);
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

  // Declaration of B2 class
  CREATE_BODY(  B2,   1,   B1,   J2);
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

  // Declaration of B3 class
  CREATE_BODY(  B3,   1,   B2,   J3);
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

  // Declaration of B4 class
  CREATE_BODY(  B4,   1,   B2,   J4);
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

  // Declaration of B5 class
  CREATE_BODY(  B5,   1,   B1,   J5);
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

  // Declaration of B6 class
  CREATE_BODY(  B6,   1,   B5,   J6);
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

  // Declaration of B7 class
  CREATE_BODY(  B7,   1,   B5,   J7);
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

  // Declaration of B8 class
  CREATE_BODY(  B8,   1,   B0,   J8);
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

  // Declaration of B9 class
  CREATE_BODY(  B9,   1,   B8,   J9);
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

  // Declaration of B10 class
  CREATE_BODY(  B10,   1,   B9,   J10);
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

  // Declaration of B11 class
  CREATE_BODY(  B11,   1,   B9,   J11);
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

  // Declaration of B12 class
  CREATE_BODY(  B12,   1,   B8,   J12);
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

  // Declaration of B13 class
  CREATE_BODY(  B13,   1,   B12,   J13);
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

  // Declaration of B14 class
  CREATE_BODY(  B14,   1,   B12,   J14);
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

} // end of namespace model_15_dof