# include "metapod/tools/bodymacros.hh"

namespace model_7_dof
{
  using namespace metapod;
  using namespace metapod::Spatial;
  // Declaration of B0 class
  CREATE_BODY(  B0,   0,   NP,   J0);
  const std::string B0::name = "B0";
  const int B0::label = 0;
  const FloatType B0::mass = 1;
  const vector3d B0::CoM = vector3d(  0.717353,   -0.12088,   0.84794);
  const matrix3d B0::inertie = matrix3dMaker(
  0.658402, -0.339326, -0.339326,
  0.786745, -0.29928, -0.29928,
  0.912937, 0.17728, 0.17728);
  Inertia B0::I = spatialInertiaMaker(  B0::mass,
  B0::CoM,
  B0::inertie);

  // Declaration of B1 class
  CREATE_BODY(  B1,   1,   B0,   J1);
  const std::string B1::name = "B1";
  const int B1::label = 1;
  const FloatType B1::mass = 1;
  const vector3d B1::CoM = vector3d(  0.762124,   0.282161,   -0.136093);
  const matrix3d B1::inertie = matrix3dMaker(
  -0.203127, 0.629534, 0.629534,
  0.821944, -0.0350187, -0.0350187,
  0.900505, 0.840257, 0.840257);
  Inertia B1::I = spatialInertiaMaker(  B1::mass,
  B1::CoM,
  B1::inertie);

  // Declaration of B2 class
  CREATE_BODY(  B2,   1,   B1,   J2);
  const std::string B2::name = "B2";
  const int B2::label = 2;
  const FloatType B2::mass = 1;
  const vector3d B2::CoM = vector3d(  -0.262673,   -0.411679,   -0.535477);
  const matrix3d B2::inertie = matrix3dMaker(
  -0.793658, -0.747849, -0.747849,
  0.52095, 0.969503, 0.969503,
  0.36889, -0.233623, -0.233623);
  Inertia B2::I = spatialInertiaMaker(  B2::mass,
  B2::CoM,
  B2::inertie);

  // Declaration of B3 class
  CREATE_BODY(  B3,   1,   B1,   J3);
  const std::string B3::name = "B3";
  const int B3::label = 3;
  const FloatType B3::mass = 1;
  const vector3d B3::CoM = vector3d(  -0.730195,   0.0404201,   -0.843536);
  const matrix3d B3::inertie = matrix3dMaker(
  -0.647579, -0.519875, -0.519875,
  0.465309, 0.313127, 0.313127,
  0.278917, 0.51947, 0.51947);
  Inertia B3::I = spatialInertiaMaker(  B3::mass,
  B3::CoM,
  B3::inertie);

  // Declaration of B4 class
  CREATE_BODY(  B4,   1,   B0,   J4);
  const std::string B4::name = "B4";
  const int B4::label = 4;
  const FloatType B4::mass = 1;
  const vector3d B4::CoM = vector3d(  -0.21662,   0.826053,   0.63939);
  const matrix3d B4::inertie = matrix3dMaker(
  0.995598, -0.891885, -0.891885,
  -0.855342, -0.991677, -0.991677,
  0.187784, -0.639255, -0.639255);
  Inertia B4::I = spatialInertiaMaker(  B4::mass,
  B4::CoM,
  B4::inertie);

  // Declaration of B5 class
  CREATE_BODY(  B5,   1,   B4,   J5);
  const std::string B5::name = "B5";
  const int B5::label = 5;
  const FloatType B5::mass = 1;
  const vector3d B5::CoM = vector3d(  0.487622,   0.806733,   0.967191);
  const matrix3d B5::inertie = matrix3dMaker(
  0.495619, 0.25782, 0.25782,
  0.495606, 0.666477, 0.666477,
  0.746543, 0.662075, 0.662075);
  Inertia B5::I = spatialInertiaMaker(  B5::mass,
  B5::CoM,
  B5::inertie);

  // Declaration of B6 class
  CREATE_BODY(  B6,   1,   B4,   J6);
  const std::string B6::name = "B6";
  const int B6::label = 6;
  const FloatType B6::mass = 1;
  const vector3d B6::CoM = vector3d(  0.0922138,   0.438537,   -0.773439);
  const matrix3d B6::inertie = matrix3dMaker(
  -0.342446, -0.537144, -0.537144,
  0.266144, -0.552687, -0.552687,
  0.0213719, 0.942931, 0.942931);
  Inertia B6::I = spatialInertiaMaker(  B6::mass,
  B6::CoM,
  B6::inertie);

} // end of namespace model_7_dof