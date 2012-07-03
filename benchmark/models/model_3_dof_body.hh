# include "metapod/tools/bodymacros.hh"

namespace model_3_dof
{
  using namespace metapod;
  using namespace metapod::Spatial;
  // Declaration of B0 class
  CREATE_BODY(  B0,   0,   NP,   J0);
  const std::string B0::name = "B0";
  const int B0::label = 0;
  const FloatType B0::mass = 1;
  const vector3d B0::CoM = vector3d(  -0.514226,   -0.725537,   0.608354);
  const matrix3d B0::inertie = matrix3dMaker(
  -0.270431, 0.0268018, 0.0268018,
  0.83239, 0.271423, 0.271423,
  -0.716795, 0.213938, 0.213938);
  Inertia B0::I = spatialInertiaMaker(  B0::mass,
  B0::CoM,
  B0::inertie);

  // Declaration of B1 class
  CREATE_BODY(  B1,   1,   B0,   J1);
  const std::string B1::name = "B1";
  const int B1::label = 1;
  const FloatType B1::mass = 1;
  const vector3d B1::CoM = vector3d(  -0.407937,   0.275105,   0.0485744);
  const matrix3d B1::inertie = matrix3dMaker(
  -0.686642, -0.198111, -0.198111,
  -0.782382, 0.997849, 0.997849,
  0.0258648, 0.678224, 0.678224);
  Inertia B1::I = spatialInertiaMaker(  B1::mass,
  B1::CoM,
  B1::inertie);

  // Declaration of B2 class
  CREATE_BODY(  B2,   1,   B0,   J2);
  const std::string B2::name = "B2";
  const int B2::label = 2;
  const FloatType B2::mass = 1;
  const vector3d B2::CoM = vector3d(  -0.959954,   -0.0845965,   -0.873808);
  const matrix3d B2::inertie = matrix3dMaker(
  -0.860489, 0.898654, 0.898654,
  -0.827888, -0.615572, -0.615572,
  0.780465, -0.302214, -0.302214);
  Inertia B2::I = spatialInertiaMaker(  B2::mass,
  B2::CoM,
  B2::inertie);

} // end of namespace model_3_dof