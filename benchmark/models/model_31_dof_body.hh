# include "metapod/tools/bodymacros.hh"

namespace model_31_dof
{
  using namespace metapod;
  using namespace metapod::Spatial;
  // Declaration of B0 class
  CREATE_BODY(  B0,   0,   NP,   J0);
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

  // Declaration of B1 class
  CREATE_BODY(  B1,   1,   B0,   J1);
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

  // Declaration of B2 class
  CREATE_BODY(  B2,   1,   B1,   J2);
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

  // Declaration of B3 class
  CREATE_BODY(  B3,   1,   B2,   J3);
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

  // Declaration of B4 class
  CREATE_BODY(  B4,   1,   B3,   J4);
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

  // Declaration of B5 class
  CREATE_BODY(  B5,   1,   B3,   J5);
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

  // Declaration of B6 class
  CREATE_BODY(  B6,   1,   B2,   J6);
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

  // Declaration of B7 class
  CREATE_BODY(  B7,   1,   B6,   J7);
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

  // Declaration of B8 class
  CREATE_BODY(  B8,   1,   B6,   J8);
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

  // Declaration of B9 class
  CREATE_BODY(  B9,   1,   B1,   J9);
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

  // Declaration of B10 class
  CREATE_BODY(  B10,   1,   B9,   J10);
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

  // Declaration of B11 class
  CREATE_BODY(  B11,   1,   B10,   J11);
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

  // Declaration of B12 class
  CREATE_BODY(  B12,   1,   B10,   J12);
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

  // Declaration of B13 class
  CREATE_BODY(  B13,   1,   B9,   J13);
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

  // Declaration of B14 class
  CREATE_BODY(  B14,   1,   B13,   J14);
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

  // Declaration of B15 class
  CREATE_BODY(  B15,   1,   B13,   J15);
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

  // Declaration of B16 class
  CREATE_BODY(  B16,   1,   B0,   J16);
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

  // Declaration of B17 class
  CREATE_BODY(  B17,   1,   B16,   J17);
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

  // Declaration of B18 class
  CREATE_BODY(  B18,   1,   B17,   J18);
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

  // Declaration of B19 class
  CREATE_BODY(  B19,   1,   B18,   J19);
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

  // Declaration of B20 class
  CREATE_BODY(  B20,   1,   B18,   J20);
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

  // Declaration of B21 class
  CREATE_BODY(  B21,   1,   B17,   J21);
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

  // Declaration of B22 class
  CREATE_BODY(  B22,   1,   B21,   J22);
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

  // Declaration of B23 class
  CREATE_BODY(  B23,   1,   B21,   J23);
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

  // Declaration of B24 class
  CREATE_BODY(  B24,   1,   B16,   J24);
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

  // Declaration of B25 class
  CREATE_BODY(  B25,   1,   B24,   J25);
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

  // Declaration of B26 class
  CREATE_BODY(  B26,   1,   B25,   J26);
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

  // Declaration of B27 class
  CREATE_BODY(  B27,   1,   B25,   J27);
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

  // Declaration of B28 class
  CREATE_BODY(  B28,   1,   B24,   J28);
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

  // Declaration of B29 class
  CREATE_BODY(  B29,   1,   B28,   J29);
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

  // Declaration of B30 class
  CREATE_BODY(  B30,   1,   B28,   J30);
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

} // end of namespace model_31_dof