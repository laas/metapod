# include "metapod/tools/bodymacros.hh"

namespace model_63_dof
{
  using namespace metapod;
  using namespace metapod::Spatial;
  // Declaration of B0 class
  CREATE_BODY(  B0,   0,   NP,   J0);
  const std::string B0::name = "B0";
  const int B0::label = 0;
  const FloatType B0::mass = 1;
  const vector3d B0::CoM = vector3d(  -0.501476,   0.293681,   -0.58744);
  const matrix3d B0::inertie = matrix3dMaker(
  0.93842, -0.407257, -0.407257,
  -0.926365, 0.267044, 0.267044,
  -0.278172, 0.479588, 0.479588);
  Inertia B0::I = spatialInertiaMaker(  B0::mass,
  B0::CoM,
  B0::inertie);

  // Declaration of B1 class
  CREATE_BODY(  B1,   1,   B0,   J1);
  const std::string B1::name = "B1";
  const int B1::label = 1;
  const FloatType B1::mass = 1;
  const vector3d B1::CoM = vector3d(  -0.0812705,   0.383491,   0.537609);
  const matrix3d B1::inertie = matrix3dMaker(
  0.473802, -0.995582, -0.995582,
  0.0740604, -0.213805, -0.213805,
  -0.83122, -0.732904, -0.732904);
  Inertia B1::I = spatialInertiaMaker(  B1::mass,
  B1::CoM,
  B1::inertie);

  // Declaration of B2 class
  CREATE_BODY(  B2,   1,   B1,   J2);
  const std::string B2::name = "B2";
  const int B2::label = 2;
  const FloatType B2::mass = 1;
  const vector3d B2::CoM = vector3d(  0.0791679,   -0.341317,   -0.140773);
  const matrix3d B2::inertie = matrix3dMaker(
  -0.939352, 0.612297, 0.612297,
  0.136759, -0.889179, -0.889179,
  0.549319, 0.584623, 0.584623);
  Inertia B2::I = spatialInertiaMaker(  B2::mass,
  B2::CoM,
  B2::inertie);

  // Declaration of B3 class
  CREATE_BODY(  B3,   1,   B2,   J3);
  const std::string B3::name = "B3";
  const int B3::label = 3;
  const FloatType B3::mass = 1;
  const vector3d B3::CoM = vector3d(  -0.0518749,   0.721174,   0.609281);
  const matrix3d B3::inertie = matrix3dMaker(
  -0.150453, 0.0258828, 0.0258828,
  -0.122152, -0.476233, -0.476233,
  -0.826808, -0.415585, -0.415585);
  Inertia B3::I = spatialInertiaMaker(  B3::mass,
  B3::CoM,
  B3::inertie);

  // Declaration of B4 class
  CREATE_BODY(  B4,   1,   B3,   J4);
  const std::string B4::name = "B4";
  const int B4::label = 4;
  const FloatType B4::mass = 1;
  const vector3d B4::CoM = vector3d(  -0.93987,   -0.135097,   -0.357907);
  const matrix3d B4::inertie = matrix3dMaker(
  0.585132, 0.723517, 0.723517,
  -0.114423, 0.137508, 0.137508,
  0.0644367, 0.987055, 0.987055);
  Inertia B4::I = spatialInertiaMaker(  B4::mass,
  B4::CoM,
  B4::inertie);

  // Declaration of B5 class
  CREATE_BODY(  B5,   1,   B4,   J5);
  const std::string B5::name = "B5";
  const int B5::label = 5;
  const FloatType B5::mass = 1;
  const vector3d B5::CoM = vector3d(  -0.567086,   0.587757,   -0.891572);
  const matrix3d B5::inertie = matrix3dMaker(
  -0.899473, -0.958272, -0.958272,
  -0.219422, 0.117047, 0.117047,
  0.206822, -0.29782, -0.29782);
  Inertia B5::I = spatialInertiaMaker(  B5::mass,
  B5::CoM,
  B5::inertie);

  // Declaration of B6 class
  CREATE_BODY(  B6,   1,   B4,   J6);
  const std::string B6::name = "B6";
  const int B6::label = 6;
  const FloatType B6::mass = 1;
  const vector3d B6::CoM = vector3d(  -0.795114,   -0.713552,   -0.760832);
  const matrix3d B6::inertie = matrix3dMaker(
  -0.540032, 0.157405, 0.157405,
  -0.321858, 0.405344, 0.405344,
  0.245976, 0.50587, 0.50587);
  Inertia B6::I = spatialInertiaMaker(  B6::mass,
  B6::CoM,
  B6::inertie);

  // Declaration of B7 class
  CREATE_BODY(  B7,   1,   B3,   J7);
  const std::string B7::name = "B7";
  const int B7::label = 7;
  const FloatType B7::mass = 1;
  const vector3d B7::CoM = vector3d(  -0.441645,   -0.968602,   0.218359);
  const matrix3d B7::inertie = matrix3dMaker(
  0.587507, 0.381003, 0.381003,
  0.585989, -0.106711, -0.106711,
  0.570692, 0.353257, 0.353257);
  Inertia B7::I = spatialInertiaMaker(  B7::mass,
  B7::CoM,
  B7::inertie);

  // Declaration of B8 class
  CREATE_BODY(  B8,   1,   B7,   J8);
  const std::string B8::name = "B8";
  const int B8::label = 8;
  const FloatType B8::mass = 1;
  const vector3d B8::CoM = vector3d(  -0.902348,   0.69001,   0.251193);
  const matrix3d B8::inertie = matrix3dMaker(
  -0.52951, 0.889394, 0.889394,
  0.651789, -0.483486, -0.483486,
  0.545412, -0.895979, -0.895979);
  Inertia B8::I = spatialInertiaMaker(  B8::mass,
  B8::CoM,
  B8::inertie);

  // Declaration of B9 class
  CREATE_BODY(  B9,   1,   B7,   J9);
  const std::string B9::name = "B9";
  const int B9::label = 9;
  const FloatType B9::mass = 1;
  const vector3d B9::CoM = vector3d(  -0.664703,   0.282376,   -0.906305);
  const matrix3d B9::inertie = matrix3dMaker(
  0.519543, 0.218712, 0.218712,
  -0.976509, 0.160097, 0.160097,
  -0.546377, 0.630587, 0.630587);
  Inertia B9::I = spatialInertiaMaker(  B9::mass,
  B9::CoM,
  B9::inertie);

  // Declaration of B10 class
  CREATE_BODY(  B10,   1,   B2,   J10);
  const std::string B10::name = "B10";
  const int B10::label = 10;
  const FloatType B10::mass = 1;
  const vector3d B10::CoM = vector3d(  0.800325,   -0.918488,   0.0227727);
  const matrix3d B10::inertie = matrix3dMaker(
  -0.838409, -0.970386, -0.970386,
  -0.925187, -0.461521, -0.461521,
  0.470295, -0.941979, -0.941979);
  Inertia B10::I = spatialInertiaMaker(  B10::mass,
  B10::CoM,
  B10::inertie);

  // Declaration of B11 class
  CREATE_BODY(  B11,   1,   B10,   J11);
  const std::string B11::name = "B11";
  const int B11::label = 11;
  const FloatType B11::mass = 1;
  const vector3d B11::CoM = vector3d(  0.78563,   -0.463833,   -0.819264);
  const matrix3d B11::inertie = matrix3dMaker(
  0.244428, -0.225978, -0.225978,
  -0.607145, -0.700236, -0.700236,
  0.36034, -0.538645, -0.538645);
  Inertia B11::I = spatialInertiaMaker(  B11::mass,
  B11::CoM,
  B11::inertie);

  // Declaration of B12 class
  CREATE_BODY(  B12,   1,   B11,   J12);
  const std::string B12::name = "B12";
  const int B12::label = 12;
  const FloatType B12::mass = 1;
  const vector3d B12::CoM = vector3d(  0.724293,   -0.486833,   -0.165021);
  const matrix3d B12::inertie = matrix3dMaker(
  0.904139, 0.51918, 0.51918,
  -0.332336, -0.124117, -0.124117,
  -0.643372, -0.879689, -0.879689);
  Inertia B12::I = spatialInertiaMaker(  B12::mass,
  B12::CoM,
  B12::inertie);

  // Declaration of B13 class
  CREATE_BODY(  B13,   1,   B11,   J13);
  const std::string B13::name = "B13";
  const int B13::label = 13;
  const FloatType B13::mass = 1;
  const vector3d B13::CoM = vector3d(  -0.791338,   -0.763747,   0.695906);
  const matrix3d B13::inertie = matrix3dMaker(
  0.718958, 0.696046, 0.696046,
  -0.425638, 0.66445, 0.66445,
  -0.699802, 0.568589, 0.568589);
  Inertia B13::I = spatialInertiaMaker(  B13::mass,
  B13::CoM,
  B13::inertie);

  // Declaration of B14 class
  CREATE_BODY(  B14,   1,   B10,   J14);
  const std::string B14::name = "B14";
  const int B14::label = 14;
  const FloatType B14::mass = 1;
  const vector3d B14::CoM = vector3d(  -0.105755,   0.485245,   0.692259);
  const matrix3d B14::inertie = matrix3dMaker(
  -0.531117, 0.793237, 0.793237,
  -0.692154, 0.191925, 0.191925,
  0.953596, -0.0891168, -0.0891168);
  Inertia B14::I = spatialInertiaMaker(  B14::mass,
  B14::CoM,
  B14::inertie);

  // Declaration of B15 class
  CREATE_BODY(  B15,   1,   B14,   J15);
  const std::string B15::name = "B15";
  const int B15::label = 15;
  const FloatType B15::mass = 1;
  const vector3d B15::CoM = vector3d(  -0.127703,   0.222187,   0.679735);
  const matrix3d B15::inertie = matrix3dMaker(
  -0.143383, 0.809077, 0.809077,
  0.0595081, -0.554541, -0.554541,
  -0.0784635, -0.0856588, -0.0856588);
  Inertia B15::I = spatialInertiaMaker(  B15::mass,
  B15::CoM,
  B15::inertie);

  // Declaration of B16 class
  CREATE_BODY(  B16,   1,   B14,   J16);
  const std::string B16::name = "B16";
  const int B16::label = 16;
  const FloatType B16::mass = 1;
  const vector3d B16::CoM = vector3d(  -0.759515,   -0.565401,   -0.457064);
  const matrix3d B16::inertie = matrix3dMaker(
  0.68208, 0.991241, 0.991241,
  0.772062, -0.481526, -0.481526,
  -0.820782, 0.375091, 0.375091);
  Inertia B16::I = spatialInertiaMaker(  B16::mass,
  B16::CoM,
  B16::inertie);

  // Declaration of B17 class
  CREATE_BODY(  B17,   1,   B1,   J17);
  const std::string B17::name = "B17";
  const int B17::label = 17;
  const FloatType B17::mass = 1;
  const vector3d B17::CoM = vector3d(  0.305176,   0.227614,   0.980329);
  const matrix3d B17::inertie = matrix3dMaker(
  0.769087, 0.588494, 0.588494,
  0.630394, 0.773473, 0.773473,
  0.485805, 0.455553, 0.455553);
  Inertia B17::I = spatialInertiaMaker(  B17::mass,
  B17::CoM,
  B17::inertie);

  // Declaration of B18 class
  CREATE_BODY(  B18,   1,   B17,   J18);
  const std::string B18::name = "B18";
  const int B18::label = 18;
  const FloatType B18::mass = 1;
  const vector3d B18::CoM = vector3d(  -0.658361,   -0.799447,   -0.166588);
  const matrix3d B18::inertie = matrix3dMaker(
  -0.62927, 0.107849, 0.107849,
  -0.774024, -0.198928, -0.198928,
  -0.84875, -0.429841, -0.429841);
  Inertia B18::I = spatialInertiaMaker(  B18::mass,
  B18::CoM,
  B18::inertie);

  // Declaration of B19 class
  CREATE_BODY(  B19,   1,   B18,   J19);
  const std::string B19::name = "B19";
  const int B19::label = 19;
  const FloatType B19::mass = 1;
  const vector3d B19::CoM = vector3d(  0.7277,   -0.201875,   0.0672727);
  const matrix3d B19::inertie = matrix3dMaker(
  -0.600124, -0.489847, -0.489847,
  0.546031, -0.798581, -0.798581,
  -0.0793812, -0.427851, -0.427851);
  Inertia B19::I = spatialInertiaMaker(  B19::mass,
  B19::CoM,
  B19::inertie);

  // Declaration of B20 class
  CREATE_BODY(  B20,   1,   B19,   J20);
  const std::string B20::name = "B20";
  const int B20::label = 20;
  const FloatType B20::mass = 1;
  const vector3d B20::CoM = vector3d(  0.545288,   -0.812425,   -0.911276);
  const matrix3d B20::inertie = matrix3dMaker(
  -0.101959, 0.631811, 0.631811,
  0.821452, -0.758332, -0.758332,
  -0.607818, -0.358456, -0.358456);
  Inertia B20::I = spatialInertiaMaker(  B20::mass,
  B20::CoM,
  B20::inertie);

  // Declaration of B21 class
  CREATE_BODY(  B21,   1,   B19,   J21);
  const std::string B21::name = "B21";
  const int B21::label = 21;
  const FloatType B21::mass = 1;
  const vector3d B21::CoM = vector3d(  -0.346264,   0.487575,   0.0535024);
  const matrix3d B21::inertie = matrix3dMaker(
  0.512528, 0.386621, 0.386621,
  0.0368711, -0.231917, -0.231917,
  0.552748, 0.666123, 0.666123);
  Inertia B21::I = spatialInertiaMaker(  B21::mass,
  B21::CoM,
  B21::inertie);

  // Declaration of B22 class
  CREATE_BODY(  B22,   1,   B18,   J22);
  const std::string B22::name = "B22";
  const int B22::label = 22;
  const FloatType B22::mass = 1;
  const vector3d B22::CoM = vector3d(  0.0371914,   0.359577,   -0.689422);
  const matrix3d B22::inertie = matrix3dMaker(
  -0.873214, 0.135634, 0.135634,
  -0.464843, -0.189823, -0.189823,
  -0.765459, -0.677294, -0.677294);
  Inertia B22::I = spatialInertiaMaker(  B22::mass,
  B22::CoM,
  B22::inertie);

  // Declaration of B23 class
  CREATE_BODY(  B23,   1,   B22,   J23);
  const std::string B23::name = "B23";
  const int B23::label = 23;
  const FloatType B23::mass = 1;
  const vector3d B23::CoM = vector3d(  -0.660504,   -0.148212,   0.942095);
  const matrix3d B23::inertie = matrix3dMaker(
  0.66022, 0.522057, 0.522057,
  -0.348964, -0.810154, -0.810154,
  -0.522253, -0.683368, -0.683368);
  Inertia B23::I = spatialInertiaMaker(  B23::mass,
  B23::CoM,
  B23::inertie);

  // Declaration of B24 class
  CREATE_BODY(  B24,   1,   B22,   J24);
  const std::string B24::name = "B24";
  const int B24::label = 24;
  const FloatType B24::mass = 1;
  const vector3d B24::CoM = vector3d(  -0.199704,   0.541194,   -0.0732437);
  const matrix3d B24::inertie = matrix3dMaker(
  0.789422, 0.452338, 0.452338,
  0.133456, 0.229937, 0.229937,
  0.902534, -0.109843, -0.109843);
  Inertia B24::I = spatialInertiaMaker(  B24::mass,
  B24::CoM,
  B24::inertie);

  // Declaration of B25 class
  CREATE_BODY(  B25,   1,   B17,   J25);
  const std::string B25::name = "B25";
  const int B25::label = 25;
  const FloatType B25::mass = 1;
  const vector3d B25::CoM = vector3d(  -0.901054,   -0.511861,   0.697824);
  const matrix3d B25::inertie = matrix3dMaker(
  0.759769, 0.240117, 0.240117,
  -0.703529, 0.565262, 0.565262,
  -0.600765, 0.354683, 0.354683);
  Inertia B25::I = spatialInertiaMaker(  B25::mass,
  B25::CoM,
  B25::inertie);

  // Declaration of B26 class
  CREATE_BODY(  B26,   1,   B25,   J26);
  const std::string B26::name = "B26";
  const int B26::label = 26;
  const FloatType B26::mass = 1;
  const vector3d B26::CoM = vector3d(  -0.551673,   0.978269,   0.0574108);
  const matrix3d B26::inertie = matrix3dMaker(
  -0.68397, 0.786636, 0.786636,
  0.497537, 0.922028, 0.922028,
  -0.197436, 0.681797, 0.681797);
  Inertia B26::I = spatialInertiaMaker(  B26::mass,
  B26::CoM,
  B26::inertie);

  // Declaration of B27 class
  CREATE_BODY(  B27,   1,   B26,   J27);
  const std::string B27::name = "B27";
  const int B27::label = 27;
  const FloatType B27::mass = 1;
  const vector3d B27::CoM = vector3d(  -0.638228,   -0.147154,   -0.905868);
  const matrix3d B27::inertie = matrix3dMaker(
  -0.653254, -0.181575, -0.181575,
  -0.505139, 0.0392793, 0.0392793,
  0.00141311, 0.35531, 0.35531);
  Inertia B27::I = spatialInertiaMaker(  B27::mass,
  B27::CoM,
  B27::inertie);

  // Declaration of B28 class
  CREATE_BODY(  B28,   1,   B26,   J28);
  const std::string B28::name = "B28";
  const int B28::label = 28;
  const FloatType B28::mass = 1;
  const vector3d B28::CoM = vector3d(  -0.282105,   -0.811855,   -0.902937);
  const matrix3d B28::inertie = matrix3dMaker(
  -0.524308, 0.128968, 0.128968,
  0.766598, 0.346539, 0.346539,
  -0.751527, 0.693284, 0.693284);
  Inertia B28::I = spatialInertiaMaker(  B28::mass,
  B28::CoM,
  B28::inertie);

  // Declaration of B29 class
  CREATE_BODY(  B29,   1,   B25,   J29);
  const std::string B29::name = "B29";
  const int B29::label = 29;
  const FloatType B29::mass = 1;
  const vector3d B29::CoM = vector3d(  -0.962024,   0.252202,   0.606869);
  const matrix3d B29::inertie = matrix3dMaker(
  -0.100276, 0.434453, 0.434453,
  0.031425, 0.00991204, 0.00991204,
  0.639576, 0.485604, 0.485604);
  Inertia B29::I = spatialInertiaMaker(  B29::mass,
  B29::CoM,
  B29::inertie);

  // Declaration of B30 class
  CREATE_BODY(  B30,   1,   B29,   J30);
  const std::string B30::name = "B30";
  const int B30::label = 30;
  const FloatType B30::mass = 1;
  const vector3d B30::CoM = vector3d(  -0.153244,   0.0436689,   0.109778);
  const matrix3d B30::inertie = matrix3dMaker(
  0.189775, 0.966994, 0.966994,
  0.150406, 0.11252, 0.11252,
  0.214773, -0.987756, -0.987756);
  Inertia B30::I = spatialInertiaMaker(  B30::mass,
  B30::CoM,
  B30::inertie);

  // Declaration of B31 class
  CREATE_BODY(  B31,   1,   B29,   J31);
  const std::string B31::name = "B31";
  const int B31::label = 31;
  const FloatType B31::mass = 1;
  const vector3d B31::CoM = vector3d(  0.935137,   -0.0094069,   -0.434331);
  const matrix3d B31::inertie = matrix3dMaker(
  -0.366231, 0.765857, 0.765857,
  0.311626, -0.349588, -0.349588,
  0.540813, 0.840188, 0.840188);
  Inertia B31::I = spatialInertiaMaker(  B31::mass,
  B31::CoM,
  B31::inertie);

  // Declaration of B32 class
  CREATE_BODY(  B32,   1,   B0,   J32);
  const std::string B32::name = "B32";
  const int B32::label = 32;
  const FloatType B32::mass = 1;
  const vector3d B32::CoM = vector3d(  -0.942999,   -0.795131,   -0.225455);
  const matrix3d B32::inertie = matrix3dMaker(
  -0.746842, 0.114005, 0.114005,
  0.601416, -0.740525, -0.740525,
  0.168242, -0.106757, -0.106757);
  Inertia B32::I = spatialInertiaMaker(  B32::mass,
  B32::CoM,
  B32::inertie);

  // Declaration of B33 class
  CREATE_BODY(  B33,   1,   B32,   J33);
  const std::string B33::name = "B33";
  const int B33::label = 33;
  const FloatType B33::mass = 1;
  const vector3d B33::CoM = vector3d(  0.116946,   0.0521239,   -0.464777);
  const matrix3d B33::inertie = matrix3dMaker(
  0.76147, -0.208024, -0.208024,
  -0.933704, 0.19755, 0.19755,
  0.0923436, 0.450708, 0.450708);
  Inertia B33::I = spatialInertiaMaker(  B33::mass,
  B33::CoM,
  B33::inertie);

  // Declaration of B34 class
  CREATE_BODY(  B34,   1,   B33,   J34);
  const std::string B34::name = "B34";
  const int B34::label = 34;
  const FloatType B34::mass = 1;
  const vector3d B34::CoM = vector3d(  0.160523,   -0.653544,   -0.687487);
  const matrix3d B34::inertie = matrix3dMaker(
  -0.40604, -0.621197, -0.621197,
  -0.595347, -0.48131, -0.48131,
  0.267468, -0.71984, -0.71984);
  Inertia B34::I = spatialInertiaMaker(  B34::mass,
  B34::CoM,
  B34::inertie);

  // Declaration of B35 class
  CREATE_BODY(  B35,   1,   B34,   J35);
  const std::string B35::name = "B35";
  const int B35::label = 35;
  const FloatType B35::mass = 1;
  const vector3d B35::CoM = vector3d(  0.975094,   0.991473,   -0.152077);
  const matrix3d B35::inertie = matrix3dMaker(
  -0.804963, -0.995774, -0.995774,
  0.574054, -0.00713963, -0.00713963,
  -0.36611, 0.586821, 0.586821);
  Inertia B35::I = spatialInertiaMaker(  B35::mass,
  B35::CoM,
  B35::inertie);

  // Declaration of B36 class
  CREATE_BODY(  B36,   1,   B35,   J36);
  const std::string B36::name = "B36";
  const int B36::label = 36;
  const FloatType B36::mass = 1;
  const vector3d B36::CoM = vector3d(  0.786518,   -0.196228,   -0.872535);
  const matrix3d B36::inertie = matrix3dMaker(
  -0.695875, -0.527021, -0.527021,
  0.339837, 0.0346812, 0.0346812,
  -0.347084, 0.229718, 0.229718);
  Inertia B36::I = spatialInertiaMaker(  B36::mass,
  B36::CoM,
  B36::inertie);

  // Declaration of B37 class
  CREATE_BODY(  B37,   1,   B35,   J37);
  const std::string B37::name = "B37";
  const int B37::label = 37;
  const FloatType B37::mass = 1;
  const vector3d B37::CoM = vector3d(  0.399206,   0.98218,   -0.0267033);
  const matrix3d B37::inertie = matrix3dMaker(
  0.516728, -0.150561, -0.150561,
  -0.569933, -0.661782, -0.661782,
  0.899152, -0.357657, -0.357657);
  Inertia B37::I = spatialInertiaMaker(  B37::mass,
  B37::CoM,
  B37::inertie);

  // Declaration of B38 class
  CREATE_BODY(  B38,   1,   B34,   J38);
  const std::string B38::name = "B38";
  const int B38::label = 38;
  const FloatType B38::mass = 1;
  const vector3d B38::CoM = vector3d(  0.686459,   0.12783,   0.879583);
  const matrix3d B38::inertie = matrix3dMaker(
  0.211701, 0.138847, 0.138847,
  -0.823599, 0.181035, 0.181035,
  0.560566, -0.302237, -0.302237);
  Inertia B38::I = spatialInertiaMaker(  B38::mass,
  B38::CoM,
  B38::inertie);

  // Declaration of B39 class
  CREATE_BODY(  B39,   1,   B38,   J39);
  const std::string B39::name = "B39";
  const int B39::label = 39;
  const FloatType B39::mass = 1;
  const vector3d B39::CoM = vector3d(  0.143509,   0.305662,   -0.736);
  const matrix3d B39::inertie = matrix3dMaker(
  0.27381, -0.588382, -0.588382,
  0.71826, 0.91756, 0.91756,
  0.575466, 0.129261, 0.129261);
  Inertia B39::I = spatialInertiaMaker(  B39::mass,
  B39::CoM,
  B39::inertie);

  // Declaration of B40 class
  CREATE_BODY(  B40,   1,   B38,   J40);
  const std::string B40::name = "B40";
  const int B40::label = 40;
  const FloatType B40::mass = 1;
  const vector3d B40::CoM = vector3d(  -0.298198,   -0.881405,   0.447676);
  const matrix3d B40::inertie = matrix3dMaker(
  -0.717587, -0.140497, -0.140497,
  0.421614, 0.126526, 0.126526,
  -0.0175725, -0.599664, -0.599664);
  Inertia B40::I = spatialInertiaMaker(  B40::mass,
  B40::CoM,
  B40::inertie);

  // Declaration of B41 class
  CREATE_BODY(  B41,   1,   B33,   J41);
  const std::string B41::name = "B41";
  const int B41::label = 41;
  const FloatType B41::mass = 1;
  const vector3d B41::CoM = vector3d(  -0.797893,   -0.21148,   0.787819);
  const matrix3d B41::inertie = matrix3dMaker(
  0.0442373, 0.306185, 0.306185,
  -0.839189, 0.0844931, 0.0844931,
  0.703573, 0.366906, 0.366906);
  Inertia B41::I = spatialInertiaMaker(  B41::mass,
  B41::CoM,
  B41::inertie);

  // Declaration of B42 class
  CREATE_BODY(  B42,   1,   B41,   J42);
  const std::string B42::name = "B42";
  const int B42::label = 42;
  const FloatType B42::mass = 1;
  const vector3d B42::CoM = vector3d(  0.814568,   0.000130012,   0.951868);
  const matrix3d B42::inertie = matrix3dMaker(
  -0.242068, -0.159564, -0.159564,
  0.506758, 0.795082, 0.795082,
  -0.191728, -0.160681, -0.160681);
  Inertia B42::I = spatialInertiaMaker(  B42::mass,
  B42::CoM,
  B42::inertie);

  // Declaration of B43 class
  CREATE_BODY(  B43,   1,   B42,   J43);
  const std::string B43::name = "B43";
  const int B43::label = 43;
  const FloatType B43::mass = 1;
  const vector3d B43::CoM = vector3d(  -0.0890465,   -0.484443,   -0.0472674);
  const matrix3d B43::inertie = matrix3dMaker(
  -0.439625, 0.375533, 0.375533,
  -0.438814, -0.749134, -0.749134,
  0.399568, 0.0087985, 0.0087985);
  Inertia B43::I = spatialInertiaMaker(  B43::mass,
  B43::CoM,
  B43::inertie);

  // Declaration of B44 class
  CREATE_BODY(  B44,   1,   B42,   J44);
  const std::string B44::name = "B44";
  const int B44::label = 44;
  const FloatType B44::mass = 1;
  const vector3d B44::CoM = vector3d(  -0.583649,   -0.526938,   0.331644);
  const matrix3d B44::inertie = matrix3dMaker(
  -0.282545, 0.097876, 0.097876,
  -0.404303, 0.351501, 0.351501,
  -0.0533106, 0.911876, 0.911876);
  Inertia B44::I = spatialInertiaMaker(  B44::mass,
  B44::CoM,
  B44::inertie);

  // Declaration of B45 class
  CREATE_BODY(  B45,   1,   B41,   J45);
  const std::string B45::name = "B45";
  const int B45::label = 45;
  const FloatType B45::mass = 1;
  const vector3d B45::CoM = vector3d(  0.953869,   0.809942,   0.55476);
  const matrix3d B45::inertie = matrix3dMaker(
  0.428182, 0.88663, 0.88663,
  -0.0759332, -0.50321, -0.50321,
  0.325761, 0.214245, 0.214245);
  Inertia B45::I = spatialInertiaMaker(  B45::mass,
  B45::CoM,
  B45::inertie);

  // Declaration of B46 class
  CREATE_BODY(  B46,   1,   B45,   J46);
  const std::string B46::name = "B46";
  const int B46::label = 46;
  const FloatType B46::mass = 1;
  const vector3d B46::CoM = vector3d(  -0.160996,   0.629964,   -0.169699);
  const matrix3d B46::inertie = matrix3dMaker(
  0.267976, -0.358287, -0.358287,
  0.447803, 0.277714, 0.277714,
  -0.233973, -0.294103, -0.294103);
  Inertia B46::I = spatialInertiaMaker(  B46::mass,
  B46::CoM,
  B46::inertie);

  // Declaration of B47 class
  CREATE_BODY(  B47,   1,   B45,   J47);
  const std::string B47::name = "B47";
  const int B47::label = 47;
  const FloatType B47::mass = 1;
  const vector3d B47::CoM = vector3d(  0.368619,   -0.49211,   0.407841);
  const matrix3d B47::inertie = matrix3dMaker(
  0.108743, 0.380134, 0.380134,
  -0.0789292, 0.792111, 0.792111,
  0.516982, 0.0600869, 0.0600869);
  Inertia B47::I = spatialInertiaMaker(  B47::mass,
  B47::CoM,
  B47::inertie);

  // Declaration of B48 class
  CREATE_BODY(  B48,   1,   B32,   J48);
  const std::string B48::name = "B48";
  const int B48::label = 48;
  const FloatType B48::mass = 1;
  const vector3d B48::CoM = vector3d(  -0.0357851,   0.106223,   -0.34143);
  const matrix3d B48::inertie = matrix3dMaker(
  -0.121655, 0.907365, 0.907365,
  0.489464, 0.0764094, 0.0764094,
  0.273566, -0.814848, -0.814848);
  Inertia B48::I = spatialInertiaMaker(  B48::mass,
  B48::CoM,
  B48::inertie);

  // Declaration of B49 class
  CREATE_BODY(  B49,   1,   B48,   J49);
  const std::string B49::name = "B49";
  const int B49::label = 49;
  const FloatType B49::mass = 1;
  const vector3d B49::CoM = vector3d(  -0.665791,   -0.672686,   -0.140573);
  const matrix3d B49::inertie = matrix3dMaker(
  0.848036, -0.0643535, -0.0643535,
  -0.452538, 0.959504, 0.959504,
  -0.843518, -0.162151, -0.162151);
  Inertia B49::I = spatialInertiaMaker(  B49::mass,
  B49::CoM,
  B49::inertie);

  // Declaration of B50 class
  CREATE_BODY(  B50,   1,   B49,   J50);
  const std::string B50::name = "B50";
  const int B50::label = 50;
  const FloatType B50::mass = 1;
  const vector3d B50::CoM = vector3d(  -0.924684,   -0.909092,   0.42464);
  const matrix3d B50::inertie = matrix3dMaker(
  0.290769, 0.591075, 0.591075,
  -0.208766, 0.69541, 0.69541,
  -0.59538, 0.543446, 0.543446);
  Inertia B50::I = spatialInertiaMaker(  B50::mass,
  B50::CoM,
  B50::inertie);

  // Declaration of B51 class
  CREATE_BODY(  B51,   1,   B50,   J51);
  const std::string B51::name = "B51";
  const int B51::label = 51;
  const FloatType B51::mass = 1;
  const vector3d B51::CoM = vector3d(  0.901787,   0.580917,   -0.33954);
  const matrix3d B51::inertie = matrix3dMaker(
  0.788795, -0.565308, -0.565308,
  -0.437198, 0.498914, 0.498914,
  0.328632, -0.210317, -0.210317);
  Inertia B51::I = spatialInertiaMaker(  B51::mass,
  B51::CoM,
  B51::inertie);

  // Declaration of B52 class
  CREATE_BODY(  B52,   1,   B50,   J52);
  const std::string B52::name = "B52";
  const int B52::label = 52;
  const FloatType B52::mass = 1;
  const vector3d B52::CoM = vector3d(  -0.106702,   -0.67119,   -0.451329);
  const matrix3d B52::inertie = matrix3dMaker(
  -0.826657, -0.66078, -0.66078,
  -0.635976, 0.977213, 0.977213,
  0.1198, 0.766008, 0.766008);
  Inertia B52::I = spatialInertiaMaker(  B52::mass,
  B52::CoM,
  B52::inertie);

  // Declaration of B53 class
  CREATE_BODY(  B53,   1,   B49,   J53);
  const std::string B53::name = "B53";
  const int B53::label = 53;
  const FloatType B53::mass = 1;
  const vector3d B53::CoM = vector3d(  0.583451,   0.328876,   0.0397207);
  const matrix3d B53::inertie = matrix3dMaker(
  -0.608494, 0.207969, 0.207969,
  -0.0662892, -0.208491, -0.208491,
  -0.942486, -0.0351483, -0.0351483);
  Inertia B53::I = spatialInertiaMaker(  B53::mass,
  B53::CoM,
  B53::inertie);

  // Declaration of B54 class
  CREATE_BODY(  B54,   1,   B53,   J54);
  const std::string B54::name = "B54";
  const int B54::label = 54;
  const FloatType B54::mass = 1;
  const vector3d B54::CoM = vector3d(  -0.0974841,   -0.843731,   -0.771891);
  const matrix3d B54::inertie = matrix3dMaker(
  -0.945432, -0.26564, -0.26564,
  -0.0406248, -0.168948, -0.168948,
  0.925035, 0.222559, 0.222559);
  Inertia B54::I = spatialInertiaMaker(  B54::mass,
  B54::CoM,
  B54::inertie);

  // Declaration of B55 class
  CREATE_BODY(  B55,   1,   B53,   J55);
  const std::string B55::name = "B55";
  const int B55::label = 55;
  const FloatType B55::mass = 1;
  const vector3d B55::CoM = vector3d(  0.153202,   0.190203,   -0.671276);
  const matrix3d B55::inertie = matrix3dMaker(
  0.668298, 0.0409637, 0.0409637,
  -0.539722, -0.82374, -0.82374,
  -0.188512, -0.769172, -0.769172);
  Inertia B55::I = spatialInertiaMaker(  B55::mass,
  B55::CoM,
  B55::inertie);

  // Declaration of B56 class
  CREATE_BODY(  B56,   1,   B48,   J56);
  const std::string B56::name = "B56";
  const int B56::label = 56;
  const FloatType B56::mass = 1;
  const vector3d B56::CoM = vector3d(  0.966078,   -0.277763,   -0.824436);
  const matrix3d B56::inertie = matrix3dMaker(
  -0.23866, -0.9401, -0.9401,
  0.731853, -0.406339, -0.406339,
  -0.955659, -0.738041, -0.738041);
  Inertia B56::I = spatialInertiaMaker(  B56::mass,
  B56::CoM,
  B56::inertie);

  // Declaration of B57 class
  CREATE_BODY(  B57,   1,   B56,   J57);
  const std::string B57::name = "B57";
  const int B57::label = 57;
  const FloatType B57::mass = 1;
  const vector3d B57::CoM = vector3d(  0.168843,   -0.333567,   0.358574);
  const matrix3d B57::inertie = matrix3dMaker(
  0.814619, 0.933737, 0.933737,
  0.735955, -0.82676, -0.82676,
  0.454524, -0.06542, -0.06542);
  Inertia B57::I = spatialInertiaMaker(  B57::mass,
  B57::CoM,
  B57::inertie);

  // Declaration of B58 class
  CREATE_BODY(  B58,   1,   B57,   J58);
  const std::string B58::name = "B58";
  const int B58::label = 58;
  const FloatType B58::mass = 1;
  const vector3d B58::CoM = vector3d(  -0.8804,   0.846728,   0.626354);
  const matrix3d B58::inertie = matrix3dMaker(
  -0.694964, -0.810913, -0.810913,
  0.0168684, -0.703846, -0.703846,
  0.746455, -0.889227, -0.889227);
  Inertia B58::I = spatialInertiaMaker(  B58::mass,
  B58::CoM,
  B58::inertie);

  // Declaration of B59 class
  CREATE_BODY(  B59,   1,   B57,   J59);
  const std::string B59::name = "B59";
  const int B59::label = 59;
  const FloatType B59::mass = 1;
  const vector3d B59::CoM = vector3d(  0.983501,   0.0561574,   -0.860223);
  const matrix3d B59::inertie = matrix3dMaker(
  -0.088598, 0.0258124, 0.0258124,
  -0.212103, 0.734253, 0.734253,
  0.351319, -0.960711, -0.960711);
  Inertia B59::I = spatialInertiaMaker(  B59::mass,
  B59::CoM,
  B59::inertie);

  // Declaration of B60 class
  CREATE_BODY(  B60,   1,   B56,   J60);
  const std::string B60::name = "B60";
  const int B60::label = 60;
  const FloatType B60::mass = 1;
  const vector3d B60::CoM = vector3d(  -0.054986,   -0.447619,   0.94973);
  const matrix3d B60::inertie = matrix3dMaker(
  -0.629354, -0.71555, -0.71555,
  -0.691213, -0.146918, -0.146918,
  -0.0762202, 0.764484, 0.764484);
  Inertia B60::I = spatialInertiaMaker(  B60::mass,
  B60::CoM,
  B60::inertie);

  // Declaration of B61 class
  CREATE_BODY(  B61,   1,   B60,   J61);
  const std::string B61::name = "B61";
  const int B61::label = 61;
  const FloatType B61::mass = 1;
  const vector3d B61::CoM = vector3d(  -0.336166,   -0.248872,   -0.00929083);
  const matrix3d B61::inertie = matrix3dMaker(
  -0.0692213, -0.0715836, -0.0715836,
  -0.670692, -0.928305, -0.928305,
  -0.802067, -0.557659, -0.557659);
  Inertia B61::I = spatialInertiaMaker(  B61::mass,
  B61::CoM,
  B61::inertie);

  // Declaration of B62 class
  CREATE_BODY(  B62,   1,   B60,   J62);
  const std::string B62::name = "B62";
  const int B62::label = 62;
  const FloatType B62::mass = 1;
  const vector3d B62::CoM = vector3d(  -0.812482,   -0.660816,   0.834737);
  const matrix3d B62::inertie = matrix3dMaker(
  0.0529006, -0.754404, -0.754404,
  -0.0993102, 0.0790972, 0.0790972,
  0.670691, -0.990124, -0.990124);
  Inertia B62::I = spatialInertiaMaker(  B62::mass,
  B62::CoM,
  B62::inertie);

} // end of namespace model_63_dof