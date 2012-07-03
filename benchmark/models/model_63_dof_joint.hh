# include "metapod/tools/jointmacros.hh"

namespace model_63_dof
{
using namespace metapod;
using namespace metapod::Spatial;
  JOINT_REVOLUTE(J0);
  const std::string J0::name = "J0";
  const int J0::label = 0;
  const int J0::positionInConf = 0;
  const Transform J0::Xt = Transform(
    matrix3dMaker(
      0.116395, 0.681608, -0.865075,
      0.407141, -0.558643, -0.987149,
      -0.912217, 0.456591, -0.906975),
    vector3d(
      0.480901, -0.106248, -0.250051));

  JOINT_REVOLUTE(J1);
  const std::string J1::name = "J1";
  const int J1::label = 1;
  const int J1::positionInConf = 1;
  const Transform J1::Xt = Transform(
    matrix3dMaker(
      0.0665052, -0.121585, 0.435557,
      0.159531, -0.183166, -0.9717,
      0.497884, -0.109531, 0.295345),
    vector3d(
      0.053654, -0.209368, 0.978966));

  JOINT_REVOLUTE(J2);
  const std::string J2::name = "J2";
  const int J2::label = 2;
  const int J2::positionInConf = 2;
  const Transform J2::Xt = Transform(
    matrix3dMaker(
      0.897416, 0.746193, -0.490187,
      0.435025, -0.200153, 0.300445,
      0.413991, 0.866352, -0.821141),
    vector3d(
      -0.958584, -0.172536, 0.126322));

  JOINT_REVOLUTE(J3);
  const std::string J3::name = "J3";
  const int J3::label = 3;
  const int J3::positionInConf = 3;
  const Transform J3::Xt = Transform(
    matrix3dMaker(
      0.0897709, -0.65034, 0.852588,
      0.948997, -0.608924, -0.319949,
      0.0753196, -0.711507, -0.573756),
    vector3d(
      0.0167388, 0.270492, 0.193904));

  JOINT_REVOLUTE(J4);
  const std::string J4::name = "J4";
  const int J4::label = 4;
  const int J4::positionInConf = 4;
  const Transform J4::Xt = Transform(
    matrix3dMaker(
      0.444753, 0.986221, -0.0523182,
      0.0540346, 0.00295953, -0.781826,
      -0.752061, -0.90727, -0.432166),
    vector3d(
      0.946293, 0.0380956, 0.226508));

  JOINT_REVOLUTE(J5);
  const std::string J5::name = "J5";
  const int J5::label = 5;
  const int J5::positionInConf = 5;
  const Transform J5::Xt = Transform(
    matrix3dMaker(
      0.643684, -0.287676, -0.0396133,
      -0.714222, -0.341383, 0.998482,
      0.512286, -0.89663, 0.984703),
    vector3d(
      0.525358, -0.347806, -0.904516));

  JOINT_REVOLUTE(J6);
  const std::string J6::name = "J6";
  const int J6::label = 6;
  const int J6::positionInConf = 6;
  const Transform J6::Xt = Transform(
    matrix3dMaker(
      -0.576791, 0.926184, -0.470895,
      -0.468363, 0.451542, 0.181299,
      -0.37288, 0.0952259, 0.893623),
    vector3d(
      0.452288, 0.49327, -0.0586519));

  JOINT_REVOLUTE(J7);
  const std::string J7::name = "J7";
  const int J7::label = 7;
  const int J7::positionInConf = 7;
  const Transform J7::Xt = Transform(
    matrix3dMaker(
      -0.239131, 0.48226, 0.0106776,
      3.71966e-05, -0.0654517, -0.496052,
      0.941385, 0.357757, -0.569868),
    vector3d(
      0.638748, 0.277374, -0.275771));

  JOINT_REVOLUTE(J8);
  const std::string J8::name = "J8";
  const int J8::label = 8;
  const int J8::positionInConf = 8;
  const Transform J8::Xt = Transform(
    matrix3dMaker(
      -0.433723, 0.819057, -0.364152,
      0.784636, 0.457805, 0.913222,
      -0.491135, -0.781326, 0.395482),
    vector3d(
      -0.246738, 0.260702, -0.39555));

  JOINT_REVOLUTE(J9);
  const std::string J9::name = "J9";
  const int J9::label = 9;
  const int J9::positionInConf = 9;
  const Transform J9::Xt = Transform(
    matrix3dMaker(
      0.6701, -0.0745602, 0.887725,
      -0.078707, 0.678701, 0.148427,
      0.525743, -0.755022, -0.0325153),
    vector3d(
      0.312197, -0.172212, -0.802285));

  JOINT_REVOLUTE(J10);
  const std::string J10::name = "J10";
  const int J10::label = 10;
  const int J10::positionInConf = 10;
  const Transform J10::Xt = Transform(
    matrix3dMaker(
      0.0652932, -0.129568, 0.935736,
      0.158988, -0.817372, -0.236476,
      0.356703, 0.852729, 0.688964),
    vector3d(
      0.488112, -0.464865, -0.34664));

  JOINT_REVOLUTE(J11);
  const std::string J11::name = "J11";
  const int J11::label = 11;
  const int J11::positionInConf = 11;
  const Transform J11::Xt = Transform(
    matrix3dMaker(
      -0.707729, 0.806787, -0.679731,
      0.315043, 0.294899, -0.144596,
      0.968403, -0.639807, 0.725836),
    vector3d(
      -0.570405, -0.993538, -0.761242));

  JOINT_REVOLUTE(J12);
  const std::string J12::name = "J12";
  const int J12::label = 12;
  const int J12::positionInConf = 12;
  const Transform J12::Xt = Transform(
    matrix3dMaker(
      -0.0208053, 0.659137, 0.832501,
      0.159931, -0.911268, 0.838963,
      0.398688, -0.618997, 0.64575),
    vector3d(
      0.336937, 0.873507, 0.296333));

  JOINT_REVOLUTE(J13);
  const std::string J13::name = "J13";
  const int J13::label = 13;
  const int J13::positionInConf = 13;
  const Transform J13::Xt = Transform(
    matrix3dMaker(
      -0.494931, -0.682826, -0.670616,
      0.340048, 0.654111, -0.797109,
      -0.363618, -0.366694, 0.862028),
    vector3d(
      0.969855, -0.407119, 0.816217));

  JOINT_REVOLUTE(J14);
  const std::string J14::name = "J14";
  const int J14::label = 14;
  const int J14::positionInConf = 14;
  const Transform J14::Xt = Transform(
    matrix3dMaker(
      0.01511, 0.994105, 0.497101,
      -0.288984, 0.96396, -0.910018,
      -0.472767, -0.530971, -0.592844),
    vector3d(
      -0.804913, 0.785443, 0.260849));

  JOINT_REVOLUTE(J15);
  const std::string J15::name = "J15";
  const int J15::label = 15;
  const int J15::positionInConf = 15;
  const Transform J15::Xt = Transform(
    matrix3dMaker(
      0.231871, -0.929972, -0.924136,
      -0.07587, -0.734885, 0.861307,
      -0.815021, 0.280225, 0.855412),
    vector3d(
      0.204061, 0.175782, -0.409381));

  JOINT_REVOLUTE(J16);
  const std::string J16::name = "J16";
  const int J16::label = 16;
  const int J16::positionInConf = 16;
  const Transform J16::Xt = Transform(
    matrix3dMaker(
      0.422869, -0.771567, -0.320536,
      0.102604, 0.432494, 0.855246,
      0.693223, -0.335636, 0.925274),
    vector3d(
      -0.0649414, 0.356136, 0.457277));

  JOINT_REVOLUTE(J17);
  const std::string J17::name = "J17";
  const int J17::label = 17;
  const int J17::positionInConf = 17;
  const Transform J17::Xt = Transform(
    matrix3dMaker(
      0.691053, 0.647318, 0.790019,
      -0.766011, -0.417624, 0.146155,
      0.691266, -0.994755, 0.374588),
    vector3d(
      0.593576, 0.406832, 0.35542));

  JOINT_REVOLUTE(J18);
  const std::string J18::name = "J18";
  const int J18::label = 18;
  const int J18::positionInConf = 18;
  const Transform J18::Xt = Transform(
    matrix3dMaker(
      0.274108, -0.00846565, 0.516579,
      0.254437, -0.414889, -0.0765884,
      -0.390143, -0.723836, -0.429271),
    vector3d(
      0.812253, 0.686358, -0.711035));

  JOINT_REVOLUTE(J19);
  const std::string J19::name = "J19";
  const int J19::label = 19;
  const int J19::positionInConf = 19;
  const Transform J19::Xt = Transform(
    matrix3dMaker(
      -0.740915, 0.291013, 0.837985,
      0.0924969, 0.103266, 0.524343,
      0.381461, -0.622626, -0.484123),
    vector3d(
      0.199145, -0.0506257, 0.637432));

  JOINT_REVOLUTE(J20);
  const std::string J20::name = "J20";
  const int J20::label = 20;
  const int J20::positionInConf = 20;
  const Transform J20::Xt = Transform(
    matrix3dMaker(
      -0.0301611, -0.164106, 0.458997,
      -0.962888, -0.96496, -0.591629,
      0.674544, -0.705876, 0.699384),
    vector3d(
      -0.296361, 0.108194, -0.339128));

  JOINT_REVOLUTE(J21);
  const std::string J21::name = "J21";
  const int J21::label = 21;
  const int J21::positionInConf = 21;
  const Transform J21::Xt = Transform(
    matrix3dMaker(
      0.918192, 0.425045, 0.882622,
      -0.993084, -0.871317, -0.00918401,
      -0.332211, 0.0985221, 0.82671),
    vector3d(
      -0.969112, 0.879757, 0.695047));

  JOINT_REVOLUTE(J22);
  const std::string J22::name = "J22";
  const int J22::label = 22;
  const int J22::positionInConf = 22;
  const Transform J22::Xt = Transform(
    matrix3dMaker(
      0.0290497, 0.566061, 0.464277,
      -0.917448, 0.596949, 0.344034,
      0.777599, 0.515141, -0.230922),
    vector3d(
      -0.782785, -0.087675, 0.976701));

  JOINT_REVOLUTE(J23);
  const std::string J23::name = "J23";
  const int J23::label = 23;
  const int J23::positionInConf = 23;
  const Transform J23::Xt = Transform(
    matrix3dMaker(
      -0.962134, -0.876479, 0.624377,
      -0.651556, -0.659264, -0.463298,
      -0.674855, 0.369786, -0.897237),
    vector3d(
      -0.504629, 0.0863297, -0.7352));

  JOINT_REVOLUTE(J24);
  const std::string J24::name = "J24";
  const int J24::label = 24;
  const int J24::positionInConf = 24;
  const Transform J24::Xt = Transform(
    matrix3dMaker(
      0.928497, -0.641563, -0.904824,
      0.870592, -0.146193, 0.181506,
      -0.864608, -0.108327, 0.305027),
    vector3d(
      -0.203421, -0.98106, 0.243388));

  JOINT_REVOLUTE(J25);
  const std::string J25::name = "J25";
  const int J25::label = 25;
  const int J25::positionInConf = 25;
  const Transform J25::Xt = Transform(
    matrix3dMaker(
      -0.94929, 0.190969, -0.870825,
      -0.0225338, 0.987548, -0.851885,
      -0.779146, 0.916045, -0.493448),
    vector3d(
      0.313799, -0.609327, -0.412018));

  JOINT_REVOLUTE(J26);
  const std::string J26::name = "J26";
  const int J26::label = 26;
  const int J26::positionInConf = 26;
  const Transform J26::Xt = Transform(
    matrix3dMaker(
      0.931765, -0.52355, -0.0997668,
      0.629589, 0.790249, 0.290906,
      -0.78243, 0.840959, -0.518125),
    vector3d(
      0.463877, -0.622497, -0.587906));

  JOINT_REVOLUTE(J27);
  const std::string J27::name = "J27";
  const int J27::label = 27;
  const int J27::positionInConf = 27;
  const Transform J27::Xt = Transform(
    matrix3dMaker(
      -0.894046, -0.896262, 0.754198,
      0.163364, 0.567615, -0.868299,
      0.575459, 0.499379, -0.391849),
    vector3d(
      -0.386195, 0.65541, 0.775929));

  JOINT_REVOLUTE(J28);
  const std::string J28::name = "J28";
  const int J28::label = 28;
  const int J28::positionInConf = 28;
  const Transform J28::Xt = Transform(
    matrix3dMaker(
      0.275465, 0.551331, -0.69478,
      0.369597, -0.834865, 0.96063,
      0.145526, -0.728911, -0.935632),
    vector3d(
      0.103362, 0.189558, 0.452373));

  JOINT_REVOLUTE(J29);
  const std::string J29::name = "J29";
  const int J29::label = 29;
  const int J29::positionInConf = 29;
  const Transform J29::Xt = Transform(
    matrix3dMaker(
      0.335119, -0.781431, 0.488299,
      0.432182, 0.321931, -0.322143,
      -0.115444, -0.402603, -0.770812),
    vector3d(
      -0.722665, 0.500674, 0.300153));

  JOINT_REVOLUTE(J30);
  const std::string J30::name = "J30";
  const int J30::label = 30;
  const int J30::positionInConf = 30;
  const Transform J30::Xt = Transform(
    matrix3dMaker(
      -0.761552, -0.278779, 0.847583,
      0.845317, -0.00144476, 0.348258,
      0.14547, -0.666326, 0.566827),
    vector3d(
      0.978118, -0.316755, -0.404618));

  JOINT_REVOLUTE(J31);
  const std::string J31::name = "J31";
  const int J31::label = 31;
  const int J31::positionInConf = 31;
  const Transform J31::Xt = Transform(
    matrix3dMaker(
      -0.299585, 0.0521224, 0.621582,
      0.810193, 0.0302401, -0.695173,
      -0.594425, 0.268688, 0.0260473),
    vector3d(
      -0.399451, -0.794634, -0.422087));

  JOINT_REVOLUTE(J32);
  const std::string J32::name = "J32";
  const int J32::label = 32;
  const int J32::positionInConf = 32;
  const Transform J32::Xt = Transform(
    matrix3dMaker(
      -0.0036948, 0.680819, 0.605326,
      0.561974, -0.718632, 0.810692,
      -0.860112, -0.018217, -0.137185),
    vector3d(
      0.543156, 0.745682, -0.385267));

  JOINT_REVOLUTE(J33);
  const std::string J33::name = "J33";
  const int J33::label = 33;
  const int J33::positionInConf = 33;
  const Transform J33::Xt = Transform(
    matrix3dMaker(
      0.599356, -0.722633, 0.633335,
      -0.626099, 0.820523, 0.379017,
      -0.0113657, -0.183172, 0.059836),
    vector3d(
      0.475223, -0.779634, 0.428466));

  JOINT_REVOLUTE(J34);
  const std::string J34::name = "J34";
  const int J34::label = 34;
  const int J34::positionInConf = 34;
  const Transform J34::Xt = Transform(
    matrix3dMaker(
      -0.401986, -0.444254, -0.184654,
      0.133236, -0.969031, 0.0357119,
      -0.438298, 0.630325, 0.313079),
    vector3d(
      -0.677734, 0.4388, 0.763222));

  JOINT_REVOLUTE(J35);
  const std::string J35::name = "J35";
  const int J35::label = 35;
  const int J35::positionInConf = 35;
  const Transform J35::Xt = Transform(
    matrix3dMaker(
      0.413044, -0.580536, 0.474539,
      0.725557, -0.258271, -0.0866607,
      0.488779, 0.339743, 0.469085),
    vector3d(
      -0.701919, 0.258941, 0.128083));

  JOINT_REVOLUTE(J36);
  const std::string J36::name = "J36";
  const int J36::label = 36;
  const int J36::positionInConf = 36;
  const Transform J36::Xt = Transform(
    matrix3dMaker(
      -0.933819, 0.412756, 0.705759,
      -0.0858952, 0.710838, -0.0352994,
      -0.957812, 0.123882, 0.384164),
    vector3d(
      0.736948, 0.437662, 0.714286));

  JOINT_REVOLUTE(J37);
  const std::string J37::name = "J37";
  const int J37::label = 37;
  const int J37::positionInConf = 37;
  const Transform J37::Xt = Transform(
    matrix3dMaker(
      0.66419, -0.578387, 0.00678713,
      0.791655, -0.841439, -0.555551,
      0.505941, -0.775258, 0.857206),
    vector3d(
      -0.470415, -0.364905, -0.796985));

  JOINT_REVOLUTE(J38);
  const std::string J38::name = "J38";
  const int J38::label = 38;
  const int J38::positionInConf = 38;
  const Transform J38::Xt = Transform(
    matrix3dMaker(
      0.0907114, -0.573812, -0.495895,
      -0.935992, -0.0442269, 0.139201,
      -0.732977, -0.380037, 0.560814),
    vector3d(
      0.152096, 0.0269817, -0.478074));

  JOINT_REVOLUTE(J39);
  const std::string J39::name = "J39";
  const int J39::label = 39;
  const int J39::positionInConf = 39;
  const Transform J39::Xt = Transform(
    matrix3dMaker(
      0.376799, -0.447313, -0.910408,
      0.256382, 0.704783, 0.116574,
      0.778308, -0.204506, 0.542762),
    vector3d(
      0.835434, -0.133772, -0.0382377));

  JOINT_REVOLUTE(J40);
  const std::string J40::name = "J40";
  const int J40::label = 40;
  const int J40::positionInConf = 40;
  const Transform J40::Xt = Transform(
    matrix3dMaker(
      0.728884, -0.16243, 0.882599,
      0.992883, -0.326995, -0.251174,
      -0.0453545, -0.950196, 0.301513),
    vector3d(
      -0.354081, 0.694061, -0.423063));

  JOINT_REVOLUTE(J41);
  const std::string J41::name = "J41";
  const int J41::label = 41;
  const int J41::positionInConf = 41;
  const Transform J41::Xt = Transform(
    matrix3dMaker(
      -0.149279, -0.52725, -0.69325,
      -0.701603, 0.118669, -0.999189,
      -0.124666, -0.152447, -0.161619),
    vector3d(
      0.320605, 0.770948, -0.811845));

  JOINT_REVOLUTE(J42);
  const std::string J42::name = "J42";
  const int J42::label = 42;
  const int J42::positionInConf = 42;
  const Transform J42::Xt = Transform(
    matrix3dMaker(
      -0.722349, -0.0941903, -0.892705,
      -0.93453, -0.773585, 0.878242,
      -0.746375, 0.0771358, -0.649008),
    vector3d(
      0.616358, -0.296297, 0.318774));

  JOINT_REVOLUTE(J43);
  const std::string J43::name = "J43";
  const int J43::label = 43;
  const int J43::positionInConf = 43;
  const Transform J43::Xt = Transform(
    matrix3dMaker(
      0.339518, 0.138396, -0.207818,
      0.291386, -0.245246, 0.495884,
      -0.38984, 0.0324058, -0.598306),
    vector3d(
      -0.527857, 0.323828, 0.792052));

  JOINT_REVOLUTE(J44);
  const std::string J44::name = "J44";
  const int J44::label = 44;
  const int J44::positionInConf = 44;
  const Transform J44::Xt = Transform(
    matrix3dMaker(
      0.891216, -0.216417, -0.144001,
      -0.156051, 0.255726, -0.820173,
      -0.363999, -0.404756, 0.318223),
    vector3d(
      -0.266434, 0.87263, -0.659558));

  JOINT_REVOLUTE(J45);
  const std::string J45::name = "J45";
  const int J45::label = 45;
  const int J45::positionInConf = 45;
  const Transform J45::Xt = Transform(
    matrix3dMaker(
      -0.260109, 0.172982, 0.939698,
      -0.928465, 0.906548, 0.812328,
      -0.588023, 0.797764, -0.404089),
    vector3d(
      0.659114, -0.243369, 0.466636));

  JOINT_REVOLUTE(J46);
  const std::string J46::name = "J46";
  const int J46::label = 46;
  const int J46::positionInConf = 46;
  const Transform J46::Xt = Transform(
    matrix3dMaker(
      -0.852352, -0.0904063, 0.854488,
      0.702408, -0.431292, -0.388881,
      0.169044, 0.308599, 0.784101),
    vector3d(
      0.944388, -0.0442757, -0.955454));

  JOINT_REVOLUTE(J47);
  const std::string J47::name = "J47";
  const int J47::label = 47;
  const int J47::positionInConf = 47;
  const Transform J47::Xt = Transform(
    matrix3dMaker(
      -0.850989, -0.887078, 0.743702,
      -0.0206882, -0.94269, -0.300574,
      0.0238575, -0.795043, 0.60902),
    vector3d(
      0.8155, 0.273917, -0.886262));

  JOINT_REVOLUTE(J48);
  const std::string J48::name = "J48";
  const int J48::label = 48;
  const int J48::positionInConf = 48;
  const Transform J48::Xt = Transform(
    matrix3dMaker(
      -0.417245, 0.991824, -0.773453,
      0.990597, 0.807324, 0.500465,
      -0.895665, 0.956335, 0.613386),
    vector3d(
      -0.547372, -0.376795, 0.718657));

  JOINT_REVOLUTE(J49);
  const std::string J49::name = "J49";
  const int J49::label = 49;
  const int J49::positionInConf = 49;
  const Transform J49::Xt = Transform(
    matrix3dMaker(
      -0.313006, -0.434906, -0.849198,
      0.345565, 0.0177225, -0.225993,
      0.0642217, 0.600478, -0.234169),
    vector3d(
      0.820535, 0.600879, 0.0445793));

  JOINT_REVOLUTE(J50);
  const std::string J50::name = "J50";
  const int J50::label = 50;
  const int J50::positionInConf = 50;
  const Transform J50::Xt = Transform(
    matrix3dMaker(
      0.733986, -0.418401, -0.410198,
      -0.406587, -0.597866, -0.809318,
      0.637993, 0.0891278, -0.244224),
    vector3d(
      0.950968, -0.75261, -0.737511));

  JOINT_REVOLUTE(J51);
  const std::string J51::name = "J51";
  const int J51::label = 51;
  const int J51::positionInConf = 51;
  const Transform J51::Xt = Transform(
    matrix3dMaker(
      0.896412, -0.939148, -0.705186,
      0.321051, -0.988179, -0.457796,
      0.58354, 0.745807, 0.123803),
    vector3d(
      -0.568724, 0.985537, -0.796094));

  JOINT_REVOLUTE(J52);
  const std::string J52::name = "J52";
  const int J52::label = 52;
  const int J52::positionInConf = 52;
  const Transform J52::Xt = Transform(
    matrix3dMaker(
      0.232325, 0.559229, -0.0807291,
      0.892786, 0.990505, -0.0951923,
      -0.903308, 0.886917, -0.03434),
    vector3d(
      -0.732726, 0.657442, 0.338354));

  JOINT_REVOLUTE(J53);
  const std::string J53::name = "J53";
  const int J53::label = 53;
  const int J53::positionInConf = 53;
  const Transform J53::Xt = Transform(
    matrix3dMaker(
      0.248272, 0.341975, 0.134538,
      0.796943, 0.609249, -0.20802,
      0.135297, -0.158426, -0.648791),
    vector3d(
      -0.801485, -0.551324, -0.194272));

  JOINT_REVOLUTE(J54);
  const std::string J54::name = "J54";
  const int J54::label = 54;
  const int J54::positionInConf = 54;
  const Transform J54::Xt = Transform(
    matrix3dMaker(
      -0.311689, 0.797234, -0.478163,
      0.728032, 0.995749, -0.0294869,
      -0.46624, 0.244021, -0.687512),
    vector3d(
      0.625803, -0.786217, 0.192961));

  JOINT_REVOLUTE(J55);
  const std::string J55::name = "J55";
  const int J55::label = 55;
  const int J55::positionInConf = 55;
  const Transform J55::Xt = Transform(
    matrix3dMaker(
      -0.181566, -0.982246, 0.707552,
      0.0465426, 0.643557, 0.921335,
      -0.760497, -0.668131, 0.71857),
    vector3d(
      0.381834, 0.115238, 0.551282));

  JOINT_REVOLUTE(J56);
  const std::string J56::name = "J56";
  const int J56::label = 56;
  const int J56::positionInConf = 56;
  const Transform J56::Xt = Transform(
    matrix3dMaker(
      -0.772939, -0.313073, 0.596595,
      -0.444215, -0.931239, -0.288167,
      -0.892933, -0.112806, -0.270413),
    vector3d(
      -0.270611, 0.533725, -0.593608));

  JOINT_REVOLUTE(J57);
  const std::string J57::name = "J57";
  const int J57::label = 57;
  const int J57::positionInConf = 57;
  const Transform J57::Xt = Transform(
    matrix3dMaker(
      -0.873514, -0.323148, -0.65723,
      -0.69795, 0.406241, 0.876495,
      -0.291559, 0.633302, -0.436578),
    vector3d(
      -0.872818, -0.289226, 0.620533));

  JOINT_REVOLUTE(J58);
  const std::string J58::name = "J58";
  const int J58::label = 58;
  const int J58::positionInConf = 58;
  const Transform J58::Xt = Transform(
    matrix3dMaker(
      0.589526, -0.529905, 0.227367,
      -0.0519005, -0.402723, 0.938141,
      -0.431368, -0.276237, -0.385007),
    vector3d(
      -0.175387, 0.301252, -0.439066));

  JOINT_REVOLUTE(J59);
  const std::string J59::name = "J59";
  const int J59::label = 59;
  const int J59::positionInConf = 59;
  const Transform J59::Xt = Transform(
    matrix3dMaker(
      -0.0440082, -0.0777877, -0.902722,
      -0.417655, 0.746825, 0.39853,
      0.143279, 0.336351, 0.868624),
    vector3d(
      0.502878, -0.197387, -0.74945));

  JOINT_REVOLUTE(J60);
  const std::string J60::name = "J60";
  const int J60::label = 60;
  const int J60::positionInConf = 60;
  const Transform J60::Xt = Transform(
    matrix3dMaker(
      -0.556826, 0.887201, 0.0451766,
      -0.417049, 0.390079, 0.847789,
      -0.166499, -0.653929, -0.229998),
    vector3d(
      -0.40045, 0.9037, 0.989019));

  JOINT_REVOLUTE(J61);
  const std::string J61::name = "J61";
  const int J61::label = 61;
  const int J61::positionInConf = 61;
  const Transform J61::Xt = Transform(
    matrix3dMaker(
      -0.931025, -0.380078, 0.307574,
      -0.981295, 0.219472, 0.211274,
      -0.992276, 0.662646, 0.0984751),
    vector3d(
      0.853498, 0.674908, -0.244807));

  JOINT_REVOLUTE(J62);
  const std::string J62::name = "J62";
  const int J62::label = 62;
  const int J62::positionInConf = 62;
  const Transform J62::Xt = Transform(
    matrix3dMaker(
      -0.821678, 0.200951, 0.0282066,
      0.169031, 0.054449, -0.296886,
      0.924224, 0.123424, 0.323036),
    vector3d(
      -0.959305, -0.462883, -0.722921));

} // end of namespace model_63_dof