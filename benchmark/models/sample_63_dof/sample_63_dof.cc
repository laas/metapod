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
 * 63 dof sample model, used for benchmarking.
 */

# include "sample_63_dof.hh"

template struct metapod::crba< metapod::sample_63_dof::Robot , true >;
template struct metapod::rnea< metapod::sample_63_dof::Robot , true >;
template struct metapod::crba< metapod::sample_63_dof::Robot , false >;
template struct metapod::rnea< metapod::sample_63_dof::Robot , false >;

namespace metapod
{
  namespace sample_63_dof
  {
    Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;

    INITIALIZE_JOINT_REVOLUTE(J0);
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
  
    INITIALIZE_JOINT_REVOLUTE(J1);
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
  
    INITIALIZE_JOINT_REVOLUTE(J2);
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
  
    INITIALIZE_JOINT_REVOLUTE(J3);
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
  
    INITIALIZE_JOINT_REVOLUTE(J4);
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
  
    INITIALIZE_JOINT_REVOLUTE(J5);
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
  
    INITIALIZE_JOINT_REVOLUTE(J6);
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
  
    INITIALIZE_JOINT_REVOLUTE(J7);
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
  
    INITIALIZE_JOINT_REVOLUTE(J8);
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
  
    INITIALIZE_JOINT_REVOLUTE(J9);
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
  
    INITIALIZE_JOINT_REVOLUTE(J10);
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
  
    INITIALIZE_JOINT_REVOLUTE(J11);
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
  
    INITIALIZE_JOINT_REVOLUTE(J12);
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
  
    INITIALIZE_JOINT_REVOLUTE(J13);
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
  
    INITIALIZE_JOINT_REVOLUTE(J14);
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
  
    INITIALIZE_JOINT_REVOLUTE(J15);
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
  
    INITIALIZE_JOINT_REVOLUTE(J16);
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
  
    INITIALIZE_JOINT_REVOLUTE(J17);
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
  
    INITIALIZE_JOINT_REVOLUTE(J18);
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
  
    INITIALIZE_JOINT_REVOLUTE(J19);
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
  
    INITIALIZE_JOINT_REVOLUTE(J20);
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
  
    INITIALIZE_JOINT_REVOLUTE(J21);
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
  
    INITIALIZE_JOINT_REVOLUTE(J22);
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
  
    INITIALIZE_JOINT_REVOLUTE(J23);
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
  
    INITIALIZE_JOINT_REVOLUTE(J24);
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
  
    INITIALIZE_JOINT_REVOLUTE(J25);
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
  
    INITIALIZE_JOINT_REVOLUTE(J26);
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
  
    INITIALIZE_JOINT_REVOLUTE(J27);
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
  
    INITIALIZE_JOINT_REVOLUTE(J28);
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
  
    INITIALIZE_JOINT_REVOLUTE(J29);
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
  
    INITIALIZE_JOINT_REVOLUTE(J30);
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
  
    INITIALIZE_JOINT_REVOLUTE(J31);
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
  
    INITIALIZE_JOINT_REVOLUTE(J32);
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
  
    INITIALIZE_JOINT_REVOLUTE(J33);
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
  
    INITIALIZE_JOINT_REVOLUTE(J34);
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
  
    INITIALIZE_JOINT_REVOLUTE(J35);
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
  
    INITIALIZE_JOINT_REVOLUTE(J36);
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
  
    INITIALIZE_JOINT_REVOLUTE(J37);
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
  
    INITIALIZE_JOINT_REVOLUTE(J38);
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
  
    INITIALIZE_JOINT_REVOLUTE(J39);
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
  
    INITIALIZE_JOINT_REVOLUTE(J40);
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
  
    INITIALIZE_JOINT_REVOLUTE(J41);
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
  
    INITIALIZE_JOINT_REVOLUTE(J42);
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
  
    INITIALIZE_JOINT_REVOLUTE(J43);
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
  
    INITIALIZE_JOINT_REVOLUTE(J44);
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
  
    INITIALIZE_JOINT_REVOLUTE(J45);
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
  
    INITIALIZE_JOINT_REVOLUTE(J46);
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
  
    INITIALIZE_JOINT_REVOLUTE(J47);
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
  
    INITIALIZE_JOINT_REVOLUTE(J48);
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
  
    INITIALIZE_JOINT_REVOLUTE(J49);
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
  
    INITIALIZE_JOINT_REVOLUTE(J50);
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
  
    INITIALIZE_JOINT_REVOLUTE(J51);
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
  
    INITIALIZE_JOINT_REVOLUTE(J52);
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
  
    INITIALIZE_JOINT_REVOLUTE(J53);
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
  
    INITIALIZE_JOINT_REVOLUTE(J54);
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
  
    INITIALIZE_JOINT_REVOLUTE(J55);
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
  
    INITIALIZE_JOINT_REVOLUTE(J56);
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
  
    INITIALIZE_JOINT_REVOLUTE(J57);
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
  
    INITIALIZE_JOINT_REVOLUTE(J58);
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
  
    INITIALIZE_JOINT_REVOLUTE(J59);
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
  
    INITIALIZE_JOINT_REVOLUTE(J60);
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
  
    INITIALIZE_JOINT_REVOLUTE(J61);
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
  
    INITIALIZE_JOINT_REVOLUTE(J62);
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

    INITIALIZE_BODY(B0);
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
  
    INITIALIZE_BODY(B1);
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
  
    INITIALIZE_BODY(B2);
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
  
    INITIALIZE_BODY(B3);
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
  
    INITIALIZE_BODY(B4);
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
  
    INITIALIZE_BODY(B5);
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
  
    INITIALIZE_BODY(B6);
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
  
    INITIALIZE_BODY(B7);
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
  
    INITIALIZE_BODY(B8);
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
  
    INITIALIZE_BODY(B9);
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
  
    INITIALIZE_BODY(B10);
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
  
    INITIALIZE_BODY(B11);
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
  
    INITIALIZE_BODY(B12);
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
  
    INITIALIZE_BODY(B13);
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
  
    INITIALIZE_BODY(B14);
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
  
    INITIALIZE_BODY(B15);
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
  
    INITIALIZE_BODY(B16);
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
  
    INITIALIZE_BODY(B17);
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
  
    INITIALIZE_BODY(B18);
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
  
    INITIALIZE_BODY(B19);
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
  
    INITIALIZE_BODY(B20);
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
  
    INITIALIZE_BODY(B21);
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
  
    INITIALIZE_BODY(B22);
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
  
    INITIALIZE_BODY(B23);
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
  
    INITIALIZE_BODY(B24);
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
  
    INITIALIZE_BODY(B25);
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
  
    INITIALIZE_BODY(B26);
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
  
    INITIALIZE_BODY(B27);
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
  
    INITIALIZE_BODY(B28);
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
  
    INITIALIZE_BODY(B29);
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
  
    INITIALIZE_BODY(B30);
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
  
    INITIALIZE_BODY(B31);
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
  
    INITIALIZE_BODY(B32);
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
  
    INITIALIZE_BODY(B33);
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
  
    INITIALIZE_BODY(B34);
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
  
    INITIALIZE_BODY(B35);
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
  
    INITIALIZE_BODY(B36);
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
  
    INITIALIZE_BODY(B37);
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
  
    INITIALIZE_BODY(B38);
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
  
    INITIALIZE_BODY(B39);
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
  
    INITIALIZE_BODY(B40);
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
  
    INITIALIZE_BODY(B41);
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
  
    INITIALIZE_BODY(B42);
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
  
    INITIALIZE_BODY(B43);
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
  
    INITIALIZE_BODY(B44);
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
  
    INITIALIZE_BODY(B45);
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
  
    INITIALIZE_BODY(B46);
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
  
    INITIALIZE_BODY(B47);
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
  
    INITIALIZE_BODY(B48);
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
  
    INITIALIZE_BODY(B49);
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
  
    INITIALIZE_BODY(B50);
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
  
    INITIALIZE_BODY(B51);
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
  
    INITIALIZE_BODY(B52);
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
  
    INITIALIZE_BODY(B53);
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
  
    INITIALIZE_BODY(B54);
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
  
    INITIALIZE_BODY(B55);
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
  
    INITIALIZE_BODY(B56);
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
  
    INITIALIZE_BODY(B57);
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
  
    INITIALIZE_BODY(B58);
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
  
    INITIALIZE_BODY(B59);
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
  
    INITIALIZE_BODY(B60);
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
  
    INITIALIZE_BODY(B61);
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
  
    INITIALIZE_BODY(B62);
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
  } // end of namespace sample_63_dof
} // end of namespace metapod
