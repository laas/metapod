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
 * This file is part of a simple humanoid robot model, used for test purpose.
 * It contains the initialization of all the robot bodies and joints.
 */

# include "simple_humanoid.hh"

template struct metapod::crba< metapod::simple_humanoid::Robot , true >;
template struct metapod::rnea< metapod::simple_humanoid::Robot , true >;
template struct metapod::crba< metapod::simple_humanoid::Robot , false >;
template struct metapod::rnea< metapod::simple_humanoid::Robot , false >;

namespace metapod
{
  namespace simple_humanoid
  {
    // Initialization of the robot global constants
    Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;

    // Init WAIST
    INITIALIZE_JOINT_FREE_FLYER(WAIST);
    const std::string WAIST::name = "WAIST";
    const int WAIST::label = 1;
    const int WAIST::positionInConf = 0;
    const Spatial::Transform WAIST::Xt = Spatial::Transform(
      matrix3dMaker(1, 0, 0,
                    0, 1, 0,
                    0, 0, 1),
      vector3d(0, 0, 0));

    // Init WAIST_P
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(WAIST_P);
    const std::string WAIST_P::name = "WAIST_P";
    const int WAIST_P::label = 2;
    const int WAIST_P::positionInConf = 32;
    const Spatial::Transform WAIST_P::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    // Init WAIST_R
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(WAIST_R);
    const std::string WAIST_R::name = "WAIST_R";
    const int WAIST_R::label = 3;
    const int WAIST_R::positionInConf = 33;
    const Spatial::Transform WAIST_R::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    // Init CHEST
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(CHEST);
    const std::string CHEST::name = "CHEST";
    const int CHEST::label = 4;
    const int CHEST::positionInConf = 34;
    const Spatial::Transform CHEST::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    1, 0, 0,
                    0, 1, 0),
      vector3d(0, 0, 0.35));

    // Init LARM_SHOULDER_P
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LARM_SHOULDER_P);
    const std::string LARM_SHOULDER_P::name = "LARM_SHOULDER_P";
    const int LARM_SHOULDER_P::label = 5;
    const int LARM_SHOULDER_P::positionInConf = 25;
    const Spatial::Transform LARM_SHOULDER_P::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    0, 1, 0,
                    -1, 0, 0),
      vector3d(0, 0, 0.21));

    // Init LARM_SHOULDER_R
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LARM_SHOULDER_R);
    const std::string LARM_SHOULDER_R::name = "LARM_SHOULDER_R";
    const int LARM_SHOULDER_R::label = 6;
    const int LARM_SHOULDER_R::positionInConf = 26;
    const Spatial::Transform LARM_SHOULDER_R::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    // Init LARM_SHOULDER_Y
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LARM_SHOULDER_Y);
    const std::string LARM_SHOULDER_Y::name = "LARM_SHOULDER_Y";
    const int LARM_SHOULDER_Y::label = 7;
    const int LARM_SHOULDER_Y::positionInConf = 27;
    const Spatial::Transform LARM_SHOULDER_Y::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    1, 0, 0,
                    0, 1, 0),
      vector3d(0, 0, -0.263));

    // Init LARM_ELBOW
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LARM_ELBOW);
    const std::string LARM_ELBOW::name = "LARM_ELBOW";
    const int LARM_ELBOW::label = 8;
    const int LARM_ELBOW::positionInConf = 28;
    const Spatial::Transform LARM_ELBOW::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    0, 1, 0,
                    -1, 0, 0),
      vector3d(0, 0, 0));

    // Init LARM_WRIST_Y
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LARM_WRIST_Y);
    const std::string LARM_WRIST_Y::name = "LARM_WRIST_Y";
    const int LARM_WRIST_Y::label = 9;
    const int LARM_WRIST_Y::positionInConf = 29;
    const Spatial::Transform LARM_WRIST_Y::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, -1,
                    0, 1, 0,
                    1, 0, 0),
      vector3d(0, 0, 0.247));

    // Init LARM_WRIST_P
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LARM_WRIST_P);
    const std::string LARM_WRIST_P::name = "LARM_WRIST_P";
    const int LARM_WRIST_P::label = 10;
    const int LARM_WRIST_P::positionInConf = 30;
    const Spatial::Transform LARM_WRIST_P::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    0, 1, 0,
                    -1, 0, 0),
      vector3d(0, 0, 0));

    // Init LARM_WRIST_R
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LARM_WRIST_R);
    const std::string LARM_WRIST_R::name = "LARM_WRIST_R";
    const int LARM_WRIST_R::label = 11;
    const int LARM_WRIST_R::positionInConf = 31;
    const Spatial::Transform LARM_WRIST_R::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    // Init RARM_SHOULDER_P
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RARM_SHOULDER_P);
    const std::string RARM_SHOULDER_P::name = "RARM_SHOULDER_P";
    const int RARM_SHOULDER_P::label = 12;
    const int RARM_SHOULDER_P::positionInConf = 12;
    const Spatial::Transform RARM_SHOULDER_P::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    0, 1, 0,
                    -1, 0, 0),
      vector3d(0, 0, -0.21));

    // Init RARM_SHOULDER_R
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RARM_SHOULDER_R);
    const std::string RARM_SHOULDER_R::name = "RARM_SHOULDER_R";
    const int RARM_SHOULDER_R::label = 13;
    const int RARM_SHOULDER_R::positionInConf = 13;
    const Spatial::Transform RARM_SHOULDER_R::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    // Init RARM_SHOULDER_Y
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RARM_SHOULDER_Y);
    const std::string RARM_SHOULDER_Y::name = "RARM_SHOULDER_Y";
    const int RARM_SHOULDER_Y::label = 14;
    const int RARM_SHOULDER_Y::positionInConf = 14;
    const Spatial::Transform RARM_SHOULDER_Y::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    1, 0, 0,
                    0, 1, 0),
      vector3d(0, 0, -0.263));

    // Init RARM_ELBOW
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RARM_ELBOW);
    const std::string RARM_ELBOW::name = "RARM_ELBOW";
    const int RARM_ELBOW::label = 15;
    const int RARM_ELBOW::positionInConf = 15;
    const Spatial::Transform RARM_ELBOW::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    0, 1, 0,
                    -1, 0, 0),
      vector3d(0, 0, 0));

    // Init RARM_WRIST_Y
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RARM_WRIST_Y);
    const std::string RARM_WRIST_Y::name = "RARM_WRIST_Y";
    const int RARM_WRIST_Y::label = 16;
    const int RARM_WRIST_Y::positionInConf = 16;
    const Spatial::Transform RARM_WRIST_Y::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, -1,
                    0, 1, 0,
                    1, 0, 0),
      vector3d(0, 0, 0.247));

    // Init RARM_WRIST_P
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RARM_WRIST_P);
    const std::string RARM_WRIST_P::name = "RARM_WRIST_P";
    const int RARM_WRIST_P::label = 17;
    const int RARM_WRIST_P::positionInConf = 17;
    const Spatial::Transform RARM_WRIST_P::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    0, 1, 0,
                    -1, 0, 0),
      vector3d(0, 0, 0));

    // Init RARM_WRIST_R
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RARM_WRIST_R);
    const std::string RARM_WRIST_R::name = "RARM_WRIST_R";
    const int RARM_WRIST_R::label = 18;
    const int RARM_WRIST_R::positionInConf = 18;
    const Spatial::Transform RARM_WRIST_R::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    // Init LLEG_HIP_R
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LLEG_HIP_R);
    const std::string LLEG_HIP_R::name = "LLEG_HIP_R";
    const int LLEG_HIP_R::label = 19;
    const int LLEG_HIP_R::positionInConf = 19;
    const Spatial::Transform LLEG_HIP_R::Xt = Spatial::Transform(
      matrix3dMaker(1, 0, 0,
                    0, 1, 0,
                    0, 0, 1),
      vector3d(0, 0.09, 0));

    // Init LLEG_HIP_P
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LLEG_HIP_P);
    const std::string LLEG_HIP_P::name = "LLEG_HIP_P";
    const int LLEG_HIP_P::label = 20;
    const int LLEG_HIP_P::positionInConf = 20;
    const Spatial::Transform LLEG_HIP_P::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    // Init LLEG_HIP_Y
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LLEG_HIP_Y);
    const std::string LLEG_HIP_Y::name = "LLEG_HIP_Y";
    const int LLEG_HIP_Y::label = 21;
    const int LLEG_HIP_Y::positionInConf = 21;
    const Spatial::Transform LLEG_HIP_Y::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, -1,
                    0, 1, 0,
                    1, 0, 0),
      vector3d(0, 0, 0.3535));

    // Init LLEG_KNEE
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LLEG_KNEE);
    const std::string LLEG_KNEE::name = "LLEG_KNEE";
    const int LLEG_KNEE::label = 22;
    const int LLEG_KNEE::positionInConf = 22;
    const Spatial::Transform LLEG_KNEE::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    0, 1, 0,
                    -1, 0, 0),
      vector3d(0, 0, 0));

    // Init LLEG_ANKLE_P
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LLEG_ANKLE_P);
    const std::string LLEG_ANKLE_P::name = "LLEG_ANKLE_P";
    const int LLEG_ANKLE_P::label = 23;
    const int LLEG_ANKLE_P::positionInConf = 23;
    const Spatial::Transform LLEG_ANKLE_P::Xt = Spatial::Transform(
      matrix3dMaker(1, 0, 0,
                    0, 1, 0,
                    0, 0, 1),
      vector3d(0, 0, 0.3));

    // Init LLEG_ANKLE_R
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(LLEG_ANKLE_R);
    const std::string LLEG_ANKLE_R::name = "LLEG_ANKLE_R";
    const int LLEG_ANKLE_R::label = 24;
    const int LLEG_ANKLE_R::positionInConf = 24;
    const Spatial::Transform LLEG_ANKLE_R::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    // Init RLEG_HIP_R
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RLEG_HIP_R);
    const std::string RLEG_HIP_R::name = "RLEG_HIP_R";
    const int RLEG_HIP_R::label = 25;
    const int RLEG_HIP_R::positionInConf = 6;
    const Spatial::Transform RLEG_HIP_R::Xt = Spatial::Transform(
      matrix3dMaker(1, 0, 0,
                    0, 1, 0,
                    0, 0, 1),
      vector3d(0, -0.09, 0));

    // Init RLEG_HIP_P
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RLEG_HIP_P);
    const std::string RLEG_HIP_P::name = "RLEG_HIP_P";
    const int RLEG_HIP_P::label = 26;
    const int RLEG_HIP_P::positionInConf = 7;
    const Spatial::Transform RLEG_HIP_P::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    // Init RLEG_HIP_Y
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RLEG_HIP_Y);
    const std::string RLEG_HIP_Y::name = "RLEG_HIP_Y";
    const int RLEG_HIP_Y::label = 27;
    const int RLEG_HIP_Y::positionInConf = 8;
    const Spatial::Transform RLEG_HIP_Y::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, -1,
                    0, 1, 0,
                    1, 0, 0),
      vector3d(0, 0, 0.3535));

    // Init RLEG_KNEE
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RLEG_KNEE);
    const std::string RLEG_KNEE::name = "RLEG_KNEE";
    const int RLEG_KNEE::label = 28;
    const int RLEG_KNEE::positionInConf = 9;
    const Spatial::Transform RLEG_KNEE::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    0, 1, 0,
                    -1, 0, 0),
      vector3d(0, 0, 0));

    // Init RLEG_ANKLE_P
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RLEG_ANKLE_P);
    const std::string RLEG_ANKLE_P::name = "RLEG_ANKLE_P";
    const int RLEG_ANKLE_P::label = 29;
    const int RLEG_ANKLE_P::positionInConf = 10;
    const Spatial::Transform RLEG_ANKLE_P::Xt = Spatial::Transform(
      matrix3dMaker(1, 0, 0,
                    0, 1, 0,
                    0, 0, 1),
      vector3d(0, 0, 0.3));

    // Init RLEG_ANKLE_R
    INITIALIZE_JOINT_REVOLUTE_AXIS_X(RLEG_ANKLE_R);
    const std::string RLEG_ANKLE_R::name = "RLEG_ANKLE_R";
    const int RLEG_ANKLE_R::label = 30;
    const int RLEG_ANKLE_R::positionInConf = 11;
    const Spatial::Transform RLEG_ANKLE_R::Xt = Spatial::Transform(
      matrix3dMaker(0, 1, 0,
                    1, 0, 0,
                    0, 0, -1),
      vector3d(0, 0, 0));

    INITIALIZE_BODY(WAIST_LINK0);
    // Initialization of WAIST_LINK0;
    const std::string WAIST_LINK0::name = "WAIST_LINK0";
    const int WAIST_LINK0::label = 0;
    const FloatType WAIST_LINK0::mass = 27;
    const vector3d WAIST_LINK0::CoM = vector3d(0, 0, 0.0375);
    const matrix3d WAIST_LINK0::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia WAIST_LINK0::I = spatialInertiaMaker(WAIST_LINK0::mass,
                                                 WAIST_LINK0::CoM,
                                                 WAIST_LINK0::inertie);

    INITIALIZE_BODY(WAIST_LINK1);
    // Initialization of WAIST_LINK1;
    const std::string WAIST_LINK1::name = "WAIST_LINK1";
    const int WAIST_LINK1::label = 1;
    const FloatType WAIST_LINK1::mass = 6;
    const vector3d WAIST_LINK1::CoM = vector3d(0, 0, 0.1);
    const matrix3d WAIST_LINK1::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia WAIST_LINK1::I = spatialInertiaMaker(WAIST_LINK1::mass,
                                                 WAIST_LINK1::CoM,
                                                 WAIST_LINK1::inertie);

    INITIALIZE_BODY(WAIST_LINK2);
    // Initialization of WAIST_LINK2;
    const std::string WAIST_LINK2::name = "WAIST_LINK2";
    const int WAIST_LINK2::label = 2;
    const FloatType WAIST_LINK2::mass = 30;
    const vector3d WAIST_LINK2::CoM = vector3d(0.11, 0, 0.25);
    const matrix3d WAIST_LINK2::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia WAIST_LINK2::I = spatialInertiaMaker(WAIST_LINK2::mass,
                                                 WAIST_LINK2::CoM,
                                                 WAIST_LINK2::inertie);

    INITIALIZE_BODY(WAIST_LINK3);
    // Initialization of WAIST_LINK3;
    const std::string WAIST_LINK3::name = "WAIST_LINK3";
    const int WAIST_LINK3::label = 3;
    const FloatType WAIST_LINK3::mass = 13;
    const vector3d WAIST_LINK3::CoM = vector3d(0, 0, 0);
    const matrix3d WAIST_LINK3::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia WAIST_LINK3::I = spatialInertiaMaker(WAIST_LINK3::mass,
                                                 WAIST_LINK3::CoM,
                                                 WAIST_LINK3::inertie);

    INITIALIZE_BODY(LARM_LINK1);
    // Initialization of LARM_LINK1;
    const std::string LARM_LINK1::name = "LARM_LINK1";
    const int LARM_LINK1::label = 4;
    const FloatType LARM_LINK1::mass = 3;
    const vector3d LARM_LINK1::CoM = vector3d(0, 0.1, 0);
    const matrix3d LARM_LINK1::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LARM_LINK1::I = spatialInertiaMaker(LARM_LINK1::mass,
                                                LARM_LINK1::CoM,
                                                LARM_LINK1::inertie);

    INITIALIZE_BODY(LARM_LINK2);
    // Initialization of LARM_LINK2;
    const std::string LARM_LINK2::name = "LARM_LINK2";
    const int LARM_LINK2::label = 5;
    const FloatType LARM_LINK2::mass = 0.6;
    const vector3d LARM_LINK2::CoM = vector3d(0, 0, -0.1);
    const matrix3d LARM_LINK2::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LARM_LINK2::I = spatialInertiaMaker(LARM_LINK2::mass,
                                                LARM_LINK2::CoM,
                                                LARM_LINK2::inertie);

    INITIALIZE_BODY(LARM_LINK3);
    // Initialization of LARM_LINK3;
    const std::string LARM_LINK3::name = "LARM_LINK3";
    const int LARM_LINK3::label = 6;
    const FloatType LARM_LINK3::mass = 1;
    const vector3d LARM_LINK3::CoM = vector3d(0, 0, 0);
    const matrix3d LARM_LINK3::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LARM_LINK3::I = spatialInertiaMaker(LARM_LINK3::mass,
                                                LARM_LINK3::CoM,
                                                LARM_LINK3::inertie);

    INITIALIZE_BODY(LARM_LINK4);
    // Initialization of LARM_LINK4;
    const std::string LARM_LINK4::name = "LARM_LINK4";
    const int LARM_LINK4::label = 7;
    const FloatType LARM_LINK4::mass = 0.6;
    const vector3d LARM_LINK4::CoM = vector3d(0, 0, 0.3);
    const matrix3d LARM_LINK4::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LARM_LINK4::I = spatialInertiaMaker(LARM_LINK4::mass,
                                                LARM_LINK4::CoM,
                                                LARM_LINK4::inertie);

    INITIALIZE_BODY(LARM_LINK5);
    // Initialization of LARM_LINK5;
    const std::string LARM_LINK5::name = "LARM_LINK5";
    const int LARM_LINK5::label = 8;
    const FloatType LARM_LINK5::mass = 0.4;
    const vector3d LARM_LINK5::CoM = vector3d(0.1, 0, 0);
    const matrix3d LARM_LINK5::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LARM_LINK5::I = spatialInertiaMaker(LARM_LINK5::mass,
                                                LARM_LINK5::CoM,
                                                LARM_LINK5::inertie);

    INITIALIZE_BODY(LARM_LINK6);
    // Initialization of LARM_LINK6;
    const std::string LARM_LINK6::name = "LARM_LINK6";
    const int LARM_LINK6::label = 9;
    const FloatType LARM_LINK6::mass = 0.4;
    const vector3d LARM_LINK6::CoM = vector3d(0, -0.1, 0);
    const matrix3d LARM_LINK6::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LARM_LINK6::I = spatialInertiaMaker(LARM_LINK6::mass,
                                                LARM_LINK6::CoM,
                                                LARM_LINK6::inertie);

    INITIALIZE_BODY(LARM_LINK7);
    // Initialization of LARM_LINK7;
    const std::string LARM_LINK7::name = "LARM_LINK7";
    const int LARM_LINK7::label = 10;
    const FloatType LARM_LINK7::mass = 0.4;
    const vector3d LARM_LINK7::CoM = vector3d(0, 0, -0.1);
    const matrix3d LARM_LINK7::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LARM_LINK7::I = spatialInertiaMaker(LARM_LINK7::mass,
                                                LARM_LINK7::CoM,
                                                LARM_LINK7::inertie);

    INITIALIZE_BODY(RARM_LINK1);
    // Initialization of RARM_LINK1;
    const std::string RARM_LINK1::name = "RARM_LINK1";
    const int RARM_LINK1::label = 11;
    const FloatType RARM_LINK1::mass = 3;
    const vector3d RARM_LINK1::CoM = vector3d(0, 0.1, 0);
    const matrix3d RARM_LINK1::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RARM_LINK1::I = spatialInertiaMaker(RARM_LINK1::mass,
                                                RARM_LINK1::CoM,
                                                RARM_LINK1::inertie);

    INITIALIZE_BODY(RARM_LINK2);
    // Initialization of RARM_LINK2;
    const std::string RARM_LINK2::name = "RARM_LINK2";
    const int RARM_LINK2::label = 12;
    const FloatType RARM_LINK2::mass = 0.6;
    const vector3d RARM_LINK2::CoM = vector3d(0, 0, -0.1);
    const matrix3d RARM_LINK2::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RARM_LINK2::I = spatialInertiaMaker(RARM_LINK2::mass,
                                                RARM_LINK2::CoM,
                                                RARM_LINK2::inertie);

    INITIALIZE_BODY(RARM_LINK3);
    // Initialization of RARM_LINK3;
    const std::string RARM_LINK3::name = "RARM_LINK3";
    const int RARM_LINK3::label = 13;
    const FloatType RARM_LINK3::mass = 1;
    const vector3d RARM_LINK3::CoM = vector3d(0, 0, 0);
    const matrix3d RARM_LINK3::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RARM_LINK3::I = spatialInertiaMaker(RARM_LINK3::mass,
                                                RARM_LINK3::CoM,
                                                RARM_LINK3::inertie);

    INITIALIZE_BODY(RARM_LINK4);
    // Initialization of RARM_LINK4;
    const std::string RARM_LINK4::name = "RARM_LINK4";
    const int RARM_LINK4::label = 14;
    const FloatType RARM_LINK4::mass = 0.6;
    const vector3d RARM_LINK4::CoM = vector3d(0, 0, 0.3);
    const matrix3d RARM_LINK4::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RARM_LINK4::I = spatialInertiaMaker(RARM_LINK4::mass,
                                                RARM_LINK4::CoM,
                                                RARM_LINK4::inertie);

    INITIALIZE_BODY(RARM_LINK5);
    // Initialization of RARM_LINK5;
    const std::string RARM_LINK5::name = "RARM_LINK5";
    const int RARM_LINK5::label = 15;
    const FloatType RARM_LINK5::mass = 0.4;
    const vector3d RARM_LINK5::CoM = vector3d(0.1, 0, 0);
    const matrix3d RARM_LINK5::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RARM_LINK5::I = spatialInertiaMaker(RARM_LINK5::mass,
                                                RARM_LINK5::CoM,
                                                RARM_LINK5::inertie);

    INITIALIZE_BODY(RARM_LINK6);
    // Initialization of RARM_LINK6;
    const std::string RARM_LINK6::name = "RARM_LINK6";
    const int RARM_LINK6::label = 16;
    const FloatType RARM_LINK6::mass = 0.4;
    const vector3d RARM_LINK6::CoM = vector3d(0, -0.1, 0);
    const matrix3d RARM_LINK6::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RARM_LINK6::I = spatialInertiaMaker(RARM_LINK6::mass,
                                                RARM_LINK6::CoM,
                                                RARM_LINK6::inertie);

    INITIALIZE_BODY(RARM_LINK7);
    // Initialization of RARM_LINK7;
    const std::string RARM_LINK7::name = "RARM_LINK7";
    const int RARM_LINK7::label = 17;
    const FloatType RARM_LINK7::mass = 0.4;
    const vector3d RARM_LINK7::CoM = vector3d(0, 0, -0.1);
    const matrix3d RARM_LINK7::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RARM_LINK7::I = spatialInertiaMaker(RARM_LINK7::mass,
                                                RARM_LINK7::CoM,
                                                RARM_LINK7::inertie);

    INITIALIZE_BODY(LLEG_LINK1);
    // Initialization of LLEG_LINK1;
    const std::string LLEG_LINK1::name = "LLEG_LINK1";
    const int LLEG_LINK1::label = 18;
    const FloatType LLEG_LINK1::mass = 2.5;
    const vector3d LLEG_LINK1::CoM = vector3d(0, 0.1, 0);
    const matrix3d LLEG_LINK1::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LLEG_LINK1::I = spatialInertiaMaker(LLEG_LINK1::mass,
                                                LLEG_LINK1::CoM,
                                                LLEG_LINK1::inertie);

    INITIALIZE_BODY(LLEG_LINK2);
    // Initialization of LLEG_LINK2;
    const std::string LLEG_LINK2::name = "LLEG_LINK2";
    const int LLEG_LINK2::label = 19;
    const FloatType LLEG_LINK2::mass = 2;
    const vector3d LLEG_LINK2::CoM = vector3d(0, 0, 0.15);
    const matrix3d LLEG_LINK2::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LLEG_LINK2::I = spatialInertiaMaker(LLEG_LINK2::mass,
                                                LLEG_LINK2::CoM,
                                                LLEG_LINK2::inertie);

    INITIALIZE_BODY(LLEG_LINK3);
    // Initialization of LLEG_LINK3;
    const std::string LLEG_LINK3::name = "LLEG_LINK3";
    const int LLEG_LINK3::label = 20;
    const FloatType LLEG_LINK3::mass = 5.1;
    const vector3d LLEG_LINK3::CoM = vector3d(0, 0, 0.04);
    const matrix3d LLEG_LINK3::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LLEG_LINK3::I = spatialInertiaMaker(LLEG_LINK3::mass,
                                                LLEG_LINK3::CoM,
                                                LLEG_LINK3::inertie);

    INITIALIZE_BODY(LLEG_LINK4);
    // Initialization of LLEG_LINK4;
    const std::string LLEG_LINK4::name = "LLEG_LINK4";
    const int LLEG_LINK4::label = 21;
    const FloatType LLEG_LINK4::mass = 7;
    const vector3d LLEG_LINK4::CoM = vector3d(0, 0, 0.3);
    const matrix3d LLEG_LINK4::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LLEG_LINK4::I = spatialInertiaMaker(LLEG_LINK4::mass,
                                                LLEG_LINK4::CoM,
                                                LLEG_LINK4::inertie);

    INITIALIZE_BODY(LLEG_LINK5);
    // Initialization of LLEG_LINK5;
    const std::string LLEG_LINK5::name = "LLEG_LINK5";
    const int LLEG_LINK5::label = 22;
    const FloatType LLEG_LINK5::mass = 2.5;
    const vector3d LLEG_LINK5::CoM = vector3d(0, -0.15, 0);
    const matrix3d LLEG_LINK5::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LLEG_LINK5::I = spatialInertiaMaker(LLEG_LINK5::mass,
                                                LLEG_LINK5::CoM,
                                                LLEG_LINK5::inertie);

    INITIALIZE_BODY(LLEG_LINK6);
    // Initialization of LLEG_LINK6;
    const std::string LLEG_LINK6::name = "LLEG_LINK6";
    const int LLEG_LINK6::label = 23;
    const FloatType LLEG_LINK6::mass = 1.9;
    const vector3d LLEG_LINK6::CoM = vector3d(0.28, 0, -0.2);
    const matrix3d LLEG_LINK6::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia LLEG_LINK6::I = spatialInertiaMaker(LLEG_LINK6::mass,
                                                LLEG_LINK6::CoM,
                                                LLEG_LINK6::inertie);

    INITIALIZE_BODY(RLEG_LINK1);
    // Initialization of RLEG_LINK1;
    const std::string RLEG_LINK1::name = "RLEG_LINK1";
    const int RLEG_LINK1::label = 24;
    const FloatType RLEG_LINK1::mass = 2.5;
    const vector3d RLEG_LINK1::CoM = vector3d(0, -0.1, 0);
    const matrix3d RLEG_LINK1::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RLEG_LINK1::I = spatialInertiaMaker(RLEG_LINK1::mass,
                                                RLEG_LINK1::CoM,
                                                RLEG_LINK1::inertie);

    INITIALIZE_BODY(RLEG_LINK2);
    // Initialization of RLEG_LINK2;
    const std::string RLEG_LINK2::name = "RLEG_LINK2";
    const int RLEG_LINK2::label = 25;
    const FloatType RLEG_LINK2::mass = 2;
    const vector3d RLEG_LINK2::CoM = vector3d(0, 0, 0.15);
    const matrix3d RLEG_LINK2::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RLEG_LINK2::I = spatialInertiaMaker(RLEG_LINK2::mass,
                                                RLEG_LINK2::CoM,
                                                RLEG_LINK2::inertie);

    INITIALIZE_BODY(RLEG_LINK3);
    // Initialization of RLEG_LINK3;
    const std::string RLEG_LINK3::name = "RLEG_LINK3";
    const int RLEG_LINK3::label = 26;
    const FloatType RLEG_LINK3::mass = 5.1;
    const vector3d RLEG_LINK3::CoM = vector3d(0, 0, -0.04);
    const matrix3d RLEG_LINK3::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RLEG_LINK3::I = spatialInertiaMaker(RLEG_LINK3::mass,
                                                RLEG_LINK3::CoM,
                                                RLEG_LINK3::inertie);

    INITIALIZE_BODY(RLEG_LINK4);
    // Initialization of RLEG_LINK4;
    const std::string RLEG_LINK4::name = "RLEG_LINK4";
    const int RLEG_LINK4::label = 27;
    const FloatType RLEG_LINK4::mass = 7;
    const vector3d RLEG_LINK4::CoM = vector3d(0, 0, 0.3);
    const matrix3d RLEG_LINK4::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RLEG_LINK4::I = spatialInertiaMaker(RLEG_LINK4::mass,
                                                RLEG_LINK4::CoM,
                                                RLEG_LINK4::inertie);

    INITIALIZE_BODY(RLEG_LINK5);
    // Initialization of RLEG_LINK5;
    const std::string RLEG_LINK5::name = "RLEG_LINK5";
    const int RLEG_LINK5::label = 28;
    const FloatType RLEG_LINK5::mass = 2.5;
    const vector3d RLEG_LINK5::CoM = vector3d(0, -0.15, 0);
    const matrix3d RLEG_LINK5::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RLEG_LINK5::I = spatialInertiaMaker(RLEG_LINK5::mass,
                                                RLEG_LINK5::CoM,
                                                RLEG_LINK5::inertie);

    INITIALIZE_BODY(RLEG_LINK6);
    // Initialization of RLEG_LINK6;
    const std::string RLEG_LINK6::name = "RLEG_LINK6";
    const int RLEG_LINK6::label = 29;
    const FloatType RLEG_LINK6::mass = 1.9;
    const vector3d RLEG_LINK6::CoM = vector3d(0.28, 0, -0.2);
    const matrix3d RLEG_LINK6::inertie = matrix3dMaker(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1);
    Spatial::Inertia RLEG_LINK6::I = spatialInertiaMaker(RLEG_LINK6::mass,
                                                RLEG_LINK6::CoM,
                                                RLEG_LINK6::inertie);
  } // end of namespace simple_humanoid
} // end of namespace metapod
