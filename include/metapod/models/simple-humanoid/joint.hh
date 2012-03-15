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
 * It contains the definition of all the robot joints.
 */

#ifndef metapod_SIMPLE_HUMANOID_JOINT_HH
# define metapod_SIMPLE_HUMANOID_JOINT_HH

# include "metapod/tools/jointmacros.hh"

namespace simplehumanoid
{

  // Init WAIST
  JOINT_FREE_FLYER(WAIST);

  matrix6d WAIST::S = matrix6d::Zero();
  matrix6d WAIST::dotS = matrix6d::Zero();
  void WAIST::jcalc(const vector6d & qi, const vector6d & dqi) { FREE_FLYER_JCALC }
  const std::string WAIST::name = "WAIST";
  const int WAIST::label = 1;
  const int WAIST::nbDof = 6;
  const int WAIST::positionInConf = 0;
  const Transform WAIST::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0, 0));
  Transform WAIST::sXp;
  Transform WAIST::Xj;
  Motion WAIST::cj;
  Motion WAIST::vj;
  Force WAIST::f;
  vector6d WAIST::torque;

  // Init WAIST_P
  JOINT_REVOLUTE(WAIST_P);

  const vector6d WAIST_P::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d WAIST_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void WAIST_P::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string WAIST_P::name = "WAIST_P";
  const int WAIST_P::label = 2;
  const int WAIST_P::nbDof = 1;
  const int WAIST_P::positionInConf = 32;
  const Transform WAIST_P::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform WAIST_P::sXp;
  Transform WAIST_P::Xj;
  Motion WAIST_P::cj;
  Motion WAIST_P::vj;
  Force WAIST_P::f;
  double WAIST_P::torque;

  // Init WAIST_R
  JOINT_REVOLUTE(WAIST_R);

  const vector6d WAIST_R::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d WAIST_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void WAIST_R::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string WAIST_R::name = "WAIST_R";
  const int WAIST_R::label = 3;
  const int WAIST_R::nbDof = 1;
  const int WAIST_R::positionInConf = 33;
  const Transform WAIST_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform WAIST_R::sXp;
  Transform WAIST_R::Xj;
  Motion WAIST_R::cj;
  Motion WAIST_R::vj;
  Force WAIST_R::f;
  double WAIST_R::torque;

  // Init CHEST
  JOINT_REVOLUTE(CHEST);

  const vector6d CHEST::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d CHEST::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void CHEST::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string CHEST::name = "CHEST";
  const int CHEST::label = 4;
  const int CHEST::nbDof = 1;
  const int CHEST::positionInConf = 34;
  const Transform CHEST::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  1, 0, 0,
                  0, 1, 0),
    vector3d(0, 0, 0.35));
  Transform CHEST::sXp;
  Transform CHEST::Xj;
  Motion CHEST::cj;
  Motion CHEST::vj;
  Force CHEST::f;
  double CHEST::torque;

  // Init LARM_SHOULDER_P
  JOINT_REVOLUTE(LARM_SHOULDER_P);

  const vector6d LARM_SHOULDER_P::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LARM_SHOULDER_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_SHOULDER_P::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_SHOULDER_P::name = "LARM_SHOULDER_P";
  const int LARM_SHOULDER_P::label = 5;
  const int LARM_SHOULDER_P::nbDof = 1;
  const int LARM_SHOULDER_P::positionInConf = 25;
  const Transform LARM_SHOULDER_P::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0.21));
  Transform LARM_SHOULDER_P::sXp;
  Transform LARM_SHOULDER_P::Xj;
  Motion LARM_SHOULDER_P::cj;
  Motion LARM_SHOULDER_P::vj;
  Force LARM_SHOULDER_P::f;
  double LARM_SHOULDER_P::torque;

  // Init LARM_SHOULDER_R
  JOINT_REVOLUTE(LARM_SHOULDER_R);

  const vector6d LARM_SHOULDER_R::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LARM_SHOULDER_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_SHOULDER_R::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_SHOULDER_R::name = "LARM_SHOULDER_R";
  const int LARM_SHOULDER_R::label = 6;
  const int LARM_SHOULDER_R::nbDof = 1;
  const int LARM_SHOULDER_R::positionInConf = 26;
  const Transform LARM_SHOULDER_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform LARM_SHOULDER_R::sXp;
  Transform LARM_SHOULDER_R::Xj;
  Motion LARM_SHOULDER_R::cj;
  Motion LARM_SHOULDER_R::vj;
  Force LARM_SHOULDER_R::f;
  double LARM_SHOULDER_R::torque;

  // Init LARM_SHOULDER_Y
  JOINT_REVOLUTE(LARM_SHOULDER_Y);

  const vector6d LARM_SHOULDER_Y::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LARM_SHOULDER_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_SHOULDER_Y::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_SHOULDER_Y::name = "LARM_SHOULDER_Y";
  const int LARM_SHOULDER_Y::label = 7;
  const int LARM_SHOULDER_Y::nbDof = 1;
  const int LARM_SHOULDER_Y::positionInConf = 27;
  const Transform LARM_SHOULDER_Y::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  1, 0, 0,
                  0, 1, 0),
    vector3d(0, 0, -0.263));
  Transform LARM_SHOULDER_Y::sXp;
  Transform LARM_SHOULDER_Y::Xj;
  Motion LARM_SHOULDER_Y::cj;
  Motion LARM_SHOULDER_Y::vj;
  Force LARM_SHOULDER_Y::f;
  double LARM_SHOULDER_Y::torque;

  // Init LARM_ELBOW
  JOINT_REVOLUTE(LARM_ELBOW);

  const vector6d LARM_ELBOW::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LARM_ELBOW::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_ELBOW::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_ELBOW::name = "LARM_ELBOW";
  const int LARM_ELBOW::label = 8;
  const int LARM_ELBOW::nbDof = 1;
  const int LARM_ELBOW::positionInConf = 28;
  const Transform LARM_ELBOW::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  Transform LARM_ELBOW::sXp;
  Transform LARM_ELBOW::Xj;
  Motion LARM_ELBOW::cj;
  Motion LARM_ELBOW::vj;
  Force LARM_ELBOW::f;
  double LARM_ELBOW::torque;

  // Init LARM_WRIST_Y
  JOINT_REVOLUTE(LARM_WRIST_Y);

  const vector6d LARM_WRIST_Y::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LARM_WRIST_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_WRIST_Y::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_WRIST_Y::name = "LARM_WRIST_Y";
  const int LARM_WRIST_Y::label = 9;
  const int LARM_WRIST_Y::nbDof = 1;
  const int LARM_WRIST_Y::positionInConf = 29;
  const Transform LARM_WRIST_Y::Xt = Transform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.247));
  Transform LARM_WRIST_Y::sXp;
  Transform LARM_WRIST_Y::Xj;
  Motion LARM_WRIST_Y::cj;
  Motion LARM_WRIST_Y::vj;
  Force LARM_WRIST_Y::f;
  double LARM_WRIST_Y::torque;

  // Init LARM_WRIST_P
  JOINT_REVOLUTE(LARM_WRIST_P);

  const vector6d LARM_WRIST_P::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LARM_WRIST_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_WRIST_P::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_WRIST_P::name = "LARM_WRIST_P";
  const int LARM_WRIST_P::label = 10;
  const int LARM_WRIST_P::nbDof = 1;
  const int LARM_WRIST_P::positionInConf = 30;
  const Transform LARM_WRIST_P::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  Transform LARM_WRIST_P::sXp;
  Transform LARM_WRIST_P::Xj;
  Motion LARM_WRIST_P::cj;
  Motion LARM_WRIST_P::vj;
  Force LARM_WRIST_P::f;
  double LARM_WRIST_P::torque;

  // Init LARM_WRIST_R
  JOINT_REVOLUTE(LARM_WRIST_R);

  const vector6d LARM_WRIST_R::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LARM_WRIST_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_WRIST_R::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_WRIST_R::name = "LARM_WRIST_R";
  const int LARM_WRIST_R::label = 11;
  const int LARM_WRIST_R::nbDof = 1;
  const int LARM_WRIST_R::positionInConf = 31;
  const Transform LARM_WRIST_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform LARM_WRIST_R::sXp;
  Transform LARM_WRIST_R::Xj;
  Motion LARM_WRIST_R::cj;
  Motion LARM_WRIST_R::vj;
  Force LARM_WRIST_R::f;
  double LARM_WRIST_R::torque;

  // Init RARM_SHOULDER_P
  JOINT_REVOLUTE(RARM_SHOULDER_P);

  const vector6d RARM_SHOULDER_P::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RARM_SHOULDER_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_SHOULDER_P::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_SHOULDER_P::name = "RARM_SHOULDER_P";
  const int RARM_SHOULDER_P::label = 12;
  const int RARM_SHOULDER_P::nbDof = 1;
  const int RARM_SHOULDER_P::positionInConf = 12;
  const Transform RARM_SHOULDER_P::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, -0.21));
  Transform RARM_SHOULDER_P::sXp;
  Transform RARM_SHOULDER_P::Xj;
  Motion RARM_SHOULDER_P::cj;
  Motion RARM_SHOULDER_P::vj;
  Force RARM_SHOULDER_P::f;
  double RARM_SHOULDER_P::torque;

  // Init RARM_SHOULDER_R
  JOINT_REVOLUTE(RARM_SHOULDER_R);

  const vector6d RARM_SHOULDER_R::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RARM_SHOULDER_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_SHOULDER_R::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_SHOULDER_R::name = "RARM_SHOULDER_R";
  const int RARM_SHOULDER_R::label = 13;
  const int RARM_SHOULDER_R::nbDof = 1;
  const int RARM_SHOULDER_R::positionInConf = 13;
  const Transform RARM_SHOULDER_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform RARM_SHOULDER_R::sXp;
  Transform RARM_SHOULDER_R::Xj;
  Motion RARM_SHOULDER_R::cj;
  Motion RARM_SHOULDER_R::vj;
  Force RARM_SHOULDER_R::f;
  double RARM_SHOULDER_R::torque;

  // Init RARM_SHOULDER_Y
  JOINT_REVOLUTE(RARM_SHOULDER_Y);

  const vector6d RARM_SHOULDER_Y::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RARM_SHOULDER_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_SHOULDER_Y::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_SHOULDER_Y::name = "RARM_SHOULDER_Y";
  const int RARM_SHOULDER_Y::label = 14;
  const int RARM_SHOULDER_Y::nbDof = 1;
  const int RARM_SHOULDER_Y::positionInConf = 14;
  const Transform RARM_SHOULDER_Y::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  1, 0, 0,
                  0, 1, 0),
    vector3d(0, 0, -0.263));
  Transform RARM_SHOULDER_Y::sXp;
  Transform RARM_SHOULDER_Y::Xj;
  Motion RARM_SHOULDER_Y::cj;
  Motion RARM_SHOULDER_Y::vj;
  Force RARM_SHOULDER_Y::f;
  double RARM_SHOULDER_Y::torque;

  // Init RARM_ELBOW
  JOINT_REVOLUTE(RARM_ELBOW);

  const vector6d RARM_ELBOW::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RARM_ELBOW::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_ELBOW::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_ELBOW::name = "RARM_ELBOW";
  const int RARM_ELBOW::label = 15;
  const int RARM_ELBOW::nbDof = 1;
  const int RARM_ELBOW::positionInConf = 15;
  const Transform RARM_ELBOW::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  Transform RARM_ELBOW::sXp;
  Transform RARM_ELBOW::Xj;
  Motion RARM_ELBOW::cj;
  Motion RARM_ELBOW::vj;
  Force RARM_ELBOW::f;
  double RARM_ELBOW::torque;

  // Init RARM_WRIST_Y
  JOINT_REVOLUTE(RARM_WRIST_Y);

  const vector6d RARM_WRIST_Y::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RARM_WRIST_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_WRIST_Y::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_WRIST_Y::name = "RARM_WRIST_Y";
  const int RARM_WRIST_Y::label = 16;
  const int RARM_WRIST_Y::nbDof = 1;
  const int RARM_WRIST_Y::positionInConf = 16;
  const Transform RARM_WRIST_Y::Xt = Transform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.247));
  Transform RARM_WRIST_Y::sXp;
  Transform RARM_WRIST_Y::Xj;
  Motion RARM_WRIST_Y::cj;
  Motion RARM_WRIST_Y::vj;
  Force RARM_WRIST_Y::f;
  double RARM_WRIST_Y::torque;

  // Init RARM_WRIST_P
  JOINT_REVOLUTE(RARM_WRIST_P);

  const vector6d RARM_WRIST_P::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RARM_WRIST_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_WRIST_P::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_WRIST_P::name = "RARM_WRIST_P";
  const int RARM_WRIST_P::label = 17;
  const int RARM_WRIST_P::nbDof = 1;
  const int RARM_WRIST_P::positionInConf = 17;
  const Transform RARM_WRIST_P::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  Transform RARM_WRIST_P::sXp;
  Transform RARM_WRIST_P::Xj;
  Motion RARM_WRIST_P::cj;
  Motion RARM_WRIST_P::vj;
  Force RARM_WRIST_P::f;
  double RARM_WRIST_P::torque;

  // Init RARM_WRIST_R
  JOINT_REVOLUTE(RARM_WRIST_R);

  const vector6d RARM_WRIST_R::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RARM_WRIST_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_WRIST_R::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_WRIST_R::name = "RARM_WRIST_R";
  const int RARM_WRIST_R::label = 18;
  const int RARM_WRIST_R::nbDof = 1;
  const int RARM_WRIST_R::positionInConf = 18;
  const Transform RARM_WRIST_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform RARM_WRIST_R::sXp;
  Transform RARM_WRIST_R::Xj;
  Motion RARM_WRIST_R::cj;
  Motion RARM_WRIST_R::vj;
  Force RARM_WRIST_R::f;
  double RARM_WRIST_R::torque;

  // Init LLEG_HIP_R
  JOINT_REVOLUTE(LLEG_HIP_R);

  const vector6d LLEG_HIP_R::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LLEG_HIP_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_HIP_R::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_HIP_R::name = "LLEG_HIP_R";
  const int LLEG_HIP_R::label = 19;
  const int LLEG_HIP_R::nbDof = 1;
  const int LLEG_HIP_R::positionInConf = 19;
  const Transform LLEG_HIP_R::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0.09, 0));
  Transform LLEG_HIP_R::sXp;
  Transform LLEG_HIP_R::Xj;
  Motion LLEG_HIP_R::cj;
  Motion LLEG_HIP_R::vj;
  Force LLEG_HIP_R::f;
  double LLEG_HIP_R::torque;

  // Init LLEG_HIP_P
  JOINT_REVOLUTE(LLEG_HIP_P);

  const vector6d LLEG_HIP_P::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LLEG_HIP_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_HIP_P::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_HIP_P::name = "LLEG_HIP_P";
  const int LLEG_HIP_P::label = 20;
  const int LLEG_HIP_P::nbDof = 1;
  const int LLEG_HIP_P::positionInConf = 20;
  const Transform LLEG_HIP_P::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform LLEG_HIP_P::sXp;
  Transform LLEG_HIP_P::Xj;
  Motion LLEG_HIP_P::cj;
  Motion LLEG_HIP_P::vj;
  Force LLEG_HIP_P::f;
  double LLEG_HIP_P::torque;

  // Init LLEG_HIP_Y
  JOINT_REVOLUTE(LLEG_HIP_Y);

  const vector6d LLEG_HIP_Y::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LLEG_HIP_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_HIP_Y::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_HIP_Y::name = "LLEG_HIP_Y";
  const int LLEG_HIP_Y::label = 21;
  const int LLEG_HIP_Y::nbDof = 1;
  const int LLEG_HIP_Y::positionInConf = 21;
  const Transform LLEG_HIP_Y::Xt = Transform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.3535));
  Transform LLEG_HIP_Y::sXp;
  Transform LLEG_HIP_Y::Xj;
  Motion LLEG_HIP_Y::cj;
  Motion LLEG_HIP_Y::vj;
  Force LLEG_HIP_Y::f;
  double LLEG_HIP_Y::torque;

  // Init LLEG_KNEE
  JOINT_REVOLUTE(LLEG_KNEE);

  const vector6d LLEG_KNEE::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LLEG_KNEE::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_KNEE::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_KNEE::name = "LLEG_KNEE";
  const int LLEG_KNEE::label = 22;
  const int LLEG_KNEE::nbDof = 1;
  const int LLEG_KNEE::positionInConf = 22;
  const Transform LLEG_KNEE::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  Transform LLEG_KNEE::sXp;
  Transform LLEG_KNEE::Xj;
  Motion LLEG_KNEE::cj;
  Motion LLEG_KNEE::vj;
  Force LLEG_KNEE::f;
  double LLEG_KNEE::torque;

  // Init LLEG_ANKLE_P
  JOINT_REVOLUTE(LLEG_ANKLE_P);

  const vector6d LLEG_ANKLE_P::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LLEG_ANKLE_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_ANKLE_P::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_ANKLE_P::name = "LLEG_ANKLE_P";
  const int LLEG_ANKLE_P::label = 23;
  const int LLEG_ANKLE_P::nbDof = 1;
  const int LLEG_ANKLE_P::positionInConf = 23;
  const Transform LLEG_ANKLE_P::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0, 0.3));
  Transform LLEG_ANKLE_P::sXp;
  Transform LLEG_ANKLE_P::Xj;
  Motion LLEG_ANKLE_P::cj;
  Motion LLEG_ANKLE_P::vj;
  Force LLEG_ANKLE_P::f;
  double LLEG_ANKLE_P::torque;

  // Init LLEG_ANKLE_R
  JOINT_REVOLUTE(LLEG_ANKLE_R);

  const vector6d LLEG_ANKLE_R::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d LLEG_ANKLE_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_ANKLE_R::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_ANKLE_R::name = "LLEG_ANKLE_R";
  const int LLEG_ANKLE_R::label = 24;
  const int LLEG_ANKLE_R::nbDof = 1;
  const int LLEG_ANKLE_R::positionInConf = 24;
  const Transform LLEG_ANKLE_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform LLEG_ANKLE_R::sXp;
  Transform LLEG_ANKLE_R::Xj;
  Motion LLEG_ANKLE_R::cj;
  Motion LLEG_ANKLE_R::vj;
  Force LLEG_ANKLE_R::f;
  double LLEG_ANKLE_R::torque;

  // Init RLEG_HIP_R
  JOINT_REVOLUTE(RLEG_HIP_R);

  const vector6d RLEG_HIP_R::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RLEG_HIP_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_HIP_R::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_HIP_R::name = "RLEG_HIP_R";
  const int RLEG_HIP_R::label = 25;
  const int RLEG_HIP_R::nbDof = 1;
  const int RLEG_HIP_R::positionInConf = 6;
  const Transform RLEG_HIP_R::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, -0.09, 0));
  Transform RLEG_HIP_R::sXp;
  Transform RLEG_HIP_R::Xj;
  Motion RLEG_HIP_R::cj;
  Motion RLEG_HIP_R::vj;
  Force RLEG_HIP_R::f;
  double RLEG_HIP_R::torque;

  // Init RLEG_HIP_P
  JOINT_REVOLUTE(RLEG_HIP_P);

  const vector6d RLEG_HIP_P::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RLEG_HIP_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_HIP_P::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_HIP_P::name = "RLEG_HIP_P";
  const int RLEG_HIP_P::label = 26;
  const int RLEG_HIP_P::nbDof = 1;
  const int RLEG_HIP_P::positionInConf = 7;
  const Transform RLEG_HIP_P::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform RLEG_HIP_P::sXp;
  Transform RLEG_HIP_P::Xj;
  Motion RLEG_HIP_P::cj;
  Motion RLEG_HIP_P::vj;
  Force RLEG_HIP_P::f;
  double RLEG_HIP_P::torque;

  // Init RLEG_HIP_Y
  JOINT_REVOLUTE(RLEG_HIP_Y);

  const vector6d RLEG_HIP_Y::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RLEG_HIP_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_HIP_Y::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_HIP_Y::name = "RLEG_HIP_Y";
  const int RLEG_HIP_Y::label = 27;
  const int RLEG_HIP_Y::nbDof = 1;
  const int RLEG_HIP_Y::positionInConf = 8;
  const Transform RLEG_HIP_Y::Xt = Transform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.3535));
  Transform RLEG_HIP_Y::sXp;
  Transform RLEG_HIP_Y::Xj;
  Motion RLEG_HIP_Y::cj;
  Motion RLEG_HIP_Y::vj;
  Force RLEG_HIP_Y::f;
  double RLEG_HIP_Y::torque;

  // Init RLEG_KNEE
  JOINT_REVOLUTE(RLEG_KNEE);

  const vector6d RLEG_KNEE::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RLEG_KNEE::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_KNEE::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_KNEE::name = "RLEG_KNEE";
  const int RLEG_KNEE::label = 28;
  const int RLEG_KNEE::nbDof = 1;
  const int RLEG_KNEE::positionInConf = 9;
  const Transform RLEG_KNEE::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  Transform RLEG_KNEE::sXp;
  Transform RLEG_KNEE::Xj;
  Motion RLEG_KNEE::cj;
  Motion RLEG_KNEE::vj;
  Force RLEG_KNEE::f;
  double RLEG_KNEE::torque;

  // Init RLEG_ANKLE_P
  JOINT_REVOLUTE(RLEG_ANKLE_P);

  const vector6d RLEG_ANKLE_P::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RLEG_ANKLE_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_ANKLE_P::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_ANKLE_P::name = "RLEG_ANKLE_P";
  const int RLEG_ANKLE_P::label = 29;
  const int RLEG_ANKLE_P::nbDof = 1;
  const int RLEG_ANKLE_P::positionInConf = 10;
  const Transform RLEG_ANKLE_P::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0, 0.3));
  Transform RLEG_ANKLE_P::sXp;
  Transform RLEG_ANKLE_P::Xj;
  Motion RLEG_ANKLE_P::cj;
  Motion RLEG_ANKLE_P::vj;
  Force RLEG_ANKLE_P::f;
  double RLEG_ANKLE_P::torque;

  // Init RLEG_ANKLE_R
  JOINT_REVOLUTE(RLEG_ANKLE_R);

  const vector6d RLEG_ANKLE_R::S = vector6dMaker(1, 0, 0, 0, 0, 0);
  const vector6d RLEG_ANKLE_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_ANKLE_R::jcalc(const vector1d & qi, const vector1d & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_ANKLE_R::name = "RLEG_ANKLE_R";
  const int RLEG_ANKLE_R::label = 30;
  const int RLEG_ANKLE_R::nbDof = 1;
  const int RLEG_ANKLE_R::positionInConf = 11;
  const Transform RLEG_ANKLE_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  Transform RLEG_ANKLE_R::sXp;
  Transform RLEG_ANKLE_R::Xj;
  Motion RLEG_ANKLE_R::cj;
  Motion RLEG_ANKLE_R::vj;
  Force RLEG_ANKLE_R::f;
  double RLEG_ANKLE_R::torque;

} // end of namespace simplehumanoid

#endif
