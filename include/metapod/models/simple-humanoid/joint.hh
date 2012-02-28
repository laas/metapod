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

  matrixN WAIST::S = matrixN::Zero(6,6);
  matrixN WAIST::dotS = matrixN::Zero(6,6);
  void WAIST::jcalc(const vectorN & qi, const vectorN & dqi) { FREE_FLYER_JCALC }
  const std::string WAIST::name = "WAIST";
  const int WAIST::label = 1;
  const int WAIST::nbDof = 6;
  const int WAIST::positionInConf = 0;
  const PluckerTransform WAIST::Xt = PluckerTransform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0, 0));
  PluckerTransform WAIST::sXp;
  PluckerTransform WAIST::Xj;
  vector6d WAIST::cj;
  vector6d WAIST::vj;
  Force WAIST::f;
  vectorN WAIST::torque;

  // Init WAIST_P
  JOINT_REVOLUTE(WAIST_P);

  const matrixN WAIST_P::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN WAIST_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void WAIST_P::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string WAIST_P::name = "WAIST_P";
  const int WAIST_P::label = 2;
  const int WAIST_P::nbDof = 1;
  const int WAIST_P::positionInConf = 32;
  const PluckerTransform WAIST_P::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform WAIST_P::sXp;
  PluckerTransform WAIST_P::Xj;
  vector6d WAIST_P::cj;
  vector6d WAIST_P::vj;
  Force WAIST_P::f;
  vectorN WAIST_P::torque;

  // Init WAIST_R
  JOINT_REVOLUTE(WAIST_R);

  const matrixN WAIST_R::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN WAIST_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void WAIST_R::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string WAIST_R::name = "WAIST_R";
  const int WAIST_R::label = 3;
  const int WAIST_R::nbDof = 1;
  const int WAIST_R::positionInConf = 33;
  const PluckerTransform WAIST_R::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform WAIST_R::sXp;
  PluckerTransform WAIST_R::Xj;
  vector6d WAIST_R::cj;
  vector6d WAIST_R::vj;
  Force WAIST_R::f;
  vectorN WAIST_R::torque;

  // Init CHEST
  JOINT_REVOLUTE(CHEST);

  const matrixN CHEST::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN CHEST::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void CHEST::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string CHEST::name = "CHEST";
  const int CHEST::label = 4;
  const int CHEST::nbDof = 1;
  const int CHEST::positionInConf = 34;
  const PluckerTransform CHEST::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  1, 0, 0,
                  0, 1, 0),
    vector3d(0, 0, 0.35));
  PluckerTransform CHEST::sXp;
  PluckerTransform CHEST::Xj;
  vector6d CHEST::cj;
  vector6d CHEST::vj;
  Force CHEST::f;
  vectorN CHEST::torque;

  // Init LARM_SHOULDER_P
  JOINT_REVOLUTE(LARM_SHOULDER_P);

  const matrixN LARM_SHOULDER_P::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LARM_SHOULDER_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_SHOULDER_P::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_SHOULDER_P::name = "LARM_SHOULDER_P";
  const int LARM_SHOULDER_P::label = 5;
  const int LARM_SHOULDER_P::nbDof = 1;
  const int LARM_SHOULDER_P::positionInConf = 25;
  const PluckerTransform LARM_SHOULDER_P::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0.21));
  PluckerTransform LARM_SHOULDER_P::sXp;
  PluckerTransform LARM_SHOULDER_P::Xj;
  vector6d LARM_SHOULDER_P::cj;
  vector6d LARM_SHOULDER_P::vj;
  Force LARM_SHOULDER_P::f;
  vectorN LARM_SHOULDER_P::torque;

  // Init LARM_SHOULDER_R
  JOINT_REVOLUTE(LARM_SHOULDER_R);

  const matrixN LARM_SHOULDER_R::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LARM_SHOULDER_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_SHOULDER_R::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_SHOULDER_R::name = "LARM_SHOULDER_R";
  const int LARM_SHOULDER_R::label = 6;
  const int LARM_SHOULDER_R::nbDof = 1;
  const int LARM_SHOULDER_R::positionInConf = 26;
  const PluckerTransform LARM_SHOULDER_R::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform LARM_SHOULDER_R::sXp;
  PluckerTransform LARM_SHOULDER_R::Xj;
  vector6d LARM_SHOULDER_R::cj;
  vector6d LARM_SHOULDER_R::vj;
  Force LARM_SHOULDER_R::f;
  vectorN LARM_SHOULDER_R::torque;

  // Init LARM_SHOULDER_Y
  JOINT_REVOLUTE(LARM_SHOULDER_Y);

  const matrixN LARM_SHOULDER_Y::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LARM_SHOULDER_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_SHOULDER_Y::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_SHOULDER_Y::name = "LARM_SHOULDER_Y";
  const int LARM_SHOULDER_Y::label = 7;
  const int LARM_SHOULDER_Y::nbDof = 1;
  const int LARM_SHOULDER_Y::positionInConf = 27;
  const PluckerTransform LARM_SHOULDER_Y::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  1, 0, 0,
                  0, 1, 0),
    vector3d(0, 0, -0.263));
  PluckerTransform LARM_SHOULDER_Y::sXp;
  PluckerTransform LARM_SHOULDER_Y::Xj;
  vector6d LARM_SHOULDER_Y::cj;
  vector6d LARM_SHOULDER_Y::vj;
  Force LARM_SHOULDER_Y::f;
  vectorN LARM_SHOULDER_Y::torque;

  // Init LARM_ELBOW
  JOINT_REVOLUTE(LARM_ELBOW);

  const matrixN LARM_ELBOW::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LARM_ELBOW::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_ELBOW::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_ELBOW::name = "LARM_ELBOW";
  const int LARM_ELBOW::label = 8;
  const int LARM_ELBOW::nbDof = 1;
  const int LARM_ELBOW::positionInConf = 28;
  const PluckerTransform LARM_ELBOW::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  PluckerTransform LARM_ELBOW::sXp;
  PluckerTransform LARM_ELBOW::Xj;
  vector6d LARM_ELBOW::cj;
  vector6d LARM_ELBOW::vj;
  Force LARM_ELBOW::f;
  vectorN LARM_ELBOW::torque;

  // Init LARM_WRIST_Y
  JOINT_REVOLUTE(LARM_WRIST_Y);

  const matrixN LARM_WRIST_Y::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LARM_WRIST_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_WRIST_Y::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_WRIST_Y::name = "LARM_WRIST_Y";
  const int LARM_WRIST_Y::label = 9;
  const int LARM_WRIST_Y::nbDof = 1;
  const int LARM_WRIST_Y::positionInConf = 29;
  const PluckerTransform LARM_WRIST_Y::Xt = PluckerTransform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.247));
  PluckerTransform LARM_WRIST_Y::sXp;
  PluckerTransform LARM_WRIST_Y::Xj;
  vector6d LARM_WRIST_Y::cj;
  vector6d LARM_WRIST_Y::vj;
  Force LARM_WRIST_Y::f;
  vectorN LARM_WRIST_Y::torque;

  // Init LARM_WRIST_P
  JOINT_REVOLUTE(LARM_WRIST_P);

  const matrixN LARM_WRIST_P::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LARM_WRIST_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_WRIST_P::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_WRIST_P::name = "LARM_WRIST_P";
  const int LARM_WRIST_P::label = 10;
  const int LARM_WRIST_P::nbDof = 1;
  const int LARM_WRIST_P::positionInConf = 30;
  const PluckerTransform LARM_WRIST_P::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  PluckerTransform LARM_WRIST_P::sXp;
  PluckerTransform LARM_WRIST_P::Xj;
  vector6d LARM_WRIST_P::cj;
  vector6d LARM_WRIST_P::vj;
  Force LARM_WRIST_P::f;
  vectorN LARM_WRIST_P::torque;

  // Init LARM_WRIST_R
  JOINT_REVOLUTE(LARM_WRIST_R);

  const matrixN LARM_WRIST_R::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LARM_WRIST_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LARM_WRIST_R::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LARM_WRIST_R::name = "LARM_WRIST_R";
  const int LARM_WRIST_R::label = 11;
  const int LARM_WRIST_R::nbDof = 1;
  const int LARM_WRIST_R::positionInConf = 31;
  const PluckerTransform LARM_WRIST_R::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform LARM_WRIST_R::sXp;
  PluckerTransform LARM_WRIST_R::Xj;
  vector6d LARM_WRIST_R::cj;
  vector6d LARM_WRIST_R::vj;
  Force LARM_WRIST_R::f;
  vectorN LARM_WRIST_R::torque;

  // Init RARM_SHOULDER_P
  JOINT_REVOLUTE(RARM_SHOULDER_P);

  const matrixN RARM_SHOULDER_P::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RARM_SHOULDER_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_SHOULDER_P::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_SHOULDER_P::name = "RARM_SHOULDER_P";
  const int RARM_SHOULDER_P::label = 12;
  const int RARM_SHOULDER_P::nbDof = 1;
  const int RARM_SHOULDER_P::positionInConf = 12;
  const PluckerTransform RARM_SHOULDER_P::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, -0.21));
  PluckerTransform RARM_SHOULDER_P::sXp;
  PluckerTransform RARM_SHOULDER_P::Xj;
  vector6d RARM_SHOULDER_P::cj;
  vector6d RARM_SHOULDER_P::vj;
  Force RARM_SHOULDER_P::f;
  vectorN RARM_SHOULDER_P::torque;

  // Init RARM_SHOULDER_R
  JOINT_REVOLUTE(RARM_SHOULDER_R);

  const matrixN RARM_SHOULDER_R::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RARM_SHOULDER_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_SHOULDER_R::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_SHOULDER_R::name = "RARM_SHOULDER_R";
  const int RARM_SHOULDER_R::label = 13;
  const int RARM_SHOULDER_R::nbDof = 1;
  const int RARM_SHOULDER_R::positionInConf = 13;
  const PluckerTransform RARM_SHOULDER_R::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform RARM_SHOULDER_R::sXp;
  PluckerTransform RARM_SHOULDER_R::Xj;
  vector6d RARM_SHOULDER_R::cj;
  vector6d RARM_SHOULDER_R::vj;
  Force RARM_SHOULDER_R::f;
  vectorN RARM_SHOULDER_R::torque;

  // Init RARM_SHOULDER_Y
  JOINT_REVOLUTE(RARM_SHOULDER_Y);

  const matrixN RARM_SHOULDER_Y::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RARM_SHOULDER_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_SHOULDER_Y::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_SHOULDER_Y::name = "RARM_SHOULDER_Y";
  const int RARM_SHOULDER_Y::label = 14;
  const int RARM_SHOULDER_Y::nbDof = 1;
  const int RARM_SHOULDER_Y::positionInConf = 14;
  const PluckerTransform RARM_SHOULDER_Y::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  1, 0, 0,
                  0, 1, 0),
    vector3d(0, 0, -0.263));
  PluckerTransform RARM_SHOULDER_Y::sXp;
  PluckerTransform RARM_SHOULDER_Y::Xj;
  vector6d RARM_SHOULDER_Y::cj;
  vector6d RARM_SHOULDER_Y::vj;
  Force RARM_SHOULDER_Y::f;
  vectorN RARM_SHOULDER_Y::torque;

  // Init RARM_ELBOW
  JOINT_REVOLUTE(RARM_ELBOW);

  const matrixN RARM_ELBOW::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RARM_ELBOW::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_ELBOW::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_ELBOW::name = "RARM_ELBOW";
  const int RARM_ELBOW::label = 15;
  const int RARM_ELBOW::nbDof = 1;
  const int RARM_ELBOW::positionInConf = 15;
  const PluckerTransform RARM_ELBOW::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  PluckerTransform RARM_ELBOW::sXp;
  PluckerTransform RARM_ELBOW::Xj;
  vector6d RARM_ELBOW::cj;
  vector6d RARM_ELBOW::vj;
  Force RARM_ELBOW::f;
  vectorN RARM_ELBOW::torque;

  // Init RARM_WRIST_Y
  JOINT_REVOLUTE(RARM_WRIST_Y);

  const matrixN RARM_WRIST_Y::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RARM_WRIST_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_WRIST_Y::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_WRIST_Y::name = "RARM_WRIST_Y";
  const int RARM_WRIST_Y::label = 16;
  const int RARM_WRIST_Y::nbDof = 1;
  const int RARM_WRIST_Y::positionInConf = 16;
  const PluckerTransform RARM_WRIST_Y::Xt = PluckerTransform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.247));
  PluckerTransform RARM_WRIST_Y::sXp;
  PluckerTransform RARM_WRIST_Y::Xj;
  vector6d RARM_WRIST_Y::cj;
  vector6d RARM_WRIST_Y::vj;
  Force RARM_WRIST_Y::f;
  vectorN RARM_WRIST_Y::torque;

  // Init RARM_WRIST_P
  JOINT_REVOLUTE(RARM_WRIST_P);

  const matrixN RARM_WRIST_P::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RARM_WRIST_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_WRIST_P::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_WRIST_P::name = "RARM_WRIST_P";
  const int RARM_WRIST_P::label = 17;
  const int RARM_WRIST_P::nbDof = 1;
  const int RARM_WRIST_P::positionInConf = 17;
  const PluckerTransform RARM_WRIST_P::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  PluckerTransform RARM_WRIST_P::sXp;
  PluckerTransform RARM_WRIST_P::Xj;
  vector6d RARM_WRIST_P::cj;
  vector6d RARM_WRIST_P::vj;
  Force RARM_WRIST_P::f;
  vectorN RARM_WRIST_P::torque;

  // Init RARM_WRIST_R
  JOINT_REVOLUTE(RARM_WRIST_R);

  const matrixN RARM_WRIST_R::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RARM_WRIST_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RARM_WRIST_R::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RARM_WRIST_R::name = "RARM_WRIST_R";
  const int RARM_WRIST_R::label = 18;
  const int RARM_WRIST_R::nbDof = 1;
  const int RARM_WRIST_R::positionInConf = 18;
  const PluckerTransform RARM_WRIST_R::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform RARM_WRIST_R::sXp;
  PluckerTransform RARM_WRIST_R::Xj;
  vector6d RARM_WRIST_R::cj;
  vector6d RARM_WRIST_R::vj;
  Force RARM_WRIST_R::f;
  vectorN RARM_WRIST_R::torque;

  // Init LLEG_HIP_R
  JOINT_REVOLUTE(LLEG_HIP_R);

  const matrixN LLEG_HIP_R::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LLEG_HIP_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_HIP_R::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_HIP_R::name = "LLEG_HIP_R";
  const int LLEG_HIP_R::label = 19;
  const int LLEG_HIP_R::nbDof = 1;
  const int LLEG_HIP_R::positionInConf = 19;
  const PluckerTransform LLEG_HIP_R::Xt = PluckerTransform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0.09, 0));
  PluckerTransform LLEG_HIP_R::sXp;
  PluckerTransform LLEG_HIP_R::Xj;
  vector6d LLEG_HIP_R::cj;
  vector6d LLEG_HIP_R::vj;
  Force LLEG_HIP_R::f;
  vectorN LLEG_HIP_R::torque;

  // Init LLEG_HIP_P
  JOINT_REVOLUTE(LLEG_HIP_P);

  const matrixN LLEG_HIP_P::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LLEG_HIP_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_HIP_P::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_HIP_P::name = "LLEG_HIP_P";
  const int LLEG_HIP_P::label = 20;
  const int LLEG_HIP_P::nbDof = 1;
  const int LLEG_HIP_P::positionInConf = 20;
  const PluckerTransform LLEG_HIP_P::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform LLEG_HIP_P::sXp;
  PluckerTransform LLEG_HIP_P::Xj;
  vector6d LLEG_HIP_P::cj;
  vector6d LLEG_HIP_P::vj;
  Force LLEG_HIP_P::f;
  vectorN LLEG_HIP_P::torque;

  // Init LLEG_HIP_Y
  JOINT_REVOLUTE(LLEG_HIP_Y);

  const matrixN LLEG_HIP_Y::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LLEG_HIP_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_HIP_Y::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_HIP_Y::name = "LLEG_HIP_Y";
  const int LLEG_HIP_Y::label = 21;
  const int LLEG_HIP_Y::nbDof = 1;
  const int LLEG_HIP_Y::positionInConf = 21;
  const PluckerTransform LLEG_HIP_Y::Xt = PluckerTransform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.3535));
  PluckerTransform LLEG_HIP_Y::sXp;
  PluckerTransform LLEG_HIP_Y::Xj;
  vector6d LLEG_HIP_Y::cj;
  vector6d LLEG_HIP_Y::vj;
  Force LLEG_HIP_Y::f;
  vectorN LLEG_HIP_Y::torque;

  // Init LLEG_KNEE
  JOINT_REVOLUTE(LLEG_KNEE);

  const matrixN LLEG_KNEE::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LLEG_KNEE::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_KNEE::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_KNEE::name = "LLEG_KNEE";
  const int LLEG_KNEE::label = 22;
  const int LLEG_KNEE::nbDof = 1;
  const int LLEG_KNEE::positionInConf = 22;
  const PluckerTransform LLEG_KNEE::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  PluckerTransform LLEG_KNEE::sXp;
  PluckerTransform LLEG_KNEE::Xj;
  vector6d LLEG_KNEE::cj;
  vector6d LLEG_KNEE::vj;
  Force LLEG_KNEE::f;
  vectorN LLEG_KNEE::torque;

  // Init LLEG_ANKLE_P
  JOINT_REVOLUTE(LLEG_ANKLE_P);

  const matrixN LLEG_ANKLE_P::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LLEG_ANKLE_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_ANKLE_P::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_ANKLE_P::name = "LLEG_ANKLE_P";
  const int LLEG_ANKLE_P::label = 23;
  const int LLEG_ANKLE_P::nbDof = 1;
  const int LLEG_ANKLE_P::positionInConf = 23;
  const PluckerTransform LLEG_ANKLE_P::Xt = PluckerTransform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0, 0.3));
  PluckerTransform LLEG_ANKLE_P::sXp;
  PluckerTransform LLEG_ANKLE_P::Xj;
  vector6d LLEG_ANKLE_P::cj;
  vector6d LLEG_ANKLE_P::vj;
  Force LLEG_ANKLE_P::f;
  vectorN LLEG_ANKLE_P::torque;

  // Init LLEG_ANKLE_R
  JOINT_REVOLUTE(LLEG_ANKLE_R);

  const matrixN LLEG_ANKLE_R::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN LLEG_ANKLE_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void LLEG_ANKLE_R::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string LLEG_ANKLE_R::name = "LLEG_ANKLE_R";
  const int LLEG_ANKLE_R::label = 24;
  const int LLEG_ANKLE_R::nbDof = 1;
  const int LLEG_ANKLE_R::positionInConf = 24;
  const PluckerTransform LLEG_ANKLE_R::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform LLEG_ANKLE_R::sXp;
  PluckerTransform LLEG_ANKLE_R::Xj;
  vector6d LLEG_ANKLE_R::cj;
  vector6d LLEG_ANKLE_R::vj;
  Force LLEG_ANKLE_R::f;
  vectorN LLEG_ANKLE_R::torque;

  // Init RLEG_HIP_R
  JOINT_REVOLUTE(RLEG_HIP_R);

  const matrixN RLEG_HIP_R::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RLEG_HIP_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_HIP_R::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_HIP_R::name = "RLEG_HIP_R";
  const int RLEG_HIP_R::label = 25;
  const int RLEG_HIP_R::nbDof = 1;
  const int RLEG_HIP_R::positionInConf = 6;
  const PluckerTransform RLEG_HIP_R::Xt = PluckerTransform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, -0.09, 0));
  PluckerTransform RLEG_HIP_R::sXp;
  PluckerTransform RLEG_HIP_R::Xj;
  vector6d RLEG_HIP_R::cj;
  vector6d RLEG_HIP_R::vj;
  Force RLEG_HIP_R::f;
  vectorN RLEG_HIP_R::torque;

  // Init RLEG_HIP_P
  JOINT_REVOLUTE(RLEG_HIP_P);

  const matrixN RLEG_HIP_P::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RLEG_HIP_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_HIP_P::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_HIP_P::name = "RLEG_HIP_P";
  const int RLEG_HIP_P::label = 26;
  const int RLEG_HIP_P::nbDof = 1;
  const int RLEG_HIP_P::positionInConf = 7;
  const PluckerTransform RLEG_HIP_P::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform RLEG_HIP_P::sXp;
  PluckerTransform RLEG_HIP_P::Xj;
  vector6d RLEG_HIP_P::cj;
  vector6d RLEG_HIP_P::vj;
  Force RLEG_HIP_P::f;
  vectorN RLEG_HIP_P::torque;

  // Init RLEG_HIP_Y
  JOINT_REVOLUTE(RLEG_HIP_Y);

  const matrixN RLEG_HIP_Y::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RLEG_HIP_Y::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_HIP_Y::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_HIP_Y::name = "RLEG_HIP_Y";
  const int RLEG_HIP_Y::label = 27;
  const int RLEG_HIP_Y::nbDof = 1;
  const int RLEG_HIP_Y::positionInConf = 8;
  const PluckerTransform RLEG_HIP_Y::Xt = PluckerTransform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.3535));
  PluckerTransform RLEG_HIP_Y::sXp;
  PluckerTransform RLEG_HIP_Y::Xj;
  vector6d RLEG_HIP_Y::cj;
  vector6d RLEG_HIP_Y::vj;
  Force RLEG_HIP_Y::f;
  vectorN RLEG_HIP_Y::torque;

  // Init RLEG_KNEE
  JOINT_REVOLUTE(RLEG_KNEE);

  const matrixN RLEG_KNEE::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RLEG_KNEE::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_KNEE::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_KNEE::name = "RLEG_KNEE";
  const int RLEG_KNEE::label = 28;
  const int RLEG_KNEE::nbDof = 1;
  const int RLEG_KNEE::positionInConf = 9;
  const PluckerTransform RLEG_KNEE::Xt = PluckerTransform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));
  PluckerTransform RLEG_KNEE::sXp;
  PluckerTransform RLEG_KNEE::Xj;
  vector6d RLEG_KNEE::cj;
  vector6d RLEG_KNEE::vj;
  Force RLEG_KNEE::f;
  vectorN RLEG_KNEE::torque;

  // Init RLEG_ANKLE_P
  JOINT_REVOLUTE(RLEG_ANKLE_P);

  const matrixN RLEG_ANKLE_P::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RLEG_ANKLE_P::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_ANKLE_P::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_ANKLE_P::name = "RLEG_ANKLE_P";
  const int RLEG_ANKLE_P::label = 29;
  const int RLEG_ANKLE_P::nbDof = 1;
  const int RLEG_ANKLE_P::positionInConf = 10;
  const PluckerTransform RLEG_ANKLE_P::Xt = PluckerTransform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0, 0.3));
  PluckerTransform RLEG_ANKLE_P::sXp;
  PluckerTransform RLEG_ANKLE_P::Xj;
  vector6d RLEG_ANKLE_P::cj;
  vector6d RLEG_ANKLE_P::vj;
  Force RLEG_ANKLE_P::f;
  vectorN RLEG_ANKLE_P::torque;

  // Init RLEG_ANKLE_R
  JOINT_REVOLUTE(RLEG_ANKLE_R);

  const matrixN RLEG_ANKLE_R::S = vector6dMaker(0, 0, 0, 1, 0, 0);
  const matrixN RLEG_ANKLE_R::dotS = vector6dMaker(0, 0, 0, 0, 0, 0);
  void RLEG_ANKLE_R::jcalc(const vectorN & qi, const vectorN & dqi) { REVOLUTE_JOINT_JCALC }
  const std::string RLEG_ANKLE_R::name = "RLEG_ANKLE_R";
  const int RLEG_ANKLE_R::label = 30;
  const int RLEG_ANKLE_R::nbDof = 1;
  const int RLEG_ANKLE_R::positionInConf = 11;
  const PluckerTransform RLEG_ANKLE_R::Xt = PluckerTransform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));
  PluckerTransform RLEG_ANKLE_R::sXp;
  PluckerTransform RLEG_ANKLE_R::Xj;
  vector6d RLEG_ANKLE_R::cj;
  vector6d RLEG_ANKLE_R::vj;
  Force RLEG_ANKLE_R::f;
  vectorN RLEG_ANKLE_R::torque;

} // end of namespace simplehumanoid

#endif
