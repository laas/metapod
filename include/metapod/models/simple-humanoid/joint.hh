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

#ifndef METAPOD_SIMPLE_HUMANOID_JOINT_HH
# define METAPOD_SIMPLE_HUMANOID_JOINT_HH

# include "metapod/tools/jointmacros.hh"

namespace simplehumanoid
{

  // Init WAIST
  JOINT_FREE_FLYER(WAIST);

  const std::string WAIST::name = "WAIST";
  const int WAIST::label = 1;
  const int WAIST::positionInConf = 0;
  const Transform WAIST::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0, 0));

  // Init WAIST_P
  JOINT_REVOLUTE(WAIST_P);

  const std::string WAIST_P::name = "WAIST_P";
  const int WAIST_P::label = 2;
  const int WAIST_P::positionInConf = 32;
  const Transform WAIST_P::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

  // Init WAIST_R
  JOINT_REVOLUTE(WAIST_R);

  const std::string WAIST_R::name = "WAIST_R";
  const int WAIST_R::label = 3;
  const int WAIST_R::positionInConf = 33;
  const Transform WAIST_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

  // Init CHEST
  JOINT_REVOLUTE(CHEST);

  const std::string CHEST::name = "CHEST";
  const int CHEST::label = 4;
  const int CHEST::positionInConf = 34;
  const Transform CHEST::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  1, 0, 0,
                  0, 1, 0),
    vector3d(0, 0, 0.35));

  // Init LARM_SHOULDER_P
  JOINT_REVOLUTE(LARM_SHOULDER_P);

  const std::string LARM_SHOULDER_P::name = "LARM_SHOULDER_P";
  const int LARM_SHOULDER_P::label = 5;
  const int LARM_SHOULDER_P::positionInConf = 25;
  const Transform LARM_SHOULDER_P::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0.21));

  // Init LARM_SHOULDER_R
  JOINT_REVOLUTE(LARM_SHOULDER_R);

  const std::string LARM_SHOULDER_R::name = "LARM_SHOULDER_R";
  const int LARM_SHOULDER_R::label = 6;
  const int LARM_SHOULDER_R::positionInConf = 26;
  const Transform LARM_SHOULDER_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

  // Init LARM_SHOULDER_Y
  JOINT_REVOLUTE(LARM_SHOULDER_Y);

  const std::string LARM_SHOULDER_Y::name = "LARM_SHOULDER_Y";
  const int LARM_SHOULDER_Y::label = 7;
  const int LARM_SHOULDER_Y::positionInConf = 27;
  const Transform LARM_SHOULDER_Y::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  1, 0, 0,
                  0, 1, 0),
    vector3d(0, 0, -0.263));

  // Init LARM_ELBOW
  JOINT_REVOLUTE(LARM_ELBOW);

  const std::string LARM_ELBOW::name = "LARM_ELBOW";
  const int LARM_ELBOW::label = 8;
  const int LARM_ELBOW::positionInConf = 28;
  const Transform LARM_ELBOW::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));

  // Init LARM_WRIST_Y
  JOINT_REVOLUTE(LARM_WRIST_Y);

  const std::string LARM_WRIST_Y::name = "LARM_WRIST_Y";
  const int LARM_WRIST_Y::label = 9;
  const int LARM_WRIST_Y::positionInConf = 29;
  const Transform LARM_WRIST_Y::Xt = Transform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.247));

  // Init LARM_WRIST_P
  JOINT_REVOLUTE(LARM_WRIST_P);

  const std::string LARM_WRIST_P::name = "LARM_WRIST_P";
  const int LARM_WRIST_P::label = 10;
  const int LARM_WRIST_P::positionInConf = 30;
  const Transform LARM_WRIST_P::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));

  // Init LARM_WRIST_R
  JOINT_REVOLUTE(LARM_WRIST_R);

  const std::string LARM_WRIST_R::name = "LARM_WRIST_R";
  const int LARM_WRIST_R::label = 11;
  const int LARM_WRIST_R::positionInConf = 31;
  const Transform LARM_WRIST_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

  // Init RARM_SHOULDER_P
  JOINT_REVOLUTE(RARM_SHOULDER_P);

  const std::string RARM_SHOULDER_P::name = "RARM_SHOULDER_P";
  const int RARM_SHOULDER_P::label = 12;
  const int RARM_SHOULDER_P::positionInConf = 12;
  const Transform RARM_SHOULDER_P::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, -0.21));

  // Init RARM_SHOULDER_R
  JOINT_REVOLUTE(RARM_SHOULDER_R);

  const std::string RARM_SHOULDER_R::name = "RARM_SHOULDER_R";
  const int RARM_SHOULDER_R::label = 13;
  const int RARM_SHOULDER_R::positionInConf = 13;
  const Transform RARM_SHOULDER_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

  // Init RARM_SHOULDER_Y
  JOINT_REVOLUTE(RARM_SHOULDER_Y);

  const std::string RARM_SHOULDER_Y::name = "RARM_SHOULDER_Y";
  const int RARM_SHOULDER_Y::label = 14;
  const int RARM_SHOULDER_Y::positionInConf = 14;
  const Transform RARM_SHOULDER_Y::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  1, 0, 0,
                  0, 1, 0),
    vector3d(0, 0, -0.263));

  // Init RARM_ELBOW
  JOINT_REVOLUTE(RARM_ELBOW);

  const std::string RARM_ELBOW::name = "RARM_ELBOW";
  const int RARM_ELBOW::label = 15;
  const int RARM_ELBOW::positionInConf = 15;
  const Transform RARM_ELBOW::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));

  // Init RARM_WRIST_Y
  JOINT_REVOLUTE(RARM_WRIST_Y);

  const std::string RARM_WRIST_Y::name = "RARM_WRIST_Y";
  const int RARM_WRIST_Y::label = 16;
  const int RARM_WRIST_Y::positionInConf = 16;
  const Transform RARM_WRIST_Y::Xt = Transform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.247));

  // Init RARM_WRIST_P
  JOINT_REVOLUTE(RARM_WRIST_P);

  const std::string RARM_WRIST_P::name = "RARM_WRIST_P";
  const int RARM_WRIST_P::label = 17;
  const int RARM_WRIST_P::positionInConf = 17;
  const Transform RARM_WRIST_P::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));

  // Init RARM_WRIST_R
  JOINT_REVOLUTE(RARM_WRIST_R);

  const std::string RARM_WRIST_R::name = "RARM_WRIST_R";
  const int RARM_WRIST_R::label = 18;
  const int RARM_WRIST_R::positionInConf = 18;
  const Transform RARM_WRIST_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

  // Init LLEG_HIP_R
  JOINT_REVOLUTE(LLEG_HIP_R);

  const std::string LLEG_HIP_R::name = "LLEG_HIP_R";
  const int LLEG_HIP_R::label = 19;
  const int LLEG_HIP_R::positionInConf = 19;
  const Transform LLEG_HIP_R::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0.09, 0));

  // Init LLEG_HIP_P
  JOINT_REVOLUTE(LLEG_HIP_P);

  const std::string LLEG_HIP_P::name = "LLEG_HIP_P";
  const int LLEG_HIP_P::label = 20;
  const int LLEG_HIP_P::positionInConf = 20;
  const Transform LLEG_HIP_P::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

  // Init LLEG_HIP_Y
  JOINT_REVOLUTE(LLEG_HIP_Y);

  const std::string LLEG_HIP_Y::name = "LLEG_HIP_Y";
  const int LLEG_HIP_Y::label = 21;
  const int LLEG_HIP_Y::positionInConf = 21;
  const Transform LLEG_HIP_Y::Xt = Transform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.3535));

  // Init LLEG_KNEE
  JOINT_REVOLUTE(LLEG_KNEE);

  const std::string LLEG_KNEE::name = "LLEG_KNEE";
  const int LLEG_KNEE::label = 22;
  const int LLEG_KNEE::positionInConf = 22;
  const Transform LLEG_KNEE::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));

  // Init LLEG_ANKLE_P
  JOINT_REVOLUTE(LLEG_ANKLE_P);

  const std::string LLEG_ANKLE_P::name = "LLEG_ANKLE_P";
  const int LLEG_ANKLE_P::label = 23;
  const int LLEG_ANKLE_P::positionInConf = 23;
  const Transform LLEG_ANKLE_P::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0, 0.3));

  // Init LLEG_ANKLE_R
  JOINT_REVOLUTE(LLEG_ANKLE_R);

  const std::string LLEG_ANKLE_R::name = "LLEG_ANKLE_R";
  const int LLEG_ANKLE_R::label = 24;
  const int LLEG_ANKLE_R::positionInConf = 24;
  const Transform LLEG_ANKLE_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

  // Init RLEG_HIP_R
  JOINT_REVOLUTE(RLEG_HIP_R);

  const std::string RLEG_HIP_R::name = "RLEG_HIP_R";
  const int RLEG_HIP_R::label = 25;
  const int RLEG_HIP_R::positionInConf = 6;
  const Transform RLEG_HIP_R::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, -0.09, 0));

  // Init RLEG_HIP_P
  JOINT_REVOLUTE(RLEG_HIP_P);

  const std::string RLEG_HIP_P::name = "RLEG_HIP_P";
  const int RLEG_HIP_P::label = 26;
  const int RLEG_HIP_P::positionInConf = 7;
  const Transform RLEG_HIP_P::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

  // Init RLEG_HIP_Y
  JOINT_REVOLUTE(RLEG_HIP_Y);

  const std::string RLEG_HIP_Y::name = "RLEG_HIP_Y";
  const int RLEG_HIP_Y::label = 27;
  const int RLEG_HIP_Y::positionInConf = 8;
  const Transform RLEG_HIP_Y::Xt = Transform(
    matrix3dMaker(0, 0, -1,
                  0, 1, 0,
                  1, 0, 0),
    vector3d(0, 0, 0.3535));

  // Init RLEG_KNEE
  JOINT_REVOLUTE(RLEG_KNEE);

  const std::string RLEG_KNEE::name = "RLEG_KNEE";
  const int RLEG_KNEE::label = 28;
  const int RLEG_KNEE::positionInConf = 9;
  const Transform RLEG_KNEE::Xt = Transform(
    matrix3dMaker(0, 0, 1,
                  0, 1, 0,
                  -1, 0, 0),
    vector3d(0, 0, 0));

  // Init RLEG_ANKLE_P
  JOINT_REVOLUTE(RLEG_ANKLE_P);

  const std::string RLEG_ANKLE_P::name = "RLEG_ANKLE_P";
  const int RLEG_ANKLE_P::label = 29;
  const int RLEG_ANKLE_P::positionInConf = 10;
  const Transform RLEG_ANKLE_P::Xt = Transform(
    matrix3dMaker(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1),
    vector3d(0, 0, 0.3));

  // Init RLEG_ANKLE_R
  JOINT_REVOLUTE(RLEG_ANKLE_R);

  const std::string RLEG_ANKLE_R::name = "RLEG_ANKLE_R";
  const int RLEG_ANKLE_R::label = 30;
  const int RLEG_ANKLE_R::positionInConf = 11;
  const Transform RLEG_ANKLE_R::Xt = Transform(
    matrix3dMaker(0, 1, 0,
                  1, 0, 0,
                  0, 0, -1),
    vector3d(0, 0, 0));

} // end of namespace simplehumanoid

#endif
