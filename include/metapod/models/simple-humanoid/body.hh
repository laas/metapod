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
 * It contains the definition of all the robot bodies.
 */

#ifndef metapod_SIMPLE_HUMANOID_BODY_HH
# define metapod_SIMPLE_HUMANOID_BODY_HH

# include "metapod/tools/bodymacros.hh"

namespace simplehumanoid
{

  using namespace metapod;
  using namespace metapod::Spatial;

  // Forward declaration of WAIST class
  class WAIST;

  // Declaration of WAIST_LINK0 class
  CREATE_BODY(WAIST_LINK0, 0, NP, WAIST);

  // Initialization of WAIST_LINK0;
  const std::string WAIST_LINK0::name = "WAIST_LINK0";
  const int WAIST_LINK0::label = 0;
  const double WAIST_LINK0::mass = 27;
  const vector3d WAIST_LINK0::CoM = vector3d(0, 0, 0.0375);
  const matrix3d WAIST_LINK0::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform WAIST_LINK0::iX0;
  Motion WAIST_LINK0::vi;
  Motion WAIST_LINK0::ai;
  Force WAIST_LINK0::Fext;
  Inertia WAIST_LINK0::I = spatialInertiaMaker(WAIST_LINK0::mass, WAIST_LINK0::CoM, WAIST_LINK0::inertie);


  // Forward declaration of WAIST_P class
  class WAIST_P;

  // Declaration of WAIST_LINK1 class
  CREATE_BODY(WAIST_LINK1, 1, WAIST_LINK0, WAIST_P);

  // Initialization of WAIST_LINK1;
  const std::string WAIST_LINK1::name = "WAIST_LINK1";
  const int WAIST_LINK1::label = 1;
  const double WAIST_LINK1::mass = 6;
  const vector3d WAIST_LINK1::CoM = vector3d(0, 0, 0.1);
  const matrix3d WAIST_LINK1::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform WAIST_LINK1::iX0;
  Motion WAIST_LINK1::vi;
  Motion WAIST_LINK1::ai;
  Force WAIST_LINK1::Fext;
  Inertia WAIST_LINK1::I = spatialInertiaMaker(WAIST_LINK1::mass, WAIST_LINK1::CoM, WAIST_LINK1::inertie);


  // Forward declaration of WAIST_R class
  class WAIST_R;

  // Declaration of WAIST_LINK2 class
  CREATE_BODY(WAIST_LINK2, 1, WAIST_LINK1, WAIST_R);

  // Initialization of WAIST_LINK2;
  const std::string WAIST_LINK2::name = "WAIST_LINK2";
  const int WAIST_LINK2::label = 2;
  const double WAIST_LINK2::mass = 30;
  const vector3d WAIST_LINK2::CoM = vector3d(0.11, 0, 0.25);
  const matrix3d WAIST_LINK2::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform WAIST_LINK2::iX0;
  Motion WAIST_LINK2::vi;
  Motion WAIST_LINK2::ai;
  Force WAIST_LINK2::Fext;
  Inertia WAIST_LINK2::I = spatialInertiaMaker(WAIST_LINK2::mass, WAIST_LINK2::CoM, WAIST_LINK2::inertie);


  // Forward declaration of CHEST class
  class CHEST;

  // Declaration of WAIST_LINK3 class
  CREATE_BODY(WAIST_LINK3, 1, WAIST_LINK2, CHEST);

  // Initialization of WAIST_LINK3;
  const std::string WAIST_LINK3::name = "WAIST_LINK3";
  const int WAIST_LINK3::label = 3;
  const double WAIST_LINK3::mass = 13;
  const vector3d WAIST_LINK3::CoM = vector3d(0, 0, 0);
  const matrix3d WAIST_LINK3::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform WAIST_LINK3::iX0;
  Motion WAIST_LINK3::vi;
  Motion WAIST_LINK3::ai;
  Force WAIST_LINK3::Fext;
  Inertia WAIST_LINK3::I = spatialInertiaMaker(WAIST_LINK3::mass, WAIST_LINK3::CoM, WAIST_LINK3::inertie);


  // Forward declaration of LARM_SHOULDER_P class
  class LARM_SHOULDER_P;

  // Declaration of LARM_LINK1 class
  CREATE_BODY(LARM_LINK1, 1, WAIST_LINK3, LARM_SHOULDER_P);

  // Initialization of LARM_LINK1;
  const std::string LARM_LINK1::name = "LARM_LINK1";
  const int LARM_LINK1::label = 4;
  const double LARM_LINK1::mass = 3;
  const vector3d LARM_LINK1::CoM = vector3d(0, 0.1, 0);
  const matrix3d LARM_LINK1::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LARM_LINK1::iX0;
  Motion LARM_LINK1::vi;
  Motion LARM_LINK1::ai;
  Force LARM_LINK1::Fext;
  Inertia LARM_LINK1::I = spatialInertiaMaker(LARM_LINK1::mass, LARM_LINK1::CoM, LARM_LINK1::inertie);


  // Forward declaration of LARM_SHOULDER_R class
  class LARM_SHOULDER_R;

  // Declaration of LARM_LINK2 class
  CREATE_BODY(LARM_LINK2, 1, LARM_LINK1, LARM_SHOULDER_R);

  // Initialization of LARM_LINK2;
  const std::string LARM_LINK2::name = "LARM_LINK2";
  const int LARM_LINK2::label = 5;
  const double LARM_LINK2::mass = 0.6;
  const vector3d LARM_LINK2::CoM = vector3d(0, 0, -0.1);
  const matrix3d LARM_LINK2::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LARM_LINK2::iX0;
  Motion LARM_LINK2::vi;
  Motion LARM_LINK2::ai;
  Force LARM_LINK2::Fext;
  Inertia LARM_LINK2::I = spatialInertiaMaker(LARM_LINK2::mass, LARM_LINK2::CoM, LARM_LINK2::inertie);


  // Forward declaration of LARM_SHOULDER_Y class
  class LARM_SHOULDER_Y;

  // Declaration of LARM_LINK3 class
  CREATE_BODY(LARM_LINK3, 1, LARM_LINK2, LARM_SHOULDER_Y);

  // Initialization of LARM_LINK3;
  const std::string LARM_LINK3::name = "LARM_LINK3";
  const int LARM_LINK3::label = 6;
  const double LARM_LINK3::mass = 1;
  const vector3d LARM_LINK3::CoM = vector3d(0, 0, 0);
  const matrix3d LARM_LINK3::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LARM_LINK3::iX0;
  Motion LARM_LINK3::vi;
  Motion LARM_LINK3::ai;
  Force LARM_LINK3::Fext;
  Inertia LARM_LINK3::I = spatialInertiaMaker(LARM_LINK3::mass, LARM_LINK3::CoM, LARM_LINK3::inertie);


  // Forward declaration of LARM_ELBOW class
  class LARM_ELBOW;

  // Declaration of LARM_LINK4 class
  CREATE_BODY(LARM_LINK4, 1, LARM_LINK3, LARM_ELBOW);

  // Initialization of LARM_LINK4;
  const std::string LARM_LINK4::name = "LARM_LINK4";
  const int LARM_LINK4::label = 7;
  const double LARM_LINK4::mass = 0.6;
  const vector3d LARM_LINK4::CoM = vector3d(0, 0, 0.3);
  const matrix3d LARM_LINK4::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LARM_LINK4::iX0;
  Motion LARM_LINK4::vi;
  Motion LARM_LINK4::ai;
  Force LARM_LINK4::Fext;
  Inertia LARM_LINK4::I = spatialInertiaMaker(LARM_LINK4::mass, LARM_LINK4::CoM, LARM_LINK4::inertie);


  // Forward declaration of LARM_WRIST_Y class
  class LARM_WRIST_Y;

  // Declaration of LARM_LINK5 class
  CREATE_BODY(LARM_LINK5, 1, LARM_LINK4, LARM_WRIST_Y);

  // Initialization of LARM_LINK5;
  const std::string LARM_LINK5::name = "LARM_LINK5";
  const int LARM_LINK5::label = 8;
  const double LARM_LINK5::mass = 0.4;
  const vector3d LARM_LINK5::CoM = vector3d(0.1, 0, 0);
  const matrix3d LARM_LINK5::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LARM_LINK5::iX0;
  Motion LARM_LINK5::vi;
  Motion LARM_LINK5::ai;
  Force LARM_LINK5::Fext;
  Inertia LARM_LINK5::I = spatialInertiaMaker(LARM_LINK5::mass, LARM_LINK5::CoM, LARM_LINK5::inertie);


  // Forward declaration of LARM_WRIST_P class
  class LARM_WRIST_P;

  // Declaration of LARM_LINK6 class
  CREATE_BODY(LARM_LINK6, 1, LARM_LINK5, LARM_WRIST_P);

  // Initialization of LARM_LINK6;
  const std::string LARM_LINK6::name = "LARM_LINK6";
  const int LARM_LINK6::label = 9;
  const double LARM_LINK6::mass = 0.4;
  const vector3d LARM_LINK6::CoM = vector3d(0, -0.1, 0);
  const matrix3d LARM_LINK6::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LARM_LINK6::iX0;
  Motion LARM_LINK6::vi;
  Motion LARM_LINK6::ai;
  Force LARM_LINK6::Fext;
  Inertia LARM_LINK6::I = spatialInertiaMaker(LARM_LINK6::mass, LARM_LINK6::CoM, LARM_LINK6::inertie);


  // Forward declaration of LARM_WRIST_R class
  class LARM_WRIST_R;

  // Declaration of LARM_LINK7 class
  CREATE_BODY(LARM_LINK7, 1, LARM_LINK6, LARM_WRIST_R);

  // Initialization of LARM_LINK7;
  const std::string LARM_LINK7::name = "LARM_LINK7";
  const int LARM_LINK7::label = 10;
  const double LARM_LINK7::mass = 0.4;
  const vector3d LARM_LINK7::CoM = vector3d(0, 0, -0.1);
  const matrix3d LARM_LINK7::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LARM_LINK7::iX0;
  Motion LARM_LINK7::vi;
  Motion LARM_LINK7::ai;
  Force LARM_LINK7::Fext;
  Inertia LARM_LINK7::I = spatialInertiaMaker(LARM_LINK7::mass, LARM_LINK7::CoM, LARM_LINK7::inertie);


  // Forward declaration of RARM_SHOULDER_P class
  class RARM_SHOULDER_P;

  // Declaration of RARM_LINK1 class
  CREATE_BODY(RARM_LINK1, 1, WAIST_LINK3, RARM_SHOULDER_P);

  // Initialization of RARM_LINK1;
  const std::string RARM_LINK1::name = "RARM_LINK1";
  const int RARM_LINK1::label = 11;
  const double RARM_LINK1::mass = 3;
  const vector3d RARM_LINK1::CoM = vector3d(0, 0.1, 0);
  const matrix3d RARM_LINK1::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RARM_LINK1::iX0;
  Motion RARM_LINK1::vi;
  Motion RARM_LINK1::ai;
  Force RARM_LINK1::Fext;
  Inertia RARM_LINK1::I = spatialInertiaMaker(RARM_LINK1::mass, RARM_LINK1::CoM, RARM_LINK1::inertie);


  // Forward declaration of RARM_SHOULDER_R class
  class RARM_SHOULDER_R;

  // Declaration of RARM_LINK2 class
  CREATE_BODY(RARM_LINK2, 1, RARM_LINK1, RARM_SHOULDER_R);

  // Initialization of RARM_LINK2;
  const std::string RARM_LINK2::name = "RARM_LINK2";
  const int RARM_LINK2::label = 12;
  const double RARM_LINK2::mass = 0.6;
  const vector3d RARM_LINK2::CoM = vector3d(0, 0, -0.1);
  const matrix3d RARM_LINK2::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RARM_LINK2::iX0;
  Motion RARM_LINK2::vi;
  Motion RARM_LINK2::ai;
  Force RARM_LINK2::Fext;
  Inertia RARM_LINK2::I = spatialInertiaMaker(RARM_LINK2::mass, RARM_LINK2::CoM, RARM_LINK2::inertie);


  // Forward declaration of RARM_SHOULDER_Y class
  class RARM_SHOULDER_Y;

  // Declaration of RARM_LINK3 class
  CREATE_BODY(RARM_LINK3, 1, RARM_LINK2, RARM_SHOULDER_Y);

  // Initialization of RARM_LINK3;
  const std::string RARM_LINK3::name = "RARM_LINK3";
  const int RARM_LINK3::label = 13;
  const double RARM_LINK3::mass = 1;
  const vector3d RARM_LINK3::CoM = vector3d(0, 0, 0);
  const matrix3d RARM_LINK3::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RARM_LINK3::iX0;
  Motion RARM_LINK3::vi;
  Motion RARM_LINK3::ai;
  Force RARM_LINK3::Fext;
  Inertia RARM_LINK3::I = spatialInertiaMaker(RARM_LINK3::mass, RARM_LINK3::CoM, RARM_LINK3::inertie);


  // Forward declaration of RARM_ELBOW class
  class RARM_ELBOW;

  // Declaration of RARM_LINK4 class
  CREATE_BODY(RARM_LINK4, 1, RARM_LINK3, RARM_ELBOW);

  // Initialization of RARM_LINK4;
  const std::string RARM_LINK4::name = "RARM_LINK4";
  const int RARM_LINK4::label = 14;
  const double RARM_LINK4::mass = 0.6;
  const vector3d RARM_LINK4::CoM = vector3d(0, 0, 0.3);
  const matrix3d RARM_LINK4::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RARM_LINK4::iX0;
  Motion RARM_LINK4::vi;
  Motion RARM_LINK4::ai;
  Force RARM_LINK4::Fext;
  Inertia RARM_LINK4::I = spatialInertiaMaker(RARM_LINK4::mass, RARM_LINK4::CoM, RARM_LINK4::inertie);


  // Forward declaration of RARM_WRIST_Y class
  class RARM_WRIST_Y;

  // Declaration of RARM_LINK5 class
  CREATE_BODY(RARM_LINK5, 1, RARM_LINK4, RARM_WRIST_Y);

  // Initialization of RARM_LINK5;
  const std::string RARM_LINK5::name = "RARM_LINK5";
  const int RARM_LINK5::label = 15;
  const double RARM_LINK5::mass = 0.4;
  const vector3d RARM_LINK5::CoM = vector3d(0.1, 0, 0);
  const matrix3d RARM_LINK5::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RARM_LINK5::iX0;
  Motion RARM_LINK5::vi;
  Motion RARM_LINK5::ai;
  Force RARM_LINK5::Fext;
  Inertia RARM_LINK5::I = spatialInertiaMaker(RARM_LINK5::mass, RARM_LINK5::CoM, RARM_LINK5::inertie);


  // Forward declaration of RARM_WRIST_P class
  class RARM_WRIST_P;

  // Declaration of RARM_LINK6 class
  CREATE_BODY(RARM_LINK6, 1, RARM_LINK5, RARM_WRIST_P);

  // Initialization of RARM_LINK6;
  const std::string RARM_LINK6::name = "RARM_LINK6";
  const int RARM_LINK6::label = 16;
  const double RARM_LINK6::mass = 0.4;
  const vector3d RARM_LINK6::CoM = vector3d(0, -0.1, 0);
  const matrix3d RARM_LINK6::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RARM_LINK6::iX0;
  Motion RARM_LINK6::vi;
  Motion RARM_LINK6::ai;
  Force RARM_LINK6::Fext;
  Inertia RARM_LINK6::I = spatialInertiaMaker(RARM_LINK6::mass, RARM_LINK6::CoM, RARM_LINK6::inertie);


  // Forward declaration of RARM_WRIST_R class
  class RARM_WRIST_R;

  // Declaration of RARM_LINK7 class
  CREATE_BODY(RARM_LINK7, 1, RARM_LINK6, RARM_WRIST_R);

  // Initialization of RARM_LINK7;
  const std::string RARM_LINK7::name = "RARM_LINK7";
  const int RARM_LINK7::label = 17;
  const double RARM_LINK7::mass = 0.4;
  const vector3d RARM_LINK7::CoM = vector3d(0, 0, -0.1);
  const matrix3d RARM_LINK7::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RARM_LINK7::iX0;
  Motion RARM_LINK7::vi;
  Motion RARM_LINK7::ai;
  Force RARM_LINK7::Fext;
  Inertia RARM_LINK7::I = spatialInertiaMaker(RARM_LINK7::mass, RARM_LINK7::CoM, RARM_LINK7::inertie);


  // Forward declaration of LLEG_HIP_R class
  class LLEG_HIP_R;

  // Declaration of LLEG_LINK1 class
  CREATE_BODY(LLEG_LINK1, 1, WAIST_LINK0, LLEG_HIP_R);

  // Initialization of LLEG_LINK1;
  const std::string LLEG_LINK1::name = "LLEG_LINK1";
  const int LLEG_LINK1::label = 18;
  const double LLEG_LINK1::mass = 2.5;
  const vector3d LLEG_LINK1::CoM = vector3d(0, 0.1, 0);
  const matrix3d LLEG_LINK1::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LLEG_LINK1::iX0;
  Motion LLEG_LINK1::vi;
  Motion LLEG_LINK1::ai;
  Force LLEG_LINK1::Fext;
  Inertia LLEG_LINK1::I = spatialInertiaMaker(LLEG_LINK1::mass, LLEG_LINK1::CoM, LLEG_LINK1::inertie);


  // Forward declaration of LLEG_HIP_P class
  class LLEG_HIP_P;

  // Declaration of LLEG_LINK2 class
  CREATE_BODY(LLEG_LINK2, 1, LLEG_LINK1, LLEG_HIP_P);

  // Initialization of LLEG_LINK2;
  const std::string LLEG_LINK2::name = "LLEG_LINK2";
  const int LLEG_LINK2::label = 19;
  const double LLEG_LINK2::mass = 2;
  const vector3d LLEG_LINK2::CoM = vector3d(0, 0, 0.15);
  const matrix3d LLEG_LINK2::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LLEG_LINK2::iX0;
  Motion LLEG_LINK2::vi;
  Motion LLEG_LINK2::ai;
  Force LLEG_LINK2::Fext;
  Inertia LLEG_LINK2::I = spatialInertiaMaker(LLEG_LINK2::mass, LLEG_LINK2::CoM, LLEG_LINK2::inertie);


  // Forward declaration of LLEG_HIP_Y class
  class LLEG_HIP_Y;

  // Declaration of LLEG_LINK3 class
  CREATE_BODY(LLEG_LINK3, 1, LLEG_LINK2, LLEG_HIP_Y);

  // Initialization of LLEG_LINK3;
  const std::string LLEG_LINK3::name = "LLEG_LINK3";
  const int LLEG_LINK3::label = 20;
  const double LLEG_LINK3::mass = 5.1;
  const vector3d LLEG_LINK3::CoM = vector3d(0, 0, 0.04);
  const matrix3d LLEG_LINK3::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LLEG_LINK3::iX0;
  Motion LLEG_LINK3::vi;
  Motion LLEG_LINK3::ai;
  Force LLEG_LINK3::Fext;
  Inertia LLEG_LINK3::I = spatialInertiaMaker(LLEG_LINK3::mass, LLEG_LINK3::CoM, LLEG_LINK3::inertie);


  // Forward declaration of LLEG_KNEE class
  class LLEG_KNEE;

  // Declaration of LLEG_LINK4 class
  CREATE_BODY(LLEG_LINK4, 1, LLEG_LINK3, LLEG_KNEE);

  // Initialization of LLEG_LINK4;
  const std::string LLEG_LINK4::name = "LLEG_LINK4";
  const int LLEG_LINK4::label = 21;
  const double LLEG_LINK4::mass = 7;
  const vector3d LLEG_LINK4::CoM = vector3d(0, 0, 0.3);
  const matrix3d LLEG_LINK4::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LLEG_LINK4::iX0;
  Motion LLEG_LINK4::vi;
  Motion LLEG_LINK4::ai;
  Force LLEG_LINK4::Fext;
  Inertia LLEG_LINK4::I = spatialInertiaMaker(LLEG_LINK4::mass, LLEG_LINK4::CoM, LLEG_LINK4::inertie);


  // Forward declaration of LLEG_ANKLE_P class
  class LLEG_ANKLE_P;

  // Declaration of LLEG_LINK5 class
  CREATE_BODY(LLEG_LINK5, 1, LLEG_LINK4, LLEG_ANKLE_P);

  // Initialization of LLEG_LINK5;
  const std::string LLEG_LINK5::name = "LLEG_LINK5";
  const int LLEG_LINK5::label = 22;
  const double LLEG_LINK5::mass = 2.5;
  const vector3d LLEG_LINK5::CoM = vector3d(0, -0.15, 0);
  const matrix3d LLEG_LINK5::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LLEG_LINK5::iX0;
  Motion LLEG_LINK5::vi;
  Motion LLEG_LINK5::ai;
  Force LLEG_LINK5::Fext;
  Inertia LLEG_LINK5::I = spatialInertiaMaker(LLEG_LINK5::mass, LLEG_LINK5::CoM, LLEG_LINK5::inertie);


  // Forward declaration of LLEG_ANKLE_R class
  class LLEG_ANKLE_R;

  // Declaration of LLEG_LINK6 class
  CREATE_BODY(LLEG_LINK6, 1, LLEG_LINK5, LLEG_ANKLE_R);

  // Initialization of LLEG_LINK6;
  const std::string LLEG_LINK6::name = "LLEG_LINK6";
  const int LLEG_LINK6::label = 23;
  const double LLEG_LINK6::mass = 1.9;
  const vector3d LLEG_LINK6::CoM = vector3d(0.28, 0, -0.2);
  const matrix3d LLEG_LINK6::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform LLEG_LINK6::iX0;
  Motion LLEG_LINK6::vi;
  Motion LLEG_LINK6::ai;
  Force LLEG_LINK6::Fext;
  Inertia LLEG_LINK6::I = spatialInertiaMaker(LLEG_LINK6::mass, LLEG_LINK6::CoM, LLEG_LINK6::inertie);


  // Forward declaration of RLEG_HIP_R class
  class RLEG_HIP_R;

  // Declaration of RLEG_LINK1 class
  CREATE_BODY(RLEG_LINK1, 1, WAIST_LINK0, RLEG_HIP_R);

  // Initialization of RLEG_LINK1;
  const std::string RLEG_LINK1::name = "RLEG_LINK1";
  const int RLEG_LINK1::label = 24;
  const double RLEG_LINK1::mass = 2.5;
  const vector3d RLEG_LINK1::CoM = vector3d(0, -0.1, 0);
  const matrix3d RLEG_LINK1::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RLEG_LINK1::iX0;
  Motion RLEG_LINK1::vi;
  Motion RLEG_LINK1::ai;
  Force RLEG_LINK1::Fext;
  Inertia RLEG_LINK1::I = spatialInertiaMaker(RLEG_LINK1::mass, RLEG_LINK1::CoM, RLEG_LINK1::inertie);


  // Forward declaration of RLEG_HIP_P class
  class RLEG_HIP_P;

  // Declaration of RLEG_LINK2 class
  CREATE_BODY(RLEG_LINK2, 1, RLEG_LINK1, RLEG_HIP_P);

  // Initialization of RLEG_LINK2;
  const std::string RLEG_LINK2::name = "RLEG_LINK2";
  const int RLEG_LINK2::label = 25;
  const double RLEG_LINK2::mass = 2;
  const vector3d RLEG_LINK2::CoM = vector3d(0, 0, 0.15);
  const matrix3d RLEG_LINK2::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RLEG_LINK2::iX0;
  Motion RLEG_LINK2::vi;
  Motion RLEG_LINK2::ai;
  Force RLEG_LINK2::Fext;
  Inertia RLEG_LINK2::I = spatialInertiaMaker(RLEG_LINK2::mass, RLEG_LINK2::CoM, RLEG_LINK2::inertie);


  // Forward declaration of RLEG_HIP_Y class
  class RLEG_HIP_Y;

  // Declaration of RLEG_LINK3 class
  CREATE_BODY(RLEG_LINK3, 1, RLEG_LINK2, RLEG_HIP_Y);

  // Initialization of RLEG_LINK3;
  const std::string RLEG_LINK3::name = "RLEG_LINK3";
  const int RLEG_LINK3::label = 26;
  const double RLEG_LINK3::mass = 5.1;
  const vector3d RLEG_LINK3::CoM = vector3d(0, 0, -0.04);
  const matrix3d RLEG_LINK3::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RLEG_LINK3::iX0;
  Motion RLEG_LINK3::vi;
  Motion RLEG_LINK3::ai;
  Force RLEG_LINK3::Fext;
  Inertia RLEG_LINK3::I = spatialInertiaMaker(RLEG_LINK3::mass, RLEG_LINK3::CoM, RLEG_LINK3::inertie);


  // Forward declaration of RLEG_KNEE class
  class RLEG_KNEE;

  // Declaration of RLEG_LINK4 class
  CREATE_BODY(RLEG_LINK4, 1, RLEG_LINK3, RLEG_KNEE);

  // Initialization of RLEG_LINK4;
  const std::string RLEG_LINK4::name = "RLEG_LINK4";
  const int RLEG_LINK4::label = 27;
  const double RLEG_LINK4::mass = 7;
  const vector3d RLEG_LINK4::CoM = vector3d(0, 0, 0.3);
  const matrix3d RLEG_LINK4::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RLEG_LINK4::iX0;
  Motion RLEG_LINK4::vi;
  Motion RLEG_LINK4::ai;
  Force RLEG_LINK4::Fext;
  Inertia RLEG_LINK4::I = spatialInertiaMaker(RLEG_LINK4::mass, RLEG_LINK4::CoM, RLEG_LINK4::inertie);


  // Forward declaration of RLEG_ANKLE_P class
  class RLEG_ANKLE_P;

  // Declaration of RLEG_LINK5 class
  CREATE_BODY(RLEG_LINK5, 1, RLEG_LINK4, RLEG_ANKLE_P);

  // Initialization of RLEG_LINK5;
  const std::string RLEG_LINK5::name = "RLEG_LINK5";
  const int RLEG_LINK5::label = 28;
  const double RLEG_LINK5::mass = 2.5;
  const vector3d RLEG_LINK5::CoM = vector3d(0, -0.15, 0);
  const matrix3d RLEG_LINK5::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RLEG_LINK5::iX0;
  Motion RLEG_LINK5::vi;
  Motion RLEG_LINK5::ai;
  Force RLEG_LINK5::Fext;
  Inertia RLEG_LINK5::I = spatialInertiaMaker(RLEG_LINK5::mass, RLEG_LINK5::CoM, RLEG_LINK5::inertie);


  // Forward declaration of RLEG_ANKLE_R class
  class RLEG_ANKLE_R;

  // Declaration of RLEG_LINK6 class
  CREATE_BODY(RLEG_LINK6, 1, RLEG_LINK5, RLEG_ANKLE_R);

  // Initialization of RLEG_LINK6;
  const std::string RLEG_LINK6::name = "RLEG_LINK6";
  const int RLEG_LINK6::label = 29;
  const double RLEG_LINK6::mass = 1.9;
  const vector3d RLEG_LINK6::CoM = vector3d(0.28, 0, -0.2);
  const matrix3d RLEG_LINK6::inertie = matrix3dMaker(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
  Transform RLEG_LINK6::iX0;
  Motion RLEG_LINK6::vi;
  Motion RLEG_LINK6::ai;
  Force RLEG_LINK6::Fext;
  Inertia RLEG_LINK6::I = spatialInertiaMaker(RLEG_LINK6::mass, RLEG_LINK6::CoM, RLEG_LINK6::inertie);

} // end of namespace simplehumanoid

#endif
