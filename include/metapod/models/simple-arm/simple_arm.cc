// Copyright 2012,
//
// Sébastien Barthélémy
//
// Aldebaran Robotics
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
 * This file is part of a simple arm robot model, used for test purpose.
 * It contains the initialization of all the robot bodies and joints.
 *
 * Here is a simple sketch of the robot, with the lengths in meter
 *
 *         0.45      0.4     0.15
 *    [ ]--------[ ]------[ ]---
 *     |
 * 1.0 |
 *    _|_
 *
 */

# include "simple_arm.hh"

template struct metapod::crba< metapod::simple_arm::Robot , true >;
template struct metapod::rnea< metapod::simple_arm::Robot , true >;
template struct metapod::crba< metapod::simple_arm::Robot , false >;
template struct metapod::rnea< metapod::simple_arm::Robot , false >;

namespace metapod
{
  namespace simple_arm
  {
    // Initialization of the robot global constants
    Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;

    INITIALIZE_JOINT_REVOLUTE_AXIS_ANY(SHOULDER, 1, 0, 0);
    const std::string SHOULDER::name = "SHOULDER";
    const int SHOULDER::label = 1;
    const int SHOULDER::positionInConf = 0;
    const Spatial::Transform SHOULDER::Xt = Spatial::Transform(
      matrix3dMaker(0, 0, 1,
                    0, 1, 0,
                   -1, 0, 0),
      vector3d(0, 0, 1));

    INITIALIZE_JOINT_REVOLUTE_AXIS_ANY(ELBOW, 1, 0, 0);
    const std::string ELBOW::name = "ELBOW";
    const int ELBOW::label = 2;
    const int ELBOW::positionInConf = 1;
    const Spatial::Transform ELBOW::Xt = Spatial::Transform(
      matrix3dMaker(1, 0, 0,
                    0, 1, 0,
                    0, 0, 1),
      vector3d(0, 0, -0.45));

    INITIALIZE_JOINT_REVOLUTE_AXIS_ANY(WRIST, 1, 0, 0);
    const std::string WRIST::name = "WRIST";
    const int WRIST::label = 3;
    const int WRIST::positionInConf = 2;
    const Spatial::Transform WRIST::Xt = Spatial::Transform(
      matrix3dMaker(1, 0, 0,
                    0, 1, 0,
                    0, 0, 1),
      vector3d(0, 0, -0.4));

    INITIALIZE_BODY(ARM);
    const std::string ARM::name = "ARM";
    const int ARM::label = 0;
    const FloatType ARM::mass = 2.75;
    const vector3d ARM::CoM = vector3d(0, 0, -0.225);
    const matrix3d ARM::inertie = matrix3dMaker(
      4.68703125e-02, 0.0,            0.0,
      0.0,            4.68703125e-02, 0.0,
      0.0,            0.0,            9.28125000e-04);
    Spatial::Inertia ARM::I = spatialInertiaMaker(ARM::mass,
                                                  ARM::CoM,
                                                  ARM::inertie);

    INITIALIZE_BODY(FOREARM);
    const std::string FOREARM::name = "FOREARM";
    const int FOREARM::label = 1;
    const FloatType FOREARM::mass = 1.75;
    const vector3d FOREARM::CoM = vector3d(0, 0, -0.2);
    const matrix3d FOREARM::inertie = matrix3dMaker(
      2.35666667e-02, 0.0,            0.0,
      0.0,            2.35666667e-02, 0.0,
      0.0,            0.0,            4.66666667e-04);
    Spatial::Inertia FOREARM::I = spatialInertiaMaker(FOREARM::mass,
                                                      FOREARM::CoM,
                                                      FOREARM::inertie);

    INITIALIZE_BODY(HAND);
    const std::string HAND::name = "HAND";
    const int HAND::label = 2;
    const FloatType HAND::mass = 0.5;
    const vector3d HAND::CoM = vector3d(0, 0, -0.075);
    const matrix3d HAND::inertie = matrix3dMaker(
      9.46875000e-04, 0.0,            0.0,
      0.0,            9.46875000e-04, 0.0,
      0.0,            0.0,            1.87500000e-05);
    Spatial::Inertia HAND::I = spatialInertiaMaker(HAND::mass,
                                                   HAND::CoM,
                                                   HAND::inertie);
  } // end of namespace simple_arm
} // end of namespace metapod
