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
 * 3 dof sample model, used for benchmarking.
 */

# include "sample_3_dof.hh"

template struct metapod::crba< metapod::sample_3_dof::Robot , true >;
template struct metapod::rnea< metapod::sample_3_dof::Robot , true >;
template struct metapod::crba< metapod::sample_3_dof::Robot , false >;
template struct metapod::rnea< metapod::sample_3_dof::Robot , false >;

namespace metapod
{
  namespace sample_3_dof
  {
    Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;

    INITIALIZE_JOINT_REVOLUTE(J0);
    const std::string J0::name = "J0";
    const int J0::label = 0;
    const int J0::positionInConf = 0;
    const Transform J0::Xt = Transform(
      matrix3dMaker(
        0.59688, 0.823295, -0.604897,
        -0.329554, 0.536459, -0.444451,
        0.10794, -0.0452059, 0.257742),
      vector3d(
        0.680375, -0.211234, 0.566198));
  
    INITIALIZE_JOINT_REVOLUTE(J1);
    const std::string J1::name = "J1";
    const int J1::label = 1;
    const int J1::positionInConf = 1;
    const Transform J1::Xt = Transform(
      matrix3dMaker(
        0.542715, 0.05349, 0.539828,
        -0.199543, 0.783059, -0.433371,
        -0.295083, 0.615449, 0.838053),
      vector3d(
        -0.012834, 0.94555, -0.414966));
  
    INITIALIZE_JOINT_REVOLUTE(J2);
    const std::string J2::name = "J2";
    const int J2::label = 2;
    const int J2::positionInConf = 2;
    const Transform J2::Xt = Transform(
      matrix3dMaker(
        0.70184, -0.466669, 0.0795207,
        -0.249586, 0.520497, 0.0250707,
        0.335448, 0.0632129, -0.921439),
      vector3d(
        -0.52344, 0.941268, 0.804416));

    INITIALIZE_BODY(B0);
    const std::string B0::name = "B0";
    const int B0::label = 0;
    const FloatType B0::mass = 1;
    const vector3d B0::CoM = vector3d(  -0.514226,   -0.725537,   0.608354);
    const matrix3d B0::inertie = matrix3dMaker(
    -0.270431, 0.0268018, 0.0268018,
    0.83239, 0.271423, 0.271423,
    -0.716795, 0.213938, 0.213938);
    Inertia B0::I = spatialInertiaMaker(  B0::mass,
    B0::CoM,
    B0::inertie);
  
    INITIALIZE_BODY(B1);
    const std::string B1::name = "B1";
    const int B1::label = 1;
    const FloatType B1::mass = 1;
    const vector3d B1::CoM = vector3d(  -0.407937,   0.275105,   0.0485744);
    const matrix3d B1::inertie = matrix3dMaker(
    -0.686642, -0.198111, -0.198111,
    -0.782382, 0.997849, 0.997849,
    0.0258648, 0.678224, 0.678224);
    Inertia B1::I = spatialInertiaMaker(  B1::mass,
    B1::CoM,
    B1::inertie);
  
    INITIALIZE_BODY(B2);
    const std::string B2::name = "B2";
    const int B2::label = 2;
    const FloatType B2::mass = 1;
    const vector3d B2::CoM = vector3d(  -0.959954,   -0.0845965,   -0.873808);
    const matrix3d B2::inertie = matrix3dMaker(
    -0.860489, 0.898654, 0.898654,
    -0.827888, -0.615572, -0.615572,
    0.780465, -0.302214, -0.302214);
    Inertia B2::I = spatialInertiaMaker(  B2::mass,
    B2::CoM,
    B2::inertie);
  } // end of namespace sample_3_dof
} // end of namespace metapod
