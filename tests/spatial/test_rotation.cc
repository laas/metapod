// Copyright 2012, 2013
//
// Olivier STASSE (LAAS/CNRS)
// Sébastien Barthélémy (Aldebaran Robotics)
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
// GNU General Lesser Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod. If not, see <http://www.gnu.org/licenses/>.
#include <iostream>

#include <tests/common.hh>

DEFAULT_FLOAT_TYPE;
#define FloatType LocalFloatType

#include <metapod/tools/joint.hh>

using namespace metapod;
using namespace metapod::Spatial;

template <typename useless>
class A
{ useless tmp_; };

using namespace std;

template <typename LocalFloatType>
void display(struct Matrix3dTpl<LocalFloatType>::Type &A,
             struct Matrix3dTpl<LocalFloatType>::Type &B,
             struct Matrix3dTpl<LocalFloatType>::Type &C,
             struct Matrix3dTpl<LocalFloatType>::Type &D,
             std::string &operation)
{
  cout << "Operands:" << endl;
  cout << A << endl;
  cout << operation << endl;
  cout << B << endl;
  cout << "=" << endl;
  cout << C << endl;
  cout << "Check:" << endl;
  cout << D << endl;
}

template <typename LocalFloatType>
void check(struct Matrix3dTpl<LocalFloatType>::Type &C,
           struct Matrix3dTpl<LocalFloatType>::Type &D)
{
  EIGEN_METAPOD_TYPEDEFS;
  Matrix3d diff_checkd_d3d = C - D;
  // Compute the norm of the difference.
  double norm_diff_checkd_d3d = diff_checkd_d3d.squaredNorm();
  if (norm_diff_checkd_d3d > 1e-10)
    { std::cout << "****************** ERROR ***************" << std::endl; }
  BOOST_CHECK(norm_diff_checkd_d3d < 1e-10);
}

template <typename LocalFloatType>
void displayAndCheck(struct Matrix3dTpl<LocalFloatType>::Type &A,
                     struct Matrix3dTpl<LocalFloatType>::Type &B,
                     struct Matrix3dTpl<LocalFloatType>::Type &C,
                     struct Matrix3dTpl<LocalFloatType>::Type &D,
                     std::string &operation)
{
  display<LocalFloatType>(A,B,C,D,operation);
  check<LocalFloatType>(C,D);
}

template< typename LocalFloatType,
          typename T >
void test_symmetric_matrix(class ltI<LocalFloatType> &altI)
{
  EIGEN_METAPOD_TYPEDEFS;
  T aRMA;

  aRMA.randomInit();
  Matrix3d R = aRMA.toMatrix();

  class ltI<LocalFloatType> d;
  d = aRMA.rotSymmetricMatrix(altI);

  Matrix3d I = altI.toMatrix();
  Matrix3d d3d = d.toMatrix();
  Matrix3d cD = static_cast<Matrix3d >(R * altI.toMatrix() * R.transpose());
  
  std::string opName("R I R^T");
  displayAndCheck<LocalFloatType>(I,R,d3d,cD,opName);

}

template< typename LocalFloatType,
          typename T >
void test_general_matrix(struct Matrix3dTpl<LocalFloatType>::Type & aI)
{
  EIGEN_METAPOD_TYPEDEFS;
  T aRMA;

  aRMA.randomInit();

  Matrix3d R = aRMA.toMatrix();

  Matrix3d d;
  d = aRMA.rotGeneralMatrix(aI);

  Matrix3d cD = static_cast<Matrix3d>(R * aI * R.transpose());
  
  std::string opName("R^T I R");
  displayAndCheck<LocalFloatType>(aI,R,d,cD,opName);

}

template< typename LocalFloatType,
          typename T>
void test_transpose()
{
  EIGEN_METAPOD_TYPEDEFS;
  T aRMA;

  aRMA.randomInit();
  Matrix3d aR = aRMA.toMatrix();
  Matrix3d Rt = aRMA.toMatrix().transpose();
  T d;
  d = aRMA.transpose();
  std::string opName("Transpose");
  Matrix3d ld=d.toMatrix();
  displayAndCheck<LocalFloatType>(aR,Rt,Rt,ld,opName);
}

template< typename LocalFloatType,
          typename T >
void test_multiplication(struct Matrix3dTpl<LocalFloatType>::Type & aI)
{
  EIGEN_METAPOD_TYPEDEFS;
  T aRMA;

  aRMA.randomInit();

  Matrix3d R = static_cast<Matrix3d>(aRMA.toMatrix() * aI);

  Matrix3d d;
  d = aRMA * aI;
  std::string opName("multiplication ");
  opName += typeid(T).name();
  Matrix3d ld = aRMA.toMatrix();
  displayAndCheck<LocalFloatType>(ld,aI,R,d,opName);
}

template <typename LocalFloatType,
          typename T,
          typename TZ=T >
struct test_mul_matrix_about
{
  METAPOD_TYPEDEFS;
  static void run()
  {
    T X,Y;
    TZ Z;
    RotationMatrix rmX,rmY,rmZ;
    Matrix3d mX, mY,mZ, R;

    X.randomInit();
    Y.randomInit();

    // Test RotationMatrixAboutX * RotationMatrixAboutX
    Z = X*Y;

    mX = X.toMatrix();
    mY = Y.toMatrix();
    mZ = Z.toMatrix();
    R = mX * mY;

    std::string opName;
    opName = typeid(T).name();

    displayAndCheck<LocalFloatType>(mX,mY,mZ,R,opName);

    // Test RotationMatrixAboutX * RotationMatrix
    rmY.randomInit();
    mY = rmY.toMatrix();
    rmZ = X*rmY;

    mZ = rmZ.toMatrix();
    R = mX * mY;
    opName = "RotationMatrix";
    displayAndCheck<LocalFloatType>(mX,mY,mZ,R,opName);
  }
};

BOOST_AUTO_TEST_CASE(test_rotation)
{
  
  Matrix3dTpl<LocalFloatType>::Type I;
  I << 0.00285998, -0.00001434,-0.00055582,
      -0.00001434, 0.00352974, 0.00000884,
      -0.00055582, 0.00000885, 0.00145427;
  cout << I << endl;
  ltI<LocalFloatType> altI(I);

  typedef rmca_traits<1,0,2,1,1,-1> PermuYXmZ;

  cout << " ************** TEST R^T*L*R ************** " << endl;
  cout << "ltI: Test X Rotation" << endl;
  test_symmetric_matrix<LocalFloatType, RotationMatrixAboutXTpl<LocalFloatType> >(altI);
  cout << "ltI: Test Y Rotation" << endl;
  test_symmetric_matrix<LocalFloatType, RotationMatrixAboutYTpl<LocalFloatType> >(altI);
  cout << "ltI: Test Z Rotation" << endl;
  test_symmetric_matrix<LocalFloatType, RotationMatrixAboutZTpl<LocalFloatType> >(altI);

  cout << "ltI: Test General Rotation Matrix" << endl;
  test_symmetric_matrix<LocalFloatType, RotationMatrixTpl<LocalFloatType> >(altI);

  cout << "ltI: Test Rotation Matrix Change Axis" << endl;
  test_symmetric_matrix<LocalFloatType, RotationMatrixChangeAxisTpl<LocalFloatType,PermuYXmZ> >(altI);

  cout << " ************** TEST R^T*A*R ************** " << endl;
  Matrix3dTpl<LocalFloatType>::Type NotSymmetrical;
  NotSymmetrical << 0.1, 0.5, 0.4,
      0.1, 0.2, 0.3,
      0.5, 0.4, 0.1 ;
  cout << NotSymmetrical << endl;

  cout << "NotSymmetrical: Test X Rotation" << endl;
  test_general_matrix<LocalFloatType, RotationMatrixAboutXTpl<LocalFloatType> >(NotSymmetrical);
  cout << "NotSymmetrical: Test Y Rotation" << endl;
  test_general_matrix<LocalFloatType, RotationMatrixAboutYTpl<LocalFloatType> >(NotSymmetrical);
  cout << "NotSymmetrical: Test Z Rotation" << endl;
  test_general_matrix<LocalFloatType, RotationMatrixAboutZTpl<LocalFloatType> >(NotSymmetrical);

  cout << "NotSymmetrical: Test General Rotation Matrix" << endl;
  test_general_matrix<LocalFloatType, RotationMatrixTpl<LocalFloatType> >(NotSymmetrical);

  cout << "NotSymmetrical: Test Rotation Matrix Change Axis" << endl;
  test_symmetric_matrix<LocalFloatType, RotationMatrixChangeAxisTpl<LocalFloatType,PermuYXmZ> >(altI);

  cout << " ************** TEST TRANSPOSE ************** " << endl;
  test_transpose<LocalFloatType, RotationMatrixAboutXTpl<LocalFloatType> >();
  test_transpose<LocalFloatType, RotationMatrixAboutYTpl<LocalFloatType> >();
  test_transpose<LocalFloatType, RotationMatrixAboutZTpl<LocalFloatType> >();
  test_transpose<LocalFloatType, RotationMatrixTpl<LocalFloatType> >();
  test_transpose<LocalFloatType, RotationMatrixChangeAxisTpl<LocalFloatType, PermuYXmZ> >();

  cout << " ************** TEST MULTIPLICATION ************** " << endl;
  // Test the rotation with a rotation matrix
  RotationMatrixTpl<LocalFloatType> aRM;
  aRM.randomInit();
  Matrix3dTpl<LocalFloatType>::Type randomRM = aRM.toMatrix();

  typedef rmca_traits_mul<PermuYXmZ,PermuYXmZ> GbToId;
  
  test_multiplication<LocalFloatType, RotationMatrixAboutXTpl<LocalFloatType> >(randomRM);
  test_multiplication<LocalFloatType, RotationMatrixAboutYTpl<LocalFloatType> >(randomRM);
  test_multiplication<LocalFloatType, RotationMatrixAboutZTpl<LocalFloatType> >(randomRM);
  test_multiplication<LocalFloatType, RotationMatrixTpl<LocalFloatType> >(randomRM);
  test_multiplication<LocalFloatType, RotationMatrixChangeAxisTpl<LocalFloatType,PermuYXmZ> >(randomRM);

  test_mul_matrix_about<LocalFloatType, RotationMatrixAboutXTpl<LocalFloatType> >::run();
  test_mul_matrix_about<LocalFloatType, RotationMatrixAboutYTpl<LocalFloatType> >::run();
  test_mul_matrix_about<LocalFloatType, RotationMatrixAboutZTpl<LocalFloatType> >::run();
  test_mul_matrix_about<LocalFloatType, RotationMatrixChangeAxisTpl<LocalFloatType, PermuYXmZ>,
    RotationMatrixChangeAxisTpl<LocalFloatType, GbToId> >::run();
  
}
