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

#include "../common.hh"

#include <metapod/tools/joint.hh>

using namespace metapod;
using namespace metapod::Spatial;

template <typename useless>
class A
{ useless tmp_; };

using namespace std;

template <typename FloatType>
void display(struct Matrix3dTpl<FloatType>::Type &A,
             struct Matrix3dTpl<FloatType>::Type &B,
             struct Matrix3dTpl<FloatType>::Type &C,
             struct Matrix3dTpl<FloatType>::Type &D,
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

template <typename FloatType>
void check(struct Matrix3dTpl<FloatType>::Type &C,
           struct Matrix3dTpl<FloatType>::Type &D)
{
  EIGEN_METAPOD_TYPEDEFS;
  Matrix3d diff_checkd_d3d = C - D;
  // Compute the norm of the difference.
  double norm_diff_checkd_d3d = diff_checkd_d3d.squaredNorm();
  if (norm_diff_checkd_d3d > 1e-10)
    { std::cout << "****************** ERROR ***************" << std::endl; }
  BOOST_CHECK(norm_diff_checkd_d3d < 1e-10);
}

template <typename FloatType>
void displayAndCheck(struct Matrix3dTpl<FloatType>::Type &A,
                     struct Matrix3dTpl<FloatType>::Type &B,
                     struct Matrix3dTpl<FloatType>::Type &C,
                     struct Matrix3dTpl<FloatType>::Type &D,
                     std::string &operation)
{
  display<FloatType>(A,B,C,D,operation);
  check<FloatType>(C,D);
}

template< typename FloatType,
          typename T >
void test_symmetric_matrix(class ltI<FloatType> &altI)
{
  EIGEN_METAPOD_TYPEDEFS;
  T aRMA;

  aRMA.randomInit();
  Matrix3d R = aRMA.toMatrix();

  class ltI<FloatType> d;
  d = aRMA.rotSymmetricMatrix(altI);

  Matrix3d I = altI.toMatrix();
  Matrix3d d3d = d.toMatrix();
  Matrix3d cD = static_cast<Matrix3d >(R * altI.toMatrix() * R.transpose());
  
  std::string opName("R I R^T");
  displayAndCheck<FloatType>(I,R,d3d,cD,opName);

}

template< typename FloatType,
          typename T >
void test_general_matrix(struct Matrix3dTpl<FloatType>::Type & aI)
{
  EIGEN_METAPOD_TYPEDEFS;
  T aRMA;

  aRMA.randomInit();

  Matrix3d R = aRMA.toMatrix();

  Matrix3d d;
  d = aRMA.rotGeneralMatrix(aI);

  Matrix3d cD = static_cast<Matrix3d>(R * aI * R.transpose());
  
  std::string opName("R^T I R");
  displayAndCheck<FloatType>(aI,R,d,cD,opName);

}

template< typename FloatType,
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
  displayAndCheck<FloatType>(aR,Rt,Rt,ld,opName);
}

template< typename FloatType,
          typename T >
void test_multiplication(struct Matrix3dTpl<FloatType>::Type & aI)
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
  displayAndCheck<FloatType>(ld,aI,R,d,opName);
}

template <typename FloatType,
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

    displayAndCheck<FloatType>(mX,mY,mZ,R,opName);

    // Test RotationMatrixAboutX * RotationMatrix
    rmY.randomInit();
    mY = rmY.toMatrix();
    rmZ = X*rmY;

    mZ = rmZ.toMatrix();
    R = mX * mY;
    opName = "RotationMatrix";
    displayAndCheck<FloatType>(mX,mY,mZ,R,opName);
  }
};

BOOST_AUTO_TEST_CASE(test_rotation)
{
  typedef double FloatType;
  
  Matrix3dTpl<FloatType>::Type I;
  I << 0.00285998, -0.00001434,-0.00055582,
      -0.00001434, 0.00352974, 0.00000884,
      -0.00055582, 0.00000885, 0.00145427;
  cout << I << endl;
  ltI<FloatType> altI(I);

  typedef rmca_traits<1,0,2,1,1,-1> PermuYXmZ;

  cout << " ************** TEST R^T*L*R ************** " << endl;
  cout << "ltI: Test X Rotation" << endl;
  test_symmetric_matrix<FloatType, RotationMatrixAboutXTpl<FloatType> >(altI);
  cout << "ltI: Test Y Rotation" << endl;
  test_symmetric_matrix<FloatType, RotationMatrixAboutYTpl<FloatType> >(altI);
  cout << "ltI: Test Z Rotation" << endl;
  test_symmetric_matrix<FloatType, RotationMatrixAboutZTpl<FloatType> >(altI);

  cout << "ltI: Test General Rotation Matrix" << endl;
  test_symmetric_matrix<FloatType, RotationMatrixTpl<FloatType> >(altI);

  cout << "ltI: Test Rotation Matrix Change Axis" << endl;
  test_symmetric_matrix<FloatType, RotationMatrixChangeAxisTpl<FloatType,PermuYXmZ> >(altI);

  cout << " ************** TEST R^T*A*R ************** " << endl;
  Matrix3dTpl<FloatType>::Type NotSymmetrical;
  NotSymmetrical << 0.1, 0.5, 0.4,
      0.1, 0.2, 0.3,
      0.5, 0.4, 0.1 ;
  cout << NotSymmetrical << endl;

  cout << "NotSymmetrical: Test X Rotation" << endl;
  test_general_matrix<FloatType, RotationMatrixAboutXTpl<FloatType> >(NotSymmetrical);
  cout << "NotSymmetrical: Test Y Rotation" << endl;
  test_general_matrix<FloatType, RotationMatrixAboutYTpl<FloatType> >(NotSymmetrical);
  cout << "NotSymmetrical: Test Z Rotation" << endl;
  test_general_matrix<FloatType, RotationMatrixAboutZTpl<FloatType> >(NotSymmetrical);

  cout << "NotSymmetrical: Test General Rotation Matrix" << endl;
  test_general_matrix<FloatType, RotationMatrixTpl<FloatType> >(NotSymmetrical);

  cout << "NotSymmetrical: Test Rotation Matrix Change Axis" << endl;
  test_symmetric_matrix<FloatType, RotationMatrixChangeAxisTpl<FloatType,PermuYXmZ> >(altI);

  cout << " ************** TEST TRANSPOSE ************** " << endl;
  test_transpose<FloatType, RotationMatrixAboutXTpl<FloatType> >();
  test_transpose<FloatType, RotationMatrixAboutYTpl<FloatType> >();
  test_transpose<FloatType, RotationMatrixAboutZTpl<FloatType> >();
  test_transpose<FloatType, RotationMatrixTpl<FloatType> >();
  test_transpose<FloatType, RotationMatrixChangeAxisTpl<FloatType, PermuYXmZ> >();

  cout << " ************** TEST MULTIPLICATION ************** " << endl;
  // Test the rotation with a rotation matrix
  RotationMatrixTpl<FloatType> aRM;
  aRM.randomInit();
  Matrix3dTpl<FloatType>::Type randomRM = aRM.toMatrix();

  typedef rmca_traits_mul<PermuYXmZ,PermuYXmZ> GbToId;
  
  test_multiplication<FloatType, RotationMatrixAboutXTpl<FloatType> >(randomRM);
  test_multiplication<FloatType, RotationMatrixAboutYTpl<FloatType> >(randomRM);
  test_multiplication<FloatType, RotationMatrixAboutZTpl<FloatType> >(randomRM);
  test_multiplication<FloatType, RotationMatrixTpl<FloatType> >(randomRM);
  test_multiplication<FloatType, RotationMatrixChangeAxisTpl<FloatType,PermuYXmZ> >(randomRM);

  test_mul_matrix_about<FloatType, RotationMatrixAboutXTpl<FloatType> >::run();
  test_mul_matrix_about<FloatType, RotationMatrixAboutYTpl<FloatType> >::run();
  test_mul_matrix_about<FloatType, RotationMatrixAboutZTpl<FloatType> >::run();
  test_mul_matrix_about<FloatType, RotationMatrixChangeAxisTpl<FloatType, PermuYXmZ>,
    RotationMatrixChangeAxisTpl<FloatType, GbToId> >::run();
  
}
