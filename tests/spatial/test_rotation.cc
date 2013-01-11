#include <iostream>

#include "../common.hh"

#include <include/metapod/tools/fwd.hh>
#include <include/metapod/tools/spatial/rotation-matrix.hh>

using namespace metapod;
using namespace metapod::Spatial;


using namespace std;

void display(Matrix3d &A,
             Matrix3d &B,
             Matrix3d &C,
             Matrix3d &D,
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

void check(Matrix3d &C,
           Matrix3d &D)
{
  Matrix3d diff_checkd_d3d = C - D;
  // Compute the norm of the difference.
  double norm_diff_checkd_d3d = diff_checkd_d3d.squaredNorm();

  BOOST_CHECK(norm_diff_checkd_d3d < 1e-10);
}

void displayAndCheck(Matrix3d &A,
                     Matrix3d &B,
                     Matrix3d &C,
                     Matrix3d &D,
                     std::string &operation)
{
  display(A,B,C,D,operation);
  check(C,D);
}

template< class T>
void test_symmetric_matrix(struct ltI &altI)
{
  T aRMA;

  aRMA.randomInit();
  Matrix3d R = aRMA.toMatrix();

  struct ltI d;
  d = aRMA.rotSymmetricMatrix(altI);

  Matrix3d I = altI.toMatrix();
  Matrix3d d3d = d.toMatrix();
  Matrix3d cD = static_cast<Matrix3d>(R * altI.toMatrix() * R.transpose());
  
  std::string opName("R I R^T");
  displayAndCheck(I,R,d3d,cD,opName);

}

template< class T>
void test_general_matrix(Matrix3d & aI)
{
  T aRMA;

  aRMA.randomInit();

  Matrix3d R = aRMA.toMatrix();

  Matrix3d d;
  d = aRMA.rotGeneralMatrix(aI);

  Matrix3d cD = static_cast<Matrix3d>(R * aI * R.transpose());
  
  std::string opName("R^T I R");
  displayAndCheck(aI,R,d,cD,opName);

}

template<class T>
void test_transpose()
{
  T aRMA;

  aRMA.randomInit();
  Matrix3d aR = aRMA.toMatrix();
  Matrix3d Rt = aRMA.toMatrix().transpose();
  T d;
  d = aRMA.transpose();
  std::string opName("Transpose");
  Matrix3d ld=d.toMatrix();
  displayAndCheck(aR,Rt,Rt,ld,opName);
}

template< class T>
void test_multiplication(Matrix3d & aI)
{
  T aRMA;

  aRMA.randomInit();

  Matrix3d R = static_cast<Matrix3d>(aRMA.toMatrix() * aI);

  Matrix3d d;
  d = aRMA * aI;
  std::string opName("multiplication ");
  opName += typeid(T).name();
  Matrix3d ld = aRMA.toMatrix();
  displayAndCheck(ld,aI,R,d,opName);
}

template <class T,
          class TZ=T>
struct test_mul_matrix_about
{
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

    displayAndCheck(mX,mY,mZ,R,opName);

    // Test RotationMatrixAboutX * RotationMatrix
    rmY.randomInit();
    mY = rmY.toMatrix();
    rmZ = X*rmY;

    mZ = rmZ.toMatrix();
    R = mX * mY;
    opName = "RotationMatrix";
    displayAndCheck(mX,mY,mZ,R,opName);
  }
};

BOOST_AUTO_TEST_CASE(test_rotation)
{
  Matrix3d I;
  I << 0.00285998, -0.00001434,-0.00055582,
      -0.00001434, 0.00352974, 0.00000884,
      -0.00055582, 0.00000885, 0.00145427;
  cout << I << endl;
  struct ltI altI(I);

  typedef rmca_traits<1,0,2,1,1,-1> PermuYXmZ;

  cout << " ************** TEST R^T*L*R ************** " << endl;
  cout << "ltI: Test X Rotation" << endl;
  test_symmetric_matrix<RotationMatrixAboutX>(altI);
  cout << "ltI: Test Y Rotation" << endl;
  test_symmetric_matrix<RotationMatrixAboutY>(altI);
  cout << "ltI: Test Z Rotation" << endl;
  test_symmetric_matrix<RotationMatrixAboutZ>(altI);

  cout << "ltI: Test General Rotation Matrix" << endl;
  test_symmetric_matrix<RotationMatrix>(altI);

  cout << "ltI: Test Rotation Matrix Change Axis" << endl;
  test_symmetric_matrix<RotationMatrixChangeAxis<PermuYXmZ> >(altI);

  cout << " ************** TEST R^T*A*R ************** " << endl;
  Matrix3d NotSymmetrical;
  NotSymmetrical << 0.1, 0.5, 0.4,
      0.1, 0.2, 0.3,
      0.5, 0.4, 0.1 ;
  cout << NotSymmetrical << endl;

  cout << "NotSymmetrical: Test X Rotation" << endl;
  test_general_matrix<RotationMatrixAboutX>(NotSymmetrical);
  cout << "NotSymmetrical: Test Y Rotation" << endl;
  test_general_matrix<RotationMatrixAboutY>(NotSymmetrical);
  cout << "NotSymmetrical: Test Z Rotation" << endl;
  test_general_matrix<RotationMatrixAboutZ>(NotSymmetrical);

  cout << "NotSymmetrical: Test General Rotation Matrix" << endl;
  test_general_matrix<RotationMatrix>(NotSymmetrical);

  cout << "NotSymmetrical: Test Rotation Matrix Change Axis" << endl;
  test_symmetric_matrix<RotationMatrixChangeAxis<PermuYXmZ> >(altI);

  cout << " ************** TEST TRANSPOSE ************** " << endl;
  test_transpose<RotationMatrixAboutX>();
  test_transpose<RotationMatrixAboutY>();
  test_transpose<RotationMatrixAboutZ>();
  test_transpose<RotationMatrix>();
  test_transpose<RotationMatrixChangeAxis<PermuYXmZ> >();

  cout << " ************** TEST MULTIPLICATION ************** " << endl;
  // Test the rotation with a rotation matrix
  RotationMatrix aRM;
  aRM.randomInit();
  Matrix3d randomRM = aRM.toMatrix();

  typedef rmca_traits_mul<PermuYXmZ,PermuYXmZ> GbToId;
  
  test_multiplication<RotationMatrixAboutX>(randomRM);
  test_multiplication<RotationMatrixAboutY>(randomRM);
  test_multiplication<RotationMatrixAboutZ>(randomRM);
  test_multiplication<RotationMatrix>(randomRM);
  test_multiplication<RotationMatrixChangeAxis<PermuYXmZ> >(randomRM);

  test_mul_matrix_about<RotationMatrixAboutX>::run();
  test_mul_matrix_about<RotationMatrixAboutY>::run();
  test_mul_matrix_about<RotationMatrixAboutZ>::run();
  test_mul_matrix_about<RotationMatrixChangeAxis<PermuYXmZ>,
      RotationMatrixChangeAxis<GbToId> >::run();
  
}
