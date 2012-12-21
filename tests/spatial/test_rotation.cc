#include <iostream>

#include "../common.hh"

#include <include/metapod/tools/fwd.hh>
#include <include/metapod/tools/spatial/rotationMatrix.hh>

using namespace metapod;
using namespace metapod::Spatial;


using namespace std;

void check(Matrix3d &aI,
	   Matrix3d &R,
	   Matrix3d d3d)
{
  Matrix3d checkd = R * aI * R.transpose();
  Matrix3d diff_checkd_d3d = checkd - d3d;


  // Compute the norm of the difference.
  double norm_diff_checkd_d3d = diff_checkd_d3d.squaredNorm();
  cout << "Result:" << endl;
  cout << d3d << endl;
  cout << "Verification: " << endl;
  cout <<  checkd << endl;
  cout << "Difference: " << norm_diff_checkd_d3d << endl;

  BOOST_CHECK(norm_diff_checkd_d3d < 1e-10);
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

  check(I,R,d3d);

}

template< class T>
void test_general_matrix(Matrix3d & aI)
{
  T aRMA;

  aRMA.randomInit();
    
  Matrix3d R = aRMA.toMatrix();
  cout << R << endl;

  Matrix3d d;
  d = aRMA.rotGeneralMatrix(aI);

  check(aI,R,d);
}

BOOST_AUTO_TEST_CASE(test_rotation)
{
  Matrix3d I;
  I << 0.00285998, -0.00001434,-0.00055582,
    -0.00001434, 0.00352974, 0.00000884,
    -0.00055582, 0.00000885, 0.00145427;
  cout << I << endl;
  struct ltI altI(I);

  cout << "ltI: Test X Rotation" << endl;
  test_symmetric_matrix<rotationMatrixAboutXAxis>(altI);
  cout << "ltI: Test Y Rotation" << endl;
  test_symmetric_matrix<rotationMatrixAboutYAxis>(altI);
  cout << "ltI: Test Z Rotation" << endl;
  test_symmetric_matrix<rotationMatrixAboutZAxis>(altI);

  cout << "ltI: Test General Rotation Matrix" << endl;
  test_symmetric_matrix<rotationMatrix>(altI);

  Matrix3d NotSymmetrical;
  NotSymmetrical << 0.1, 0.5, 0.4,
    0.1, 0.2, 0.3,
    0.5, 0.4, 0.1 ;
  cout << NotSymmetrical << endl;

  cout << "NotSymmetrical: Test X Rotation" << endl;
  test_general_matrix<rotationMatrixAboutXAxis>(NotSymmetrical);
  cout << "NotSymmetrical: Test Y Rotation" << endl;
  test_general_matrix<rotationMatrixAboutYAxis>(NotSymmetrical);
  cout << "NotSymmetrical: Test Z Rotation" << endl;
  test_general_matrix<rotationMatrixAboutZAxis>(NotSymmetrical);

  cout << "NotSymmetrical: Test General Rotation Matrix" << endl;
  test_general_matrix<rotationMatrix>(NotSymmetrical);

  
}
