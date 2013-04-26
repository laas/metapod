// Disable vector operations and alignment assertions to ensure the test
// won't crash or abort.
// This does *not* disable Eigen's alignment constraints which we are testing
// here.
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "../common.hh"
#include <Eigen/Eigen>
#include <cmath>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/fusion/sequence.hpp>
#include <boost/fusion/include/sequence.hpp>
#include <boost/fusion/include/vector.hpp>

template <typename T>
inline bool is_aligned(T *addr) {
  return (reinterpret_cast<size_t>(addr) & 0x0f) == 0;
}

#define PRINT_ADDRESS(addr) std::cout << "" #addr " " << addr << std::endl;
#define CHECK_IS_ALIGNED(addr) BOOST_CHECK(is_aligned(addr))

#if 0
// this will check if a pointer we do not expect to be aligned is aligned.
// Hence we'll see the test fail, except if the pointer is aligned by chance,
// which is quite likely on 64 bits systems.
//
// Use this test in the following way:
//
//     test_eigenalign | grep -E "as expected .* is not aligned" | sort | uniq
//
// to verify your expectations
# define MAY_NOT_BE_ALIGNED(addr) \
    BOOST_CHECK_MESSAGE(is_aligned(addr), "as expected " #addr " is not aligned")
#else
# define MAY_NOT_BE_ALIGNED(addr)
#endif

class Foo1f {
  public:
    char c;
    int i;
    Eigen::Matrix<float, 6, 1> S;
    Foo1f() : c(0), i(0) {}
};

class FooA1f {
  public:
    // the following operator is not needed since S has no alignment
    // requirements.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    char c;
    int i;
    Eigen::Matrix<float, 6, 1> S;
    FooA1f() : c(0), i(0) {}
};

class Foo6f {
  public:
    char c;
    int i;
    Eigen::Matrix<float, 6, 6> S;
    Foo6f() : c(0), i(0) {}
};

class FooA6f {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    char c;
    int i;
    Eigen::Matrix<float, 6, 6> S;
    FooA6f() : c(0), i(0) {}
};

class FooVector {
  public:
    char c;
    int i;
    Foo1f m0;
    FooA1f m1;
    Foo6f m2;
    FooA6f m3;
    FooVector(): c(0), i(0) {}
};

class FooVectorA {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    char c;
    int i;
    Foo1f m0;
    FooA1f m1;
    Foo6f m2;
    FooA6f m3;
    FooVectorA(): c(0), i(0) {}
};

BOOST_AUTO_TEST_CASE(test_eigenalign) {

  for (size_t i=64; i>0; --i) {

    // add some noise to the heap
    boost::scoped_array<char> pc0(new char[i % 16]);

    Foo1f foo1f;
    boost::scoped_ptr<Foo1f> pfoo1f(new Foo1f);

    FooA1f fooA1f;
    boost::scoped_ptr<FooA1f> pfooA1f(new FooA1f);
    // If is not useful, but we asked the struct to be aligned
    CHECK_IS_ALIGNED(pfooA1f.get());
    // note that the S member has no reason to be aligned
    MAY_NOT_BE_ALIGNED(&pfooA1f->S);

    Foo6f foo6f;
    CHECK_IS_ALIGNED(&foo6f);
    CHECK_IS_ALIGNED(&foo6f.S);
    boost::scoped_ptr<Foo6f> pfoo6f(new Foo6f);
    // we did not define a proper new operator, so S may be misaligned.
    MAY_NOT_BE_ALIGNED(&pfoo6f->S);

    FooA6f fooA6f;
    CHECK_IS_ALIGNED(&fooA6f.S);
    boost::scoped_ptr<FooA6f> pfooA6f(new FooA6f);
    CHECK_IS_ALIGNED(&pfooA6f->S);

    FooVector fv;
    CHECK_IS_ALIGNED(&fv.m2.S);
    CHECK_IS_ALIGNED(&fv.m3.S);
    boost::scoped_ptr<FooVector> pfv(new FooVector);
    MAY_NOT_BE_ALIGNED(&pfv->m3.S);

    FooVectorA fvA;
    CHECK_IS_ALIGNED(&fvA.m2.S);
    CHECK_IS_ALIGNED(&fvA.m3.S);
    boost::scoped_ptr<FooVectorA> pfvA(new FooVectorA);
    CHECK_IS_ALIGNED(&pfvA->m3.S);

    typedef boost::fusion::vector4<
        Foo1f,
        FooA1f,
        Foo6f,
        FooA6f>
        FusionFooVector;
    FusionFooVector ffv;
    CHECK_IS_ALIGNED(&ffv.m2.S);
    CHECK_IS_ALIGNED(&ffv.m3.S);
    boost::scoped_ptr<FusionFooVector> pffv(new FusionFooVector);
    MAY_NOT_BE_ALIGNED(&pffv->m3.S);
  }
}
