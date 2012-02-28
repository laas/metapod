metapod
========

This software provides robot dynamics algorithms.
It uses a combination of a specific representation of robot models and C++ templates,
such that each algorithm remains model-independant, yet is optimized for a particular robot at compile-time.

As of today, the following algorithms have been implemented:
 - RNEA (Recursive Newton Euler Algorithm) for inverse dynamics

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The package depends on several packages which have to be available on
your machine.

 - Libraries:
   - Eigen (>=3.0.0)
   - Boost (>=1.48.0)
     Boost Test is used in the test suite
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)
