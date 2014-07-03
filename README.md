metapod
========

[![Build Status](https://travis-ci.org/laas/metapod.png?branch=master)](https://travis-ci.org/laas/metapod)
[![Coverage Status](https://coveralls.io/repos/laas/metapod/badge.png?branch=master)](https://coveralls.io/r/laas/metapod?branch=master)

This software provides robot dynamics algorithms.
It uses a combination of a specific representation of robot models and C++
templates, such that each algorithm remains model-independant, yet is optimized
for a particular robot at compile-time.
It makes use of R. Featherstone's Spatial Algebra to describe forces, motions
and inertias (cf. Rigid Body Dynamics Algorithms, Roy Featherstone).

Content
-------

  * include/metapod/spatial: header-only spatial algebra library.

  * include/metapod: header-only library of robot dynamics algorithms.
    Algorithms consist of the combination of *compile-time* traversal
    algorithms and visitors. They can be applied to a robot model which is
    a class with a specific structure. The number representation
    (double by default) is also a template parameter.

  * robotbuilder: a library to help generating the source code of robot
    model classes.

  * embedfile: an utility executable used to embed templates in the
    robotbuilder library.

  * metapodfromurdf: an utility executable based on robotbuilder and liburdf
    which generates the the source code of robot model classes from an URDF
    description.

  * binarytreemodel: an utility executable based on robotbuilder which
    generates the source code of robot model classes with a binary kinematic
    tree. Those models are named sample_[1-4]and are used for benckmarks.

  * data: URDF definition of the simple_arm and simple_humanoid models which
    are used for examples, tests and benchmarks.

  * tests: some tests for metapod and metapod::Spatial

  * timer: a portable timer library used for the benchmarks.

  * benchmark: some benchmarks

  * doc: some doc.

Dependencies
------------

The package depends on several packages which have to be available on
your machine.

 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)
 - Libraries:
   - Eigen (>=3.0.0)
   - Boost (>=1.40.0)
     Boost Test is used in the test suite
   - optionally, liburdfdom or liburdf (as provided by ROS)

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_METAPODFROMURDF=OFF -DMETAPOD_DEFAULT_FLOAT_TYPE=double ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.

Remark: Release build type is necessary to obtain full performances.

In order to build the urdf converter, you'll need to install liburdfdom or
liburdf. There are several options:

 - Install urdfdom alone

       git clone git://github.com/ros/console_bridge.git && cd console_bridge
       git checkout 0.1.5
       mkdir build && cd build
       cmake ..
       make
       sudo make install

       git clone git://github.com/ros/urdfdom_headers && cd urdfdom_headers
       git checkout 0.2.2
       mkdir build && cd build
       cmake ..
       make
       sudo make install

       git clone git://github.com/ros/urdfdom && cd urdfdom
       git checkout 0.2.7
       mkdir build && cd build
       cmake ..
       make
       sudo make install

 - Or install ROS groovy using the ubuntu packages and just do

       source /opt/ros/groovy/setup.bash

   before running cmake.

 - Or install ROS fuerte using the ubuntu packages and just do

       source /opt/ros/fuerte/setup.bash

   before running cmake.

Documentation
-------------

Development branch documentation is [available
online](http://laas.github.com/metapod/doxygen/HEAD/).

Known Bugs
----------

 * Benchmark times different with different combination of outer and inner loop.

 Martin Felis pointed out that an outer loop of one, with an inner
 loop of 100000 was not equivalent with the benchmarks, to
 an outer loop of 100 and an inner loop of 1000.

 He also found out that it is related to a bug with timers in boost 1.40.
 Please use boot 1.48 to fix it.
 

 * NP is mandatory
 
 Benjamin Chretien pointed out that a dummy root link is necessary as
 a Galilean Frame for the urdf file parser of metapod.