metapod
========

[![Build Status](https://travis-ci.org/laas/metapod.png?branch=master)](https://travis-ci.org/laas/metapod)

This software provides robot dynamics algorithms.
It uses a combination of a specific representation of robot models and C++
templates, such that each algorithm remains model-independant, yet is optimized
for a particular robot at compile-time.
It makes use of R. Featherstone's Spatial Algebra to describe forces, motions
and inertias (cf. Rigid Body Dynamics Algorithms, Roy Featherstone).

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
    cmake -DBUILD_METAPODFROMURDF=OFF
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


In order to build the urdf converter, you'll need to install liburdfdom or
liburdf. There are several options:

 - Install urdfdom alone

       git clone git://github.com/ros/console_bridge.git && cd console_bridge
       git checkout 0.1.5
       mkdir build && cd build
       cmake ..
       make
       sudo make install

       hg clone https://bitbucket.org/osrf/urdfdom_headers && cd urdfdom_headers
       hg checkout 0.2.2
       mkdir build && cd build
       cmake ..
       make
       sudo make install

       hg clone https://bitbucket.org/osrf/urdfdom && cd urdfdom
       hg checkout 0.2.7
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
