/* Copyright 2011, 2012,
*
* Olivier STASSE, LAAS/CNRS
*
* This file is part of metapod.
* metapod is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* metapod is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Lesser Public License for more details.
* You should have received a copy of the GNU Lesser General Public License
* along with metapod.  If not, see <http://www.gnu.org/licenses/>.
*
* Creation: 02/03/2013
*/

/**
\page installation Installation

\section Dependencies
This package depends upon :
<ul>
   <li> System tools:
     <ul>
       <li> CMake (>=2.6) </li>
       <li> pkg-config </li>
       <li> usual compilation tools </li>
     </ul>
   </li>
   <li> Libraries:
     <ul>
       <li> Eigen (>=3.0.0) </li>
       <li> Boost (1.40.0 and 1.46.0) Boost Test is used in the test suite </li>
       <li> Optionally liburdf, as provided by ROS </li>
     </ul>
   </li>
</ul>

If you want to build the urdf converter, you'll need to install ROS too, and to run

\verbatim
source /opt/ros/fuerte/setup.bash
\endverbatim

before running cmake.

There is no currently no binary package.

\section getting_the_code Getting the code
To compile the code you can clone it from github:
\verbatim
git clone https://github.com/laas/metapod.git --submodule
\endverbatim

\section building Building
To compile this package, it is recommended to create a separate build directory:
\verbatim
mkdir _build
cd _build
cmake -DBUILD_METAPODFROMURDF=OFF
make install
\endverbatim

Please note that CMake produces a CMakeCache.txt file which should be deleted to reconfigure a package from scratch.


*/

