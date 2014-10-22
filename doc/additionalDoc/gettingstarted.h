/* Copyright 2014
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
*/

/*!
   @page gettingstarted Getting Started
   
   \section GettingStartedInstallation Preparing a robot model.
   
   In order to use Metapod, you will need to extract the source file and install the header files. <br>
   You also need a robot model. They are two ways to build a robot model. 
   <ul>
   <li> You can do it by yourself
   from the examples, but this is not recommanded. </li>
   <li> In order to make your/our life easier, metapod integrates a tool calls metapodfromurdf
   to generate models from a urdf file. <br>
   Two urdf models are included in the initial repository: 
   a simple planar arm, and a humanoid robot.   </li>
   </ul>

   Once your robot model is available, using Metapod is very similar to Eigen.
   You do not need additional library. <br>
   Follow the tutorial on \subpage include_your_robot_model
   to use your URDF robot model with metapod.
   The remaining examples use a robot model provided with metapod.

   \section Examples
   
   A set of \subpage tutorials is available.
*/
