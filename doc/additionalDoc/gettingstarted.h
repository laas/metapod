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
   
   \section GettingStartedInstallation Creating the template of your robot model.
   
   In order to use Metapod, you will need to extract the source file and install the header files. <br>
   You also need a robot model. They are two ways to build a robot model. 
   <ul>
   <li> You can do it by yourself
   from the examples, but this is not recommanded. </li>
   <li> In order to make your/our life easier, metapod integrates a tool calls metapodfromurdf
   to generate models from a urdf file. Two urdf models are included in the initial repository: 
   a simple planar arm, and a humanoid robot.   </li>
   </ul>
   The following examples are based on the simple_arm.urdf example.
   We will assume that the related robot model is in a repository called simple_arm.
   Once your robot model is available, using Metapod is very similar to Eigen.
   You do not need additional library. 

   \section Examples
   
   Once you have generated the source files related to your model, it is very simple to use it.
   Let us assume that you are using the simple_arm model and that you want to compute
   the torques of your robot \f$\tau\f$ for a given set of \f$({\bf q},\dot{\bf q},\ddot{\bf q})\f$.

   First the header which describes the model has to be specified:
   \code
   #include <metapod/models/simple_arm.hh>
   \endcode
   
   Then the way real are represented in the model has to be set:   
   
   \code typedef simple_humanoid<double> Robot;\endcode

   It is now possible to create an instance of the robot from the newly defined type:
   \code 
   int main(void)
   {
      Robot arobot;
   \endcode

   To specify a robot state we can defined the configuration vectors related to the robot:
   \code
     Robot::confVector q,dq,ddq, torques;
   \endcode
   
   The configuration vector can be initialized by reading files for instance.

   \code 
     std::ifstream qconf(TEST_DIRECTORY "/q.conf");
     std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
     std::ifstream ddqconf(TEST_DIRECTORY "/ddq.conf");

     initConf< Robot >::run(qconf, q);
     initConf< Robot >::run(dqconf, dq);
     initConf< Robot >::run(ddqconf, ddq);

     qconf.close();
     dqconf.close();
     ddqconf.close();`
   \endcode
   
   RNEA is the algorithm computing the inverse dynamics.
   To apply it to the robot, the rnea template is instanciated with 
   the model of the robot like this:
   \code
     rnea< Robot, true >::run(robot, q, dq, ddq);
   \endcode

   To get the torques:
   \code
     getTorques(robot,torques);
   \endcode
   
   
 */
