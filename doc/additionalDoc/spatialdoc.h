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

/**
   \defgroup spatial_algebra Spatial Algebra

   This module implements the spatial algebra as described in Featherstone book.
   
   \section Definitions
   The motion space \f$M\f$ is used to represent velocity, acceleration, infinitesimal displacement,
   and finally directions of motion freedom and constraint.

   The elements of \f$F\f$ describe the forces acting on a solid body, and the related quantities:
   momentum, impulse, and directions of force freedom and constraint.

   A very important object is the <i>Transform</i> which allows to transfer
   a spatial vector in the motion space \f$M\f$ or in the force space \f$F\f$ from frame \f$A\f$ to another frame \f$B\f$.
   
   The \ref Transform  uses Plucker coordinates to represent motion with a point \f${\bf p}\f$ and 
   a rotation matrix \f${\bf R}\f$.

   \section quick_start Quick start
   \code
   #include <metapod/tools/spatial.hh>
   \endcode

   \defgroup transform_group Transforms
   \ingroup spatial_algebra

   \class Transform
   \brief 

   \defgroup rotations Rotations
   \ingroup transform_group

   The following joints use specific rotations matrices which implement the method
   recommended in Featherstone's Book Appendix.2.

   <ul>
   <li> RevoluteAxisXJoint : Implements specific rotation around x-axis.</li>

   <li> RevoluteAxisYJoint : Implements specific rotation around y-axis.</li>

   <li> RevoluteAxisXJoint : Implements specific rotation around z-axis.</li>
   
   </ul>
   
   An extension would be to write planar motion 
   \f$R_{\theta_x,\theta_y}(\theta_x,\theta_y)\f$
   such that:
   \f[ 
    R_{\theta_x,\theta_y}(\theta_x,\theta_y) = 
     R_x(\theta_x)  R_y(\theta_y) = 
     \left[ \begin{matrix} 
     1 & 0 & 0 \\ 
     0 & c_x & s_x \\ 
     0 & -s_x & c_x 
     \end{matrix} \right] 
     \left[
      \begin{matrix}
       c_y & 0 & -s_y \\
       0 & 1 & 0 \\
       s_y & 0 & c_y 
     \end{matrix}
    \right] =
     \left[ \begin{matrix} 
     c_y    & 0    & -s_y \\ 
     s_xs_y & c_x  & s_xc_y \\ 
     c_xs_y & -s_x & c_xc_y 
     \end{matrix} \right] 
  \f]
  On the other hand
   \f[ 
    R_{\theta_y,\theta_x}(\theta_y,\theta_x) = 
     R_y(\theta_y)  R_x(\theta_x) = 
     \left[
      \begin{matrix}
       c_y & 0 & -s_y \\
       0 & 1 & 0 \\
       s_y & 0 & c_y 
     \end{matrix}
    \right] 
     \left[ \begin{matrix} 
     1 & 0 & 0 \\ 
     0 & c_x & s_x \\ 
     0 & -s_x & c_x 
     \end{matrix} \right] 
    =
     \left[ \begin{matrix} 
     c_y &  s_xs_y  & -c_xs_y \\ 
     0   &  c_x     &  s_x \\ 
     s_y & -s_xc_y  &  c_xc_y 
     \end{matrix} \right] 
  \f]


 */

