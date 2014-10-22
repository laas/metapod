/** \page gettorques Inverse dynamics: how to read the torques of your model.

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
