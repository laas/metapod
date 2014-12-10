/** \page forward_kinematics bcalc: how to compute the forward kinematics of your model.

   Let us assume that you are using the simple_arm model and that you want to compute

   the position and orientation of each part of your robot knowing \f$q\f$ the configuration vector

   include the joint angles and the position and orientation of the base.

   First the header which describes the model has to be specified:
   \code
   #include <metapod/models/simple_arm.hh>
   \endcode

   Secondly we call the bcalc include:
   \code
   #include <metapod/tools/bcalc.hh>
   \endcode

   for the tutorial we used the metapod namespace :
   \code
   using namespace metapod;
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
     Robot::confVector q ;
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

   you can acces the configuration vector by simply doing :
   \code
        for(unsigned int i=0 ; i < qconf.size(); ++i)
            q(i,0) = InitialPosition(i) ;
   \endcode

   bcalc is the algorithm computing the forward kinematics.
   To apply it to the robot, the bcalc template is instanciated with
   the model of the robot like this:
   \code
     bcalc<robot>::run(arobot,q);
   \endcode

   see this page to access to the position

*/

