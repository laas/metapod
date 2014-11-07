/** \page NodesExplanation Nodes : read information of you robot model during run time.

    Let us assume that you are using the simple_arm model and that you

    have computed the forward kinematics for example, and you to display or save

    the state of your robot somewhere.

    we assume that you have read the tutorial here :
    \link forward_kinematics tutorial on forward kinematics \endlink

    First the headers:
    \code
        #include <metapod/models/simple_arm.hh>
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

    After initialization you can call whatever suitable function you need, e.g. bcalc, jcalc, rnea...

    Then you want to access to one of the body position. A body, as well as a joint is attached to "Node".
    One node contains every information concerning the \f$j^{th}\f$ joint and the \f$j^{th}\f$ body.

    In order to access one node we need to introduce a "typedef" that will describe one node in the kinematic tree.
    \code
        typedef Nodes< Robot, id >::type Node_id;
    \endcode

    this type will be used to instanciated a reference pointing to the \f$id^{th}\f$ "Node"

    \code
        Node_id & a_node_id = boost::fusion::at_c<id>(arobot.nodes);;
    \endcode
    carefull: here we use both a type "Robot" and a instance of this type "arobot".
    A classic mistake is to mix up the type and the instance.
    All the possible id are describe in the include file in the Robot class.

    Here is a list (non exhaustive) of information you can extract
    (for more information read the code : metapod/models/simple_arm.hh) ;

    \code
        a_node_id.id              ;  // the id of the node
        a_node_id.jointFwdDyn     ;  // <dynamics> fwd_dyn field, used by chda
        a_node_id.jointNuOfFwdDyn ;  // subtree supported by at least one fwdDyn joint
        a_node_id.joint_name      ;  // the name of the id_th joint
        a_node_id.body_name       ;  // the name of the id_th body
        a_node_id.parent_id       ;  // the id of the parent node in the kinematic tree
        a_node_id.childX_id       ;  // the ids of the child nodes in the kinematic tree, X in {1,...,4}
        a_node_id.q_idx           ;  // "TODO complete this part"
        a_node_id.joint_F         ;  // "TODO complete this part"
        //...
        a_node_id.body            ;  // id_th body
        a_node_id.joint           ;  // id_th joint
        //...
        a_node_id.joint.f         ;  // force apllied to the id_th joint
        a_node_id.Xt              ;  // transform from base i to the body 0 (see below)
        a_node_id.sXp             ;  // transform from the successor frame to the predecessor frame (see below)
        a_node_id.joint.Xj        ;  // transform relative to the joint (see below)
        a_node_id.joint.iX0       ;  // transform from base 0 to the body i (see below)
    \endcode

    The transform needs a little bit more explanation (see the Featherstone book):
    \f$ ^{B}X_{A} =
    \begin{bmatrix}
        ^{B}R_{A}                        &   0    \\
        -^{B}R_{A} * ^{A}r_{\overrightarrow{ab}}     &   ^{B}R_{A}
    \end{bmatrix}
    \f$

    with :
    - \f$^{B}X_{A}\f$, being the transformation from frame A to frame B,
    - \f$^{B}R_{A}\f$, the rotational matrix from frame A to frame B,
    - \f$^{A}r_{\overrightarrow{ab}}\f$, the vector \overrightarrow{ab}
    where a is the origin of the frame A and b the origin of the frame B
    This vector is expressed in the frame A

    This is why when you print the trnasformation \f$ ^{i}X_{0}\f$
    \code
        cout << a_node_id.joint.iX0 << endl ;
    \endcode
    you get the rotation matrix : \f$ ^{i}R_{0}\f$.
    And the vector : \f$ ^{0}r_{\overrightarrow{ob_i}} \f$ which is the position of the body "i" in the world frame.


    In the case where you want to display every nodes, you will need to use a templated structure like this one :
    \code
        struct print_iXo
        {
            template <typename T>
            void operator()(T & x) const
            {
                std::cout << "body.iX0 = \n" << x.body.iX0 ;
            }
        };
    \endcode

    In the main :
    \code
        //...
        #include <boost/fusion/algorithm/iteration/for_each.hpp>
        #include <boost/fusion/include/for_each.hpp>
        int main(void)
        {
            Robot arobot;
            Robot::confVector q_init ;

            // .. init the q_init here ..

            bcalc<Robot>::run(arobot,q_init);

            boost::fusion::for_each(arobot.nodes ,  print_iXo() );
        }
    \endcode

    see <a href="http://www.boost.org/doc/libs/1_55_0/libs/fusion/doc/html/fusion/quick_start.html">link Boost fusion web site</a>
    for more information
*/

