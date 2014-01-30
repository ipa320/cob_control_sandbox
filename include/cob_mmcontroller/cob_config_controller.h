#include "ros/ros.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cob_srvs/Trigger.h>
#include <brics_actuator/JointVelocities.h>

#include <tf_conversions/tf_kdl.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <cob_mmcontroller/augmented_solver.h>

class cob_config_controller
{
public:
    cob_config_controller();
    
private:
    KDL::JntArray parseJointStates(std::vector<std::string> names, std::vector<double> positions, std::vector<double> velocities, KDL::JntArray& q, KDL::JntArray& q_dot);
    void cartTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void baseTwistCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool SyncMMTriggerStart(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
    bool SyncMMTriggerStop(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
    void sendVel(KDL::JntArray q_t, KDL::JntArray q_dot, KDL::JntArray q_dot_base);
    void sendCartPose();
    void controllerStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    ros::NodeHandle n;
    ros::Time last;
    
    int zeroCounter;
    int zeroCounter_base;
    int zeroCounterTwist;

    //configuration
    std::string arm_base_name_;
    std::string arm_ee_name_;
    std::string kinematic_mode_;

    bool started;
    
    KDL::Chain arm_base_chain;
    KDL::Chain arm_chain;
    KDL::JntArray VirtualQ;
    KDL::JntArray q;
    KDL::JntArray q_dot;
    KDL::JntArray q_last;
    KDL::Twist extTwist;

    KDL::Frame base_odom_;
    KDL::Twist base_twist_;
    KDL::Frame arm_pose_;
    KDL::FrameVel arm_vel_;

    KDL::ChainFkSolverPos_recursive *  fksolver1;
    KDL::ChainFkSolverVel_recursive * fksolver1_vel;
    augmented_solver * iksolver1v;  //Inverse velocity solver

    bool RunSyncMM;
    ros::Publisher arm_pub_;    //publish topic arm_controller/command
    ros::Publisher base_pub_;   //publish topic base_controller/command
    ros::Publisher debug_cart_pub_;
    ros::Publisher cart_position_pub_;
    ros::Publisher cart_twist_pub_;

    ros::ServiceServer serv_start;
    ros::ServiceServer serv_stop;

    ros::Subscriber sub;
    ros::Subscriber cart_vel_sub;
    ros::Subscriber plat_odom_sub;

};
