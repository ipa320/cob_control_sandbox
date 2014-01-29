#include "ros/ros.h"

#include <math.h>
#include <map>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_srvs/Trigger.h>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <cob_articulation_controller/ParamMsg.h>
#include <cob_articulation_controller/TrackMsg.h>
#include <cob_articulation_controller/ModelMsg.h>
#include <cob_articulation_controller/MoveModel.h>
#include <cob_articulation_controller/MovePrismatic.h>
#include <cob_articulation_controller/MoveRotational.h>
#include <cob_articulation_controller/ArticulationModelAction.h>
#include <actionlib/server/simple_action_server.h>



using namespace KDL;
using namespace std;

class cob_cartesian_trajectories
{
public:
    cob_cartesian_trajectories();

private:
    ros::NodeHandle n;
    actionlib::SimpleActionServer<cob_articulation_controller::ArticulationModelAction> as_model_;
    
    void cartStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void cartTwistStateCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void stopTrajectory();
    geometry_msgs::Twist getTwist(double dt, Frame F_current);

    // trajectory generation
    void getTargetPosition(double dt, KDL::Frame &F_target);
    void getPriTarget(double dt, KDL::Frame &F_target);
    void getPriStart(KDL::Frame &F_track_start);
    void getRotTarget(double dt, KDL::Frame &F_target);
    void getRotStart(KDL::Frame &F_track_start);
    double getParamValue(std::string param_name);
    double unwrapRPY(std::string axis,  double angle);    

    // tools
    void vector3dKDLToEigen(const KDL::Vector &from, Eigen::Vector3d &to);
    Eigen::Vector3d vector3dKDLToEigen(const KDL::Vector &from);
    void vector3dEigenToKDL(const Eigen::Vector3d &from, KDL::Vector &to);
    KDL::Vector vector3dEigenToKDL(const Eigen::Vector3d &from);

    // controller
    geometry_msgs::Twist PIDController(const double dt, const KDL::Frame &F_target, const KDL::Frame &F_Current);

    // joint limits
    void getJointLimits(std::vector<double> &UpperLimits, std::vector<double> &LowerLimits);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    std::vector<double> parseJointStates(std::vector<std::string> names, std::vector<double> positions);
    
    // visualization functions
    void pubTrack(const int track_id, const ros::Duration pub_duration, const KDL::Frame &F_pub);
    void pubTwistMarkers(const ros::Duration pub_duration, const geometry_msgs::Twist &Twist, const KDL::Frame &F_current);
    void sendMarkers();

    // action callbacks
    void moveModelActionCB(const cob_articulation_controller::ArticulationModelGoalConstPtr& goal);
    
    // service callbacks
    //bool moveCircCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
    //bool moveLinCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
    bool movePriCB(cob_articulation_controller::MovePrismatic::Request& request, cob_articulation_controller::MovePrismatic::Response& response);
    bool moveRotCB(cob_articulation_controller::MoveRotational::Request& request, cob_articulation_controller::MoveRotational::Response& response);
    bool moveModelCB(cob_articulation_controller::MoveModel::Request& request, cob_articulation_controller::MoveModel::Response& response);

    bool start();
    
    ros::Subscriber cart_state_sub_;
    ros::Subscriber cart_twist_state_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher cart_command_pub;
    ros::Publisher debug_cart_pub_;
    ros::Publisher map_pub_;
    ros::Publisher twist_pub_;
    ros::Publisher track_pub_;
    ros::Publisher model_pub_;
    ros::ServiceServer serv_prismatic;
    ros::ServiceServer serv_rotational;
    ros::ServiceServer serv_model;


    // VARIABLES
    bool bRun;
    bool bStarted;
    double currentDuration;
    double targetDuration;

    //controller configuration
    double p_gain_;
    double i_gain_;
    double d_gain_;

    ros::Time timer;
    ros::Time tstart;
    KDL::Frame F_EE_start;
    KDL::Frame F_track_start;
    KDL::Twist Error;
    KDL::Twist Error_sum;
    KDL::Twist Error_dot;
    KDL::Twist Error_last;
    KDL::Twist Error_last2;

    // action
    bool success;                       // status of finished action
    cob_articulation_controller::ArticulationModelFeedback feedback_;
    cob_articulation_controller::ArticulationModelResult result_;

    string mode;                        // prismatic, rotational, trajectory, model
    std::vector<cob_articulation_controller::ParamMsg> params;    // articulation parameters
    std::vector<geometry_msgs::Pose> track;     // trajectory

    geometry_msgs::PoseStamped current_hinge;

    // visualization
    std::vector<geometry_msgs::Point> trajectory_points;
    std::map<int, cob_articulation_controller::TrackMsg> track_map;     //stores track_ids and tracks for publishing
    ros::Time pub_timer;
    int pub_counter;

    // joint limits
    std::vector<double> UpperLimits;
    std::vector<double> LowerLimits;

    std::vector<double> q_last;
    std::vector<double> jointStates;

    map<std::string, double> last_rpy_angles;       // stores the last angles for R-P-Y unwrapping function

    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    double rot_radius;

    bool bHandle;
    bool debug;

    // error vector
    std::vector<KDL::Twist> vec_err_p;
    std::vector<KDL::Twist> vec_err_tb;
    std::vector<KDL::Frame> vec_frames;
    std::vector<KDL::Twist> vec_vel_soll;
    std::vector<KDL::Twist> vec_vel_ist;
    std::vector<KDL::Vector> vec_pos_ist;
    std::vector<KDL::Vector> vec_pos_soll;

    KDL::Frame F_last_soll;
    KDL::Frame F_last_ist;
};

