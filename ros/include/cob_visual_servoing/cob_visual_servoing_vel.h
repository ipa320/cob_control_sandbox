/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_mobile_manipulation
 * \note
 *   ROS package name: cob_visual_servoing
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: November, 2013
 *
 * \brief
 *   This package provides visual servoing for the Care-O-bot
 *
 ****************************************************************/
#ifndef COB_VISUAL_SERVOING_VEL_H
#define COB_VISUAL_SERVOING_VEL_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
#include <cob_srvs/Trigger.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_listener.h>

class CobVisualServoingVel
{
private:
	ros::NodeHandle nh_;
	tf::TransformListener m_tf_listener;
	
	ros::ServiceServer serv_start;
	ros::ServiceServer serv_stop;
	
	ros::Subscriber js_sub;
	
	ros::Publisher torso_cmd_vel_pub;
	//ros::Publisher torso_lower_pub;
	//ros::Publisher torso_pan_pub;
	//ros::Publisher torso_upper_pub;
	
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *torso_ac; 
	
	bool b_initial_focus;
	bool b_servoing;
	unsigned int throttle_;
	
	KDL::Chain chain_arm_;
	KDL::Chain chain_lookat_;
	
	KDL::ChainFkSolverPos_recursive* p_fksolver_pos_arm_;
	KDL::ChainFkSolverVel_recursive* p_fksolver_vel_arm_;
	KDL::ChainIkSolverPos_NR* p_iksolver_pos_arm_;
	KDL::ChainIkSolverVel_pinv* p_iksolver_vel_arm_;
	KDL::ChainFkSolverPos_recursive* p_fksolver_pos_lookat_;
	KDL::ChainFkSolverVel_recursive* p_fksolver_vel_lookat_;
	//KDL::ChainIkSolverPos_NR* p_iksolver_pos_lookat_;
	KDL::ChainIkSolverPos_NR_JL* p_iksolver_pos_lookat_;
	KDL::ChainIkSolverVel_pinv* p_iksolver_vel_lookat_;
	
	std::vector<std::string> torso_joints_;		//later: use this for configuring visual servoing for 3DoF or 4DoF torso
	std::vector<float> torso_limits_min_;
	std::vector<float> torso_limits_max_;
	
	//helper functions
	bool parseArmJointStates(std::vector<std::string> names, std::vector<double> positions, std::vector<double> velocities, KDL::JntArray& q, KDL::JntArray& q_dot);
	bool parseLookatJointStates(std::vector<std::string> names, std::vector<double> positions, std::vector<double> velocities, KDL::JntArray& q, KDL::JntArray& q_dot);
	
	
public:
	CobVisualServoingVel() {;}
	~CobVisualServoingVel();
	
	void initialize();
	void run();
	
	
	bool start_cb(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	bool stop_cb(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	
	bool initial_focus();
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);

};
#endif

