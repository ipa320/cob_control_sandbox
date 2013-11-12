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
#ifndef COB_VISUAL_SERVOING_H
#define COB_VISUAL_SERVOING_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <cob_srvs/Trigger.h>
#include <controller_manager_msgs/SwitchController.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

class CobVisualServoing
{
private:
	ros::NodeHandle nh_;
	
	ros::ServiceClient switch_controller_client;
	
	ros::ServiceServer serv_start;
	ros::ServiceServer serv_stop;
	
	ros::Subscriber js_sub;
	
	ros::Publisher torso_lower_pub;
	ros::Publisher torso_pan_pub;
	ros::Publisher torso_upper_pub;
	
	bool b_servoing;
	unsigned int throttle_;
	
	KDL::Chain chain_arm_;
	KDL::Chain chain_lookat_;
	
	KDL::ChainFkSolverPos_recursive* p_fksolver_pos_arm_;
	KDL::ChainIkSolverVel_pinv* p_iksolver_vel_arm_;
	KDL::ChainIkSolverPos_NR* p_iksolver_pos_arm_;
	KDL::ChainFkSolverPos_recursive* p_fksolver_pos_lookat_;
	KDL::ChainIkSolverVel_pinv* p_iksolver_vel_lookat_;
	KDL::ChainIkSolverPos_NR* p_iksolver_pos_lookat_;
	
	
	
	//helper functions
	bool parseJointStates(std::vector<std::string> names, std::vector<double> positions, std::vector<double> velocities, KDL::JntArray& q, KDL::JntArray& q_dot);
	
	
public:
	CobVisualServoing() {;}
	~CobVisualServoing();
	
	void initialize();
	void run();
	
	
	bool start_cb(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	bool stop_cb(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);

};
#endif

