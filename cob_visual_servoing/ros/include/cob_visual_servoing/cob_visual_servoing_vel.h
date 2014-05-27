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
	ros::Publisher lookat_cmd_vel_pub;
	
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *torso_ac;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *lookat_ac; 
	
	KDL::Chain chain_lookat_;
	KDL::ChainFkSolverPos_recursive* p_fksolver_pos_lookat_;
	KDL::ChainFkSolverVel_recursive* p_fksolver_vel_lookat_;
	//KDL::ChainIkSolverPos_NR* p_iksolver_pos_lookat_;
	KDL::ChainIkSolverPos_NR_JL* p_iksolver_pos_lookat_;
	KDL::ChainIkSolverVel_pinv* p_iksolver_vel_lookat_;
	
	std::vector<std::string> torso_joints_;		//later: use this for configuring visual servoing for 3DoF or 4DoF torso
	std::vector<std::string> lookat_joints_;
	KDL::JntArray lookat_min_;
	KDL::JntArray lookat_max_;
	KDL::JntArray lookat_vel_max_;
	
	ros::Time m_last_time_pub;
	KDL::JntArray m_last_q_lookat;
	KDL::JntArray m_last_q_dot_lookat;
	std::string m_goal_focus_frame;
	
	bool b_initial_focus;
	bool b_servoing;
	double update_frequency;
	
	
	
	bool update_goal(KDL::JntArray& q_lookat_goal);
	bool update_commands(KDL::JntArray& q_lookat_goal);
	void send_commands(KDL::JntArray goal_velocities);
	bool initial_focus();
	void stop();
	
	
public:
	CobVisualServoingVel() {;}
	~CobVisualServoingVel();
	
	void initialize();
	void run();
	
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	
	bool start_cb(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	bool stop_cb(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
};
#endif

