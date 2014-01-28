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
#include <ros/ros.h>

#include <cob_visual_servoing/cob_visual_servoing_vel.h>

#include <kdl_conversions/kdl_msg.h>


void CobVisualServoingVel::initialize()
{
	KDL::Tree my_tree;

	std::string robot_desc_string;
	nh_.param("/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return;
	}
	my_tree.getChain("base_link", "arm_7_link", chain_arm_);
	my_tree.getChain("base_link", "lookat_focus_frame", chain_lookat_);
	if(chain_arm_.getNrOfJoints() == 0 || chain_lookat_.getNrOfJoints() == 0)
	{
		ROS_ERROR("Failed to initialize kinematic chains");
		return;
	}
	
	//Initializing configuration control solver
	p_fksolver_pos_arm_ = new KDL::ChainFkSolverPos_recursive(chain_arm_);
	p_fksolver_vel_arm_ = new KDL::ChainFkSolverVel_recursive(chain_arm_);
	p_iksolver_vel_arm_ = new KDL::ChainIkSolverVel_pinv(chain_arm_, 0.001, 5);
	p_iksolver_pos_arm_ = new KDL::ChainIkSolverPos_NR(chain_arm_, *p_fksolver_pos_arm_, *p_iksolver_vel_arm_, 5, 0.001);
	p_fksolver_pos_lookat_ = new KDL::ChainFkSolverPos_recursive(chain_lookat_);
	p_fksolver_vel_lookat_ = new KDL::ChainFkSolverVel_recursive(chain_lookat_);
	p_iksolver_vel_lookat_ = new KDL::ChainIkSolverVel_pinv(chain_lookat_, 0.001, 5);
	p_iksolver_pos_lookat_ = new KDL::ChainIkSolverPos_NR(chain_lookat_, *p_fksolver_pos_lookat_, *p_iksolver_vel_lookat_, 5, 0.001);
	
	
	
	serv_start = nh_.advertiseService("/visual_servoing/start", &CobVisualServoingVel::start_cb, this);
	serv_stop = nh_.advertiseService("/visual_servoing/stop", &CobVisualServoingVel::stop_cb, this);
	
	js_sub = nh_.subscribe("/joint_states", 1, &CobVisualServoingVel::jointstate_cb, this);
	
	torso_lower_pub = nh_.advertise<std_msgs::Float64>("/torso_lower_neck_tilt_velocity_controller/command", 10);
	torso_pan_pub = nh_.advertise<std_msgs::Float64>("/torso_pan_velocity_controller/command", 10);
	torso_upper_pub = nh_.advertise<std_msgs::Float64>("/torso_upper_neck_tilt_velocity_controller/command", 10);
	
	b_servoing = false;
	throttle_ = 0;
	ROS_INFO("...initialized!");
}

void CobVisualServoingVel::run()
{
	ROS_INFO("cob_visual_servoing...spinning");
	ros::spin();
}

bool CobVisualServoingVel::start_cb(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	if(b_servoing)
	{
		ROS_INFO("Already started");
	}
	else
	{
		ROS_INFO("Start servoing...");
		b_servoing = true;
	}
	return true;
}


bool CobVisualServoingVel::stop_cb(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	if(!b_servoing)
	{
		ROS_INFO("Already stopped");
	}
	else
	{
		ROS_INFO("Stopping servoing...");
		b_servoing = false;
	}
	return true;
}


void CobVisualServoingVel::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
	ROS_INFO("Throttle %d", throttle_);
	if(b_servoing)
	{
		KDL::JntArray q_arm(chain_arm_.getNrOfJoints());
		KDL::JntArray q_dot_arm(chain_arm_.getNrOfJoints());

		bool success = parseJointStates(msg->name, msg->position, msg->velocity, q_arm, q_dot_arm);
		
		if(success)
		{
			if(throttle_==10)
			{
				//ROS_INFO_STREAM("Current arm_config: "<<q_arm(0)<<", "<<q_arm(1)<<", "<<q_arm(2)<<", "<<q_arm(3)<<", "<<q_arm(4)<<", "<<q_arm(5)<<", "<<q_arm(6));
				
				/***
				KDL::Frame frame_arm;
				int ret_fk = p_fksolver_pos_arm_->JntToCart(q_arm, frame_arm);
				
				geometry_msgs::Pose pose_arm;
				tf::poseKDLToMsg(frame_arm, pose_arm);
				
				//ROS_INFO_STREAM("Current arm_pose: " << pose_arm);
				
				KDL::JntArray q_init_lookat(chain_lookat_.getNrOfJoints());
				KDL::JntArray q_ik_lookat(chain_lookat_.getNrOfJoints());
				
				int ret_ik = p_iksolver_pos_lookat_->CartToJnt(q_init_lookat, frame_arm, q_ik_lookat);
				***/
				
				
				KDL::FrameVel frame_vel_arm;
				KDL::JntArrayVel vel_arm_in(q_arm, q_dot_arm);
				int ret_fk = p_fksolver_vel_arm_->JntToCart(vel_arm_in, frame_vel_arm);
				
				KDL::JntArray q_init_lookat(chain_lookat_.getNrOfJoints());
				KDL::Twist twist_arm = frame_vel_arm.GetTwist();
				KDL::JntArray q_dot_ik_lookat(chain_lookat_.getNrOfJoints());
				
				int ret_ik = p_iksolver_vel_lookat_->CartToJnt(q_init_lookat, twist_arm, q_dot_ik_lookat);
				
				ROS_INFO_STREAM("Goal torso_velocities: "<<q_dot_ik_lookat(0)<<", "<<q_dot_ik_lookat(1)<<", "<<q_dot_ik_lookat(2));
				
				std_msgs::Float64 msg_lower;
				std_msgs::Float64 msg_pan;
				std_msgs::Float64 msg_upper;
				msg_lower.data = q_dot_ik_lookat(0);
				msg_pan.data = q_dot_ik_lookat(1);
				msg_upper.data = q_dot_ik_lookat(2);
				
				torso_lower_pub.publish(msg_lower);
				torso_pan_pub.publish(msg_pan);
				torso_upper_pub.publish(msg_upper);
				throttle_ = 0;
			}
			throttle_++;
		}
	}

}




/* ~~~~~~~~~~~~~~~~
 * Helper Functions 
 * ~~~~~~~~~~~~~~~~*/

bool CobVisualServoingVel::parseJointStates(std::vector<std::string> names, std::vector<double> positions, std::vector<double> velocities, KDL::JntArray& q, KDL::JntArray& q_dot)
{
	KDL::JntArray q_temp(7);
	KDL::JntArray q_dot_temp(7);
	int count = 0;
	bool parsed = false;
	for(unsigned int i = 0; i < names.size(); i++)
	{
		if(strncmp(names[i].c_str(), "arm_", 4) == 0)
		{
			q_temp(count) = positions[i];
			q_dot_temp(count) = velocities[i];
			count++;
		}
	}
	
	if(count == 7)
		parsed = true;
	
	q = q_temp;
	q_dot = q_dot_temp;
	return parsed;
}











int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_visual_servoing_node");
	CobVisualServoingVel *cob_visual_servoing_server = new CobVisualServoingVel();
	
	cob_visual_servoing_server->initialize();
	cob_visual_servoing_server->run();
	
	return 0;
}
