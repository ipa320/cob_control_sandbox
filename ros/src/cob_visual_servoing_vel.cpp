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
#include <tf/transform_datatypes.h>


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
	
	//hardcoded for now
	torso_joints_.push_back("torso_lower_neck_tilt_joint");
	torso_joints_.push_back("torso_pan_joint");
	torso_joints_.push_back("torso_upper_neck_tilt_joint");
	
	torso_limits_min_.push_back(-0.25);
	torso_limits_min_.push_back(-0.12);
	torso_limits_min_.push_back(-0.37);
	
	torso_limits_max_.push_back(0.21);
	torso_limits_max_.push_back(0.12);
	torso_limits_max_.push_back(0.37);
	
	torso_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh_, "/torso_controller/follow_joint_trajectory",  true);
	ROS_INFO("Wait for ActionServer...");
	torso_ac->waitForServer(ros::Duration(10.0));
	
	//Initializing configuration control solver
	p_fksolver_pos_arm_ = new KDL::ChainFkSolverPos_recursive(chain_arm_);
	p_fksolver_vel_arm_ = new KDL::ChainFkSolverVel_recursive(chain_arm_);
	p_iksolver_vel_arm_ = new KDL::ChainIkSolverVel_pinv(chain_arm_, 0.001, 5);
	p_iksolver_pos_arm_ = new KDL::ChainIkSolverPos_NR(chain_arm_, *p_fksolver_pos_arm_, *p_iksolver_vel_arm_, 5, 0.001);
	p_fksolver_pos_lookat_ = new KDL::ChainFkSolverPos_recursive(chain_lookat_);
	p_fksolver_vel_lookat_ = new KDL::ChainFkSolverVel_recursive(chain_lookat_);
	p_iksolver_vel_lookat_ = new KDL::ChainIkSolverVel_pinv(chain_lookat_, 0.001, 5);
	//p_iksolver_pos_lookat_ = new KDL::ChainIkSolverPos_NR(chain_lookat_, *p_fksolver_pos_lookat_, *p_iksolver_vel_lookat_, 5, 0.001);
	KDL::JntArray torso_min(torso_limits_min_.size());
	KDL::JntArray torso_max(torso_limits_max_.size());
	for(unsigned int i=0; i<torso_limits_min_.size(); i++)
	{
		torso_min(i)=torso_limits_min_[i];
		torso_max(i)=torso_limits_max_[i];
	}
	p_iksolver_pos_lookat_ = new KDL::ChainIkSolverPos_NR_JL(chain_lookat_, torso_min, torso_max, *p_fksolver_pos_lookat_, *p_iksolver_vel_lookat_, 50, 0.001);
	
	serv_start = nh_.advertiseService("/visual_servoing/start", &CobVisualServoingVel::start_cb, this);
	serv_stop = nh_.advertiseService("/visual_servoing/stop", &CobVisualServoingVel::stop_cb, this);
	
	js_sub = nh_.subscribe("/joint_states", 1, &CobVisualServoingVel::jointstate_cb, this);
	
	torso_cmd_vel_pub = nh_.advertise<brics_actuator::JointVelocities>("/torso_controller/command_vel", 10);
	//torso_lower_pub = nh_.advertise<std_msgs::Float64>("/torso_lower_neck_tilt_velocity_controller/command", 10);
	//torso_pan_pub = nh_.advertise<std_msgs::Float64>("/torso_pan_velocity_controller/command", 10);
	//torso_upper_pub = nh_.advertise<std_msgs::Float64>("/torso_upper_neck_tilt_velocity_controller/command", 10);
	
	b_initial_focus = false;
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
		ROS_INFO("Received request...");
		
		///Move Lookat so that lookat_focus_frame is equal to arm_7_link (i.e. same Pose)
		while(!b_initial_focus)
		{
			bool success = initial_focus();
			if(success)
			{
				ROS_INFO("Start servoing...");
				break;
			}
			else
			{
				ROS_WARN("Not yet able to focus goal...Move Goal into field-of-view");
				ros::Duration(0.5).sleep();
			}
		}
		
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
		b_initial_focus = false;
		b_servoing = false;
	}
	return true;
}


bool CobVisualServoingVel::initial_focus()
{
	ROS_INFO("Trying to initially focus goal!");
	b_initial_focus = false;
	
	///get goal_pose
	tf::StampedTransform transform_tf;
	geometry_msgs::TransformStamped transform_msg;
	geometry_msgs::Pose pose_msg;
	try{
		m_tf_listener.lookupTransform("/base_link", "/arm_7_link", ros::Time(0), transform_tf);
		transformStampedTFToMsg(transform_tf, transform_msg);
		ROS_INFO_STREAM("Current Position of Goal:\n" << transform_msg);
		pose_msg.position.x = transform_msg.transform.translation.x;
		pose_msg.position.y = transform_msg.transform.translation.y;
		pose_msg.position.z = transform_msg.transform.translation.z;
		pose_msg.orientation = transform_msg.transform.rotation;
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ROS_ERROR("Cannot lookupTransform for current Goal");
		b_initial_focus = false;
		return false;
	}
	
	///calculate IK for lookat chain (pos)
	KDL::JntArray q_init_lookat(chain_lookat_.getNrOfJoints());
	KDL::Frame frame_arm;
	tf::poseMsgToKDL(pose_msg, frame_arm);
	KDL::JntArray q_ik_lookat(chain_lookat_.getNrOfJoints());
	
	int ret_ik = p_iksolver_pos_lookat_->CartToJnt(q_init_lookat, frame_arm, q_ik_lookat);
	
	if(ret_ik < 0)
	{
		ROS_ERROR("No IK-Solution found! Unable to focus goal!");
		b_initial_focus = false;
		return false;
	}
	else
	{
		ROS_INFO("Found IK-Solution: (%f, %f, %f, %f, %f, %f, %f)", q_ik_lookat(0), q_ik_lookat(1), q_ik_lookat(2), q_ik_lookat(3), q_ik_lookat(4), q_ik_lookat(5), q_ik_lookat(6));
	}
	
	
	///move torso (follow_joint_trajectory)
	control_msgs::FollowJointTrajectoryGoal  torso_goal;
	torso_goal.trajectory.header.stamp  =  ros::Time::now();
	torso_goal.trajectory.header.frame_id  =  "base_link";
	torso_goal.trajectory.joint_names = torso_joints_;
	trajectory_msgs::JointTrajectoryPoint  torso_point;
	for(unsigned int i=0; i<chain_lookat_.getNrOfJoints(); i++)
	{
		torso_point.positions.push_back(q_ik_lookat(i));
	}
	torso_point.time_from_start  = ros::Duration(2.0);
	torso_goal.trajectory.points.push_back(torso_point);

	torso_ac->sendGoal(torso_goal);
	
	bool finished_before_timeout = torso_ac->waitForResult(ros::Duration(5.0));
	
	if(finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = torso_ac->getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
		
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Successfully moved torso!");
			b_initial_focus = true;
			return true;
		}
		else
		{
			ROS_ERROR("Something went wrong while moving torso!");
			b_initial_focus = false;
			return false;
		}
	}
	else
	{
		ROS_ERROR("Something went wrong while moving torso!");
		b_initial_focus = false;
		return false;
	}
}



void CobVisualServoingVel::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
	ROS_DEBUG("Throttle %d", throttle_);
	if(b_servoing)
	{
		KDL::JntArray q_arm(chain_arm_.getNrOfJoints());
		KDL::JntArray q_dot_arm(chain_arm_.getNrOfJoints());
		
		bool success = parseArmJointStates(msg->name, msg->position, msg->velocity, q_arm, q_dot_arm);
		
		if(success)
		{
			if(throttle_==10)
			{
				///Test and Debug output
				KDL::Frame frame_arm;
				int test_fk_arm = p_fksolver_pos_arm_->JntToCart(q_arm, frame_arm);
				geometry_msgs::Pose pose_arm;
				tf::poseKDLToMsg(frame_arm, pose_arm);
				ROS_INFO_STREAM("Current arm_config: "<<q_arm(0)<<", "<<q_arm(1)<<", "<<q_arm(2)<<", "<<q_arm(3)<<", "<<q_arm(4)<<", "<<q_arm(5)<<", "<<q_arm(6));
				ROS_INFO_STREAM("Current arm_pose:\n" << pose_arm);
				
				
				KDL::JntArray q_lookat(chain_lookat_.getNrOfJoints());
				KDL::JntArray q_dot_lookat(chain_lookat_.getNrOfJoints());
				bool test_success = parseLookatJointStates(msg->name, msg->position, msg->velocity, q_lookat, q_dot_lookat);
				
				KDL::Frame frame_lookat;
				int test_fk_lookat = p_fksolver_pos_lookat_->JntToCart(q_lookat, frame_lookat);
				geometry_msgs::Pose pose_lookat;
				tf::poseKDLToMsg(frame_lookat, pose_lookat);
				ROS_INFO_STREAM("Current lookat_config: "<<q_lookat(0)<<", "<<q_lookat(1)<<", "<<q_lookat(2)<<", "<<q_lookat(3)<<", "<<q_lookat(4)<<", "<<q_lookat(5)<<", "<<q_lookat(6));
				ROS_INFO_STREAM("Current lookat_pose:\n" << pose_lookat);
				
				
				
				
				/***
				KDL::JntArray q_init_lookat(chain_lookat_.getNrOfJoints());
				KDL::JntArray q_ik_lookat(chain_lookat_.getNrOfJoints());
				
				int ret_ik = p_iksolver_pos_lookat_->CartToJnt(q_init_lookat, frame_arm, q_ik_lookat);
				***/
				
				
				
				///Actual Work starts here
				
				KDL::FrameVel frame_vel_arm;
				KDL::JntArrayVel vel_arm_in(q_arm, q_dot_arm);
				int ret_fk = p_fksolver_vel_arm_->JntToCart(vel_arm_in, frame_vel_arm);
				
				KDL::JntArray q_init_lookat(chain_lookat_.getNrOfJoints());
				KDL::Twist twist_arm = frame_vel_arm.GetTwist();
				KDL::JntArray q_dot_ik_lookat(chain_lookat_.getNrOfJoints());
				
				int ret_ik = p_iksolver_vel_lookat_->CartToJnt(q_init_lookat, twist_arm, q_dot_ik_lookat);
				
				ROS_INFO_STREAM("Goal torso_velocities: "<<q_dot_ik_lookat(0)<<", "<<q_dot_ik_lookat(1)<<", "<<q_dot_ik_lookat(2));
				
				
				brics_actuator::JointVelocities msg;
				msg.velocities.resize(torso_joints_.size());
				for(int i=0; i<torso_joints_.size(); i++)
				{
					msg.velocities[i].joint_uri = torso_joints_[i].c_str();
					msg.velocities[i].unit = "rad";
					msg.velocities[i].value = q_dot_ik_lookat(i);
				}
				torso_cmd_vel_pub.publish(msg);
				
				//std_msgs::Float64 msg_lower;
				//std_msgs::Float64 msg_pan;
				//std_msgs::Float64 msg_upper;
				//msg_lower.data = q_dot_ik_lookat(0);
				//msg_pan.data = q_dot_ik_lookat(1);
				//msg_upper.data = q_dot_ik_lookat(2);
				
				//torso_lower_pub.publish(msg_lower);
				//torso_pan_pub.publish(msg_pan);
				//torso_upper_pub.publish(msg_upper);
				
				throttle_ = 0;
			}
			throttle_++;
		}
	}

}




/* ~~~~~~~~~~~~~~~~
 * Helper Functions 
 * ~~~~~~~~~~~~~~~~*/

bool CobVisualServoingVel::parseArmJointStates(std::vector<std::string> names, std::vector<double> positions, std::vector<double> velocities, KDL::JntArray& q, KDL::JntArray& q_dot)
{
	KDL::JntArray q_temp(7);
	KDL::JntArray q_dot_temp(7);
	int count = 0;
	bool parsed = false;
	
	ROS_DEBUG("New JointState");
	
	for(unsigned int i = 0; i < names.size(); i++)
	{
		if(strncmp(names[i].c_str(), "arm_", 4) == 0)
		{
			ROS_DEBUG("%d: %s", i, names[i].c_str());
			q_temp(count) = positions[i];
			q_dot_temp(count) = velocities[i];
			count++;
		}
	}
	
	if(count == 7)
		parsed = true;
	
	ROS_DEBUG("Done Parsing");
	
	q = q_temp;
	q_dot = q_dot_temp;
	return parsed;
}


bool CobVisualServoingVel::parseLookatJointStates(std::vector<std::string> names, std::vector<double> positions, std::vector<double> velocities, KDL::JntArray& q, KDL::JntArray& q_dot)
{
	KDL::JntArray q_temp(7);
	KDL::JntArray q_dot_temp(7);
	int count = 0;
	bool parsed = false;
	
	ROS_DEBUG("New JointState");
	
	for(unsigned int i = 0; i < names.size(); i++)
	{
		if(strncmp(names[i].c_str(), "torso_", 6) == 0)
		{
			ROS_DEBUG("%d: %s", i, names[i].c_str());
			q_temp(count) = positions[i];
			q_dot_temp(count) = velocities[i];
			count++;
		}
	}
	
	if(count == 3)
	{
		q_temp(3) = 1.0;		//lookat_link_joint
		q_temp(4) = 0.0;		//lookat_x_joint
		q_temp(5) = 0.0;		//lookat_y_joint
		q_temp(6) = 0.0;		//lookat_z_joint
		
		q_dot_temp(3) = 0.0;		//lookat_link_joint
		q_dot_temp(4) = 0.0;		//lookat_x_joint
		q_dot_temp(5) = 0.0;		//lookat_y_joint
		q_dot_temp(6) = 0.0;		//lookat_z_joint
		
		parsed = true;
	}
	
	ROS_DEBUG("Done Parsing");
	
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
