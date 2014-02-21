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
	///parse robot_description and generate KDL chains
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
	
	///get some more parameter
	//hardcoded for now -- get this from URDF later
	torso_joints_.push_back("torso_lower_neck_tilt_joint");
	torso_joints_.push_back("torso_pan_joint");
	torso_joints_.push_back("torso_upper_neck_tilt_joint");
		
	lookat_joints_.push_back("lookat_lin_joint");
	lookat_joints_.push_back("lookat_x_joint");
	lookat_joints_.push_back("lookat_y_joint");
	lookat_joints_.push_back("lookat_z_joint");
	
	torso_limits_min_.push_back(-0.25);
	torso_limits_min_.push_back(-0.12);
	torso_limits_min_.push_back(-0.37);
	torso_limits_max_.push_back(0.21);
	torso_limits_max_.push_back(0.12);
	torso_limits_max_.push_back(0.37);
	
	KDL::JntArray torso_min(torso_limits_min_.size());
	KDL::JntArray torso_max(torso_limits_max_.size());
	for(unsigned int i=0; i<torso_limits_min_.size(); i++)
	{
		torso_min(i)=torso_limits_min_[i];
		torso_max(i)=torso_limits_max_[i];
	}
	
	///initialize configuration control solver
	p_fksolver_pos_arm_ = new KDL::ChainFkSolverPos_recursive(chain_arm_);
	p_fksolver_vel_arm_ = new KDL::ChainFkSolverVel_recursive(chain_arm_);
	p_iksolver_vel_arm_ = new KDL::ChainIkSolverVel_pinv(chain_arm_, 0.001, 5);
	p_iksolver_pos_arm_ = new KDL::ChainIkSolverPos_NR(chain_arm_, *p_fksolver_pos_arm_, *p_iksolver_vel_arm_, 5, 0.001);
	p_fksolver_pos_lookat_ = new KDL::ChainFkSolverPos_recursive(chain_lookat_);
	p_fksolver_vel_lookat_ = new KDL::ChainFkSolverVel_recursive(chain_lookat_);
	p_iksolver_vel_lookat_ = new KDL::ChainIkSolverVel_pinv(chain_lookat_, 0.001, 5);
	//p_iksolver_pos_lookat_ = new KDL::ChainIkSolverPos_NR(chain_lookat_, *p_fksolver_pos_lookat_, *p_iksolver_vel_lookat_, 5, 0.001);
	p_iksolver_pos_lookat_ = new KDL::ChainIkSolverPos_NR_JL(chain_lookat_, torso_min, torso_max, *p_fksolver_pos_lookat_, *p_iksolver_vel_lookat_, 50, 0.001);
	
	
	///initialize ROS interfaces (ActionServer, Subscriber/Publisher, Services)
	torso_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh_, "/torso_controller/follow_joint_trajectory",  true);
	ROS_INFO("Wait for ActionServer...");
	torso_ac->waitForServer(ros::Duration(10.0));
	lookat_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh_, "/lookat_controller/follow_joint_trajectory",  true);
	ROS_INFO("Wait for ActionServer...");
	lookat_ac->waitForServer(ros::Duration(10.0));
	
	serv_start = nh_.advertiseService("/visual_servoing/start", &CobVisualServoingVel::start_cb, this);
	serv_stop = nh_.advertiseService("/visual_servoing/stop", &CobVisualServoingVel::stop_cb, this);
	
	js_sub = nh_.subscribe("/joint_states", 1, &CobVisualServoingVel::jointstate_cb, this);
	//pose_sub = nh_.subscribe("/lookat_pose", 1, &CobVisualServoingVel::lookat_pose_cb, this);
	
	torso_cmd_vel_pub = nh_.advertise<brics_actuator::JointVelocities>("/torso_controller/command_vel", 10);
	lookat_cmd_vel_pub = nh_.advertise<brics_actuator::JointVelocities>("/lookat_controller/command_vel", 10);
	
	
	///initialize variables and current joint values and velocities
	last_q_arm_ = KDL::JntArray(chain_arm_.getNrOfJoints());
	last_q_dot_arm_ = KDL::JntArray(chain_arm_.getNrOfJoints());
	last_q_lookat_ = KDL::JntArray(chain_lookat_.getNrOfJoints());
	last_q_dot_lookat_ = KDL::JntArray(chain_lookat_.getNrOfJoints());
	
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
		
		///Move Lookat so that lookat_focus_frame is equal to goal_focus_frame (i.e. same Pose)
		std::string goal_focus_frame = "/arm_7_link";
		while(!b_initial_focus)
		{
			bool success = initial_focus(goal_focus_frame);
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


bool CobVisualServoingVel::initial_focus(std::string goal_focus_frame)
{
	ROS_INFO("Trying to initially focus goal!");
	b_initial_focus = false;
	
	///get goal_pose
	tf::StampedTransform transform_tf;
	geometry_msgs::TransformStamped transform_msg;
	geometry_msgs::Pose pose_msg;
	try{
		m_tf_listener.lookupTransform("/base_link", goal_focus_frame, ros::Time(0), transform_tf);
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
	KDL::Frame frame_goal;
	tf::poseMsgToKDL(pose_msg, frame_goal);
	KDL::JntArray q_ik_lookat(chain_lookat_.getNrOfJoints());
	
	int ret_ik = p_iksolver_pos_lookat_->CartToJnt(q_init_lookat, frame_goal, q_ik_lookat);
	
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
	for(unsigned int i=0; i<torso_joints_.size(); i++)
	{
		torso_point.positions.push_back(q_ik_lookat(i));
	}
	torso_point.time_from_start  = ros::Duration(2.0);
	torso_goal.trajectory.points.push_back(torso_point);
	
	///move lookat (follow_joint_trajectory)
	control_msgs::FollowJointTrajectoryGoal  lookat_goal;
	lookat_goal.trajectory.header.stamp  =  ros::Time::now();
	lookat_goal.trajectory.header.frame_id  =  "base_link";
	lookat_goal.trajectory.joint_names = lookat_joints_;
	trajectory_msgs::JointTrajectoryPoint  lookat_point;
	for(unsigned int i=0; i<lookat_joints_.size(); i++)
	{
		lookat_point.positions.push_back(q_ik_lookat(i+torso_joints_.size()));
	}
	lookat_point.time_from_start  = ros::Duration(2.0);
	lookat_goal.trajectory.points.push_back(lookat_point);

	torso_ac->sendGoal(torso_goal);
	lookat_ac->sendGoal(lookat_goal);
	
	//bool finished_before_timeout = torso_ac->waitForResult(ros::Duration(5.0));
	bool finished_before_timeout = torso_ac->waitForResult(ros::Duration(5.0)) && lookat_ac->waitForResult(ros::Duration(5.0));
	
	if(finished_before_timeout)
	{
		actionlib::SimpleClientGoalState torso_state = torso_ac->getState();
		ROS_INFO("Torso Action finished: %s",torso_state.toString().c_str());
		
		actionlib::SimpleClientGoalState lookat_state = lookat_ac->getState();
		ROS_INFO("Lookat Action finished: %s",lookat_state.toString().c_str());
		
		//if(torso_state == actionlib::SimpleClientGoalState::SUCCEEDED)
		if(torso_state == actionlib::SimpleClientGoalState::SUCCEEDED && lookat_state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Successfully moved!");
			b_initial_focus = true;
			return true;
		}
		else
		{
			ROS_ERROR("Something went wrong while moving!");
			b_initial_focus = false;
			return false;
		}
	}
	else
	{
		ROS_ERROR("Not finished before timeout!");
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
		KDL::JntArray q_lookat(chain_lookat_.getNrOfJoints());
		KDL::JntArray q_dot_lookat(chain_lookat_.getNrOfJoints());
		
		bool success = parseJointStates(msg->name, msg->position, msg->velocity);
		
		q_arm = last_q_arm_;
		q_dot_arm = last_q_dot_arm_;
		q_lookat = last_q_lookat_;
		q_dot_lookat = last_q_dot_lookat_;
		
		
		if(success)
		{
			if(throttle_==10)
			{
				/////Test and Debug output
				//KDL::Frame frame_arm;
				//int test_fk_arm = p_fksolver_pos_arm_->JntToCart(q_arm, frame_arm);
				//geometry_msgs::Pose pose_arm;
				//tf::poseKDLToMsg(frame_arm, pose_arm);
				//ROS_INFO_STREAM("Current arm_config: "<<q_arm(0)<<", "<<q_arm(1)<<", "<<q_arm(2)<<", "<<q_arm(3)<<", "<<q_arm(4)<<", "<<q_arm(5)<<", "<<q_arm(6));
				//ROS_INFO_STREAM("Current arm_pose:\n" << pose_arm);
				
				
				//KDL::Frame frame_lookat;
				//int test_fk_lookat = p_fksolver_pos_lookat_->JntToCart(q_lookat, frame_lookat);
				//geometry_msgs::Pose pose_lookat;
				//tf::poseKDLToMsg(frame_lookat, pose_lookat);
				//ROS_INFO_STREAM("Current lookat_config: "<<q_lookat(0)<<", "<<q_lookat(1)<<", "<<q_lookat(2)<<", "<<q_lookat(3)<<", "<<q_lookat(4)<<", "<<q_lookat(5)<<", "<<q_lookat(6));
				//ROS_INFO_STREAM("Current lookat_pose:\n" << pose_lookat);
				
				
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
				//KDL::Twist twist_arm = KDL::Twist::Zero();
				//twist_arm.vel.x(0.20);
				KDL::JntArray q_dot_ik_lookat(chain_lookat_.getNrOfJoints());
				
				ROS_INFO("Twist_Arm Vel (%f, %f, %f)", twist_arm.vel.x(), twist_arm.vel.y(), twist_arm.vel.z());
				ROS_INFO("Twist_Arm Rot (%f, %f, %f)", twist_arm.rot.x(), twist_arm.rot.y(), twist_arm.rot.z());
				
				int ret_ik = p_iksolver_vel_lookat_->CartToJnt(last_q_lookat_, twist_arm, q_dot_ik_lookat);
				
				if(ret_ik < 0)
				{
					ROS_ERROR("No Vel-IK found!");
				}
				else
				{
					ROS_INFO_STREAM("Goal torso_velocities: "<<q_dot_ik_lookat(0)<<", "<<q_dot_ik_lookat(1)<<", "<<q_dot_ik_lookat(2));
					
					
					brics_actuator::JointVelocities torso_msg;
					torso_msg.velocities.resize(torso_joints_.size());
					for(int i=0; i<torso_joints_.size(); i++)
					{
						torso_msg.velocities[i].joint_uri = torso_joints_[i].c_str();
						torso_msg.velocities[i].unit = "rad";
						torso_msg.velocities[i].value = q_dot_ik_lookat(i);
					}
					
					brics_actuator::JointVelocities lookat_msg;
					lookat_msg.velocities.resize(lookat_joints_.size());
					for(int i=0; i<lookat_joints_.size(); i++)
					{
						lookat_msg.velocities[i].joint_uri = lookat_joints_[i].c_str();
						lookat_msg.velocities[i].unit = "rad";
						lookat_msg.velocities[i].value = q_dot_ik_lookat(i+torso_joints_.size());
					}
					
					torso_cmd_vel_pub.publish(torso_msg);
					lookat_cmd_vel_pub.publish(lookat_msg);
					
				}
				
				throttle_ = 0;
			}
			throttle_++;
		}
	}

}




/* ~~~~~~~~~~~~~~~~
 * Helper Functions 
 * ~~~~~~~~~~~~~~~~*/

bool CobVisualServoingVel::parseJointStates(std::vector<std::string> names, std::vector<double> positions, std::vector<double> velocities)
{
	KDL::JntArray q_arm_temp(7);
	KDL::JntArray q_dot_arm_temp(7);
	int count_arm = 0;
	KDL::JntArray q_torso_temp(3);
	KDL::JntArray q_dot_torso_temp(3);
	int count_torso = 0;
	KDL::JntArray q_lookat_temp(4);
	KDL::JntArray q_dot_lookat_temp(4);
	int count_lookat = 0;
	
	ROS_DEBUG("New JointState");
	
	for(unsigned int i = 0; i < names.size(); i++)
	{
		if(strncmp(names[i].c_str(), "arm_", 4) == 0)
		{
			ROS_DEBUG("%d: %s", i, names[i].c_str());
			q_arm_temp(count_arm) = positions[i];
			q_dot_arm_temp(count_arm) = velocities[i];
			count_arm++;
		}
		if(strncmp(names[i].c_str(), "torso_", 6) == 0)
		{
			ROS_DEBUG("%d: %s", i, names[i].c_str());
			q_torso_temp(count_torso) = positions[i];
			q_dot_torso_temp(count_torso) = velocities[i];
			count_torso++;
		}
		if(strncmp(names[i].c_str(), "lookat_", 7) == 0)
		{
			ROS_DEBUG("%d: %s", i, names[i].c_str());
			q_lookat_temp(count_lookat) = positions[i];
			q_dot_lookat_temp(count_lookat) = velocities[i];
			count_lookat++;
		}
	}
	
	if(count_arm == 7)
	{
		ROS_DEBUG("Done Parsing");
		last_q_arm_ = q_arm_temp;
		last_q_dot_arm_ = q_dot_arm_temp;
	}
	if(count_torso == 3)
	{
		ROS_DEBUG("Done Parsing");
		for(unsigned int i=0; i<q_torso_temp.rows(); i++)
		{
			last_q_lookat_(i) = q_torso_temp(i);
			last_q_dot_lookat_(i) = q_dot_torso_temp(i);
		}
	}
	if(count_lookat == 4)
	{
		ROS_DEBUG("Done Parsing");
		for(unsigned int i=0; i<q_lookat_temp.rows(); i++)
		{
			last_q_lookat_(i+q_torso_temp.rows()) = q_lookat_temp(i);
			last_q_dot_lookat_(i+q_torso_temp.rows()) = q_dot_lookat_temp(i);
		}
	}
	
	return true;
}









int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_visual_servoing_node");
	CobVisualServoingVel *cob_visual_servoing_server = new CobVisualServoingVel();
	
	cob_visual_servoing_server->initialize();
	cob_visual_servoing_server->run();
	
	return 0;
}
