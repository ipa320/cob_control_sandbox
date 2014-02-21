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
	my_tree.getChain("base_link", "lookat_focus_frame", chain_lookat_);
	if(chain_lookat_.getNrOfJoints() == 0)
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
	
	std::vector<double> torso_limits_min;
	std::vector<double> torso_limits_max;
	std::vector<double> torso_vel_max;
	torso_limits_min.push_back(-0.25);
	torso_limits_min.push_back(-0.12);
	torso_limits_min.push_back(-0.37);
	torso_limits_max.push_back(0.21);
	torso_limits_max.push_back(0.12);
	torso_limits_max.push_back(0.37);
	torso_vel_max.push_back(0.5);
	torso_vel_max.push_back(0.5);
	torso_vel_max.push_back(0.5);
	
	std::vector<double> lookat_limits_min;
	std::vector<double> lookat_limits_max;
	std::vector<double> lookat_vel_max;
	lookat_limits_min.push_back(-5.0);
	lookat_limits_min.push_back(-3.1415);
	lookat_limits_min.push_back(-3.1415);
	lookat_limits_min.push_back(-3.1415);
	lookat_limits_max.push_back(5.0);
	lookat_limits_max.push_back(3.1415);
	lookat_limits_max.push_back(3.1415);
	lookat_limits_max.push_back(3.1415);
	lookat_vel_max.push_back(1.0);
	lookat_vel_max.push_back(1.0);
	lookat_vel_max.push_back(1.0);
	lookat_vel_max.push_back(1.0);
	
	lookat_min_.resize(chain_lookat_.getNrOfJoints());
	lookat_max_.resize(chain_lookat_.getNrOfJoints());
	lookat_vel_max_.resize(chain_lookat_.getNrOfJoints());
	for(unsigned int i=0; i<torso_limits_min.size(); i++)
	{
		lookat_min_(i)=torso_limits_min[i];
		lookat_max_(i)=torso_limits_max[i];
		lookat_vel_max_(i)=torso_vel_max[i];
	}
	for(unsigned int i=0; i<lookat_limits_min.size(); i++)
	{
		lookat_min_(i+torso_limits_min.size())=lookat_limits_min[i];
		lookat_max_(i+torso_limits_max.size())=lookat_limits_max[i];
		lookat_vel_max_(i+torso_vel_max.size())=lookat_vel_max[i];
	}
	
	///initialize configuration control solver
	p_fksolver_pos_lookat_ = new KDL::ChainFkSolverPos_recursive(chain_lookat_);
	p_fksolver_vel_lookat_ = new KDL::ChainFkSolverVel_recursive(chain_lookat_);
	p_iksolver_vel_lookat_ = new KDL::ChainIkSolverVel_pinv(chain_lookat_, 0.001, 5);
	//p_iksolver_pos_lookat_ = new KDL::ChainIkSolverPos_NR(chain_lookat_, *p_fksolver_pos_lookat_, *p_iksolver_vel_lookat_, 5, 0.001);
	p_iksolver_pos_lookat_ = new KDL::ChainIkSolverPos_NR_JL(chain_lookat_, lookat_min_, lookat_max_, *p_fksolver_pos_lookat_, *p_iksolver_vel_lookat_, 50, 0.001);
	
	
	///initialize ROS interfaces (ActionServer, Subscriber/Publisher, Services)
	torso_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh_, "/torso_controller/follow_joint_trajectory",  true);
	ROS_INFO("Wait for ActionServer...");
	torso_ac->waitForServer(ros::Duration(10.0));
	lookat_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh_, "/lookat_controller/follow_joint_trajectory",  true);
	ROS_INFO("Wait for ActionServer...");
	lookat_ac->waitForServer(ros::Duration(10.0));
	
	serv_start = nh_.advertiseService("/visual_servoing/start", &CobVisualServoingVel::start_cb, this);
	serv_stop = nh_.advertiseService("/visual_servoing/stop", &CobVisualServoingVel::stop_cb, this);
	
	js_sub = nh_.subscribe("/joint_states", 100, &CobVisualServoingVel::jointstate_cb, this);
	
	torso_cmd_vel_pub = nh_.advertise<brics_actuator::JointVelocities>("/torso_controller/command_vel", 10);
	lookat_cmd_vel_pub = nh_.advertise<brics_actuator::JointVelocities>("/lookat_controller/command_vel", 10);
	
	
	///initialize variables and current joint values and velocities
	m_last_q_lookat = KDL::JntArray(chain_lookat_.getNrOfJoints());
	m_last_q_dot_lookat = KDL::JntArray(chain_lookat_.getNrOfJoints());
	
	m_goal_focus_frame= "/arm_7_link";		//default;
	
	b_initial_focus = false;
	b_servoing = false;
	update_frequency = 10.0;
	m_last_time_pub = ros::Time::now();
	ROS_INFO("...initialized!");
}

void CobVisualServoingVel::run()
{
	ROS_INFO("cob_visual_servoing...spinning");
	//ros::spin();
	
	bool success = false;
	ros::Rate r(update_frequency); // 10 hz
	
	while(ros::ok())
	{
		if(b_servoing)
		{
			ROS_INFO("Update Loop");
			KDL::JntArray q_lookat_goal;
			success = update_goal(q_lookat_goal);
			
			if(success)
			{
				success = update_commands(q_lookat_goal);
				
				if(!success)
				{
					ROS_ERROR("Goal Values exceed limits! Aborting...!");
					stop();
					b_initial_focus = false;
					b_servoing = false;
				}
			}
			else
			{
				ROS_ERROR("Not able to follow goal anymore! Abort Servoing!");
				stop();
				b_initial_focus = false;
				b_servoing = false;
			}
		}
		
		ros::spinOnce();
		r.sleep();
	}
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
		
		///Move Lookat so that lookat_focus_frame is equal to m_goal_focus_frame (i.e. same Pose)
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
			ros::spinOnce();
		}
		
		//wait some time to give joint_states time to update
		//ros::Duration(5.0).sleep();
		ros::spinOnce();
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
		stop();
		b_initial_focus = false;
		b_servoing = false;
	}
	return true;
}

void CobVisualServoingVel::stop()
{
	KDL::JntArray zero(chain_lookat_.getNrOfJoints());
	KDL::SetToZero(zero);
	
	send_commands(zero);
}


bool CobVisualServoingVel::initial_focus()
{
	ROS_INFO("Trying to initially focus goal!");
	b_initial_focus = false;
	
	///update goal_config
	KDL::JntArray q_lookat_goal;
	if(!update_goal(q_lookat_goal))
	{
		ROS_ERROR("No IK found!");
		return false;
	}
	
	///move torso (follow_joint_trajectory)
	control_msgs::FollowJointTrajectoryGoal  torso_goal;
	torso_goal.trajectory.header.stamp  =  ros::Time::now();
	torso_goal.trajectory.header.frame_id  =  "base_link";
	torso_goal.trajectory.joint_names = torso_joints_;
	trajectory_msgs::JointTrajectoryPoint  torso_point;
	for(unsigned int i=0; i<torso_joints_.size(); i++)
	{
		torso_point.positions.push_back(q_lookat_goal(i));
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
		lookat_point.positions.push_back(q_lookat_goal(i+torso_joints_.size()));
	}
	lookat_point.time_from_start  = ros::Duration(2.0);
	lookat_goal.trajectory.points.push_back(lookat_point);

	torso_ac->sendGoal(torso_goal);
	lookat_ac->sendGoal(lookat_goal);
	
	///evaluate result
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



bool CobVisualServoingVel::update_goal(KDL::JntArray& q_lookat_goal)
{
	///get goal_pose
	tf::StampedTransform transform_tf;
	geometry_msgs::TransformStamped transform_msg;
	geometry_msgs::Pose pose_msg;
	try{
		m_tf_listener.lookupTransform("/base_link", m_goal_focus_frame, ros::Time(0), transform_tf);
		transformStampedTFToMsg(transform_tf, transform_msg);
		ROS_DEBUG_STREAM("Current Position of Goal:\n" << transform_msg);
		pose_msg.position.x = transform_msg.transform.translation.x;
		pose_msg.position.y = transform_msg.transform.translation.y;
		pose_msg.position.z = transform_msg.transform.translation.z;
		pose_msg.orientation = transform_msg.transform.rotation;
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ROS_ERROR("Cannot lookupTransform for current Goal");
		return false;
	}
	
	///calculate IK for lookat chain (pos)
	KDL::JntArray q_lookat_init(chain_lookat_.getNrOfJoints());
	KDL::Frame frame_goal;
	tf::poseMsgToKDL(pose_msg, frame_goal);
	q_lookat_goal.resize(chain_lookat_.getNrOfJoints());
	
	int ret_ik = p_iksolver_pos_lookat_->CartToJnt(q_lookat_init, frame_goal, q_lookat_goal);
	
	if(ret_ik < 0)
	{
		ROS_ERROR("No IK-Solution found! Unable to focus goal!");
		return false;
	}
	else
	{
		ROS_INFO("Goal Config: (%f, %f, %f, %f, %f, %f, %f)", q_lookat_goal(0), q_lookat_goal(1), q_lookat_goal(2), q_lookat_goal(3), q_lookat_goal(4), q_lookat_goal(5), q_lookat_goal(6));
		return true;
	}
}


bool CobVisualServoingVel::update_commands(KDL::JntArray& q_lookat_goal)
{
	///safety check: joint limits:
	for(unsigned int i=0; i<q_lookat_goal.rows(); i++)
	{
		if(q_lookat_goal(i) < lookat_min_(i) || q_lookat_goal(i) > lookat_max_(i))
		{
			ROS_ERROR("Goal value %d exceeds pos limits: %f", i, q_lookat_goal(i));
			return false;
		}
	}
	
	///calculate goal velocities
	double delta_t = ros::Time::now().toSec() - m_last_time_pub.toSec();
	m_last_time_pub = ros::Time::now();  
	
	ROS_INFO("Current Config: (%f, %f, %f, %f, %f, %f, %f)", m_last_q_lookat(0), m_last_q_lookat(1), m_last_q_lookat(2), m_last_q_lookat(3), m_last_q_lookat(4), m_last_q_lookat(5), m_last_q_lookat(6));
	
	KDL::JntArray q_lookat_diff(chain_lookat_.getNrOfJoints());
	KDL::Subtract(q_lookat_goal, m_last_q_lookat, q_lookat_diff);
	
	ROS_INFO("Diff Config: (%f, %f, %f, %f, %f, %f, %f)", q_lookat_diff(0), q_lookat_diff(1), q_lookat_diff(2), q_lookat_diff(3), q_lookat_diff(4), q_lookat_diff(5), q_lookat_diff(6));
	
	KDL::JntArray q_dot_lookat(chain_lookat_.getNrOfJoints());
	KDL::Divide(q_lookat_diff, delta_t, q_dot_lookat);
	
	ROS_INFO("Vel Command: (%f, %f, %f, %f, %f, %f, %f)", q_dot_lookat(0), q_dot_lookat(1), q_dot_lookat(2), q_dot_lookat(3), q_dot_lookat(4), q_dot_lookat(5), q_dot_lookat(6));
	
	
	///safety check: velocity limits
	for(unsigned int i=0; i<q_dot_lookat.rows(); i++)
	{
		if(std::fabs(q_dot_lookat(i)) > lookat_vel_max_(i))
		{
			ROS_ERROR("Goal value %d exceeds vel limits: %f", i, q_dot_lookat(i));
			return false;
		}
	}
	
	
	///send commands
	send_commands(q_dot_lookat);
	
	return true;
}




/* ~~~~~~~~~~~~~~~~
 * Helper Functions 
 * ~~~~~~~~~~~~~~~~*/
void CobVisualServoingVel::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
	KDL::JntArray q_torso_temp(3);
	KDL::JntArray q_dot_torso_temp(3);
	int count_torso = 0;
	KDL::JntArray q_lookat_temp(4);
	KDL::JntArray q_dot_lookat_temp(4);
	int count_lookat = 0;
	
	for(unsigned int i = 0; i < msg->name.size(); i++)
	{
		if(strncmp(msg->name[i].c_str(), "torso_", 6) == 0)
		{
			ROS_DEBUG("%s: %f", msg->name[i].c_str(), msg->position[i]);
			q_torso_temp(count_torso) = msg->position[i];
			q_dot_torso_temp(count_torso) = msg->velocity[i];
			count_torso++;
		}
		if(strncmp(msg->name[i].c_str(), "lookat_", 7) == 0)
		{
			ROS_DEBUG("%s: %f", msg->name[i].c_str(), msg->position[i]);
			q_lookat_temp(count_lookat) = msg->position[i];
			q_dot_lookat_temp(count_lookat) = msg->velocity[i];
			count_lookat++;
		}
	}
	
	if(count_torso == 3)
	{
		ROS_DEBUG("Done Parsing");
		for(unsigned int i=0; i<q_torso_temp.rows(); i++)
		{
			m_last_q_lookat(i) = q_torso_temp(i);
			m_last_q_dot_lookat(i) = q_dot_torso_temp(i);
		}
	}
	if(count_lookat == 4)
	{
		ROS_DEBUG("Done Parsing");
		for(unsigned int i=0; i<q_lookat_temp.rows(); i++)
		{
			m_last_q_lookat(i+q_torso_temp.rows()) = q_lookat_temp(i);
			m_last_q_dot_lookat(i+q_torso_temp.rows()) = q_dot_lookat_temp(i);
		}
	}
}



void CobVisualServoingVel::send_commands(KDL::JntArray goal_velocities)
{
	if(goal_velocities.rows() != chain_lookat_.getNrOfJoints())
	{
		ROS_ERROR("DoF do not match! Not sending velocities!");
		return;
	}
	
	///send velocities
	brics_actuator::JointVelocities torso_msg;
	torso_msg.velocities.resize(torso_joints_.size());
	for(int i=0; i<torso_joints_.size(); i++)
	{
		torso_msg.velocities[i].joint_uri = torso_joints_[i].c_str();
		torso_msg.velocities[i].unit = "rad";
		torso_msg.velocities[i].value = goal_velocities(i);
	}
	
	brics_actuator::JointVelocities lookat_msg;
	lookat_msg.velocities.resize(lookat_joints_.size());
	for(int i=0; i<lookat_joints_.size(); i++)
	{
		lookat_msg.velocities[i].joint_uri = lookat_joints_[i].c_str();
		lookat_msg.velocities[i].unit = "rad";
		lookat_msg.velocities[i].value = goal_velocities(i+torso_joints_.size());
	}
	
	torso_cmd_vel_pub.publish(torso_msg);
	lookat_cmd_vel_pub.publish(lookat_msg);
}









int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_visual_servoing_node");
	CobVisualServoingVel *cob_visual_servoing_server = new CobVisualServoingVel();
	
	cob_visual_servoing_server->initialize();
	cob_visual_servoing_server->run();
	
	return 0;
}
