#include "ros/ros.h"

#include <cob_mmcontroller/cob_config_controller.h>


cob_config_controller::cob_config_controller()
{
    started = false;
    RunSyncMM = false;
    //parsing urdf for KDL chain
    KDL::Tree my_tree;
    ros::NodeHandle node;
    double base_to_arm_ratio_ = 0.1;
    node.param("/arm_controller/arm_mmcontroller_node/arm_base", arm_base_name_, std::string("arm_0_link"));
    node.param("/arm_controller/arm_mmcontroller_node/arm_end_effector", arm_ee_name_, std::string("arm_7_link"));
    node.param("/arm_controller/arm_mmcontroller_node/default_control_mode", kinematic_mode_, std::string("arm_base"));
    node.param("/arm_controller/arm_mmcontroller_node/base_to_arm_ratio", base_to_arm_ratio_, 0.1);
    ROS_INFO("Starting node with arm_end_effector: %s; default_control_mode: %s", arm_ee_name_.c_str(), kinematic_mode_.c_str());

    std::string robot_desc_string;
    node.param("/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }
    my_tree.getChain("base_link",arm_ee_name_, arm_base_chain);
    if(arm_base_chain.getNrOfJoints() == 0)
    {
        ROS_ERROR("Specified arm_end_effector doesn't seem to exist in urdf description");
        exit(0);
    }
    my_tree.getChain(arm_base_name_,arm_ee_name_, arm_chain);

    //Initializing configuration control solver
    iksolver1v = new augmented_solver(arm_base_chain);//Inverse velocity solver
    fksolver1 = new KDL::ChainFkSolverPos_recursive(arm_base_chain);
    fksolver1_vel = new KDL::ChainFkSolverVel_recursive(arm_base_chain);
    iksolver1v->setBaseToArmFactor(base_to_arm_ratio_);

    //Initializing communication
    sub = n.subscribe("/joint_states", 1, &cob_config_controller::controllerStateCallback, this);
    cart_vel_sub = n.subscribe("/arm_controller/cart_command", 1, &cob_config_controller::cartTwistCallback, this);
    plat_odom_sub = n.subscribe("/base_controller/odometry", 1, &cob_config_controller::baseTwistCallback, this);

    ROS_INFO("Creating publishers");
    arm_pub_ = n.advertise<brics_actuator::JointVelocities>("/arm_controller/command_vel",1);
    base_pub_ = n.advertise<geometry_msgs::Twist>("/base_controller/command_direct",1);
    debug_cart_pub_ = n.advertise<geometry_msgs::PoseArray>("/arm_controller/debug/cart",1);
    cart_position_pub_ = n.advertise<geometry_msgs::PoseStamped>("/arm_controller/cart_state",1);
    cart_twist_pub_ = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_twist_state",1);

    serv_start = n.advertiseService("/mm/start", &cob_config_controller::SyncMMTriggerStart, this);
    serv_stop = n.advertiseService("/mm/stop", &cob_config_controller::SyncMMTriggerStop, this);

    ROS_INFO("Running cartesian velocity controller.");
    zeroCounter = 0;
    zeroCounter_base = 0;
    zeroCounterTwist = 0;
}


KDL::JntArray cob_config_controller::parseJointStates(std::vector<std::string> names, std::vector<double> positions, std::vector<double> velocities, KDL::JntArray& q, KDL::JntArray& q_dot)
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
            parsed = true;
        }
    }
    if(!parsed)
        return q_last;
    
    q_last = q_temp;
    //ROS_INFO("CurrentConfig: %f %f %f %f %f %f %f", q_temp(0), q_temp(1), q_temp(2), q_temp(3), q_temp(4), q_temp(5), q_temp(6));
    if(!started)
    {
        KDL::JntArray zero(7);
        sendVel(zero,zero,zero);
        VirtualQ = q_temp;
        started = true;
        last = ros::Time::now();

        ROS_INFO("Starting up controller with first configuration");
        std::cout << VirtualQ(0) << "\n";
    }
    q = q_temp;
    q_dot = q_dot_temp;
    
    return q_temp;
}

void cob_config_controller::cartTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    extTwist.vel.x(msg->linear.x);
    extTwist.vel.y(msg->linear.y);
    extTwist.vel.z(msg->linear.z);

    extTwist.rot.x(msg->angular.x);
    extTwist.rot.y(msg->angular.y);
    extTwist.rot.z(msg->angular.z);
}

void cob_config_controller::baseTwistCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf::poseMsgToKDL(msg->pose.pose, base_odom_);
    tf::twistMsgToKDL(msg->twist.twist, base_twist_);
}


bool cob_config_controller::SyncMMTriggerStart(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
    ros::ServiceClient client = n.serviceClient<cob_srvs::Trigger>("/arm_controller/reset_brics_interface");
    cob_srvs::Trigger srv;
    client.call(srv);
    if(RunSyncMM)
    {
        ROS_INFO("Already started");
    }    
    else
    {
        ROS_INFO("Starting MM interface");
        started = false;
        RunSyncMM = true;
    }    
    return true;
}
bool cob_config_controller::SyncMMTriggerStop(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
    if(RunSyncMM)
    {
        ROS_INFO("Stopping MM interface");
        RunSyncMM = false;
    }    
    else
    {
        ROS_INFO("Already stopped");
    }    
    return true;
}

void cob_config_controller::sendVel(KDL::JntArray q_t, KDL::JntArray q_dot, KDL::JntArray q_dot_base)
{
    ros::Time now = ros::Time::now();
    double dt = now.toSec() - last.toSec();
    last = now;
    double horizon = 3.0*dt;

    brics_actuator::JointVelocities target_joint_vel;
    target_joint_vel.velocities.resize(7);
    bool nonzero = false;
    for(unsigned int i=0; i<7; i++)
    {
        std::stringstream joint_name;
        joint_name << "arm_" << (i+1) << "_joint";
        target_joint_vel.velocities[i].joint_uri = joint_name.str();
        target_joint_vel.velocities[i].unit = "rad";
        if(q_dot(i) != 0.0)
        {
            target_joint_vel.velocities[i].value = q_dot(i);
            nonzero = true;
            zeroCounter = 0;
        }
        else
        {
            target_joint_vel.velocities[i].value = 0.0;
        }

    }
    if(zeroCounter <= 4)
    {
        zeroCounter++;
        if(!nonzero)
            std::cout << "Sending additional zero\n";
        nonzero = true;
    }
    if(!started)
        nonzero = true;
    if(nonzero)
    {
        arm_pub_.publish(target_joint_vel);
    }

    //send to base
    bool nonzero_base = false;
    geometry_msgs::Twist cmd;
    if(q_dot_base(0) != 0.0 || q_dot_base(1) != 0.0 || q_dot_base(2) != 0.0)
    {
        cmd.linear.x = q_dot_base(0);
        cmd.linear.y = q_dot_base(1);
        cmd.angular.z = q_dot_base(2);
        nonzero_base = true;
        zeroCounter_base = 0;
    }
    else
    {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
    }
    if(zeroCounter_base <= 4)
    {
        zeroCounter_base++;
        if(!nonzero_base)
            std::cout << "Sending additional zero twist to base\n";
        nonzero_base = true;
    }
    if(!started)
        nonzero = true;
    if(nonzero_base)
    {
        base_pub_.publish(cmd);
    }
}

void cob_config_controller::sendCartPose()
{
    KDL::Frame F_current;
    //KDL::Frame F_test;
    F_current = base_odom_ * arm_pose_;
    //std::cout << "Test fwcalc: " << F_current.p.x() << ", " << F_current.p.y() << " | " << arm_pose_.p.x() + base_odom_.p.x() << ", " << arm_pose_.p.y() + base_odom_.p.y() << "\n";
    //F_current.p.x(arm_pose_.p.x() + base_odom_.p.x());
    //F_current.p.y(arm_pose_.p.y() + base_odom_.p.y());
    geometry_msgs::PoseStamped pose;
    tf::poseKDLToMsg(F_current, pose.pose);
    pose.header.stamp = ros::Time::now();
    cart_position_pub_.publish(pose);

    geometry_msgs::Twist twist;
    KDL::Twist twist_sum = arm_vel_.GetTwist() + base_twist_;
    tf::twistKDLToMsg(twist_sum, twist);
    cart_twist_pub_.publish(twist);
}

void cob_config_controller::controllerStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_out(7);
    KDL::JntArray q_dot_base(3);
    std::vector<std::string> names = msg->name;
    std::vector<double> positions = msg->position;
    std::vector<double> velocities = msg->velocity;
    q = parseJointStates(names,positions,velocities,q,q_dot);
    fksolver1->JntToCart(q, arm_pose_);
    KDL::JntArrayVel q_dot_array(q,q_dot);
    fksolver1_vel->JntToCart(q_dot_array, arm_vel_); 
    sendCartPose();
    if(RunSyncMM)
    {
        if(extTwist.vel.x() != 0.0 || extTwist.vel.y() != 0.0 || extTwist.vel.z() != 0.0 || extTwist.rot.x() != 0.0 || extTwist.rot.y() != 0.0 || extTwist.rot.z() != 0.0)
        {
            zeroCounterTwist = 0;
            int ret = iksolver1v->CartToJnt(q, extTwist, q_out, q_dot_base);
            if(ret >= 0)
            {
                sendVel(q, q_out, q_dot_base);
                //std::cout << q_out(0) << " " << q_out(1) << " " << q_out(2) << " " << q_out(3) << " " << q_out(4) << " " << q_out(5) << " " << q_out(6)  << "\n";
            }    
            else
                std::cout << "Something went wrong" << "\n";
        }
        else
        {
            if(zeroCounterTwist >= 4)
            {
                int ret = iksolver1v->CartToJnt(q, extTwist, q_out, q_dot_base);
                if(ret >= 0)
                {
                    sendVel(q, q_out, q_dot_base);
                    //std::cout << q_out(0) << " " << q_out(1) << " " << q_out(2) << " " << q_out(3) << " " << q_out(4) << " " << q_out(5) << " " << q_out(6)  << "\n";
                }    
                else
                    std::cout << "Something went wrong" << "\n";
            }
            zeroCounterTwist++;
        }    
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_mm_controller");
    cob_config_controller cob_config_controller;
    ros::spin();

    return 0;
}
