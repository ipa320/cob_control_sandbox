#include <cob_articulation_controller/cob_cartesian_trajectories_PID.h>

cob_cartesian_trajectories::cob_cartesian_trajectories() : as_model_(n, "moveModel", boost::bind(&cob_cartesian_trajectories::moveModelActionCB, this, _1), false)
{
    ros::NodeHandle node;
    node.param("cob_cartesian_trajectories_PID/p_gain", p_gain_, 1.0);
    node.param("cob_cartesian_trajectories_PID/i_gain", i_gain_, 0.002);
    node.param("cob_cartesian_trajectories_PID/d_gain", d_gain_, 0.0);
    ROS_INFO("Starting PID controller with P: %e, I: %e, D: %e", p_gain_, i_gain_, d_gain_);
    
    cart_state_sub_ = n.subscribe("/arm_controller/cart_state", 1, &cob_cartesian_trajectories::cartStateCallback, this);
    cart_twist_state_sub_ = n.subscribe("/arm_controller/cart_twist_state", 1, &cob_cartesian_trajectories::cartTwistStateCallback, this);
    joint_state_sub_ = n.subscribe("/joint_states", 1, &cob_cartesian_trajectories::jointStateCallback, this);
    cart_command_pub = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_command",1);
    debug_cart_pub_ = n.advertise<geometry_msgs::PoseArray>("/mm/debug",1);
    serv_prismatic = n.advertiseService("/mm/move_pri", &cob_cartesian_trajectories::movePriCB, this);      // new service
    serv_rotational = n.advertiseService("/mm/move_rot", &cob_cartesian_trajectories::moveRotCB, this);     // new service
    serv_model = n.advertiseService("/mm/move_model", &cob_cartesian_trajectories::moveModelCB, this);       // new service to work with models
    map_pub_ = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    twist_pub_ = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);   // publish twist to be visualized inj rviz
    track_pub_ = n.advertise<cob_articulation_controller::TrackMsg>("/track", 1);       // publish generated trajectory for debugging
    model_pub_ = n.advertise<cob_articulation_controller::ModelMsg>("/model", 1);       // publish given model for debugging
    bRun = false;
    as_model_.start();
    targetDuration = 0;
    currentDuration = 0;
    mode = "prismatic";     // prismatic, rotational, trajectory, model
   
    pub_timer = ros::Time::now();
    pub_counter = 0;
    double const PI = 4.0*std::atan(1.0);

    getJointLimits(UpperLimits, LowerLimits);   // get arm joint limits from topic /robot_description

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
        
    bHandle = false;
    debug = true;

}


// to avoid reaching the arm joint limits
// get arm joint limits from topic /robot_description
void cob_cartesian_trajectories::getJointLimits(std::vector<double> &UpperLimits, std::vector<double> &LowerLimits)
{
    ros::NodeHandle param_node;
    const unsigned int DOF = 7;
    std::vector<std::string> JointNames;
    std::string param_name = "robot_description";
    std::string full_param_name;
    std::string xml_string;

    for (unsigned int i = 1; i <= DOF; i++)
    {
        stringstream ss;
        ss << i;
        JointNames.push_back("arm_" + ss.str() + "_joint");
    }

    param_node.searchParam(param_name, full_param_name);
    if (param_node.hasParam(full_param_name))
    {
        param_node.getParam(full_param_name.c_str(), xml_string);
        //std::cout << "Parameter name: " << full_param_name << "\n";
    }

    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
        param_node.shutdown();
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
        param_node.shutdown();
    }
    ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

    /// Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        param_node.shutdown();
    }
    ROS_DEBUG("Successfully parsed urdf file");

    /// Get lower limits out of urdf model
    for (unsigned int i = 0; i < DOF; i++)
    {
        LowerLimits.push_back(model.getJoint(JointNames[i].c_str())->limits->lower);
    }

    // Get upper limits out of urdf model
    for (unsigned int i = 0; i < DOF; i++)
    {
        UpperLimits.push_back(model.getJoint(JointNames[i].c_str())->limits->upper);
    }
    param_node.shutdown();
}

// get arm joint states 
std::vector<double> cob_cartesian_trajectories::parseJointStates(std::vector<std::string> names, std::vector<double> positions)
{
    std::vector<double> q_temp(7);
    bool parsed = false;
    unsigned int count = 0;
    for(unsigned int i = 0; i < names.size(); i++)
    {
            if(strncmp(names[i].c_str(), "arm_", 4) == 0)
            {
                q_temp[count] = positions[i];
                count++;
                parsed = true;
            }
    }

    if(!parsed)
        return q_last;

    q_last = q_temp;
    return q_temp;

}

// check if an arm joint limit is reached
void cob_cartesian_trajectories::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (bRun)
    {
        std::vector<std::string> names = msg->name;
        std::vector<double> positions = msg->position;
        jointStates = parseJointStates(names,positions);

        // stopping to run trajectory 
        for (unsigned int i = 0; i < jointStates.size(); i++)
        {
            if (jointStates[i] <= (LowerLimits[i] + 0.04))
            {
                ROS_INFO("Stopping trajectory because arm joint %d reached almost lower joint limit!", i+1);
                result_.exit_code = 2;
                stopTrajectory();
                //std::cout << "Stopping trajectory because arm joint " << i+1 << " reached almost lower joint limit!" << "\n";
            }
            else if (jointStates[i] >= (UpperLimits[i] - 0.04))
            {
                ROS_INFO("Stopping trajectory because arm joint %d reached almost upper joint limit!", i+1);
                result_.exit_code = 2;
                stopTrajectory();
                //std::cout << "Stopping trajectory because arm joint " << i+1 << " reached almost upper joint limit!" << "\n";
            }
            //std::cout << "arm_" << i+1 << "_joint is " << jointStates[i] << " >> " << -1.0 *(fabs(LowerLimits[i])+jointStates[i]) << " to lower and " << UpperLimits[i]-jointStates[i] << " to upper limit" << "\n";
        }
    }

}


// action for model 
void cob_cartesian_trajectories::moveModelActionCB(const cob_articulation_controller::ArticulationModelGoalConstPtr& goal)
{
    cob_articulation_controller::ModelMsg pub_model;
    //tf broadcaster 
    tf::Transform transform_articulation;

    mode = goal->model.name;
    std::cout << "Mode:" << mode << "\n";
    targetDuration = goal->target_duration.toSec();
    params = goal->model.params;

    //set up articulation frame
    if (mode == "rotational")
    {
        transform_articulation.setOrigin( tf::Vector3(getParamValue("rot_center.x"), getParamValue("rot_center.y"), getParamValue("rot_center.z")) );
        transform_articulation.setRotation( tf::Quaternion(getParamValue("rot_axis.x"), getParamValue("rot_axis.y"), getParamValue("rot_axis.z"), getParamValue("rot_axis.w")) );
    }
    else
    {
        transform_articulation.setOrigin( tf::Vector3(getParamValue("rigid_position.x"), getParamValue("rigid_position.y"), getParamValue("rigid_position.z")) );
        transform_articulation.setRotation( tf::Quaternion(getParamValue("rigid_orientation.x"), getParamValue("rigid_orientation.y"), getParamValue("rigid_orientation.z"), getParamValue("rigid_orientation.w")) );
    }
    //publish articulation frame
    br.sendTransform(tf::StampedTransform(transform_articulation, ros::Time::now(), "/map", "/articulation_center"));

    if(start())
    {
        while(bRun)
        {
            //wait until finished
            //publish articulation frame TODO: necessary
            br.sendTransform(tf::StampedTransform(transform_articulation, ros::Time::now(), "/map", "/articulation_center"));
        
            //publish model
            pub_model = goal->model;
            pub_model.header.stamp = ros::Time::now();
            pub_model.header.frame_id = "/map";

            model_pub_.publish(pub_model);
            //publish feedback
            feedback_.time_left = targetDuration - currentDuration;
            as_model_.publishFeedback(feedback_);

            sleep(1);
        }
        if (result_.exit_code != 0)
            as_model_.setAborted(result_);
        else
            as_model_.setSucceeded(result_);
    }
    return;
}

bool cob_cartesian_trajectories::movePriCB(cob_articulation_controller::MovePrismatic::Request& request, cob_articulation_controller::MovePrismatic::Response& response)    //TODO // prismatic callback
{
    mode = "prismatic";
    targetDuration = request.target_duration.toSec();
    params = request.params;
    std::cout << targetDuration << "\n";
    return start();
}

bool cob_cartesian_trajectories::moveRotCB(cob_articulation_controller::MoveRotational::Request& request, cob_articulation_controller::MoveRotational::Response& response)    //TODO // rotational callback
{
    mode = "rotational";
    targetDuration = request.target_duration.toSec();
    params = request.params;
    std::cout << targetDuration << "\n";
    return start();
}

/*bool cob_cartesian_trajectories::moveTrajCB(cob_articulation_controller::MoveTrajectory::Request& request, cob_articulation_controller::MoveTrajectory::Response& response)   // TODO // trajectory callback
{
    mode = "trajectory";
    targetDuration = request.target_duration.toSec();
    track = request.pose
    return start();
}*/

bool cob_cartesian_trajectories::moveModelCB(cob_articulation_controller::MoveModel::Request& request, cob_articulation_controller::MoveModel::Response& response)  //TODO // model callback
{
    mode = "model";
    targetDuration = request.target_duration.toSec();
    //model_params = request.model.params            // model parameters
    track = request.model.track.pose_projected;         // trajectory
    return start();
}

bool cob_cartesian_trajectories::start() //TODO request->model.params // start 
{
    if(bRun)
    {
        ROS_ERROR("Already running trajectory");
        return false;
    }
    else
    {
        bRun = true;
        bStarted = false;
        timer = ros::Time::now();
        tstart = ros::Time::now();
        currentDuration = 0;
        trajectory_points.clear();
        result_.exit_code = 1;
        Error_last = Twist::Zero();
        Error_last2 = Twist::Zero();
        return true;
    }    
}

void cob_cartesian_trajectories::cartTwistStateCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    geometry_msgs::Twist aux_twist;
    aux_twist.linear = msg->linear;
    aux_twist.angular = msg->angular;
    KDL::Twist aux_twist_KDL;
    tf::twistMsgToKDL(aux_twist, aux_twist_KDL);
    vec_vel_ist.push_back(aux_twist_KDL);
}


//Pose is global pose with odometry
void cob_cartesian_trajectories::cartStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(bRun)
    {
        ros::Duration dt = ros::Time::now() - timer;
        cout << "\n" << "===================================" << "\n\n" << "dt: " << dt << "\n";
        timer = ros::Time::now();
        if((targetDuration-currentDuration) <= 0)
        {
            geometry_msgs::Twist twist;
            cart_command_pub.publish(twist);
            ROS_INFO("finished trajectory in %f", ros::Time::now().toSec() - tstart.toSec());
            result_.exit_code = 0;
            stopTrajectory();
            return;
        }
        KDL::Frame current;
        KDL::Frame myhinge;
        tf::poseMsgToKDL(msg->pose, current);
        tf::poseMsgToKDL(current_hinge.pose, myhinge);
        KDL::Vector unitz = myhinge.M.UnitZ();
        std::cout << "Radius because of Hinge: " << (myhinge.p - current.p) << "UnitZ of hinge: " << unitz.z() << "\n";

        // get twist to be published
        geometry_msgs::Twist twist;
        twist = getTwist(currentDuration, current); 

        // publish twist
        cart_command_pub.publish(twist);

        // add needed time
        currentDuration+=dt.toSec();

        // add position to be visualized in rviz
        geometry_msgs::Point p;
        p.x = msg->pose.position.x;
        p.y = msg->pose.position.y;
        p.z = msg->pose.position.z;
        trajectory_points.push_back(p);
    }
    else
    {
        //publish zero    
        geometry_msgs::Twist twist;
        cart_command_pub.publish(twist);
    }
}

void cob_cartesian_trajectories::stopTrajectory()
{
    geometry_msgs::Twist twist;
    cart_command_pub.publish(twist);
    bRun = false;
    bStarted = false;
    sendMarkers();
    Error = Twist::Zero();
    Error_sum = Twist::Zero();
    Error_dot = Twist::Zero();

    //print error vector
    std::cout << "error vector:" << "\n";
    for(std::vector<KDL::Twist>::iterator it = vec_err_p.begin(); it != vec_err_p.end(); ++it) {
        std::cout << *it << ", ";
    }
    std::cout << "\n\n\n\n\n\n\n\n";
    sleep(2);

    //print error vector
    std::cout << "tb error vector:" << "\n";
    for(std::vector<KDL::Twist>::iterator it = vec_err_tb.begin(); it != vec_err_tb.end(); ++it) {
        std::cout << *it << ", ";
    }
    std::cout << "\n\n\n\n\n\n\n\n";
    sleep(2);

    //print error vector
    std::cout << "vel ist vector:" << "\n";
    for(std::vector<KDL::Twist>::iterator it = vec_vel_ist.begin(); it != vec_vel_ist.end(); ++it) {
        std::cout << *it << ", ";
    }
    std::cout << "\n\n\n\n\n\n\n\n";
    sleep(2);

    ////print error vector
    //std::cout << "vel ist vector:" << "\n";
    //for(std::vector<KDL::Twist>::iterator it = vec_vel_ist.begin(); it != vec_vel_ist.end(); ++it) {
    //    std::cout << *it << ", ";
    //}
    //std::cout << "\n\n\n\n\n\n\n\n";
    //sleep(2);

    //print error vector
    std::cout << "pos ist vector:" << "\n";
    for(std::vector<KDL::Vector>::iterator it = vec_pos_ist.begin(); it != vec_pos_ist.end(); ++it) {
        std::cout << *it << ", ";
    }
    std::cout << "\n\n\n\n\n\n\n\n";
    sleep(2);

    //print error vector
    std::cout << "pos soll vector:" << "\n";
    for(std::vector<KDL::Vector>::iterator it = vec_pos_soll.begin(); it != vec_pos_soll.end(); ++it) {
        std::cout << *it << ", ";
    }
    std::cout << "\n\n\n\n\n\n\n\n";
    sleep(2);
}

geometry_msgs::Twist cob_cartesian_trajectories::getTwist(double dt, Frame F_current)
{
    KDL::Frame F_target;
    KDL::Frame F_diff;
    geometry_msgs::Twist ControllTwist;
    double start_roll, start_pitch, start_yaw = 0.0;
    double current_roll, current_pitch, current_yaw = 0.0;
    double target_roll, target_pitch, target_yaw = 0.0;

    if(!bStarted)
    {
        F_EE_start = F_current;
        F_last_ist = F_current;
        F_last_soll = F_current;
        F_EE_start.M.GetRPY(last_rpy_angles["target_roll"], last_rpy_angles["target_pitch"], last_rpy_angles["target_yaw"]);
        F_EE_start.M.GetRPY(last_rpy_angles["current_roll"], last_rpy_angles["current_pitch"], last_rpy_angles["current_yaw"]);
        bStarted = true;
        bHandle = true;
    }
    
    getTargetPosition(dt, F_target);
   
    F_EE_start.M.GetRPY(start_roll, start_pitch, start_yaw);
    F_current.M.GetRPY(current_roll, current_pitch, current_yaw);
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);

    //std::cout << "AngleDiff: " << (current_yaw-start_yaw) << " vs " << (soll_angle) << " error: " << (soll_angle-current_yaw-start_yaw) << "\n";

    
    std::cout << "Target (x,y):  " << F_target.p.x() << ", " << F_target.p.y() << "\n";
    std::cout << "Error (x,y):   " << F_target.p.x()-F_current.p.x() << ", " << F_target.p.y()-F_current.p.y() << "\n";
    std::cout << "Start (x,y):   " << F_EE_start.p.x() << ", " << F_EE_start.p.y() << "\n";
    std::cout << "Current (x,y): " << F_current.p.x() << ", " << F_current.p.y() << "\n";

    // calling PIDController to calculate the actuating variable and get back twist
    ControllTwist = PIDController(dt, F_target, F_current);
    
    //DEBUG
    F_diff.p.x(F_target.p.x()-F_current.p.x());
    F_diff.p.y(F_target.p.y()-F_current.p.y());
    F_diff.p.z(F_target.p.z()-F_current.p.z());
    geometry_msgs::PoseArray poses;
    poses.poses.resize(3);
    tf::poseKDLToMsg(F_current, poses.poses[0]);
    tf::poseKDLToMsg(F_target, poses.poses[1]);
    tf::poseKDLToMsg(F_diff, poses.poses[2]);
    debug_cart_pub_.publish(poses);
    //

    return ControllTwist;
}

void cob_cartesian_trajectories::getTargetPosition(double dt, KDL::Frame &F_target)    //TODO //
{
    if (mode == "prismatic")
        getPriTarget(dt, F_target);
    else if (mode == "rotational")
        getRotTarget(dt, F_target);
    /*else if (mode == "trajectory");
        getTrajTarget(dt, F_target);
    else if (mode == "model");
        getModelTarget(dt, F_target);*/
    else
    {
        ROS_ERROR("Invalid mode");
        F_target = F_EE_start;
    }
}

// linear trajectory 
void cob_cartesian_trajectories::getPriTarget(double dt, KDL::Frame &F_target)
{
    double length;
    double partial_length;

    KDL::Frame F_track;

    // get start frame of trajectory
    if (bHandle)
        getPriStart(F_track_start);
    
    length = getParamValue("action");
    
    partial_length = length * (1. - cos(PI*(dt/(targetDuration-1.))))/2.;
    if (dt > (targetDuration-1.))
            partial_length = length;

    //partial_length = length * (dt/targetDuration);
    
    std::cout << "partial length: " << partial_length << "\n";
    
    // calculate F_track
    F_track.p.z(partial_length);

    F_target.p = F_track_start*F_track.p;
    F_target.M = F_EE_start.M; 

    // tf transform F_track_start
    tf::Transform transform_track_start;
    geometry_msgs::Transform transform_track_start_msg;
    tf::transformKDLToMsg(F_track_start, transform_track_start_msg);
    tf::transformMsgToTF(transform_track_start_msg, transform_track_start);
    br.sendTransform(tf::StampedTransform(transform_track_start, ros::Time::now(), "/map", "/track_start"));
    
    // tf transform F_track
    tf::Transform transform_track;
    geometry_msgs::Transform transform_track_msg;
    tf::transformKDLToMsg(F_track, transform_track_msg);
    tf::transformMsgToTF(transform_track_msg, transform_track);
    br.sendTransform(tf::StampedTransform(transform_track, ros::Time::now(), "/track_start", "/track"));

    // tf transform F_target
    tf::Transform transform_target;
    geometry_msgs::Transform transform_target_msg;
    tf::transformKDLToMsg(F_target, transform_target_msg);
    tf::transformMsgToTF(transform_target_msg, transform_target);
    br.sendTransform(tf::StampedTransform(transform_target, ros::Time::now(), "/map", "/target"));
    
}


void cob_cartesian_trajectories::getPriStart(KDL::Frame &F_handle)
{
    int axis_no;

    KDL::Frame F_articulation;

    map<int, KDL::Vector> handle_rot;

    //tf transform
    tf::Transform transform_handle;
    tf::StampedTransform transform_map_base;

    // lookup transform from map to base_link
    try
    {
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform_map_base);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    // convert to KDL::Frame
    KDL::Frame F_base_link;
    geometry_msgs::TransformStamped transform_map_base_msg;
    tf::transformStampedTFToMsg(transform_map_base, transform_map_base_msg);
    tf::transformMsgToKDL(transform_map_base_msg.transform, F_base_link);

    // set up articulation frame
    F_articulation.p.x(getParamValue("rigid_position.x"));
    F_articulation.p.y(getParamValue("rigid_position.y"));
    F_articulation.p.z(getParamValue("rigid_position.z"));
    F_articulation.M = KDL::Rotation::Quaternion(getParamValue("rigid_orientation.x"), getParamValue("rigid_orientation.y"), getParamValue("rigid_orientation.z"), getParamValue("rigid_orientation.w"));
    debug ? (std::cout << "F_articulation" << "\n" <<  F_articulation << "\n" ) : (std::cout << ""); //debug

    // origin of track start frame correlates with EE start
    F_handle.p = F_EE_start.p;
    debug ? (std::cout << "F_EE_start" << "\n" <<  F_EE_start << "\n") : (std::cout << ""); //debug

    // Vector prismatic_dir
    KDL::Vector prismatic_dir_ART = KDL::Vector(getParamValue("prismatic_dir.x"), getParamValue("prismatic_dir.y"), getParamValue("prismatic_dir.z"));
    debug ? (std::cout << "prismatic_dir_ART" << "\n" <<  prismatic_dir_ART << "\n") : (std::cout << ""); //debug

    // transform prismatic_dir_ART in global frame
    KDL::Vector prismatic_dir = F_articulation.M*prismatic_dir_ART;
    debug ? (std::cout << "prismatic_dir" << "\n" <<  prismatic_dir << "\n") : (std::cout << ""); //debug

    // transform prismatic_dir in F_base_link
    KDL::Vector prismatic_dir_BL = F_base_link.M.Inverse()*prismatic_dir;
    debug ? (std::cout << "prismatic_dir_BL" << "\n" <<  prismatic_dir_BL << "\n") : (std::cout << ""); //debug
    Eigen::Vector3d(abs(prismatic_dir_BL[0]), abs(prismatic_dir_BL[1]), abs(prismatic_dir_BL[2])).maxCoeff(&axis_no);
    if (prismatic_dir_BL[axis_no] < 0.0)
        handle_rot[2] = prismatic_dir * (-1.0);
    else
        handle_rot[2] = prismatic_dir;
    debug ? (std::cout << "prismatic_dir" << "\n" <<  prismatic_dir << "\n") : (std::cout << ""); //debug
    handle_rot[2].Normalize();
    std::cout << "rot vector z" << "\n" << handle_rot[2] << "\n"; //debug

    // set up arbitrary vector perpendicular to prismatic_dir as x-axis
    handle_rot[0] = KDL::Vector(handle_rot[2][1], -handle_rot[2][0], 0.0);
    handle_rot[0].Normalize();
    std::cout << "rot vector x" << "\n" << handle_rot[0] << "\n"; //debug

    // than set up the y-axis via cross product
    handle_rot[1] = vector3dEigenToKDL(vector3dKDLToEigen(handle_rot[2]).cross(vector3dKDLToEigen(handle_rot[0])));
    handle_rot[1].Normalize();
    std::cout << "rot vector y" << "\n" << handle_rot[1] << "\n"; //debug

    // set up F_handle rotation
    F_handle.M = KDL::Rotation(handle_rot[0], handle_rot[1], handle_rot[2]);
    std::cout << "F_handle" << "\n" << F_handle << "\n"; //debug

    // broadcast F_handle
    geometry_msgs::Transform transform_handle_msg;
    tf::transformKDLToMsg(F_handle, transform_handle_msg);
    tf::transformMsgToTF(transform_handle_msg, transform_handle);
    br.sendTransform(tf::StampedTransform(transform_handle, ros::Time::now(), "/map", "/handle"));


    bHandle = false;
}


//rotational 6D-trajectory from rot_axis, rot_radius and angle
void cob_cartesian_trajectories::getRotTarget(double dt, KDL::Frame &F_target)
{
    double angle;
    double partial_angle;

    KDL::Frame F_track;

    // get start frame of trajectory
    if (bHandle)
        getRotStart(F_track_start);

    angle = getParamValue("action");

    // calculating partial_angle
    partial_angle = angle * (1. - cos(PI*(dt/(targetDuration-1.))))/2.;
    if (dt > (targetDuration-1.))
            partial_angle = angle;
    //partial_angle = angle * (dt/targetDuration);

    // creating trajectory frame w.r.t. the track_start frame
    // orientation is like sdh_tip_link frame
    F_track.p[0] = rot_radius*(1-cos(partial_angle));
    F_track.p[2] = rot_radius*sin(partial_angle);

    F_track.M.DoRotY(partial_angle);

    // transformation of trajectory frame into base_link frame ??? map frame
    F_target.p = F_track_start*F_track.p;       // transform F_Track in F_track_start 
    F_target.M = F_track_start.M*F_track.M*F_track_start.M.Inverse()*F_EE_start.M;        // rotation with respect to gripper start frame (F_EE_start)

    // tf transform F_track_start
    tf::Transform transform_track_start;
    geometry_msgs::Transform transform_track_start_msg;
    tf::transformKDLToMsg(F_track_start, transform_track_start_msg);
    tf::transformMsgToTF(transform_track_start_msg, transform_track_start);
    br.sendTransform(tf::StampedTransform(transform_track_start, ros::Time::now(), "/map", "/track_start"));
    
    // tf transform F_track
    tf::Transform transform_track;
    geometry_msgs::Transform transform_track_msg;
    tf::transformKDLToMsg(F_track, transform_track_msg);
    tf::transformMsgToTF(transform_track_msg, transform_track);
    br.sendTransform(tf::StampedTransform(transform_track, ros::Time::now(), "/track_start", "/track"));

    // tf transform F_target
    tf::Transform transform_target;
    geometry_msgs::Transform transform_target_msg;
    tf::transformKDLToMsg(F_target, transform_target_msg);
    tf::transformMsgToTF(transform_target_msg, transform_target);
    br.sendTransform(tf::StampedTransform(transform_target, ros::Time::now(), "/map", "/target"));
}


//calculate start position of trajectory from rot_axis, rot_radius and gripper position
void cob_cartesian_trajectories::getRotStart(KDL::Frame &F_handle)
{
    // auxiliary variables
    double rot_radius_actual;

    int axis_no;

    KDL::Frame F_articulation;
    
    Eigen::Vector3d articulation_Z;
    Eigen::Vector3d articulation_O;
    Eigen::Vector3d handle_O;
    Eigen::Vector3d perpendicular;

    Eigen::Hyperplane<double, 3> handle_plane;
    
    map<int, KDL::Vector> handle_rot;

    //tf transform
    tf::Transform transform_handle;
    tf::StampedTransform transform_map_base;

    // lookup transform from map to base_link
    try
    {
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform_map_base);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    // convert to KDL::Frame
    KDL::Frame F_base_link;
    geometry_msgs::TransformStamped transform_map_base_msg;
    tf::transformStampedTFToMsg(transform_map_base, transform_map_base_msg);
    tf::transformMsgToKDL(transform_map_base_msg.transform, F_base_link);
    debug ? (std::cout << "F_base_link" <<  F_base_link << "\n" ) : (std::cout << ""); //debug

    // set up articulation frame
    F_articulation.p.x(getParamValue("rot_center.x"));
    F_articulation.p.y(getParamValue("rot_center.y"));
    F_articulation.p.z(getParamValue("rot_center.z"));
    F_articulation.M = KDL::Rotation::Quaternion(getParamValue("rot_axis.x"), getParamValue("rot_axis.y"), getParamValue("rot_axis.z"), getParamValue("rot_axis.w"));
    debug ? (std::cout << "F_articulation" <<  F_articulation << "\n" ) : (std::cout << ""); //debug

    // EE start position is sdh_tip_link frame
    debug ? (std::cout << "F_EE_start" <<  F_EE_start << "\n") : (std::cout << ""); //debug

    // origin of track start frame correlates with EE start
    F_handle.p = F_EE_start.p;

    // z-axis of articulation frame in global coordinates to use as normal for rotation plane
    KDL::Vector articulation_Z_KDL = F_articulation.M.UnitZ(); 
    vector3dKDLToEigen(articulation_Z_KDL, articulation_Z);
    std::cout << "articulation_Z_KDL" << "\n" <<  articulation_Z_KDL << "\n"; //debug
    std::cout << "articulation_Z" << "\n" <<  articulation_Z << "\n"; //debug

    // origin of handle frame as point in plane of rotation
    vector3dKDLToEigen(F_handle.p, handle_O);
    std::cout << "handle_O" << "\n" <<  handle_O << "\n"; //debug

    // origin of articulation frame
    vector3dKDLToEigen(F_articulation.p, articulation_O);
    std::cout << "articulation_O" << "\n" <<  articulation_O << "\n"; //debug

    // handle plane calculation --> three point 
    handle_plane = Eigen::Hyperplane<double, 3>::Through(articulation_O, articulation_O + articulation_Z, handle_O);
    std::cout << "handle_plane_offset" << "\n" <<  handle_plane.offset() << "\n"; //debug
    std::cout << "handle_plane_coeffs" << "\n" <<  handle_plane.coeffs() << "\n"; //debug
    std::cout << "handle_plane_distance to articulation_O" << "\n" <<  handle_plane.absDistance(articulation_O) << "\n"; //debug
    std::cout << "handle_plane_distance to handle" << "\n" <<  handle_plane.absDistance(handle_O) << "\n"; //debug

    // perpendicular of articulation_Z through handle_O (already normalized)
    perpendicular = articulation_Z.cross(handle_plane.normal());
    std::cout << "perdendicular" << "\n" <<  perpendicular << "\n"; //debug
    if (perpendicular.norm() < 1e-6)
        ROS_ERROR("Normals are parallel");

    // translation from EE start to articulation in global coord
    KDL::Vector trans_ee_art_KDL = F_articulation.p - F_EE_start.p;
    std::cout << "trans_ee_art_KDL" << "\n" <<  trans_ee_art_KDL << "\n"; //debug

    // transform trans_ee_art into F_EE_start
    KDL::Vector trans_ee_art_KDL_EE = F_EE_start.M.Inverse()*trans_ee_art_KDL;
    std::cout << "trans_ee_art_KDL_EE" << "\n" <<  trans_ee_art_KDL_EE << "\n"; //debug

    // transform perpendicular into EE frame
    KDL::Vector perpendicular_EE_KDL = F_EE_start.M.Inverse()*KDL::Vector(perpendicular[0], perpendicular[1], perpendicular[2]);
    std::cout << "perpendicular_EE_KDL" << "\n" <<  perpendicular_EE_KDL << "\n"; //debug

    // get actual rot_radius
    rot_radius_actual = dot(trans_ee_art_KDL_EE, perpendicular_EE_KDL);
    std::cout << "rot_radius_actual" << "\n" <<  rot_radius_actual << "\n"; //debug

    // set direction of perpendicular and set as handle_rot x-axis
    if (rot_radius_actual < 0.0)
        perpendicular *= (-1.0);
    vector3dEigenToKDL(perpendicular, handle_rot[0]);
        
    // absolute value
    rot_radius_actual = abs(rot_radius_actual);

    // check direction of articulation_Z and set up second vector of handle_rot (parallel to articulation_Z)
    KDL::Vector articulation_Z_KDL_BL = F_base_link.M.Inverse()*articulation_Z_KDL;
    Eigen::Vector3d(abs(articulation_Z_KDL_BL.x()), abs(articulation_Z_KDL_BL.y()), abs(articulation_Z_KDL_BL.z())).maxCoeff(&axis_no);
    if (articulation_Z_KDL_BL[axis_no] < 0.0)
        handle_rot[1] = (-1.0)*articulation_Z_KDL;
    else
        handle_rot[1] = articulation_Z_KDL;
    
    // calculate cross produkt to get handle_rot z-axis 
    handle_rot[2] = vector3dEigenToKDL(vector3dKDLToEigen(handle_rot[0]).cross(vector3dKDLToEigen(handle_rot[1])));

    std::cout << "rot vector x" << "\n" << handle_rot[0] << "\n"; //debug
    std::cout << "rot vector y" << "\n" << handle_rot[1] << "\n"; //debug
    std::cout << "rot vector z" << "\n" << handle_rot[2] << "\n"; //debug

    // set up F_handle rotation
    F_handle.M = KDL::Rotation(handle_rot[0], handle_rot[1], handle_rot[2]);
    std::cout << "F_handle" << "\n" << F_handle << "\n"; //debug

    // broadcast F_handle
    geometry_msgs::Transform transform_handle_msg;
    tf::transformKDLToMsg(F_handle, transform_handle_msg);
    tf::transformMsgToTF(transform_handle_msg, transform_handle);
    br.sendTransform(tf::StampedTransform(transform_handle, ros::Time::now(), "/map", "/handle"));
    
    // calculate, check and set up rot_radius TODO: abort if tolerance is exceeded
    rot_radius = getParamValue("rot_radius");
    if (rot_radius == 0.0)
    {
        ROS_DEBUG("Parameter rot_radius no set, rot_radius_actual will be taken!");
    }
    std::cout << "radius difference" << "\n" << rot_radius-rot_radius_actual << "\n"; //debug
    if (abs(rot_radius - rot_radius_actual) > 0.05)
    {
        std::cout << "rot_radius" << "\n" << rot_radius << "\n"; //debug
        std::cout << "rot_radius_actual" << "\n" << rot_radius_actual << "\n"; //debug
        ROS_ERROR("model radius and actual radius differ quite much");
    }
    rot_radius = rot_radius_actual;

    bHandle = false;
}


double cob_cartesian_trajectories::getParamValue(std::string param_name)
{
    for (unsigned int i = 0; i < params.size(); i++)
    {
        if (strncmp(params[i].name.c_str(), param_name.c_str(), param_name.size()) == 0)
        {
            std::cout << " -- " << params[i].name << " -- " << params[i].value << "\n";
            return params[i].value;
        }
    }
    ROS_ERROR("Missing parameter");
    std::cout << "No value found for parameter " << param_name << "\n";
    return 0.0;
}


geometry_msgs::Twist cob_cartesian_trajectories::PIDController(const double dt, const KDL::Frame &F_target, const KDL::Frame &F_current)
{
    geometry_msgs::Twist twist;
    if (dt < 0.00001)
        return twist;
    double current_roll = 0.0, current_pitch = 0.0, current_yaw = 0.0;
    double target_roll = 0.0, target_pitch = 0.0, target_yaw = 0.0;
    
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);
    target_roll = unwrapRPY("target_roll", target_roll);
    target_pitch = unwrapRPY("target_pitch", target_pitch);
    target_yaw = unwrapRPY("target_yaw", target_yaw);
    F_current.M.GetRPY(current_roll, current_pitch, current_yaw);
    current_roll = unwrapRPY("current_roll", current_roll);
    current_pitch = unwrapRPY("current_pitch", current_pitch);
    current_yaw = unwrapRPY("current_yaw", current_yaw);
        
    Error.vel.x(F_target.p.x() - F_current.p.x());
    Error.vel.y(F_target.p.y() - F_current.p.y());
    Error.vel.z(F_target.p.z() - F_current.p.z());
    Error.rot.x(target_roll - current_roll);
    Error.rot.y(target_pitch - current_pitch);
    Error.rot.z(target_yaw - current_yaw);
    
    cout << "Error twist: " << "\n" << Error << "\n";
    
    Error_sum.vel.x(Error_sum.vel.x() + Error.vel.x() * dt);
    Error_sum.vel.y(Error_sum.vel.y() + Error.vel.y() * dt);
    Error_sum.vel.z(Error_sum.vel.z() + Error.vel.z() * dt);
    Error_sum.rot.x(Error_sum.rot.x() + Error.rot.x() * dt);
    Error_sum.rot.y(Error_sum.rot.y() + Error.rot.y() * dt);
    Error_sum.rot.z(Error_sum.rot.z() + Error.rot.z() * dt); 
    
    cout << "Error_sum twist: " << "\n" << Error_sum << "\n";
    
    Error_dot.vel.x((Error.vel.x() - Error_last.vel.x()) / dt);
    Error_dot.vel.y((Error.vel.y() - Error_last.vel.y()) / dt);
    Error_dot.vel.z((Error.vel.z() - Error_last.vel.z()) / dt);
    Error_dot.rot.x((Error.rot.x() - Error_last.rot.x()) / dt);
    Error_dot.rot.y((Error.rot.y() - Error_last.rot.y()) / dt);
    Error_dot.rot.z((Error.rot.z() - Error_last.rot.z()) / dt);
    
    cout << "Error_dot twist: " << "\n" << Error_dot << "\n";
    
    // create twist
    p_gain_ = 1.5;
    twist.linear.x = p_gain_*Error.vel.x() + i_gain_*Error_sum.vel.x() + d_gain_*Error_dot.vel.x();
    p_gain_ = 1.5;
    twist.linear.y = p_gain_*Error.vel.y() + i_gain_*Error_sum.vel.y() + d_gain_*Error_dot.vel.y(); 
    p_gain_ = 1.;
    twist.linear.z = p_gain_*Error.vel.z() + i_gain_*Error_sum.vel.z() + d_gain_*Error_dot.vel.z();
    p_gain_ = 1.;
    twist.angular.x = -(p_gain_*Error.rot.x() + i_gain_*Error_sum.rot.x() + d_gain_*Error_dot.rot.x());
    p_gain_ = 1.;
    twist.angular.y = -(p_gain_*Error.rot.y() + i_gain_*Error_sum.rot.y() + d_gain_*Error_dot.rot.y());
    p_gain_ = 1.;
    twist.angular.z = p_gain_*Error.rot.z() + i_gain_*Error_sum.rot.z() + d_gain_*Error_dot.rot.z();

    pubTrack(1, ros::Duration(0.5), F_target);
    pubTrack(9, ros::Duration(0.5), F_current);
    pubTwistMarkers(ros::Duration(1.0), twist, F_current);
    
    Error_last2.vel.x(Error_last.vel.x());
    Error_last2.vel.y(Error_last.vel.y());
    Error_last2.vel.z(Error_last.vel.z());
    Error_last2.rot.x(Error_last.rot.x());
    Error_last2.rot.y(Error_last.rot.y());
    Error_last2.rot.z(Error_last.rot.z());

    Error_last.vel.x(Error.vel.x());
    Error_last.vel.y(Error.vel.y());
    Error_last.vel.z(Error.vel.z());
    Error_last.rot.x(Error.rot.x());
    Error_last.rot.y(Error.rot.y());
    Error_last.rot.z(Error.rot.z());


    vec_frames.push_back(F_target);
    if((int)vec_frames.size() > 30){
        vec_err_tb.push_back(KDL::diff(*vec_frames.begin(), F_current, 1.));
        vec_frames.erase(vec_frames.begin());
    }

    // target and current position vector
    vec_pos_ist.push_back(F_current.p);
    vec_pos_soll.push_back(F_target.p);

    // vector of velocities
    //vec_vel_ist.push_back(KDL::diff(F_current, F_last_ist, dt));
    //vec_vel_soll.push_back(KDL::diff(F_target, F_last_soll, dt));
    //F_last_ist = F_current;
    //F_last_soll = F_target;

    // add error to vector
    vec_err_p.push_back(Error);

    // broadcast twist
    tf::Transform transform_twist;
    KDL::Frame F_twist;
    F_twist.p.x(twist.linear.x);
    F_twist.p.y(twist.linear.y);
    F_twist.p.z(twist.linear.z);
    F_twist.M = KDL::Rotation::RPY(twist.angular.x, twist.angular.y, twist.angular.z);
    geometry_msgs::Transform transform_twist_msg;
    tf::transformKDLToMsg(F_twist, transform_twist_msg);
    tf::transformMsgToTF(transform_twist_msg, transform_twist);
    br.sendTransform(tf::StampedTransform(transform_twist, ros::Time::now(), "/sdh_tip_link", "/twist"));

    return twist;
}


//VISUALIZATION IN RVIZ
//
// publish generated trajectory
void cob_cartesian_trajectories::pubTrack(const int track_id, const ros::Duration pub_duration, const KDL::Frame &F_pub)
{
    cob_articulation_controller::TrackMsg track;
    // set up a new TrackMsg
    if (track_map.find(track_id) == track_map.end())
    {
        cout << "new track: " << track_id << "\n";
        track.header.stamp = (ros::Time::now() - pub_duration);
        track.header.frame_id = "/map";
        track.id = track_id;
        track_map[track_id] = track;
    }

    // store pose in corresponding TrackMsg
    if ((ros::Time::now() - track_map[track_id].header.stamp) >= pub_duration)
    {
        track_map[track_id].header.stamp = ros::Time::now();
        track_map[track_id].header.frame_id = "/map";
        track_map[track_id].id = track_id;
        geometry_msgs::Pose pose; 
        

        pose.position.x = F_pub.p.x();
        pose.position.y = F_pub.p.y();
        pose.position.z = F_pub.p.z();

        F_pub.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        track_map[track_id].pose.push_back(pose);
        
        track_pub_.publish(track_map[track_id]);
    }

}

// publish twist
void cob_cartesian_trajectories::pubTwistMarkers(const ros::Duration pub_duration, const geometry_msgs::Twist &Twist, const KDL::Frame &F_current)
{
    if ((ros::Time::now() - pub_timer) >= pub_duration)
    {
        double color_mixer;
        double offset = 0.0;

        //setting up marker msg
        visualization_msgs::Marker twist_marker;
        twist_marker.header.frame_id = "/map";
        twist_marker.header.stamp = ros::Time::now();
        twist_marker.ns = "twist";
        twist_marker.id = pub_counter;
        twist_marker.type = visualization_msgs::Marker::ARROW;
        twist_marker.action = visualization_msgs::Marker::ADD;
        twist_marker.lifetime = ros::Duration();

        twist_marker.scale.x = 0.01;
        twist_marker.scale.y = 0.02;

        //calculate the color of the marker depending on twist size
        color_mixer = 11.0*sqrt(Twist.linear.x*Twist.linear.x + Twist.linear.y*Twist.linear.y + Twist.linear.z*Twist.linear.z);
        if (color_mixer > 1.0) offset = color_mixer - 1.0;
        color_mixer = color_mixer - offset;
        twist_marker.color.a = 1.0;
        twist_marker.color.g = 1.0 - color_mixer;
        twist_marker.color.r = color_mixer;

        //add two markers for start and end point of the arrow
        geometry_msgs::Point p;
        p.x = F_current.p.x();
        p.y = F_current.p.y();
        p.z = F_current.p.z();
        twist_marker.points.push_back(p);
        p.x = F_current.p.x() + Twist.linear.x;
        p.y = F_current.p.y() + Twist.linear.y;
        p.z = F_current.p.z() + Twist.linear.z;
        twist_marker.points.push_back(p);
        
        twist_pub_.publish(twist_marker);
        pub_timer = ros::Time::now();
        pub_counter++;
    }
}

//old function to publish trajectory markers
void cob_cartesian_trajectories::sendMarkers()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_values";
    marker.id = 10;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    for(unsigned int i=0; i<trajectory_points.size(); i++)
    {
        //ROS_INFO("line %f %f %f %f\n", iX1, iY1, iX2, iY2);
        marker.points.push_back(trajectory_points[i]);
    }
    //map_pub_.publish(marker);
}

//TOOLS
//
void cob_cartesian_trajectories::vector3dKDLToEigen(const KDL::Vector &from, Eigen::Vector3d &to)
{
    to = Eigen::Vector3d(from.x(), from.y(), from.z());
}

Eigen::Vector3d cob_cartesian_trajectories::vector3dKDLToEigen(const KDL::Vector &from)
{
    return Eigen::Vector3d(from.x(), from.y(), from.z());
}

void cob_cartesian_trajectories::vector3dEigenToKDL(const Eigen::Vector3d &from, KDL::Vector &to)
{
    to = KDL::Vector(from[0], from[1], from[2]);
}

KDL::Vector cob_cartesian_trajectories::vector3dEigenToKDL(const Eigen::Vector3d &from)
{
    return KDL::Vector(from[0], from[1], from[2]);
}


// to avoid the jump from positive to negative and the other way around 
// when crossing pi or -pi for roll and yaw angles and pi/2 and -pi/2 for pitch
double cob_cartesian_trajectories::unwrapRPY(std::string axis, double angle)
{
    double unwrapped_angle = 0.0;
    double fractpart, intpart;
    double discont = PI;    //point of discontinuity

    // FORMULA: phi(n)_adjusted = MODULO( phi(n) - phi(n-1) + PI, 2*PI) + phi(n-1) - PI
    
    // the pitch angle is only defined from -PI/2 to PI/2
    if (strncmp(&axis[strlen(axis.c_str())-6], "_pitch", 6) == 0)
        discont = PI/2;
    
    fractpart = modf(((angle - last_rpy_angles[axis] + discont) / (2*discont)), &intpart);    // modulo for double and getting only the fractal part of the division
    
    if (fractpart >= 0.0) // to get always the positive modulo
        unwrapped_angle = (fractpart)*(2*discont) - discont + last_rpy_angles[axis];
    else
        unwrapped_angle = (1+fractpart)*(2*discont) - discont + last_rpy_angles[axis];

    std::cout << "modulo: " << intpart << " + " << (fractpart) << "\n";
    
    std::cout << axis << " angle: " << angle << "\n";
    std::cout << "unwrapped " << axis << " angle: " << unwrapped_angle << "\n";
    last_rpy_angles[axis] = unwrapped_angle;
    return unwrapped_angle;
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "cob_cartesian_trajectories");
    cob_cartesian_trajectories controller ;
    ros::spin();

    return 0;
}
