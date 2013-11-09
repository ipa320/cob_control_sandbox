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


class CobVisualServoing
{
private:
	ros::NodeHandle nh_;
	
public:
	CobVisualServoing() {;}
	~CobVisualServoing();
	
	void initialize();
	void run();

};
#endif

