/*
/**
*
* \file main.cpp
*
* \brief Example Master App for Profile Position (on PC)
*
*
*
* Copyright (c) 2014, Synapticon GmbH
* All rights reserved.
* Author: Pavan Kanajar <pkanajar@synapticon.com> & Christian Holl <choll@synapticon.com>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 3. Execution of this software or parts of it exclusively takes place on hardware
*    produced by Synapticon GmbH.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
* either expressed or implied, of the Synapticon GmbH.
*
*/
extern "C"
{
#include <ctrlproto_m.h>
#include "ethercat_setup.h"
#include "profile.h"
#include "drive_function.h"
#include <motor_define.h>
}

#include <ecrt.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>


/***ROS includes***/
#include <ros/ros.h>
#include "boost/thread.hpp"


#include <geometry_msgs/Twist.h>
#include <ros_ecat_master/MoveJoint.h>
boost::mutex mutex_ethercat_update;


void cb_move_joints(geometry_msgs::TwistConstPtr &msg)
{
	mutex_ethercat_update.lock();
	mutex_ethercat_update.unlock();
}

void ecat_master()
{

	ros::Rate r(2000); // 2000 hz

	 while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();


	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ethercatExampleNode");
	ros::NodeHandle n("~");
	//ros::Subscriber sub0=n.subscribe("/ecat",1, cb_move_joints);

	ecat_master();

	return 0;
}



