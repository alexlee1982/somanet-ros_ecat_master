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
#include <profile.h>
#include <drive_function.h>
#include <motor_define.h>

#include "ethercat_setup.h"
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
#include <ros_ecat_master/MotorState.h>
boost::mutex mutex_ethercat_update;


enum {ECAT_SLAVE_0};

ros::Publisher pub0;


int desired_speed=0;
int desired_speed_current=0;


void cb_move_joints(const geometry_msgs::TwistConstPtr &msg)
{
	mutex_ethercat_update.lock();
		if(msg->linear.x<1)
		{
			desired_speed=msg->linear.x*2000;
		}
		else
		{
			desired_speed=1*2000;
		}


	mutex_ethercat_update.unlock();
}

void ecat_master()
{

	ros::Rate r(2000); // 2000 hz

    int acceleration = 1000; //rpm/s
    int deceleration = 1000; //rpm/s

    int actual_velocity = 0; // rpm
    int actual_position=0; // ticks
    float actual_torque=0; // mNm
    int steps = 0;
    int velocity_ramp = 0; // rpm

    /* Initialize Ethercat Master */
    init_master(&master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);

    /* Initialize torque parameters */
    initialize_torque(ECAT_SLAVE_0, slv_handles);

    /* Initialize all connected nodes with Mandatory Motor Configurations (specified in config/motor/)*/
    init_nodes(&master_setup, slv_handles, TOTAL_NUM_OF_SLAVES,1);

    /* Initialize the node specified with ECAT_SLAVE_0 with CSV configurations (specified in config/motor/)*/
    set_operation_mode(CSV, ECAT_SLAVE_0, &master_setup, slv_handles,
            TOTAL_NUM_OF_SLAVES);

    /* Enable operation of node in CSV mode */
    enable_operation(ECAT_SLAVE_0, &master_setup, slv_handles,
            TOTAL_NUM_OF_SLAVES);

    ros_ecat_master::MotorState msg;

    int step=0;
    bool init=1;
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();

		/* Update the process data (EtherCat packets) sent/received from the node */
		pdo_handle_ecat(&master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);

		mutex_ethercat_update.lock();

		if(desired_speed != desired_speed_current)
		{
			step=0;
			desired_speed_current=desired_speed;
		    steps = init_velocity_profile_params(desired_speed,
		            actual_velocity, acceleration, deceleration, ECAT_SLAVE_0,
		            slv_handles);
		}

		if(steps>0 || init)
		{
			init=false;
            velocity_ramp = generate_profile_velocity(step, ECAT_SLAVE_0,
                    slv_handles);


            set_velocity_rpm(velocity_ramp, ECAT_SLAVE_0, slv_handles);

            steps--;
            step++;
		}

		mutex_ethercat_update.unlock();





		/* Read actual node sensor values */
		actual_velocity
				= get_velocity_actual_rpm(ECAT_SLAVE_0, slv_handles);
		actual_position = get_position_actual_ticks(ECAT_SLAVE_0,
				slv_handles);
		actual_torque = get_torque_actual_mNm(ECAT_SLAVE_0, slv_handles);

		msg.actual_position=actual_position;
		msg.actual_velocity=actual_velocity;
		msg.actual_torque=actual_torque;

		pub0.publish(msg);
	}


    /* Quick stop velocity mode (for emergency) */
    quick_stop_velocity(ECAT_SLAVE_0, &master_setup, slv_handles,
            TOTAL_NUM_OF_SLAVES);

    /* Regain control of node to continue after quick stop */
    renable_ctrl_quick_stop(CSV, ECAT_SLAVE_0, &master_setup, slv_handles,
            TOTAL_NUM_OF_SLAVES);

    set_operation_mode(CSV, ECAT_SLAVE_0, &master_setup, slv_handles,
            TOTAL_NUM_OF_SLAVES);

    enable_operation(ECAT_SLAVE_0, &master_setup, slv_handles,
            TOTAL_NUM_OF_SLAVES);

    /* Shutdown node operations */
    shutdown_operation(CSV, ECAT_SLAVE_0, &master_setup, slv_handles,
            TOTAL_NUM_OF_SLAVES);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ethercatExampleNode");
	ros::NodeHandle n("~");
	ros::Subscriber sub0=n.subscribe("/ecat",1, cb_move_joints);
	pub0=n.advertise<ros_ecat_master::MotorState>("/motor_state",1,0);
	ecat_master();

	return 0;
}



