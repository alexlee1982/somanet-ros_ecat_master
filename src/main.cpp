
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
#include "ros_ecat_master/MoveJoint.h"

#define NODE_1 0
#define NODE_2 1
#define NODE_3 2
#define NODE_4 3

boost::mutex mutex_ethercat_update;

double move_degree_j1 = 0.0, move_degree_j2 = 0.0, move_degree_j3 = 0.0, move_degree_j4 = 0.0,
    move_speed_j1 = 0.0, move_speed_j2 = 0.0, move_speed_j3 = 0.0, move_speed_j4 = 0.0;
double move_degree_j1_old = 0.0, move_degree_j2_old = 0.0, move_degree_j3_old = 0.0, move_degree_j4_old = 0.0,
    move_speed_j1_old = 0.0, move_speed_j2_old = 0.0, move_speed_j3_old = 0.0, move_speed_j4_old = 0.0;
int cmd = 3;
float temp;

void cb_move_joints(const ros_ecat_master::MoveJointConstPtr &move)
{
	mutex_ethercat_update.lock();

	if(move->TurnDegree.size()>=1) move_degree_j1 = move->TurnDegree[0];
	if(move->TurnDegree.size()>=2)move_degree_j2 = move->TurnDegree[1];
	if(move->TurnDegree.size()>=3)move_degree_j3 = move->TurnDegree[2];
	if(move->TurnDegree.size()>=4)move_degree_j4 = move->TurnDegree[3];

	if(move_degree_j1 > 350) move_degree_j1 = 350;
	if(move_degree_j1 < -350) move_degree_j1 = -350;

	if(move_degree_j2 > 350) move_degree_j2 = 350;
	if(move_degree_j2 < -350) move_degree_j2 = -350;

	if(move_degree_j3 > 350) move_degree_j3 = 350;
	if(move_degree_j3 < -350) move_degree_j3 = -350;

	if(move_degree_j4 > 350) move_degree_j4 = 350;
	if(move_degree_j4 < -350) move_degree_j4 = -350;


	if(move->VelocityRPM.size()>=1)move_speed_j1 = move->VelocityRPM[0];
	if(move->VelocityRPM.size()>=2)move_speed_j2 = move->VelocityRPM[1];
	if(move->VelocityRPM.size()>=3)move_speed_j3 = move->VelocityRPM[2];
	if(move->VelocityRPM.size()>=4)move_speed_j4 = move->VelocityRPM[3];
	mutex_ethercat_update.unlock();
}

void ecat_master()
{
	std::cout<<"Master starting...."<<std::endl;

	int flag = 0;

	int acceleration = 4000;	// rpm/s
	int deceleration = 4000;	// rpm/s
	int velocity_1 = 0;	// rpm
	int velocity_2 = 0;	// rpm
	int velocity_3 = 0;	// rpm
	int velocity_4 = 0;	// rpm
	float actual_position_1 = 0.0f;		// degree
	float actual_position_2 = 0.0f;		// degree
	float actual_position_3 = 0.0f;		// degree
	float actual_position_4 = 0.0f;		// degree

	int steps_1 = 0, steps_2 = 0, steps_3 = 0, steps_4 = 0;
	int i_1 = 1, i_2 = 1, i_3 = 1, i_4 = 1;
	int position_ramp_1 = 0, position_ramp_2 = 0, position_ramp_3 = 0, position_ramp_4 = 0;


	profile_pos_param profile_pos_param_1;
	profile_pos_param profile_pos_param_2;
	profile_pos_param profile_pos_param_3;
	profile_pos_param profile_pos_param_4;


	init_master(&master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);

	for (int j = 0; j < TOTAL_NUM_OF_SLAVES; j++)
	{
		std::cout << "1" << std::endl;
		init_node(j, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
		std::cout << "2" << std::endl;
		set_operation_mode(CSP, j, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
		std::cout << "3" << std::endl;
		enable_operation(j, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	}

	init_position_profile_limits(GEAR_RATIO_1, MAX_ACCELERATION_1, MAX_NOMINAL_SPEED_1, &profile_pos_param_1);// one time
	init_position_profile_limits(GEAR_RATIO_2, MAX_ACCELERATION_2, MAX_NOMINAL_SPEED_2, &profile_pos_param_2);// one time
	init_position_profile_limits(GEAR_RATIO_3, MAX_ACCELERATION_3, MAX_NOMINAL_SPEED_3, &profile_pos_param_3);// one time
	init_position_profile_limits(GEAR_RATIO_4, MAX_ACCELERATION_4, MAX_NOMINAL_SPEED_4, &profile_pos_param_4);// one time

	ros::Rate r(2000); // 2000 hz

    while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();

       
    		pdo_handle_ecat(&master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);

    		if(master_setup.op_flag)	//Check if the master is active
    		{
    			mutex_ethercat_update.lock();
    			{
					//NODE 1
					if(move_degree_j1 != move_degree_j1_old)
					{
						i_1 = 0;
						actual_position_1 =  get_position_actual_degree(NODE_1, slv_handles);
						velocity_1 = move_speed_j1;
						steps_1 = init_position_profile_params(move_degree_j1, actual_position_1, velocity_1, acceleration, deceleration, &profile_pos_param_1);
					}

					if(i_1<steps_1 && flag == 0)
					{
						position_ramp_1 = position_profile_generate(i_1, &profile_pos_param_1);
						set_position_degree(position_ramp_1, NODE_1, slv_handles);
						i_1 = i_1 + 1;
					}


					//NODE 2
					if(move_degree_j2 != move_degree_j2_old)
					{
						i_2 = 0;
						actual_position_2 =  get_position_actual_degree(NODE_2, slv_handles);
						velocity_2 = move_speed_j2;
						steps_2 = init_position_profile_params(move_degree_j2, actual_position_2, velocity_2, acceleration, deceleration, &profile_pos_param_2);
					}

					if(i_2<steps_2 && flag == 0)
					{
						position_ramp_2 = position_profile_generate(i_2, &profile_pos_param_2);
						set_position_degree(position_ramp_2, NODE_2, slv_handles);
						i_2 = i_2 + 1;
					}

					//NODE 3
					if(move_degree_j3 != move_degree_j3_old)
					{
						i_3 = 0;
						actual_position_3 =  get_position_actual_degree(NODE_3, slv_handles);
						velocity_3 = move_speed_j3;
						steps_3 = init_position_profile_params(move_degree_j3, actual_position_3, velocity_3, acceleration, deceleration, &profile_pos_param_3);
					}

					if(i_3<steps_3 && flag == 0)
					{
						position_ramp_3 = position_profile_generate(i_3, &profile_pos_param_3);
						set_position_degree(position_ramp_3, NODE_3, slv_handles);
						i_3 = i_3 + 1;
					}

					//NODE 4
					if(move_degree_j4 != move_degree_j4_old)
					{
						i_4 = 0;
						actual_position_4 =  get_position_actual_degree(NODE_4, slv_handles);
						velocity_4 = move_speed_j4;
						steps_4 = init_position_profile_params(move_degree_j4, actual_position_4, velocity_4, acceleration, deceleration, &profile_pos_param_4);
					}

					if(i_4<steps_4 && flag == 0)
					{
						position_ramp_4 = position_profile_generate(i_4, &profile_pos_param_4);
						set_position_degree(position_ramp_4, NODE_4, slv_handles);
						i_4 = i_4 + 1;
					}

					move_degree_j1_old = move_degree_j1;
					move_degree_j2_old = move_degree_j2;
					move_degree_j3_old = move_degree_j3;
					move_degree_j4_old = move_degree_j4;

					mutex_ethercat_update.unlock();
    			}
    		}


	}

    std::cout << "Ethercat terminated" << std::endl;
//----------------------------------Quick stop----------------------------------------
	quick_stop_position(NODE_1, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	quick_stop_position(NODE_2, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	quick_stop_position(NODE_3, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	quick_stop_position(NODE_4, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);

	renable_ctrl_quick_stop(CSP, NODE_1, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES); //after quick-stop
	renable_ctrl_quick_stop(CSP, NODE_2, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES); //after quick-stop
	renable_ctrl_quick_stop(CSP, NODE_3, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES); //after quick-stop
	renable_ctrl_quick_stop(CSP, NODE_4, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES); //after quick-stop

	set_operation_mode(CSP, NODE_1, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	set_operation_mode(CSP, NODE_2, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	set_operation_mode(CSP, NODE_3, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	set_operation_mode(CSP, NODE_4, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);

	enable_operation(NODE_1, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	enable_operation(NODE_2, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	enable_operation(NODE_3, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	enable_operation(NODE_4, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);

//----------------------------------Shut down----------------------------------------
	shutdown_operation(CSP, NODE_1, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	shutdown_operation(CSP, NODE_2, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	shutdown_operation(CSP, NODE_3, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
	shutdown_operation(CSP, NODE_4, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);


}


int main(int argc, char **argv)
{


	ros::init(argc, argv, "SymanNode");
	ros::NodeHandle n("~");
	ros::Subscriber sub0=n.subscribe("/move_joints",1, cb_move_joints);

	ecat_master();





	return 0;
}



