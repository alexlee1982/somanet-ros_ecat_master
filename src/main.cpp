/**
 * @file main.cpp
 * @brief Example ROS Master App for Cyclic Synchronous Velocity (on PC)
 * @author Synapticon GmbH (www.synapticon.com)
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


enum {ECAT_SLAVE_0, ECAT_SLAVE_1, ECAT_SLAVE_2, ECAT_SLAVE_3};

ros::Publisher pub0;

void cb_move_joints(const geometry_msgs::TwistConstPtr &msg)
{
	mutex_ethercat_update.lock();
//		if(msg->linear.x<1)
//		{
//			desired_speed=msg->linear.x*2000;
//		}
//		else
//		{
//			desired_speed=1*2000;
//		}

	mutex_ethercat_update.unlock();
}

void ecat_master()
{

    ros::Rate r(1000); // 1000 hz

    int acceleration = 1000; //rpm/s
    int deceleration = 1000; //rpm/s

    int actual_velocity[TOTAL_NUM_OF_SLAVES]; // rpm
    int actual_position[TOTAL_NUM_OF_SLAVES]; // ticks
    float actual_torque[TOTAL_NUM_OF_SLAVES]; // mNm
    int steps[TOTAL_NUM_OF_SLAVES];
    int inc[TOTAL_NUM_OF_SLAVES];
    int velocity_ramp = 0; // rpm

    int desired_velocity[TOTAL_NUM_OF_SLAVES];
    int desired_velocity_old[TOTAL_NUM_OF_SLAVES];

    for(int i = 0; i < TOTAL_NUM_OF_SLAVES; i++) {
        desired_velocity[i] = 0;
        desired_velocity_old[i] = 0;
        actual_velocity[i] = 0;
        actual_position[i] = 0; // ticks
        actual_torque[i] = 0;// mNm

    }

    /* Initialize Ethercat Master */
    init_master(&master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);

    /* Initialize all connected nodes with Mandatory Motor Configurations (specified in config/motor/)*/
    init_nodes(&master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);


    for (int i = 0; i < TOTAL_NUM_OF_SLAVES; i++ ) {
        /* Initialize the node specified with ECAT_SLAVE_0 with CSV configurations (specified in config/motor/)*/
        set_operation_mode(CSV, i, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
        /* Enable operation of node in CSV mode */
        enable_operation(i, &master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);
    }

    ros_ecat_master::MotorState msg;

    while(ros::ok())
    {
      ros::spinOnce();
      r.sleep();

      /* Update the process data (EtherCat packets) sent/received from the node */
      pdo_handle_ecat(&master_setup, slv_handles, TOTAL_NUM_OF_SLAVES);

      mutex_ethercat_update.lock();


			if(master_setup.op_flag) /*Check if the master is active*/
			{

          for (int i = 0; i < TOTAL_NUM_OF_SLAVES; i++ )
          {
              /* Generating progile */
             if(desired_velocity[i] != desired_velocity_old[i]) {
                 steps[i] = init_velocity_profile_params(desired_velocity[i], actual_velocity[i], acceleration, deceleration, i, slv_handles);
                 inc[i] = 1;
             }
             /* Handling Slaves */
             else if(inc[i] < steps[i]){
                 /* Generate target velocity steps */
                 int target_velocity = generate_profile_velocity( inc[i], i, slv_handles);

                 /* Send target velocity for the node specified by slave_number */
                 set_velocity_rpm(target_velocity, i, slv_handles);
                 /* Update step */
                 inc[i]++;
             }

             actual_velocity[i] = get_velocity_actual_rpm(i, slv_handles);
             desired_velocity_old[i] = desired_velocity[i];
          }

      }

      mutex_ethercat_update.unlock();

      /* Publish actual values from the slaves */

      msg.actual_position=actual_position[ECAT_SLAVE_0];
      msg.actual_velocity=actual_velocity[ECAT_SLAVE_0];
      msg.actual_torque=actual_torque[ECAT_SLAVE_0];

			pub0.publish(msg);
		}

		for (int i = 0; i < TOTAL_NUM_OF_SLAVES; i++ ) {
				/* Quick stop velocity mode (for emergency) */
				quick_stop_velocity(i, &master_setup, slv_handles,
								TOTAL_NUM_OF_SLAVES);

				/* Regain control of node to continue after quick stop */
				renable_ctrl_quick_stop(CSV, i, &master_setup, slv_handles,
								TOTAL_NUM_OF_SLAVES);

				/* Initialize the node specified with ECAT_SLAVE_0 with CSV configurations (specified in config/motor/)*/
				set_operation_mode(CSV, i, &master_setup, slv_handles,
								TOTAL_NUM_OF_SLAVES);

				/* Enable operation of node in CSV mode */
				enable_operation(i, &master_setup, slv_handles,
								TOTAL_NUM_OF_SLAVES);

				/* Shutdown node operations */
				shutdown_operation(CSV, i, &master_setup, slv_handles,
								TOTAL_NUM_OF_SLAVES);
		}


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



