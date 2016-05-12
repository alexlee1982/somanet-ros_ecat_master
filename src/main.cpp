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


#include <geometry_msgs/TwistStamped.h>
#include <ros_ecat_master/MoveJoint.h>
#include <ros_ecat_master/MotorState.h>
boost::mutex mutex_ethercat_update;


enum {ECAT_SLAVE_0, ECAT_SLAVE_1, ECAT_SLAVE_2, ECAT_SLAVE_3};

ros::Publisher pub0;
int desired_velocity[TOTAL_NUM_OF_SLAVES];
geometry_msgs::TwistStamped vel_;

void cb_command_velocity(const geometry_msgs::TwistStamped &vel)
{

	double wheelRadius = 0.0475;//in meters
	double LengthBetweenFrontAndRearWheels = 0.47;//in meters
	double LengthBetweenFrontWheels = 0.3;//in meters
	double l1 = LengthBetweenFrontWheels/2.0;
	double l2 = LengthBetweenFrontAndRearWheels/2.0;

	mutex_ethercat_update.lock();

	desired_velocity[ECAT_SLAVE_0] = -1.0 * (-vel.twist.linear.x + vel.twist.linear.y - (l1 + l2) *  vel.twist.angular.z) / wheelRadius;
	desired_velocity[ECAT_SLAVE_1] = -1.0 * (-vel.twist.linear.x - vel.twist.linear.y + (l1 + l2) * (-1.0) * vel.twist.angular.z) / wheelRadius;
	desired_velocity[ECAT_SLAVE_2] = (-vel.twist.linear.x - vel.twist.linear.y - (l1 + l2) * (-1.0) * vel.twist.angular.z) / wheelRadius;
	desired_velocity[ECAT_SLAVE_3] = (-vel.twist.linear.x + vel.twist.linear.y + (l1 + l2) * vel.twist.angular.z) / wheelRadius;
	vel_.header.stamp = vel.header.stamp;
	vel_.header.seq = vel.header.seq;

	mutex_ethercat_update.unlock();
}


void watchdog(const ros::TimerEvent &e)
{
	ros::Time now = ros::Time::now();
	double secs = now.toSec();
	if ((now - vel_.header.stamp) > ros::Duration(1.0))
	{
		ROS_WARN("Something is wrong with connection!");
		//Emergency stop
		for (int i = 0; i < TOTAL_NUM_OF_SLAVES; i++ )
		{
			desired_velocity[i] = 0;
		}
	}


}


void ecat_master()
{

    ros::Rate r(1000); // 1000 hz

    int acceleration = 3000; //rpm/s
    int deceleration = 3000; //rpm/s

    int actual_velocity[TOTAL_NUM_OF_SLAVES]; // rpm
    int actual_position[TOTAL_NUM_OF_SLAVES]; // ticks
    float actual_torque[TOTAL_NUM_OF_SLAVES]; // mNm
    int steps[TOTAL_NUM_OF_SLAVES];
    int inc[TOTAL_NUM_OF_SLAVES];

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

      ros::spinOnce();
      r.sleep();

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
	ros::Subscriber sub0=n.subscribe("/command_velocity_joy",1, cb_command_velocity);
	ros::Timer watchdog_timer = n.createTimer(ros::Duration(1.0), watchdog);
	pub0=n.advertise<ros_ecat_master::MotorState>("/motor_state",1,0);
	ecat_master();

	return 0;
}



