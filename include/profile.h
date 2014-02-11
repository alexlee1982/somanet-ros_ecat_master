
/**
 *
 * \file profile.h
 *
 * \brief Profile Generation for Position, Velocity and Torque
 * 	Implements position profile based on Linear Function with
 * 	Parabolic Blends, velocity profile and torque profiles are
 * 	based on linear functions.
 *
 *
 * Copyright (c) 2013, Synapticon GmbH
 * All rights reserved.
 * Author: Pavan Kanajar <pkanajar@synapticon.com>
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

#include <stdio.h>
#include <math.h>
#ifndef _PROFILE_H_
#define _PROFILE_H_

typedef struct
{
	float max_acceleration;     // max acceleration
	float max_velocity;

	/*User Inputs*/

	float acc;					// acceleration
	float dec; 					// deceleration
	float vi;					// velocity
	float qi;					// initial position
	float qf; 				    // final position

	/*Profile time*/

	float T;    				// total no. of Samples
	float s_time; 				// sampling time

	int direction;
	int acc_too_low;			// flag for low acceleration constraint
	float acc_min;				// constraint minimum acceleration

	/*LFPB motion profile constants*/

	float ai;
	float bi;
	float ci;
	float di;
	float ei;
	float fi;
	float gi;

	/*internal velocity variables*/

	float qid;					// initial velocity
	float qfd;					// final velocity

	float distance_cruise;
	float total_distance;
	float distance_acc;			// distance covered during acceleration
	float distance_dec;			// distance covered during deceleration
	float distance_left;		// distance left for cruise velocity

	float tb_acc;				// blend time for acceleration profile
	float tb_dec;				// blend time for deceleration profile
	float tf;					// total time for profile
	float t_cruise;				// time for cruise velocity profile
	float ts;					// variable to hold current sample time

	float q;					// position profile

	float gear_ratio;

} 	profile_pos_param;

typedef struct
{
	float max;												// shaft output speed in deg/s

	float vi;
	float qi;
	float qf; 												// user input variables

	float  t;
	float ts;

	int dirn;

	float ai, bi, ci, di, ei, fi, gi; 						// motion profile constants

	float qid, qfd;

	float dist, d_cruise;

	float t_cruise, tb, tf, t_int; 							// motion profile params


	float q, qd, q2d;
	float samp; float len;

	float cur_pos_s;
	float acc;
	int negative_s;
} pos_quick_stop_param;   											// position quick stop parameters



/*Profile Position Mode*/

/**
 * \brief Initialise Position Profile Limits
 *
 *  Input
 * \param gear_ratio
 * \param max_acceleration for the position profile
 * \param max_velocity for the position profile
 *
 */
extern void init_position_profile_limits(int gear_ratio, int max_acceleration, int max_velocity, profile_pos_param *profile_pos_params);

/**
 * \brief Initialise Position Profile
 *
 *  Input
 * \param target_position
 * \param actual_position
 * \param velocity for the position profile
 * \param acceleration for the position profile
 * \param deceleration for the position profile
 *
 * Output
 * \return no. of steps for position profile : range [1 - steps]
 */
int init_position_profile(int target_position, int actual_position,	int velocity, int acceleration, \
        						 int deceleration, profile_pos_param *profile_pos_params);

/**
 * \brief Generate Position Profile
 *
 *  Input
 * \param step current step of the profile
 *
 * Output
 * \return corresponding target position at the step input
 */
int position_profile_generate(int step, profile_pos_param *profile_pos_params);

/*Profile Position Quick Stop*/

/**
 * \brief Initialise Quick Stop Position Profile
 *
 *  Input
 * \param actual_velocity
 * \param actual_position
 * \param max_acceleration defines the deceleration for quick stop profile
 *
 * Output
 * \return no. of steps for quick stop profile : range [1 - steps]
 */
int init_quick_stop_position_profile(int actual_velocity, int actual_position, int max_acceleration, pos_quick_stop_param *pos_param_s);

/**
 * \brief Generate Quick Stop Position Profile
 *
 *  Input
 * \param step current step of the profile
 * \param actual_velocity
 *
 * Output
 * \return corresponding target position at the step input
 */
int quick_stop_position_profile_generate(int steps, int actual_velocity, pos_quick_stop_param *pos_param_s);



#endif /* _PROFILE_H_ */
