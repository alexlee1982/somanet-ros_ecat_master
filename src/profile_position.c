
/**
 *
 * \file profile_position.c
 *
 * \brief Profile Generation for Position
 * 	Implements position profile based on Linear Function with
 * 	Parabolic Blends.
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

#include <profile.h>



void init_position_profile_limits(int gear_ratio, int max_acceleration, int max_velocity, profile_pos_param *profile_pos_params)
{
	profile_pos_params->max_acceleration = (max_acceleration * 6)/ gear_ratio;
	profile_pos_params->max_velocity = (max_velocity * 6)/ gear_ratio ;
	profile_pos_params->gear_ratio = (float) gear_ratio;
}

int init_position_profile(int target_position, int actual_position,	int velocity, int acceleration, \
		                  int deceleration, profile_pos_param *profile_pos_params)//ctrlproto_slv_handle *
{
	profile_pos_params->qf = (float) target_position;

	profile_pos_params->qf = profile_pos_params->qf/10000.0f;

	profile_pos_params->qi = (float) actual_position;

	profile_pos_params->qi = profile_pos_params->qi/10000.0f;

	profile_pos_params->vi = (float) (velocity * 6)/profile_pos_params->gear_ratio;

	profile_pos_params->acc = (float) (acceleration * 6)/profile_pos_params->gear_ratio;

	profile_pos_params->dec = (float) (deceleration * 6)/profile_pos_params->gear_ratio;


	if(profile_pos_params->acc > profile_pos_params->max_acceleration)
		profile_pos_params->acc = profile_pos_params->max_acceleration;

	if(profile_pos_params->dec > profile_pos_params->max_acceleration)
		profile_pos_params->dec = profile_pos_params->max_acceleration;

	if(profile_pos_params->vi > profile_pos_params->max_velocity)
		profile_pos_params->vi = profile_pos_params->max_velocity;


	/* Internal params */

	profile_pos_params->acc_too_low = 0;

	profile_pos_params->qid = 0.0f;

	/* leads to shorter blend times in the begining (if init condition != 0) non zero case - not yet considered */

	profile_pos_params->qfd = 0.0f;


	/* compute distance */

	profile_pos_params->total_distance = profile_pos_params->qf - profile_pos_params->qi;

	profile_pos_params->direction = 1;

	if (profile_pos_params->total_distance < 0)
	{
		profile_pos_params->total_distance = -profile_pos_params->total_distance;

		profile_pos_params->direction = -1;
	}

	profile_pos_params->tb_acc = profile_pos_params->vi / profile_pos_params->acc;

	profile_pos_params->tb_dec = profile_pos_params->vi / profile_pos_params->dec;

	profile_pos_params->distance_acc = (profile_pos_params->acc * profile_pos_params->tb_acc 	\
			                        * profile_pos_params->tb_acc) / 2.0f;

	profile_pos_params->distance_dec = (profile_pos_params->dec * profile_pos_params->tb_dec	\
			                        * profile_pos_params->tb_dec) / 2.0f;

	profile_pos_params->distance_left = profile_pos_params->total_distance 	\
									 - profile_pos_params->distance_acc   	\
									 - profile_pos_params->distance_dec;


	/*check velocity and distance constraint*/

	if (profile_pos_params->distance_left < 0)
	{
		profile_pos_params->acc_too_low = 1;

		/* acc too low to meet distance/vel constraint */

        if(profile_pos_params->vi > profile_pos_params->total_distance)
        {
        	profile_pos_params->vi = profile_pos_params->total_distance;

        	profile_pos_params->acc_min = profile_pos_params->vi;

        	if(profile_pos_params->acc < profile_pos_params->acc_min)
        	{
        		profile_pos_params->acc = profile_pos_params->acc_min;
        	}

        	if(profile_pos_params->dec < profile_pos_params->acc_min)
        	{
        		profile_pos_params->dec = profile_pos_params->acc_min;
        	}
        }
        else if(profile_pos_params->vi < profile_pos_params->total_distance)
        {
        	profile_pos_params->acc_min = profile_pos_params->vi;

        	if(profile_pos_params->acc < profile_pos_params->acc_min)
        	{
        		profile_pos_params->acc = profile_pos_params->acc_min;
        	}
        	if(profile_pos_params->dec < profile_pos_params->acc_min)
        	{
        		profile_pos_params->dec = profile_pos_params->acc_min;
        	}
        }

        profile_pos_params->tb_acc = profile_pos_params->vi / profile_pos_params->acc;

        profile_pos_params->tb_dec = profile_pos_params->vi / profile_pos_params->dec;

        profile_pos_params->distance_acc = (profile_pos_params->acc * profile_pos_params->tb_acc 	\
        								* profile_pos_params->tb_acc)/2.0f;

        profile_pos_params->distance_dec = (profile_pos_params->dec * profile_pos_params->tb_dec 	\
        								* profile_pos_params->tb_dec)/2.0f;

        profile_pos_params->distance_left = profile_pos_params->total_distance 	\
        								 - profile_pos_params->distance_acc 		\
        								 - profile_pos_params->distance_dec;

	}
	else if (profile_pos_params->distance_left > 0)
	{
		profile_pos_params->acc_too_low = 0;
	}


	/* check velocity and min acceleration constraint */

	if (profile_pos_params->distance_left < 0)
	{
		profile_pos_params->acc_too_low = 1;

		/* acc too low to meet distance/velocity constraint */

		profile_pos_params->acc_min = profile_pos_params->vi;

        if(profile_pos_params->acc < profile_pos_params->acc_min)
        {
        	profile_pos_params->acc = profile_pos_params->acc_min;
        }

        if(profile_pos_params->dec < profile_pos_params->acc_min)
        {
        	profile_pos_params->dec = profile_pos_params->acc_min;
        }

        profile_pos_params->tb_acc = profile_pos_params->vi / profile_pos_params->acc;

        profile_pos_params->tb_dec = profile_pos_params->vi / profile_pos_params->dec;

        profile_pos_params->distance_acc = (profile_pos_params->acc * profile_pos_params->tb_acc 	\
        								* profile_pos_params->tb_acc)/2.0f;

        profile_pos_params->distance_dec = (profile_pos_params->dec * profile_pos_params->tb_dec 	\
        								* profile_pos_params->tb_dec)/2.0f;

        profile_pos_params->distance_left = profile_pos_params->total_distance	\
        								 - profile_pos_params->distance_acc		\
        								 - profile_pos_params->distance_dec;
	}
	else if (profile_pos_params->distance_left > 0)
	{
		profile_pos_params->acc_too_low = 0;
	}

	profile_pos_params->distance_cruise = profile_pos_params->distance_left;

	profile_pos_params->t_cruise = (profile_pos_params->distance_cruise) / profile_pos_params->vi;

	profile_pos_params->tf = profile_pos_params->tb_acc + profile_pos_params->tb_dec \
			              + profile_pos_params->t_cruise;

	if (profile_pos_params->direction == -1)
	{
		profile_pos_params->vi = -profile_pos_params->vi;
	}

	/* compute LFPB motion constants */

	profile_pos_params->ai = profile_pos_params->qi;

	profile_pos_params->bi = profile_pos_params->qid;

	profile_pos_params->ci = (profile_pos_params->vi - profile_pos_params->qid) / (2.0f * profile_pos_params->tb_acc);

	profile_pos_params->di = profile_pos_params->ai + profile_pos_params->tb_acc * profile_pos_params->bi 		\
						  + profile_pos_params->ci * profile_pos_params->tb_acc * profile_pos_params->tb_acc 	\
						  - profile_pos_params->vi * profile_pos_params->tb_acc;

	profile_pos_params->ei = profile_pos_params->qf;

	profile_pos_params->fi = profile_pos_params->qfd;

	profile_pos_params->gi = (profile_pos_params->di + (profile_pos_params->tf - profile_pos_params->tb_dec) 	\
			 	 	 	  * profile_pos_params->vi + profile_pos_params->fi * profile_pos_params->tb_dec 		\
			              - profile_pos_params->ei) / (profile_pos_params->tb_dec * profile_pos_params->tb_dec);

	profile_pos_params->T = profile_pos_params->tf / 0.001f;        	// 1 ms

	profile_pos_params->s_time = 0.001f;								// 1 ms

	return (int) round(profile_pos_params->T);
}

int position_profile_generate(int step, profile_pos_param *profile_pos_params)
{
	profile_pos_params->ts = profile_pos_params->s_time * step ;

	if (profile_pos_params->ts < profile_pos_params->tb_acc)
	{
		profile_pos_params->q = profile_pos_params->ai + profile_pos_params->ts * profile_pos_params->bi 	\
							 + profile_pos_params->ci * profile_pos_params->ts * profile_pos_params->ts;
	}

	else if (profile_pos_params->tb_acc <= profile_pos_params->ts 										\
			 &&  profile_pos_params->ts < (profile_pos_params->tf - profile_pos_params->tb_dec) )
	{
		profile_pos_params->q = profile_pos_params->di + profile_pos_params->vi * profile_pos_params->ts;
	}

	else if ( (profile_pos_params->tf - profile_pos_params->tb_dec) <= profile_pos_params->ts				\
			                             && profile_pos_params->ts <= profile_pos_params->tf)
	{
		profile_pos_params->q = profile_pos_params->ei + (profile_pos_params->ts - profile_pos_params->tf)	\
							 * profile_pos_params->fi + (profile_pos_params->ts - profile_pos_params->tf) 	\
							 * (profile_pos_params->ts - profile_pos_params->tf) * profile_pos_params->gi;
	}

	return (int) round(profile_pos_params->q * 10000.0f);
}




int init_quick_stop_position_profile(int actual_velocity, int actual_position, int max_acceleration, pos_quick_stop_param *pos_param_s)  //emergency stop
{
	pos_param_s->qi = 0;
	pos_param_s->qf = (float) actual_velocity;   //always positive

	 if(pos_param_s->qf < 0)
		 pos_param_s->qf = 0 - pos_param_s->qf;

	 pos_param_s->acc = pos_param_s->qf;
	 if(pos_param_s->acc > max_acceleration)
		 pos_param_s->acc = max_acceleration;

	 pos_param_s->cur_pos_s = (float) actual_position;

	 pos_param_s->tb = pos_param_s->qf / pos_param_s->acc;

	 pos_param_s->ci = - pos_param_s->acc / 2;
	 pos_param_s->samp = pos_param_s->tb/1.0e-3;
	 pos_param_s->t_int = pos_param_s->tb/pos_param_s->samp;
	 if(pos_param_s->samp<0)
		 pos_param_s->samp= 0 - pos_param_s->samp;
	 return (int)  round((pos_param_s->samp));
}

int quick_stop_position_profile_generate(int steps, int actual_velocity, pos_quick_stop_param *pos_param_s)
{
	pos_param_s->ts = pos_param_s->t_int*steps;
	pos_param_s->q = pos_param_s->qf * pos_param_s->ts + pos_param_s->ci*pos_param_s->ts*pos_param_s->ts;
    if(actual_velocity >= 0)
    {
    	return (int) round( pos_param_s->cur_pos_s + pos_param_s->q*10000.0f);
    }
    else if(actual_velocity < 0)
    {
	    return (int) round( pos_param_s->cur_pos_s - pos_param_s->q*10000.0f);
    }
    return 0;
}


