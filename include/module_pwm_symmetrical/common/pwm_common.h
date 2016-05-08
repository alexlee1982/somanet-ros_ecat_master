/*
 *
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and/or its licensors.
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2010
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#pragma once

/****** CONFIGURATION. DO NOT TOUCH!! ******/

/* All timing values depend on reference clock setting. For 100 MHz reference clock each tick
 * is equivalent to 10ns (for 250 MHz clock it's 4ns)
 */
#define PWM_DEAD_TIME 700 //DC100/300 - 300, DC1K/5K - 700

/* This defines the PWM resolution. Because the PWM is driven by a fixed clock,
 * this also configures the PWM frequency at the same time. The relation is as follows:
 * PWM_MAX_VALUE = PWM_CLOCK / PWM_FREQUENCY (e.g. 250 MHz / 18 kHz = 13889)
 */
#define PWM_MAX_VALUE 12500 //DC100/300 - 13889, DC1K/5K - 12500
/*******************************************/

#ifndef __ASSEMBLER__

typedef enum PWM_OUTPUT_CAT
{
	LONG_SINGLE,
	SINGLE,
	DOUBLE
} e_pwm_cat;

/* if changing this then change the corresponding value in dsc_pwm_common.h */
typedef struct PWM_OUT_DATA
{
	/* N */
	unsigned ts0;  // 0
	unsigned out0; // 1
	unsigned ts1;  // 2
	unsigned out1; // 3

	/* N' */
	unsigned inv_ts0;  // 4
	unsigned inv_out0; // 5
	unsigned inv_ts1;  // 6
	unsigned inv_out1; // 7

	/* other info */
	e_pwm_cat cat;
	unsigned value;
} t_out_data;

// Shared memory structure for the client->server
typedef struct {
	unsigned chan_id_buf[2][3];
	unsigned mode_buf[2];
	t_out_data pwm_out_data_buf[2][3];
	unsigned pwm_cur_buf;
} t_pwm_control;

#endif

// The offset and size of components in the PWM control structure
#define OFFSET_OF_CHAN_ID  0
#define OFFSET_OF_MODE_BUF 24
#define OFFSET_OF_DATA_OUT 32
#define SIZE_OF_T_DATA_OUT 40

// Define this to limit the symmetrical PWM duty cycle to a smaller range, enabling faster update
//#define PWM_CLIPPED_RANGE

// The number of PWM channels that are supported by the symmetrical PWM
#ifndef PWM_CHAN_COUNT
#define PWM_CHAN_COUNT 3
#endif

// The number of clocks to increment between each phase
#ifndef SYNC_INCREMENT
#define SYNC_INCREMENT (PWM_MAX_VALUE)
#endif

// The initial number of clocks to wait before starting the PWM loops
#ifndef INIT_SYNC_INCREMENT
#define INIT_SYNC_INCREMENT (SYNC_INCREMENT)
#endif

