/* A version of the lkdriver.h the header file for low_cost_lkdriver.cpp.
 * 
 * This includes the addition of descriptive comments along with 
 * reformatting to follow good software engineering practice and increase clarity.
 * 
 * Originally written by Emo Todorov
 *
 * Copyright (C) 2016 Roboti LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * 
 * Aidan Lethaby
 * 4 June 2020
 */


#ifndef LOW_COST_LKDRIVER_H  // Ensures declarations are only included once
#define LOW_COST_LKDRIVER_H

// Sensor ordering and number
enum {
	lkSNS_WRIST_ROTATION = 0,
	lkSNS_WRIST_DEVIATION,
	lkSNS_THUMB_YAW,
	lkSNS_THUMB_PITCH,
	lkSNS_INDEX,
	lkSNS_MRP,

	lkSNS_COMPLIANCE_THUMB_ROLL,
	lkSNS_COMPLIANCE_THUMB_PITCH,
	lkSNS_COMPLIANCE_INDEX,

	lkSNS_TOUCH_INDEX_LATERAL,
	lkSNS_TOUCH_INDEX_TIP,
	lkSNS_TOUCH_MIDDLE,
	lkSNS_TOUCH_RING,
	lkSNS_TOUCH_PINKY,
	lkSNS_PALM_DISTAL,
	lkSNS_PALM_PROXIMAL,
	lkSNS_HAND_EDGE,
	lkSNS_HAND_DORSAL,
	lkSNS_THUMB_ULNAR,
	lkSNS_THUMB_RADIAL,
	lkSNS_THUMB_TIP,
	lkSNS_THUMB_DORSAL,

	lkNSENSOR  // Used to represent the size = 22, useful for iteration
};


// Command ordering and number
enum {
	lkCMD_WRIST_ROTATION = 0,  // Wrist rotation is command 0
	lkCMD_WRIST_DEVIATION,     // Wrist deviation (inward, outward) is command 1
	lkCMD_THUMB_YAW,           // Thumb yaw (inward, outward) is command 2
	lkCMD_THUMB_PITCH,         // Thumb pitch (toward palm, away from palm) is command 3
	lkCMD_INDEX,               // Index finger is command 4
	lkCMD_MRP,                 // Middle, ring, and pinky finger is command 5

	lkNCOMMAND  // Used to represent the size = 6, useful for iteration
};


/* Start thread responsible for hand control
 * 
 * Return: 0 = success, thread started; 1 = fail, thread already started
 */
int lk_start(void);


/* Stop thread responsible for hand control 
 * 
 * Return: 0 = success, active thread stopped; 1 = fail, no thread to stop
 */
int lk_stop(void);


/* Get sensor data (data can be NULL) from hand sensors
 * 
 * Input: data array of doubles to put sensor data into
 * 
 * Return: framecount 
 */
int lk_get_sensor(double* data);


/* Set motor command
 * 
 * Input: data array of doubles containing hand commands
 */
void lk_set_command(const double* data);


/* Checks if the thread is running
 * 
 * Return: 0 = thread not running; 1 = thread running
 */
int lk_running(void);

#endif
