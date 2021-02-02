/* A version of the lkdriver.cpp wrapped for use by low_cost_lkdriver.py.
 *
 * This includes modification to allow the python wrapper to work and has the addition of descriptive comments along with 
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
 * 29 June 2020
 */

#define PY_SSIZE_T_CLEAN
#include <Python.h>  // Library containing the Python API *FOR WRAPPING*

#include "PCANBasic.h"  // File containing shorthands for PCAN ports, definitions, etc.
#include "low_cost_lkdriver.h"  // File containing enum and function declarations for this file

#include <iostream>  // Allows console print and read (io streams)
#include <stdio.h>   // Input/output library that uses streams to operate with physical devices
#include <curses.h>  // Terminal control library
#include <mutex>     // Class representing a lockable object used to manage threads
#include <thread>    // Class representing individual threads of execution
#include <chrono>    // Time library that deals with durations, time points, and clocks


using namespace std;


// Internal globals
static mutex guard;                 // Protect global data, used to lock sections of code to prevent race conditions by different threads
static thread thandle;              // Thread handle to identify the thread
static bool exitrequest = false;    // Request to exit thread
static double command[lkNCOMMAND];  // Motor command to send
static double sensor[lkNSENSOR];    // Sensor data received
static int framecount = 0;          // Received sensor framecount ????IDK what this is for????


//////////////////////////////THREAD FUNCTION AND UTILS//////////////////////////////

/* Encodes one command
 * 
 * Converts motor command so arm contol input module(ACI) can understand 
 */
static void encode(BYTE* DATA, int adr,  double cmd)
{
    int icmd = (int)round(cmd);  // Round
    if(icmd < -1023)  // Limit negative
        icmd = -1023;
    else if(icmd > 1023)  // Limit positive
        icmd = 1023;

    if(icmd > 0)  // Set low and high bytes: positive
    {
        DATA[4 * adr] = (BYTE)((icmd >> 8) & 0xFF);
        DATA[4 * adr + 1] = (BYTE)(icmd & 0xFF);
		DATA[4 * adr + 2] = 0;
		DATA[4 * adr + 3] = 0;
    }
    else  // Set low and high bytes: negative
    {
        icmd = -icmd;
		DATA[4 * adr] = 0;
		DATA[4 * adr + 1] = 0;
        DATA[4 * adr + 2] = (BYTE)((icmd >> 8) & 0xFF);
        DATA[4 * adr + 3] = (BYTE)(icmd & 0xFF);
    }
}


/* Decode one sensor
 *
 * Converts sensor data so we can understand
 * 
 * Returns a double representing the sensor
 */
static double decode(const BYTE* DATA, int adr, int power)
{
    union  // Convert big-endian device data to little-endian Intel processor data ????We're on a pi so what does this mean for us????
    {  
        BYTE bytes[2];
        int16_t result;
    };
    bytes[0] = DATA[2 * adr + 1];
    bytes[1] = DATA[2 * adr];
    
    return ((double)result) / ((double)(1 << power));  // Convert to double, divide by 2^power
}


/* Decode one touch sensor
 * 
 * Converts sensor touch data so we can understand
 * 
 * Returns a double representing the force
 */
static double touch(BYTE DATA, BYTE status, BYTE mask)
{
	double force = 0.1 * (double)DATA;  // Byte divided by 10

	if((status & mask))  // Flip sign if saturated
		force = -force;

    return force;
}


/* Thread function
 * 
 * This is the thread function that is run when a connection is set up with the hand
 * It ensures a clean start and the checks every 1 ms for an exit request or movement/motor command message
 * 
 * See the public API functions below as they change variables such as the command array and exitrequest boolean
 * that this function looks at in its loop
 */
static void threadfunc(void)
{
    int n;  // Increment variable used in for loops
    TPCANStatus status;
    TPCANMsg ACI[4], msg;
    TPCANHandle channel = 0;

    // To ensure a clean start, clear command, sensor, framecount
    guard.lock();
    for(n = 0; n < lkNCOMMAND; n++)
        command[n] = 0.0;
    for(n = 0; n < lkNSENSOR; n++)
        sensor[n] = 0.0;
    framecount = 0;
    guard.unlock();

    // Initiate ACI messages and clear
    ACI[0].ID = 0x210;  // Thumb pitch up, down; thumb yaw right, left
    ACI[1].ID = 0x211;  // Index flex, extend; MRP flex, extend
    ACI[2].ID = 0x212;  // Wrist pronate, supinate; wrist extend, flex
    ACI[3].ID = 0x213;  // Mode select, 0; 0, 0
    for(n = 0; n < 4; n++)
    {
        ACI[n].MSGTYPE = PCAN_MESSAGE_STANDARD;
        ACI[n].LEN = 8;
        for(int k = 0; k < 8; k++)
            ACI[n].DATA[k] = 0;
    }

    // Find CAN-USB device and initialize, exit if not found
    const TPCANHandle channels[16] = {
        PCAN_USBBUS1,  PCAN_USBBUS2,  PCAN_USBBUS3,  PCAN_USBBUS4,
        PCAN_USBBUS5,  PCAN_USBBUS6,  PCAN_USBBUS7,  PCAN_USBBUS8,
        PCAN_USBBUS9,  PCAN_USBBUS10, PCAN_USBBUS11, PCAN_USBBUS12,
        PCAN_USBBUS13, PCAN_USBBUS14, PCAN_USBBUS15, PCAN_USBBUS16
    };
    for(n = 0; n < 16; n++)
    {
        status = CAN_Initialize(channels[n], PCAN_BAUD_1M, 0, 0, 0);
        if(status == PCAN_ERROR_OK)
            channel = channels[n];
        break;  // ????Is this supposed to be in the if statement because this breaks the loop the first time through every time????
    }
    if(!channel)
    {
        printf("CAN device not found\n");
        return;
    }

    while(!exitrequest)  // Loop until asked to exit
    {
        while((status = CAN_Read(channel, &msg, NULL)) == PCAN_ERROR_QRCVEMPTY)  // Check every 1 ms until message received or exit request
        {  
            if(exitrequest)  // Exit if requested, close CAN first
            {
                CAN_Uninitialize(channel);
                return;
            }

            this_thread::sleep_for(chrono::milliseconds(1));  // Sleep thread for 1 ms
        }

        if(status != PCAN_ERROR_OK)  // CAN error: exit
        {
            CAN_Uninitialize(channel);
            printf("CAN_Read error %x\n", status);
            return;
        }

        switch(msg.ID)  // Process message from device
        {
        case 0x80:  // Sync request: reply with ACIs
            if(framecount < 200)  // Enable on first 200 counts... not documented ????Maybe framecount is used for some setup purpose????
            {
                ACI[3].DATA[0] = 0x02;
                ACI[3].DATA[1] = 0xFF;
            }
            else
                ACI[3].DATA[0] = ACI[3].DATA[1] = 0;

            // Encode motor command into ACIs
            guard.lock();
            encode(ACI[0].DATA, 0, -command[3]);     // 3: thumb pitch
            encode(ACI[0].DATA, 1, -command[2]);     // 2: thumb yaw
            encode(ACI[1].DATA, 0, command[4]);      // 4: index
            encode(ACI[1].DATA, 1, command[5]);      // 5: MRP
            encode(ACI[2].DATA, 0, -command[0]);     // 0: wrist rotate
            encode(ACI[2].DATA, 1, command[1]);      // 1: wrist deviate
            guard.unlock();

            
            for(n = 0; n < 4; n++)  // Send ACIs, exit on error
            {
                if((status = CAN_Write(channel, ACI+n)) != PCAN_ERROR_OK)
                {
                    CAN_Uninitialize(channel);
                    printf("CAN_Write error %x in ACI %d\n", status, n);
                    return;
                }
            }
            break;

        case 0x4AA:  // Wrist and index position sensors
            guard.lock();
            sensor[lkSNS_WRIST_ROTATION] = decode(msg.DATA, 0, 6);
            sensor[lkSNS_WRIST_DEVIATION] = -decode(msg.DATA, 1, 6);
            sensor[lkSNS_INDEX]	= decode(msg.DATA, 2, 6);
            sensor[lkSNS_MRP] = decode(msg.DATA, 3, 6);
            framecount++;  // Framecount on 0x4AA ????Why is it only incremented here????
            guard.unlock();
            break;

        case 0x4BF:  // Thumb position sensors
            guard.lock();
            sensor[lkSNS_THUMB_YAW]	= decode(msg.DATA, 1, 6);
            sensor[lkSNS_THUMB_PITCH] = decode(msg.DATA, 0, 6);
            guard.unlock();
            break;

        case 0x4AC:  // Compliance sensors
            guard.lock();
            sensor[lkSNS_COMPLIANCE_THUMB_PITCH] = decode(msg.DATA, 0, 4);
            sensor[lkSNS_COMPLIANCE_THUMB_ROLL] = decode(msg.DATA, 1, 4);
            sensor[lkSNS_COMPLIANCE_INDEX] = decode(msg.DATA, 2, 4);
            guard.unlock();
            break;

		case 0x241:  // Touch 1
            guard.lock();
            sensor[lkSNS_TOUCH_INDEX_LATERAL] = touch(msg.DATA[0], msg.DATA[5], 1);
            sensor[lkSNS_TOUCH_INDEX_TIP] = touch(msg.DATA[1], msg.DATA[5], 2);
            sensor[lkSNS_TOUCH_MIDDLE] = touch(msg.DATA[2], msg.DATA[5], 4);
            sensor[lkSNS_TOUCH_RING] = touch(msg.DATA[3], msg.DATA[5], 8);
            sensor[lkSNS_TOUCH_PINKY] = touch(msg.DATA[4], msg.DATA[5], 16);
            guard.unlock();
            break;

		case 0x341:  // Touch 2
            guard.lock();
            sensor[lkSNS_PALM_DISTAL] = touch(msg.DATA[0], msg.DATA[4], 1);
            sensor[lkSNS_PALM_PROXIMAL]	= touch(msg.DATA[1], msg.DATA[4], 2);
            sensor[lkSNS_HAND_EDGE]	= touch(msg.DATA[2], msg.DATA[4], 4);
            sensor[lkSNS_HAND_DORSAL] = touch(msg.DATA[3], msg.DATA[4], 8);
            guard.unlock();
            break;

		case 0x4C2:  // Touch 3
            guard.lock();
            sensor[lkSNS_THUMB_ULNAR] = touch(msg.DATA[0], msg.DATA[4], 1);
            sensor[lkSNS_THUMB_RADIAL] = touch(msg.DATA[1], msg.DATA[4], 2);
            sensor[lkSNS_THUMB_TIP]	= touch(msg.DATA[2], msg.DATA[4], 4);
            sensor[lkSNS_THUMB_DORSAL] = touch(msg.DATA[3], msg.DATA[4], 8);
            guard.unlock();
            break;
		}
    }
}


//////////////////////////////API//////////////////////////////

/* Start thread responsible for hand control
 * 
 * Return: 0 = success, thread started; 1 = fail, thread already started
 */
int lk_start(void)
{
    if(thandle.joinable())  // If the thread is already started
        return 1;

    exitrequest = false;
    thandle = thread(threadfunc);  //Start, creates a new thread that will run the threadfunc() function

    return 0;
}


/* Stop thread responsible for hand control 
 * 
 * Return: 0 = success, active thread stopped; 1 = fail, no thread to stop
 */
int lk_stop(void)
{
    if(!thandle.joinable())  // If thread controlling hand is not started
        return 1;

    exitrequest = true;  // Request exit
    thandle.join();  // Wait, blocks current thread till thread controlling the hand is stopped/done

    return 0;
}


/* Get sensor data (data can be NULL)
 * 
 * Input: data array of doubles to put sensor data into
 * 
 * Return: framecount 
 */
int lk_get_sensor(double* data)
{
    // Protected data access with lock
    guard.lock();
    if(data)
        for(int n=0; n<lkNSENSOR; n++)
            data[n] = sensor[n];
    int result = framecount;
    guard.unlock();

    return result;
}


/* Set motor command
 * 
 * Input: data array of doubles containing hand commands
 */
void lk_set_command(const double* data)
{
    // Protected data access with lock
    guard.lock();
    for(int n=0; n<lkNCOMMAND; n++)  // Copy data array of doubles into command array of doubles,
        command[n] = data[n];  // this suggests a similar format to handihand that a command is represented by one double value varying from -1 to 1 or something similar
    guard.unlock();
}


/* Checks if the thread is running
 * 
 * Return: 0 = thread not running; 1 = thread running
 */
int lk_running(void)
{
    return (int)thandle.joinable();  // Returns true if the thread is active and false otherwise
}


//////////////////////////////WRAPPED API//////////////////////////////

/* A python callable function to wrap the lk_start function.
 */
static PyObject* lk_start_wrapper(PyObject* self)  // Takes no arguments
{
    return Py_BuildValue("i", lk_start());  // Returns lk_start's int return value as a python value
}


/* A python callable function to wrap the lk_stop function.
 */
static PyObject* lk_stop_wrapper(PyObject* self)  // Takes no arguments
{
    return Py_BuildValue("i", lk_stop());  // Returns lk_stop's int return value as a python value
}


/* A python callable function to wrap the lk_get_sensor function.
 * 
 * Return: a python list of doubles containing the sensor values.
 */
static PyObject* lk_get_sensor_wrapper(PyObject* self) // Takes no arguments
{
    double sensor_data[lkNSENSOR];  // Declare an array to store sensor data
    
    for(int i = 0; i < lkNSENSOR; i++)  // Initialize array with zeros 
    {
	    sensor_data[i] = 0.0;
    }
    
    lk_get_sensor(sensor_data);  // Populate array with sensor data
    
    const char* double_list = "[d, d, d, d, d, d, d, d, d, d, d, d, d, d, d, d, d, d, d, d, d, d]"; // Format string representing the python type of a double list, length lkNSENSOR = 22
    
    return Py_BuildValue(double_list, sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4], sensor_data[5], sensor_data[6],
                         sensor_data[7], sensor_data[8], sensor_data[9], sensor_data[10], sensor_data[11], sensor_data[12], sensor_data[13], 
                         sensor_data[14], sensor_data[15], sensor_data[16], sensor_data[17], sensor_data[18], sensor_data[19], sensor_data[20], sensor_data[21]); // Returns a array as a python list
}


/* A python callable function to wrap the lk_set_command function.
 * Takes in an array of doubles that represent a command.
 */
static PyObject* lk_set_command_wrapper(PyObject* self, PyObject* args)  // Takes arguments
{
	PyObject* buf_obj;
    Py_buffer buffer;
    double command_data[lkNCOMMAND];
  
    if(!PyArg_ParseTuple(args, "O", &buf_obj)) // Parse the PyObject args into a buffer
    {
	    return NULL; // If it fails then return NULL
    }
  
    if(PyObject_GetBuffer(buf_obj, &buffer, PyBUF_ANY_CONTIGUOUS | PyBUF_FORMAT) == -1)
    {
          return NULL;
    }
    
    for(int i = 0; i < lkNCOMMAND; i++)  // Copy buffer to a double array for use in lk_set_command function
    {
	    command_data[i] = ((double*)buffer.buf)[i];
    }
 
    PyBuffer_Release(&buffer);  // Free the buffer memory
    
    lk_set_command(command_data);  // Send the commands to the lk_set_command function
  
    const char* double_list = "[d, d, d, d, d, d]";  // Format string representing the python type of a double list, length lkNCOMMAND = 6
    return Py_BuildValue(double_list, command_data[0], command_data[1], command_data[2], command_data[3], command_data[4], command_data[5]);  // Return array as python list for testing purposes
    //return Py_RETURN_NONE; // Return nothing
}


/* A python callable function to wrap the lk_running function.
 */
static PyObject* lk_running_wrapper(PyObject* self)  // Takes no arguments
{
    return Py_BuildValue("i", lk_running());  // Returns lk_running's int return value as a python value
}


//////////////////////////////WRAPPER REQUIRED STRUCTURES//////////////////////////////

/* Methods definition required by python.
 */ 
static PyMethodDef lk_methods[] = { 
    {"lk_start", (PyCFunction)lk_start_wrapper, METH_NOARGS, "Start thread responsible for hand control."}, // python function name, c++ wrapped function name, arguments?, documentation
    {"lk_stop", (PyCFunction)lk_stop_wrapper, METH_NOARGS, "Stop thread responsible for hand control."},
    {"lk_get_sensor", (PyCFunction)lk_get_sensor_wrapper, METH_NOARGS, "Get sensor data (data can be NULL)."},
    {"lk_set_command", lk_set_command_wrapper, METH_VARARGS, "Set motor command."},
    {"lk_running", (PyCFunction)lk_running_wrapper, METH_NOARGS, "Checks if the thread is running."},
    {NULL, NULL, 0, NULL} // Null terminator
  };
      
/* Module definition required by python.
 */
static struct PyModuleDef lk_module = { 
    PyModuleDef_HEAD_INIT, // Method definition head initializer
    "lk_module", // Name of module
    "lkdriver Module", // Documentation (can be string of variable containing large amount of documentation data)
    -1, // Specify state, -1 = global
    lk_methods // Method definition
  };
      
/* Python module initializer function required by python.
 */
PyMODINIT_FUNC PyInit_lk_module(void)
{
  return PyModule_Create(&lk_module);
}
