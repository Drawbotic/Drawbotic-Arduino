/*!
 * \file Drawbotic_DB1_Defines.h
 * \author Elliott Wilson (elliott.wilson@monash.edu)
 * \brief Defines for various constants for the Drawbotic DB1 Arduino library
 * \version 1.0
 * \date 2023-03-13
 */

#ifndef DRAWBOTIC_DB1_DEFINES_H
#define DRAWBOTIC_DB1_DEFINES_H

//! Number of RGB LEDs on board
#define LIGHT_COUNT         8
//! Number of Time of Flight sensor on board
#define TOF_COUNT           3
//! Number of IR line detectors on board
#define IR_COUNT            5

//! I2C Addresses of Time of Flight sensors, set during initialisation
const int TOF_ADDRESSES[TOF_COUNT] = { 0x1C, 0x1B, 0x1A };
//! Enable pins for Time of Flight sensors, defined in DB1 board package
const int TOF_EN_PINS[TOF_COUNT] = { TOF3_EN, TOF2_EN, TOF1_EN };

//! Delay time for Time of Flight sensors to boot
#define TOF_BOOT_DELAY_MS   20

//! Number of IR sensor samples during calibration
#define IR_CALIBRATION_COUNT    1000
//! Delay between IR calibration samples
#define IR_CALIBRATION_DELAY_MS 10

//! Number of Colour sensor samples during calibration 
#define COLOUR_CALIBRATION_COUNT    5
//! Delay between Colour calibration samples
#define COLOUR_CALIBRATION_DELAY_MS 500

//! Default IMU Orientation sample rate in milliseconds
#define IMU_ORIENT_MS_DEFAULT  5
//! Default IMU Acceleration sample rate in milliseconds
#define IMU_ACCEL_MS_DEFAULT   5

//! Default Time of Flight sample rate in milliseconds
#define TOF_MS_DEFAULT         5

//! Default Servo Up position
#define SERVO_UP_DEFAULT    25
//! Default Servo Down position
#define SERVO_DOWN_DEFAULT  90

//! Default Timeout of Time of Flight Sensors
#define TOF_TIMEOUT_DEFAULT         500
//! Default Signal Limit of Time of Flight Sensors
#define TOF_SIGLIM_DEFAULT          0.25f
//! Default Timing Budget of Time of Flight Sensors
#define TOF_TIMING_BUDGET_DEFAULT   33000
//! Default Pre Pulse Period of Time of Flight Sensors
#define TOF_PRE_PCLKS_DEFAULT       14
//! Default Final Pulse Period of Time of Flight Sensors
#define TOF_FIN_PCLKS_DEFAULT       10

//! Default IR Average Window Size
#define IR_READ_COUNT_DEFAULT       4

//! Voltage for Low Battery level
#define BATT_VOLT_LOW               6.0f
//! Voltage for High Battery level
#define BATT_VOLT_HIGH              8.4f

#endif