#ifndef DRAWBOTICDB1DEFINES_H
#define DRAWBOTICDB1DEFINES_H

#define LIGHT_COUNT         8
#define TOF_COUNT           3
#define IR_COUNT            5

const int TOF_ADDRESSES[TOF_COUNT] = { 0x1C, 0x1B, 0x1A };
const int TOF_EN_PINS[TOF_COUNT] = { TOF3_EN, TOF2_EN, TOF1_EN };

#define TOF_BOOT_DELAY_MS   20
#define TCS_BOOT_DELAY_MS   20

#define IR_CALIBRATION_COUNT    1000
#define IR_CALIBRATION_DELAY_MS 10

#define SERVO_UP_DEFAULT    25
#define SERVO_DOWN_DEFAULT  90

#define TOF_TIMEOUT_DEFAULT         500
#define TOF_SIGLIM_DEFAULT          0.25f
#define TOF_TIMING_BUDGET_DEFAULT   33000
#define TOF_PRE_PCLKS_DEFAULT       14
#define TOF_FIN_PCLKS_DEFAULT       10

#define BATT_VOLT_LOW               6.0f
#define BATT_VOLT_HIGH              8.4f

#endif