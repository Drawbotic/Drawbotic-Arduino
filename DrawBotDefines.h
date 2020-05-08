#ifndef DRAWBOTDEFINES_H
#define DRAWBOTDEFINES_H

#define LIGHT_COUNT         8
#define TOF_COUNT           3
#define IR_COUNT            5

const int TOF_ADDRESSES[TOF_COUNT] = { 0x1A, 0x1B, 0x1C };
const int TOF_EN_PINS[TOF_COUNT] = { TOF1_EN, TOF2_EN, TOF3_EN };

#define TOF_BOOT_DELAY_MS   20
#define TCS_BOOT_DELAY_MS   20

#define IR_CALIBRATION_COUNT    1000
#define IR_CALIBRATION_DELAY_MS 10

#endif