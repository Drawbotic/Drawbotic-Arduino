#ifndef DRAWBOTICDB1_H
#define DRAWBOTICDB1_H

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>
#include <BMX160.h>

#include "Drawbotic_DB1_Defines.h"

struct DB1_Colour
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct DB1_Lights
{
    DB1_Colour colours[LIGHT_COUNT];
};

struct DB1_Vector3
{
    float x, y, z;
};

struct DB1_Motion
{
    DrawBot_Vector3 accel;
    DrawBot_Vector3 gyro;
    DrawBot_Vector3 mag;
};

struct DB1_IRArray
{
    uint8_t farLeft;
    uint8_t left;
    uint8_t centre;
    uint8_t right;
    uint8_t farRight;
};

struct DB1_IMUSettings
{
    BMX160_AccelRate accelRate;
    BMX160_AccelRange accelRange;
    BMX160_GyroRate gyroRate;
    BMX160_GyroRange gyroRange;
};

struct DB1_ColourSettings
{
    tcs34725Gain_t gain;
    tcs34725IntegrationTime_t intergrationTime;
};

struct DB1_ServoSettings
{
    int pin;
    double penUpPosition;
    double penDownPosition;
};

struct DB1_ToFSettings
{
    int timeout;
    float signalRateLimit;
    uint32_t measurementTimingBudget;
    uint8_t prePclks;
    uint8_t finalPclks;
};

struct DB1_Settings
{
    DB1_IMUSettings imu;
    DB1_ColourSettings colourSensor;
    DB1_ServoSettings servo;
    DB1_ToFSettings tof;
    bool useEncoders;
    bool m1Flipped;
    bool m2Flipped;
    bool whiteLightOn;
    int irDimLevel;
};

enum DB1_ToFLocation
{
    TOF_LEFT,
    TOF_CENTRE,
    TOF_RIGHT
};

class DB1
{
public:
    DB1();

    //Overall bot setup
    bool Initialise();
    bool Initialise(DB1_Settings settings);
    
    //Setting individual bot elements
    void SetWhiteLight(bool on);
    void SetIRDimLevel(int level);
    void SetupIMU(DB1_IMUSettings settings);
    void SetupColourSensor(DB1_ColourSettings settings);
    void SetupServo(DB1_ServoSettings settings);
    void SetupToFSensor(int number, DB1_ToFSettings settings);
    void SetupMotors(bool useEncoders);

    //Sensor calibrations
    void CalibrateIRArray();
    void CalibrateIMU();
    
    //RGB Lighting
    void SetLights(DB1_Lights lights);
    DB1_Lights GetCurrentLights() { return m_currentLights; }

    //Battery Fuel Guage
    float UpdateBatteryLevel(bool lights = true);

    //Servo control
    void SetPenUp(bool up);
    void SetPenServo(double pos);

    //Motor control
    void SetMotorSpeed(int motor, double speed);
    long GetM1Encoder() { return m_m1En; }
    long GetM2Encoder() { return m_m2En; }

    long GetM1EncoderDelta();
    long GetM2EncoderDelta();

    //Sensor access
    DB1_Colour ReadColour();
    int ReadToFSensor(DB1_ToFLocation location);
    DB1_IRArray ReadIRSensors(bool calibrated = true);
    DB1_Motion ReadIMU();
    void EnableBumpInterrupt(uint8_t threshold = 20, uint8_t duration = 10, BMX160_InterruptPin pin = BMX160_INT_PIN_1);
    void DisableBumpInterrupt();

    //Settings structure
    DB1_Settings GetCurrentSettings() { return m_currentSettings; }
    static DB1_Settings GetDefaultSettings() { return s_defaultSettings; }

private:
    DB1_Settings m_currentSettings;

    Servo m_penLift;
    Adafruit_NeoPixel m_lights;
    Adafruit_TCS34725 m_colourTCS;
    VL53L0X m_tofs[TOF_COUNT];
    BMX160* m_imu;

    DB1_Lights m_currentLights;

    int m_irHigh[IR_COUNT];
    int m_irLow[IR_COUNT];

    long m_m1En;
    long m_m2En;
    long m_lastM1En;
    long m_lastM2En;
    double m_m1Speed;
    double m_m2Speed;

    static DB1* s_instance;
    const static DB1_Settings s_defaultSettings;

    static void m1EncoderCallBack();
    static void m2EncoderCallBack();

    double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif