#ifndef DRAWBOT_H
#define DRAWBOT_H

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>
#include <BMX160.h>

#include "DrawBotDefines.h"

struct DrawBot_Colour
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct DrawBot_Lights
{
    DrawBot_Colour colours[LIGHT_COUNT];
};

struct DrawBot_Vector3
{
    float x, y, z;
};

struct DrawBot_Motion
{
    DrawBot_Vector3 accel;
    DrawBot_Vector3 gyro;
    DrawBot_Vector3 mag;
};

struct DrawBot_IRArray
{
    uint8_t farLeft;
    uint8_t left;
    uint8_t centre;
    uint8_t right;
    uint8_t farRight;
};

struct DrawBot_IMUSettings
{
    BMX160_AccelRate accelRate;
    BMX160_AccelRange accelRange;
    BMX160_GyroRate gyroRate;
    BMX160_GyroRange gyroRange;
};

struct DrawBot_ColourSettings
{
    tcs34725Gain_t gain;
    tcs34725IntegrationTime_t intergrationTime;
};

struct DrawBot_ServoSettings
{
    int pin;
    double penUpPosition;
    double penDownPosition;
};

struct DrawBot_ToFSettings
{
    int timeout;
    float signalRateLimit;
    uint32_t measurementTimingBudget;
    uint8_t prePclks;
    uint8_t finalPclks;
};

struct DrawBot_MotorSettings
{
    bool enabled;
    bool useEncoders;
};

struct DrawBot_Settings
{
    DrawBot_IMUSettings imu;
    DrawBot_ColourSettings colourSensor;
    DrawBot_ServoSettings servo;
    DrawBot_ToFSettings tof;
    DrawBot_MotorSettings motors;
    bool whiteLightOn;
    int irDimLevel;
};

enum DrawBot_ToFLocation
{
    TOF_LEFT,
    TOF_CENTRE,
    TOF_RIGHT
};

class DrawBot
{
public:
    DrawBot();

    //Overall bot setup
    bool Initialise();
    bool Initialise(DrawBot_Settings settings);
    
    //Setting individual bot elements
    void SetWhiteLight(bool on);
    void SetIRDimLevel(int level);
    void SetupIMU(DrawBot_IMUSettings settings);
    void SetupColourSensor(DrawBot_ColourSettings settings);
    void SetupServo(DrawBot_ServoSettings settings);
    void SetupToFSensor(int number, DrawBot_ToFSettings settings);
    void SetupMotors(DrawBot_MotorSettings settings);

    //Sensor calibrations
    void CalibrateIRArray();
    void CalibrateIMU();
    
    //RGB Lighting
    void SetLights(DrawBot_Lights lights);
    DrawBot_Lights GetCurrentLights() { return m_currentLights; }

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
    DrawBot_Colour ReadColour();
    int ReadToFSensor(DrawBot_ToFLocation location);
    DrawBot_IRArray ReadIRSensors(bool calibrated = true);
    DrawBot_Motion ReadIMU();
    void EnableBumpInterrupt(uint8_t threshold = 20, uint8_t duration = 10, BMX160_InterruptPin pin = BMX160_INT_PIN_1);
    void DisableBumpInterrupt();

    //Settings structure
    DrawBot_Settings GetCurrentSettings() { return m_currentSettings; }
    static DrawBot_Settings GetDefaultSettings() { return s_defaultSettings; }

private:
    DrawBot_Settings m_currentSettings;

    Servo m_penLift;
    Adafruit_NeoPixel m_lights;
    Adafruit_TCS34725 m_colourTCS;
    VL53L0X m_tofs[TOF_COUNT];
    BMX160* m_imu;

    DrawBot_Lights m_currentLights;

    int m_irHigh[IR_COUNT];
    int m_irLow[IR_COUNT];

    long m_m1En;
    long m_m2En;
    long m_lastM1En;
    long m_lastM2En;
    double m_m1Speed;
    double m_m2Speed;

    static DrawBot* s_instance;
    const static DrawBot_Settings s_defaultSettings;

    static void m1EncoderCallBack();
    static void m2EncoderCallBack();

    double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif