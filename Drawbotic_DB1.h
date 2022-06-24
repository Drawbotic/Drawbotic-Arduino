#ifndef DRAWBOTICDB1_H
#define DRAWBOTICDB1_H

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Drawbotic_VEML6040.h>
#include <VL53L0X.h>
//#include <BMX160.h>

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
    DB1_Vector3 accel;
    DB1_Vector3 gyro;
    DB1_Vector3 mag;
    float heading;
};

struct DB1_IRArray
{
    int farLeft;
    int left;
    int centre;
    int right;
    int farRight;
};

/*struct DB1_IMUSettings
{
    BMX160_AccelRate accelRate;
    BMX160_AccelRange accelRange;
    BMX160_GyroRate gyroRate;
    BMX160_GyroRange gyroRange;
};*/

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
    //DB1_IMUSettings imu;
    DB1_ServoSettings servo;
    DB1_ToFSettings tof;
    VEML6040_IntegrationTime colourIntTime;
    bool useEncoders;
    bool whiteLightOn;
    int irReadCount;
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
    //void SetupIMU(DB1_IMUSettings settings);
    void SetupColourSensor(VEML6040_IntegrationTime colourIntTime);
    void SetupServo(DB1_ServoSettings settings);
    void SetupToFSensor(int number, DB1_ToFSettings settings);
    void SetupMotors(bool useEncoders);

    //Sensor calibrations
    void CalibrateIRArray();
    void SetIRCalibration(DB1_IRArray low, DB1_IRArray high);
    void CalibrateColourSensor();
    //void CalibrateIMU();
    
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
    VEML6040_Colour ReadColour(bool calibrated = true);
    int ReadToFSensor(DB1_ToFLocation location);
    DB1_IRArray ReadIRSensors(bool calibrated = true);
    //DB1_Motion ReadIMU();
    //void EnableBumpInterrupt(uint8_t threshold = 20, uint8_t duration = 10, BMX160_InterruptPin pin = BMX160_INT_PIN_1);
    //void DisableBumpInterrupt();
    //void EnableIMUReadyInterrupt(BMX160_InterruptPin pin = BMX160_INT_PIN_1);
    //void DisableIMUReadyInterrupt();

    //Settings structure
    DB1_Settings GetCurrentSettings() { return m_currentSettings; }
    static DB1_Settings GetDefaultSettings() { return s_defaultSettings; }

private:
    DB1_Settings m_currentSettings;

    Servo m_penLift;
    Adafruit_NeoPixel m_lights;
    Drawbotic_VEML6040 m_colourSensor;
    VL53L0X m_tofs[TOF_COUNT];
    //BMX160* m_imu;

    DB1_Lights m_currentLights;

    DB1_IRArray m_irHigh;
    DB1_IRArray m_irLow;

    VEML6040_Colour m_colourHigh;

    int m_m1EnALastState;
    int m_m2EnALastState;

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