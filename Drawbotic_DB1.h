#ifndef DRAWBOTICDB1_H
#define DRAWBOTICDB1_H

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Drawbotic_VEML6040.h>
#include <VL53L0X.h>
#include <Adafruit_BNO08x.h>

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

struct DB1_Quaternion
{
  float r, i, j, k;
};

struct DB1_Orientation
{
  float heading, pitch, roll;
};

struct DB1_IRArray
{
  int farLeft;
  int left;
  int centre;
  int right;
  int farRight;
};

struct DB1_IMUSettings
{
  uint16_t orientationRate_ms;
  uint16_t accelerationRate_ms;
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

typedef void (*DB1_BumpInt_t)();

class DB1
{
public:
  DB1();

  //Overall bot setup
  bool init();
  bool init(DB1_Settings settings);
  
  //Setting individual bot elements
  void setWhiteLight(bool on);
  void setupIMU(DB1_IMUSettings settings);
  void setupColourSensor(VEML6040_IntegrationTime colourIntTime);
  void setupServo(DB1_ServoSettings settings);
  void setupToFSensor(int number, DB1_ToFSettings settings);
  void setupMotors(bool useEncoders);

  //Sensor calibrations
  void calibrateIRArray();
  void setIRCalibration(DB1_IRArray low, DB1_IRArray high);
  void calibrateColourSensor();
  
  //RGB Lighting
  void setLights(DB1_Lights lights);
  DB1_Lights getCurrentLights() { return m_currentLights; }
  void setTopLight(DB1_Colour light);
  DB1_Colour getCurrentTopLight() { return m_currentTopLight; }

  //Battery Fuel Guage
  float updateBatteryLevel(bool lights = true);

  //Servo control
  void setPen(bool down);
  void setPenServo(double pos);

  //Motor control
  void setMotorSpeed(int motor, double speed);
  long getM1Encoder() { return m_m1En; }
  long getM2Encoder() { return m_m2En; }

  long getM1EncoderDelta();
  long getM2EncoderDelta();

  //Sensor access
  VEML6040_Colour readColour(bool calibrated = true);
  int readToFSensor(DB1_ToFLocation location);
  DB1_IRArray readIRSensors(bool calibrated = true);
  DB1_Quaternion getRotation() { return m_currentRotation; }
  DB1_Orientation getOrientation() { return DB1::QuaternionToEuler(m_currentRotation); }
  DB1_Vector3 getAcceleration() { return m_currentAccel; }
  void enableBumpInterrupt(DB1_BumpInt_t callback, uint32_t threshold_mg = 1000);
  void disableBumpInterrupt();

  //Settings structure
  DB1_Settings getCurrentSettings() { return m_currentSettings; }
  static DB1_Settings getDefaultSettings() { return s_defaultSettings; }

  //Helper Functions
  static DB1_Orientation QuaternionToEuler(DB1_Quaternion q);

private:
  DB1_Settings m_currentSettings;

  Servo m_penLift;
  Adafruit_NeoPixel m_lights;
  Adafruit_NeoPixel m_topLight;
  Drawbotic_VEML6040 m_colourSensor;
  VL53L0X m_tofs[TOF_COUNT];
  Adafruit_BNO08x m_imu;

  DB1_Quaternion m_currentRotation;
  DB1_Vector3 m_currentAccel;

  DB1_Lights m_currentLights;
  DB1_Colour m_currentTopLight;

  DB1_IRArray m_irHigh;
  DB1_IRArray m_irLow;

  VEML6040_Colour m_colourLow;
  VEML6040_Colour m_colourHigh;

  int m_m1EnALastState;
  int m_m2EnALastState;

  long m_m1En;
  long m_m2En;
  long m_lastM1En;
  long m_lastM2En;
  double m_m1Speed;
  double m_m2Speed;

  DB1_BumpInt_t m_bumpCallback;
  uint32_t m_bumpThreshold;

  static DB1* s_instance;
  const static DB1_Settings s_defaultSettings;

  static void m1EncoderCallback();
  static void m2EncoderCallback();
  static void bnoIntCallback();

  double mapf(double val, double in_min, double in_max, double out_min, double out_max)
  {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
};

#endif