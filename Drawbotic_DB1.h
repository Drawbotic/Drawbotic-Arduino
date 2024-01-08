/*!
 * \file Drawbotic_DB1.h
 * \author Elliott Wilson (elliott.wilson@monash.edu)
 * \brief Declares the DB1 class and required structs and enums
 * \version 1.0
 * \date 2023-03-13
 */

#ifndef DRAWBOTIC_DB1_H
#define DRAWBOTIC_DB1_H

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Drawbotic_VEML6040.h>
#include <VL53L0X.h>
#include <Adafruit_BNO08x.h>

#include "Drawbotic_DB1_Defines.h"

/*!
 * \brief A struct to represent an RGB colour
 */
struct DB1_Colour {
  uint8_t red;    //!< The red component
  uint8_t green;  //!< The green component
  uint8_t blue;   //!< The blue component
};

/*!
 * \brief A struct containing the RGB colours for all of the RGB LEDs on board
 */
struct DB1_Lights {
  DB1_Colour colours[LIGHT_COUNT];  //!< An array containing the colour value for each light
};

/*!
 * \brief A 3D vector struct used to represent DB1 acceleration
 */
struct DB1_Vector3 {
  float x;  //!< The x component
  float y;  //!< The y component
  float z;  //!< The z component
};

/*!
 * \brief A 4D quaternion stuct used to represent DB1 orientation
 */
struct DB1_Quaternion {
  float r;  //!< The real component
  float i;  //!< The i component
  float j;  //!< The j component
  float k;  //!< The k component
};

/*!
 * \brief A struct used to represent the Euler Orientation of the DB1
 */
struct DB1_Orientation {
  float heading;  //!< The heading (Z) component
  float pitch;    //!< The pitch (X) component
  float roll;     //!< The roll (Y) component
};

/*!
 * \brief A struct used to represent the state of the IR line detectors
 */
struct DB1_IRArray {
  int farLeft;  //!< The value of the far left IR line detector
  int left;     //!< The value of the left IR line detector
  int centre;   //!< The value of the centre IR line detector
  int right;    //!< The value of the right IR line detector
  int farRight; //!< The value of the far right IR line detector
};

/*!
 * \brief A struct containing the settings for the BNO085 IMU
 */
struct DB1_IMUSettings {
  uint16_t orientationRate_ms;  //!< The sample rate of the orientation in milliseconds
  uint16_t accelerationRate_ms; //!< The sample rate of the acceleration in milliseconds
};

/*!
 * \brief A struct containing the settings for the Pen Lift Servo
 */
struct DB1_ServoSettings {
  int pin;                //!< The pin that the servo is attached to
  double penUpPosition;   //!< The servo position for when the pen is up
  double penDownPosition; //!< The servo position for when the pen is down
};

/*!
 * \brief A struct containing the settings for the VL53L0X Time of Flight sensors
 */
struct DB1_ToFSettings {
  int timeout; //!< The timeout period in milliseconds, used to determine the amount of time before a reading timeout is raised
  float signalRateLimit; //!< The signal rate limit in mega counts per second, this limits the amplitude of the signal reflected from the target and detected by the device.
  uint32_t measurementTimingBudget; //!< The total measurement timing budget in microseconds, this is overall time each measurement takes.
  uint8_t prePclks; //!< The Pre VCSEL Pulse Period
  uint8_t finalPclks; //!< The Final VCSEL Pulse Period
  uint16_t rate_ms; //!< The sample rate of the sensor
};

/*!
 * \brief A struct containing all of the settings need to initialise DB1
 */
struct DB1_Settings {
  DB1_IMUSettings imu;                    //!< The IMU settings, see DB1_IMUSettings
  DB1_ServoSettings servo;                //!< The Servo settings, see DB1_ServoSettings
  DB1_ToFSettings tof;                    //!< The Time of Flight settings, see DB1_ToFSettings
  VEML6040_IntegrationTime colourIntTime; //!< The integration time for the VEML6040 colour sensor. \n Supported values are:\n VEML6040_IT_40MS\n VEML6040_IT_80MS\n VEML6040_IT_160MS\n VEML6040_IT_320MS\n VEML6040_IT_640MS\n VEML6040_IT_1280MS
  bool useEncoders;                       //!< Sets if the encoders should be enabled
  bool whiteLightOn;                      //!< Sets of the white LED for the colour sensor should be on by default
  int irReadCount;                        //!< Sets the average window size for reading the IR line detectors, higher values increase accuracy but changes take longer to appear
};

/*!
 * \brief An enum describing the Time of Flight sensor locations, see getToFSensor
 */
enum DB1_ToFLocation {
  TOF_LEFT,
  TOF_CENTRE,
  TOF_RIGHT
};

//! The bump interrupt callback type
typedef void (*DB1_BumpInt_t)();

/*!
 * \brief The class containing all of the functionality of the Drawbotic DB1. 
 * \note This class is implemented as a static Singleton. No instance of the class should be created by the user. The shared instance can be access via the globally defined DB1 variable.
 */
class Drawbotic_DB1 {
public:
  //Singleton setup
  /*!
   * \brief Accessor for the shared Singleton instance of the Drawbotic_DB1 class
   * \note A simpler way to access the shared instance is to use the globally defined `DB1` variable
   * \return A reference to the shared instance
   */
  static Drawbotic_DB1 &getInstance(); // Accessor for singleton instance

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
  /*!
   * \brief The low calibration value of each sensor
   * \return The low calibration value of each sensor 
   */
  DB1_IRArray getIRLowCalibration() { return m_irLow; }
  /*!
   * \brief The high calibration value of each sensor
   * \return The high calibration value of each sensor 
   */
  DB1_IRArray getIRHighCalibration() { return m_irHigh; }
  void calibrateColourSensor();
  void setColourCalibration(VEML6040_Colour low, VEML6040_Colour high);
  /*!
   * \brief The low/black calibration value of the sensor
   * \return The low/black calibration value of the sensor
   */
  VEML6040_Colour getColourLowCalibration() { return m_colourLow; }
  /*!
   * \brief The high/white calibration value of the sensor
   * \return The high/white calibration value of the sensor 
   */
  VEML6040_Colour getColourHighCalibration() { return m_colourHigh; }
  
  //RGB Lighting
  void setLights(DB1_Lights lights);
  /*!
   * \brief The current colours of the bottom side RGB LEDs
   * \return The current colours of the bottom side RGB LEDs 
   */
  DB1_Lights getCurrentLights() { return m_currentLights; }
  void setTopLight(DB1_Colour light);
  /*!
   * \brief The current colour of the top side RGB LED
   * \return The current colour of the top side RGB LED
   */
  DB1_Colour getCurrentTopLight() { return m_currentTopLight; }

  //Battery Fuel Guage
  float updateBatteryLevel(bool lights = true);

  //Servo control
  void setPen(bool down);
  void setPenServo(double pos);

  //Motor control
  void setMotorSpeed(int motor, double speed);
  /*!
   * \brief The current value of the Motor 1 encoder
   * \return The current value of the Motor 1 encoder 
   */
  long getM1Encoder() { return m_m1En; }
  /*!
   * \brief The current value of the Motor 2 encoder
   * \return The current value of the Motor 2 encoder 
   */
  long getM2Encoder() { return m_m2En; }

  long getM1EncoderDelta();
  long getM2EncoderDelta();
  void resetEncoderDeltas();

  //Sensor access
  void startSensors();
  void stopSensors();

  VEML6040_Colour getColour(bool calibrated = true);
  DB1_IRArray getIRSensors(bool calibrated = true);
  int getToFSensor(DB1_ToFLocation location);
  /*!
   * \brief The current Rotation (Quaternion) of the DB1
   * \return The current Rotation (Quaternion) of the DB1 
   */
  DB1_Quaternion getRotation() { return m_currentRotation; }
  /*!
   * \brief The current Orientation (heading, pitch, roll) of the DB1
   * \return The current Orientation (heading, pitch, roll) of the DB1 
   */
  DB1_Orientation getOrientation() { return Drawbotic_DB1::QuaternionToEuler(m_currentRotation); }
  /*!
   * \brief The current Linear Acceleration of the DB1
   * \return The current Linear Acceleration of the DB1 
   */
  DB1_Vector3 getAcceleration() { return m_currentAccel; }
  void enableBumpInterrupt(DB1_BumpInt_t callback, float threshold_g = 1.0f);
  void disableBumpInterrupt();

  //Settings structure
  /*!
   * \brief The current settings for the DB1
   * \return The current settings for the DB1 
   */
  DB1_Settings getCurrentSettings() { return m_currentSettings; }

  //Singleton Specific
  /*!
   * \brief Delete the copy constructor for Singleton pattern
   */
  Drawbotic_DB1(const Drawbotic_DB1 &) = delete; // no copying
  /*!
   * \brief Delete the assignment operator for Singleton pattern
   */
  Drawbotic_DB1 &operator=(const Drawbotic_DB1 &) = delete;

  /*!
   * \brief The default settings for the DB1
   * \return The default settings for the DB1 
   */
  static DB1_Settings getDefaultSettings() { return s_defaultSettings; }

  //Helper Functions
  static DB1_Orientation QuaternionToEuler(DB1_Quaternion q);

private:
  //Private constructor for singleton
  Drawbotic_DB1();
  DB1_Settings m_currentSettings;

  Servo m_penLift;
  Adafruit_NeoPixel m_lights;
  Adafruit_NeoPixel m_topLight;
  Drawbotic_VEML6040 m_colourSensor;
  VL53L0X m_tofs[TOF_COUNT];
  Adafruit_BNO08x m_imu;
  SoftwareTimer m_sensorTimer;
  bool m_sensorsRunning;

  DB1_Quaternion m_currentRotation;
  DB1_Vector3 m_currentAccel;

  int m_tofValues[TOF_COUNT];
  VEML6040_Colour m_currentColour;

  DB1_Lights m_currentLights;
  DB1_Colour m_currentTopLight;

  //IR calibration values
  DB1_IRArray m_irHigh;
  DB1_IRArray m_irLow;

  //Colour calibration values
  VEML6040_Colour m_colourLow;
  VEML6040_Colour m_colourHigh;

  //Encoder memory
  int m_m1EnALastState;
  int m_m2EnALastState;
  long m_m1En;
  long m_m2En;
  long m_lastM1En;
  long m_lastM2En;

  //Motor speeds
  double m_m1Speed;
  double m_m2Speed;

  //Bump values
  DB1_BumpInt_t m_bumpCallback;
  float m_bumpThreshold;

  const static DB1_Settings s_defaultSettings;

  static void m1EncoderCallback();
  static void m2EncoderCallback();

  static void sensorTask(void*);
  static void setIMUReports();
  static long s_lastSensor;
  static int s_tofTimeBank;
  static int s_colourTimebank;

  double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
};

/*!
 * \brief The global reference to the shared instace of the Drawbotic_DB1 class
*/
extern Drawbotic_DB1 &DB1;

#endif