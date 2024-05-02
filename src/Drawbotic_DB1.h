/*!
 * \file Drawbotic_DB1.h
 * \author Elliott Wilson (elliott.wilson@monash.edu)
 * \brief Declares the DB1 class
 * \version 1.0
 * \date 2023-03-13
 */

#ifndef DRAWBOTIC_DB1_H
#define DRAWBOTIC_DB1_H

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#include "db1/inc/drawbotic_types.h"
#include "db1/inc/drawbotic_hal.h"
#include "db1/inc/drawbotic_db1_drv.h"

class Drawbotic_DB1
{
public:
  // Singleton setup
  /*!
   * \brief Accessor for the shared Singleton instance of the Drawbotic_DB1 class
   * \note A simpler way to access the shared instance is to use the globally defined `DB1` variable
   * \return A reference to the shared instance
   */
  static Drawbotic_DB1 &getInstance(); // Accessor for singleton instance

  /*!
   * \brief Initialise DB1 with Default Settings
   * \return true - Initialisation Completed Successfully
   * \return false - Initialisation failed
   */
  int init();

  /*!
   * \brief Initialise DB1 with Custom Settings
   * \param settings A db1_settings_t struct containing the desired settings
   * \return true - Initialisation Completed Successfully
   * \return false - Initialisation failed
   */
  int init(db1_settings_t settings);

  /*!
   * \brief Configues the BNO085 IMU to use the provided settings
   * \param settings A db1_imu_settings_t struct containing the desired settings
   */
  void setupIMU(db1_imu_settings_t settings) {
    db1_init_imu(settings);
  }

  /*!
   * \brief Configures the VEML6040 RGBW colour sensor
   * \param colourIntTime The desired integration time for the sensor
   */
  void setupColourSensor(veml6040_int_time_t colourIntTime) {
    db1_init_colour_sensor(colourIntTime);
  }
  
  /*!
   * \brief Configures the Pen Lift servo to use the desired settings
   * \param settings A db1_servo_settings_t struct containing the desired settings
   */
  void setupServo(db1_servo_settings_t settings) {
    db1_init_servo(settings);
  }
  
  /*!
   * \brief Configures one of the VL53L0X time of flight sensors to the desired config mode
   * \param location The location of the sensor to configure
   * \param config A vl53l0x_config_t value
   */
  void setupToFSensor(db1_tof_location_t location, vl53l0x_config_t config) {
    db1_init_tof(location, config);
  }
  
  /*!
   * \brief Configures the motors
   * \param encoders If true, the encoders will be set up and start counting steps
   */
  void setupMotors(bool useEncoders) {
    db1_init_motors(useEncoders);
  }

  /*!
   * \brief Calibrates the IR array. 
   * \note If motor power is present, the DB1 will spin on the spot and find the highest and lowest values to use as calibration min and max. Make sure you place the DB1 on a sample that shows the expected darkest and lightest value.
   */
  void calibrateIRArray() {
    db1_calibrate_ir_array();
  }

  /*!
   * \brief Sets the IR calibration min and max to the provided values. Useful if you have already determined the lightest and darkest features
   * \param low The desired lowest values for each sensor
   * \param high The desired highest value for each sensor
   */
  void setIRCalibration(db1_ir_array_t low, db1_ir_array_t high) {
    db1_set_ir_calibration(low, high);
  }
  /*!
   * \brief The low calibration value of each sensor
   * \return The low calibration value of each sensor
   */
  db1_ir_array_t getIRLowCalibration() {
    return db1_get_ir_low();
  }
  /*!
   * \brief The high calibration value of each sensor
   * \return The high calibration value of each sensor
   */
  db1_ir_array_t getIRHighCalibration() {
    return db1_get_ir_high();
  }

  /*!
   * \brief Calibrates the VEML6040 colour sensor. 
   * \note The DB1 will turn on the white LED and take an average high/white reading then turn off the LED and take an average low/black reading. Make sure you place the DB1 on a white surface
   */
  void calibrateColourSensor() {
    db1_calibrate_colour();
  }

  /*!
   * \brief Sets the colour calibration to the provided high/white and low/black readings
   * \param low The desired low/black value
   * \param high The desired high/white value
   */
  void setColourCalibration(db1_colour_reading_t low, db1_colour_reading_t high) {
    db1_set_colour_calibration(low, high);
  }
  /*!
   * \brief The low/black calibration value of the sensor
   * \return The low/black calibration value of the sensor
   */
  db1_colour_reading_t getColourLowCalibration() {
    return db1_get_colour_low();
  }

  /*!
   * \brief The high/white calibration value of the sensor
   * \return The high/white calibration value of the sensor
   */
  db1_colour_reading_t getColourHighCalibration() {
    return db1_get_colour_high();
  }

  /*!
   * \brief Turns the White LED for the colour sensor on/off
   * \param on The desired state for the LED
   */
  void setWhiteLight(bool on) {
    db1_set_white_led(on);
  }

  /*!
   * \brief Sets the bottom side RGB LEDs to the provided colours
   * \param lights A db1_lights_t struct containing the desired colours
   */
  void setLights(db1_lights_t lights) {
    db1_set_lights(lights);
  }

  /*!
   * \brief The current colours of the bottom side RGB LEDs
   * \return The current colours of the bottom side RGB LEDs
   */
  db1_lights_t getCurrentLights() {
    return db1_get_lights();
  }

  /*!
   * \brief Sets the top side RGB LED to the provided colour
   * \param light A db1_colour_t containing the desired colour
   */
  void setTopLight(db1_colour_t light) {
    db1_set_top_light(light);
  }

  /*!
   * \brief The current colour of the top side RGB LED
   * \return The current colour of the top side RGB LED
   */
  db1_colour_t getCurrentTopLight() {
    return db1_get_top_light();
  }

  /*!
   * \brief Reads the current battery level
   * \param lights (Default: true) If true, updates the battery LEDs to reflect current level
   * \return The remaining battery percentage
   */
  float updateBatteryLevel(bool lights = true) {
    return db1_update_battery(lights);
  }

  /*!
   * \brief Updates the state of the pen
   * \param down If true, sets the pen to the down position. If false, sets the pen to the up position
   */
  void setPen(bool down) {
    db1_set_pen(down);
  }

  /*!
  * \brief Sets the poistion of the pen to an arbitrary point 
  * 
  * \param pos A position between 0.0 - 1.0 where 0.0 is all the way up and 1.0 is all the way down
  */
  void setPenServo(float pos) {
    db1_set_pen_pos(pos);
  }

  /*!
   * \brief Sets the speed of a specific motor
   * \param motor The motor to set the speed of. Valid values are DB1_M1 or DB1_M2
   * \param speed The speed to set. Valid values are -1.0 to 1.0 where 1.0 is full speed forward, -1.0 is full spped backwards and 0.0 is no movement
   */
  void setMotorSpeed(db1_motor_t motor, float speed) {
    db1_set_motor_speed(motor, speed);
  }

  /*!
   * \brief The current value of the Motor 1 encoder
   * \return The current value of the Motor 1 encoder
   */
  long getM1Encoder() {
    return db1_encoder_val(DB1_M1);
  }

  /*!
   * \brief The current value of the Motor 2 encoder
   * \return The current value of the Motor 2 encoder
   */
  long getM2Encoder() {
    return db1_encoder_val(DB1_M2);
  }

  /*!
   * \brief The change in the Motor 1 Encoder since the last call to this method
   * \return The change in the Motor 1 Encoder since the last call to this method
   */
  long getM1EncoderDelta() {
    return db1_encoder_delta(DB1_M1);
  }

  /*!
   * \brief The change in the Motor 2 Encoder since the last call to this method
   * \return The change in the Motor 2 Encoder since the last call to this method
   */
  long getM2EncoderDelta() {
    return db1_encoder_delta(DB1_M2);
  }

  /*!
   * \brief Resets the delta values of the encoders.
   * \note This should be done when starting a new encoder based operation as the deltas can be incorrect if not periodically read.
   */
  void resetEncoderDeltas() {
    db1_reset_encoder_deltas();
  }

  /*!
   * \brief Reads the current colour from the VEML6040 colour sensor
   * \param calibrated (Default: true) If true, the returned Colour will be corrected by the current calibration values
   * \return The read colour value 
   */
  db1_colour_reading_t getColour(bool calibrated = true) {
    return db1_read_colour(calibrated);
  }
  
  /*!
   * \brief Reads the current values from the IR line detectors
   * \param calibrated (Default: true) If true, the returned values will be corrected by the current calibration values
   * \return The read IR values 
   */
  db1_ir_array_t getIRSensors(bool calibrated = true) {
    return db1_read_ir(calibrated);
  }

  /*!
   * \brief Reads the current range from a specific VL53L0X Time of Flight Sensor
   * \param location Which Time of Flight Sensor to read. Valid values are:\n TOF_LEFT\n TOF_CENTRE\n TOF_RIGHT
   * \return The read range in millimetres
   */
  int getToFSensor(db1_tof_location_t location) {
    return db1_read_tof(location);
  }

  /*!
   * \brief The current Rotation (Quaternion) of the DB1
   * \return The current Rotation (Quaternion) of the DB1
   */
  db1_quaternion_t getRotation() {
    return db1_read_rotation();
  }

  /*!
   * \brief The current Orientation (heading, pitch, roll) of the DB1
   * \return The current Orientation (heading, pitch, roll) of the DB1
   */
  db1_orientation_t getOrientation() {
    return db1_read_orientation();
  }

  /*!
   * \brief The current Linear Acceleration of the DB1
   * \return The current Linear Acceleration of the DB1
   */
  db1_vector_t getAcceleration() {
    return db1_read_acceleration();
  }

  /*!
   * \brief Checks to see if the DB1 experienced a high g bump
   * \note This method needs to be called periodically to work correctly (with 50-100 milliseconds)
   * \return true if a bump was detected, false if not.
   */
  bool wasBumped() {
    return db1_check_bump();
  }

  /*!
   * \brief The current settings for the DB1
   * \return The current settings for the DB1
   */
  db1_settings_t getCurrentSettings() {
    return db1_get_current_settings();
  }

  /*!
   * \brief Delete the copy constructor for Singleton pattern
   */
  Drawbotic_DB1(const Drawbotic_DB1 &) = delete; // no copying
  
  /*!
   * \brief Delete the assignment operator for Singleton pattern
   */
  Drawbotic_DB1 &operator=(const Drawbotic_DB1 &) = delete;

  void writeServo(uint32_t pin, uint8_t val);
  void setPixel(uint32_t pin, uint16_t index, uint8_t r, uint8_t g, uint8_t b);
  void fillPixels(uint32_t pin, uint8_t r, uint8_t g, uint8_t b);

private:
  // Private constructor for singleton
  Drawbotic_DB1();

  db1_hal_t m_hal;
  Servo m_pen_servo;
  Adafruit_NeoPixel m_top_light;
  Adafruit_NeoPixel m_lights;
};

/*!
 * \brief The global reference to the shared instace of the Drawbotic_DB1 class
 */
extern Drawbotic_DB1 &DB1;

#endif