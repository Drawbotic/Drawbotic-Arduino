#include "Drawbotic_DB1.h"

//------- Static Methods -------//
DB1 *DB1::s_instance = NULL;

const DB1_Settings DB1::s_defaultSettings = {
    {IMU_ORIENT_MS_DEFAULT, IMU_ACCEL_MS_DEFAULT},      // IMU defaults
    {SERVO_PWM, SERVO_UP_DEFAULT, SERVO_DOWN_DEFAULT},  // Servo defaults
    {TOF_TIMEOUT_DEFAULT, TOF_SIGLIM_DEFAULT,
     TOF_TIMING_BUDGET_DEFAULT, TOF_PRE_PCLKS_DEFAULT,
     TOF_FIN_PCLKS_DEFAULT},                            // ToF defaults
    VEML6040_IT_40MS,                                   // Default colour sensor int time
    true,                                               // Use encoders
    false,                                              // White Light default
    IR_READ_COUNT_DEFAULT,                              // IR Read Count
};

/*!
 * \brief Converts a DB1_Quaternion to a Heading, Pitch and Roll Orientation
 * 
 * \param q - The Quaternion to convert
 * \return DB1_Orientation - A new Heading, Pitch and Roll
 */
DB1_Orientation DB1::QuaternionToEuler(DB1_Quaternion q) {
  DB1_Orientation result;

  float sqr = q.r * q.r;
  float sqi = q.i * q.i;
  float sqj = q.j * q.j;
  float sqk = q.k * q.k;

  result.heading = atan2(2.0 * (q.i * q.j + q.k * q.r), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
  result.pitch = asin(-2.0 * (q.i * q.k - q.j * q.r) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
  result.roll = atan2(2.0 * (q.j * q.k + q.i * q.r), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;

  return result;
}

void DB1::m1EncoderCallback() {
  if (s_instance) {
    int m1EnAState = digitalRead(M1_E_A);

    if (m1EnAState != s_instance->m_m1EnALastState && m1EnAState == 1) {
      if (digitalRead(M1_E_B) != m1EnAState)
        s_instance->m_m1En--;
      else
        s_instance->m_m1En++;
    }

    s_instance->m_m1EnALastState = m1EnAState;
  }
}

void DB1::m2EncoderCallback() {
  if (s_instance) {
    int m2EnAState = digitalRead(M2_E_A);

    if (m2EnAState != s_instance->m_m2EnALastState && m2EnAState == 1) {
      if (digitalRead(M2_E_B) != m2EnAState)
        s_instance->m_m2En++;
      else
        s_instance->m_m2En--;
    }

    s_instance->m_m2EnALastState = m2EnAState;
  }
}

void DB1::bnoIntCallback() {
  if(s_instance) {
    sh2_SensorValue_t sensorValue;
    if (!s_instance->m_imu.getSensorEvent(&sensorValue))
      return;

    switch(sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        s_instance->m_currentAccel.x = sensorValue.un.linearAcceleration.x;
        s_instance->m_currentAccel.y = sensorValue.un.linearAcceleration.y;
        s_instance->m_currentAccel.z = sensorValue.un.linearAcceleration.z;
        if(s_instance->m_bumpCallback != NULL) {
          //Check threshold
          float mag_sq = s_instance->m_currentAccel.x * s_instance->m_currentAccel.x + 
                         s_instance->m_currentAccel.y * s_instance->m_currentAccel.y + 
                         s_instance->m_currentAccel.z * s_instance->m_currentAccel.z;
          if(mag_sq > (s_instance->m_bumpThreshold * s_instance->m_bumpThreshold)) {
            s_instance->m_bumpCallback();
          }
        }
        break;
      case SH2_ARVR_STABILIZED_RV:
        s_instance->m_currentRotation.r = sensorValue.un.arvrStabilizedRV.real;
        s_instance->m_currentRotation.i = sensorValue.un.arvrStabilizedRV.i;
        s_instance->m_currentRotation.j = sensorValue.un.arvrStabilizedRV.j;
        s_instance->m_currentRotation.k = sensorValue.un.arvrStabilizedRV.k;
        break;
    }
  }
}

//------- Public Methods -------//
/*!
 * \brief Construct a new DB1 object
 * 
 */
DB1::DB1() : m_lights(LIGHT_COUNT, RGB_DOUT, NEO_GRB + NEO_KHZ800),
             m_topLight(1, STAT_DOUT, NEO_GRB + NEO_KHZ800),
             m_imu(IMU_RESET) {
  for (int i = 0; i < LIGHT_COUNT; i++) {
    m_currentLights.colours[i].red = 0;
    m_currentLights.colours[i].green = 0;
    m_currentLights.colours[i].blue = 0;
  }
  m_currentTopLight = {0};
  m_irHigh = {0};
  m_irLow = {0};
  m_m1En = 0;
  m_m2En = 0;
  m_lastM1En = 0;
  m_lastM2En = 0;

  m_currentSettings = getDefaultSettings();

  s_instance = this;
}

/*!
 * \brief Initialise DB1 with Default Settings
 * 
 * \return true - Initialisation Completed Successfully
 * \return false - Initialisation failed
 */
bool DB1::init() {
  return init(DB1::getDefaultSettings());
}

/*!
 * \brief Initialise DB1 with Custom Settings
 * 
 * \param settings - A DB1_Settings struct containing the desired settings
 * \return true - Initialisation Completed Successfully
 * \return false - Initialisation failed
 */
bool DB1::init(DB1_Settings settings) {
  Wire.begin();
  analogWriteResolution(16);

  // Set up motor driver
  setupMotors(settings.useEncoders);
  Serial.println("Motors Done");

  pinMode(LINE1, INPUT);
  pinMode(LINE2, INPUT);
  pinMode(LINE3, INPUT);
  pinMode(LINE4, INPUT);
  pinMode(LINE5, INPUT);

  pinMode(BATT_LVL1, OUTPUT);
  pinMode(BATT_LVL2, OUTPUT);
  pinMode(BATT_LVL3, OUTPUT);
  pinMode(BATT_LVL4, OUTPUT);

  digitalWrite(BATT_LVL1, LOW);
  digitalWrite(BATT_LVL2, LOW);
  digitalWrite(BATT_LVL3, LOW);
  digitalWrite(BATT_LVL4, LOW);

  // Setup white LED
  pinMode(LED_EN, OUTPUT);
  setWhiteLight(settings.whiteLightOn);
  m_currentSettings.whiteLightOn = settings.whiteLightOn;
  Serial.println("White Done");

  // Setup servo
  m_penLift.attach(settings.servo.pin);
  Serial.println("Servo Done");

  // Turn off ToFs
  for (int i = 0; i < TOF_COUNT; i++) {
    pinMode(TOF_EN_PINS[i], OUTPUT);
    digitalWrite(TOF_EN_PINS[i], LOW);
  }
  Serial.println("Turn off ToFs");

  // Setup tof sensors
  for (int i = 0; i < TOF_COUNT; i++) {
    digitalWrite(TOF_EN_PINS[i], HIGH);
    Serial.print("Turnning on pin ");
    Serial.println(TOF_EN_PINS[i]);
    delay(TOF_BOOT_DELAY_MS);
    m_tofs[i] = VL53L0X();
    m_tofs[i].setAddress(TOF_ADDRESSES[i]);
    setupToFSensor(i, settings.tof);
    Serial.print("Set up complete for ToF 0x");
    Serial.println(TOF_ADDRESSES[i], HEX);
  }
  Serial.println("Tofs Done");

  setupColourSensor(settings.colourIntTime);

  // Setup IMU
  setupIMU(settings.imu);
  Serial.println("IMU Done");

  // Setup neopixels
  m_lights.begin();
  setLights(m_currentLights);
  m_lights.show();

  m_topLight.begin();
  setTopLight(m_currentTopLight);
  m_topLight.show();

  Serial.println("RGB Done");

  setPen(false);
  setWhiteLight(false);

  return true;
}

/*!
 * \brief Turns the White LED for the colour sensor on/off
 * 
 * \param on - the desired state for the LED
 */
void DB1::setWhiteLight(bool on) {
  m_currentSettings.whiteLightOn = on;
  digitalWrite(LED_EN, on);
}

/*!
 * \brief Configues the BNO085 IMU to use the provided settings
 * 
 * \param settings - A DB1_IMUSettings struct containing the desired settings
 */
void DB1::setupIMU(DB1_IMUSettings settings) {
  pinMode(IMU_INT, INPUT_PULLUP);
  if(m_imu.begin_I2C())
    Serial.println("BNO085 started");
  else
    Serial.println("Could not start BNO085");

  m_imu.enableReport(SH2_LINEAR_ACCELERATION, settings.accelerationRate_ms * 1000);
  m_imu.enableReport(SH2_ARVR_STABILIZED_RV, settings.orientationRate_ms * 1000);

  attachInterrupt(IMU_INT, bnoIntCallback, FALLING);
  m_currentSettings.imu = settings;
}

/*!
 * \brief Configures the VEML6040 RGBW colour sensor
 * 
 * \param colourIntTime - The desired integration time for the sensor
 */
void DB1::setupColourSensor(VEML6040_IntegrationTime colourIntTime) {
  m_colourSensor.begin();
  m_colourSensor.setConfig(colourIntTime);
  m_currentSettings.colourIntTime = colourIntTime;
}

/*!
 * \brief Configures the Pen Lift servo to use the desired settings
 * 
 * \param settings - A DB1_ServoSettings struct containing the desired settings
 */
void DB1::setupServo(DB1_ServoSettings settings) {
  if (!m_penLift.attached())
    m_penLift.attach(settings.pin);

  m_currentSettings.servo = settings;
}

/*!
 * \brief Configures one of the VL53L0X time of flight sensors to use the desired settings
 * 
 * \param index - The index of the sensor to configure
 * \param settings - A DB1_ToFSettings struct containing the desired settings
 */

void DB1::setupToFSensor(int index, DB1_ToFSettings settings) {
  m_tofs[index].init();
  m_tofs[index].setTimeout(settings.timeout);
  m_tofs[index].setSignalRateLimit(settings.signalRateLimit);
  m_tofs[index].setMeasurementTimingBudget(settings.measurementTimingBudget);
  m_tofs[index].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, settings.prePclks);
  m_tofs[index].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, settings.finalPclks);
  m_tofs[index].startContinuous();

  m_currentSettings.tof = settings;
}

/*!
 * \brief Configures the motors
 * 
 * \param encoders - if true, the encoders will be set up and start counting steps
 */
void DB1::setupMotors(bool encoders) {
  pinMode(M1_DIR_A, OUTPUT);
  pinMode(M1_DIR_B, OUTPUT);
  pinMode(M2_DIR_A, OUTPUT);
  pinMode(M2_DIR_B, OUTPUT);

  pinMode(M1_E_A, INPUT);
  pinMode(M1_E_B, INPUT);
  pinMode(M2_E_A, INPUT);
  pinMode(M2_E_B, INPUT);

  if (encoders) {
    attachInterrupt(M1_E_A, m1EncoderCallback, CHANGE);
    attachInterrupt(M1_E_B, m1EncoderCallback, CHANGE);
    attachInterrupt(M2_E_A, m2EncoderCallback, CHANGE);
    attachInterrupt(M2_E_B, m2EncoderCallback, CHANGE);
  }
  else {
    detachInterrupt(M1_E_A);
    detachInterrupt(M1_E_B);
    detachInterrupt(M2_E_A);
    detachInterrupt(M2_E_B);
  }

  m_currentSettings.useEncoders = encoders;
}

/*!
 * \brief Calibrates the IR array. If motor power is present, the DB1 will spin on the spot and find the highest and lowest values to use as calibration min and max. Make sure you place the DB1 on a sample that shows the expected darkest and lightest value.
 */
void DB1::calibrateIRArray() {
  m_irLow.centre = 1024;
  m_irLow.left = 1024;
  m_irLow.right = 1024;
  m_irLow.farLeft = 1024;
  m_irLow.farRight = 1024;

  m_irHigh.centre = 0;
  m_irHigh.left = 0;
  m_irHigh.right = 0;
  m_irHigh.farLeft = 0;
  m_irHigh.farRight = 0;

  setMotorSpeed(1, 0.1);
  setMotorSpeed(2, -0.1);
  for (int i = 0; i < IR_CALIBRATION_COUNT; i++) {
    DB1_IRArray ir = readIRSensors(false);
    if (ir.centre < m_irLow.centre)
      m_irLow.centre = ir.centre;
    else if (ir.centre > m_irHigh.centre)
      m_irHigh.centre = ir.centre;

    if (ir.left < m_irLow.left)
      m_irLow.left = ir.left;
    else if (ir.left > m_irHigh.left)
      m_irHigh.left = ir.left;

    if (ir.right < m_irLow.right)
      m_irLow.right = ir.right;
    else if (ir.right > m_irHigh.right)
      m_irHigh.right = ir.right;

    if (ir.farLeft < m_irLow.farLeft)
      m_irLow.farLeft = ir.farLeft;
    else if (ir.farLeft > m_irHigh.farLeft)
      m_irHigh.farLeft = ir.farLeft;

    if (ir.farRight < m_irLow.farRight)
      m_irLow.farRight = ir.farRight;
    else if (ir.farRight > m_irHigh.farRight)
      m_irHigh.farRight = ir.farRight;

    delay(IR_CALIBRATION_DELAY_MS);
  }
  setMotorSpeed(1, 0.0);
  setMotorSpeed(2, 0.0);

  Serial.println("IR Calibration Complete:");
  Serial.print("Far Left - Low: ");
  Serial.print(m_irLow.farLeft);
  Serial.print(" High: ");
  Serial.println(m_irHigh.farLeft);
  Serial.print("Left - Low: ");
  Serial.print(m_irLow.left);
  Serial.print(" High: ");
  Serial.println(m_irHigh.left);
  Serial.print("Centre - Low: ");
  Serial.print(m_irLow.centre);
  Serial.print(" High: ");
  Serial.println(m_irHigh.centre);
  Serial.print("Right - Low: ");
  Serial.print(m_irLow.right);
  Serial.print(" High: ");
  Serial.println(m_irHigh.right);
  Serial.print("Far Right - Low: ");
  Serial.print(m_irLow.farRight);
  Serial.print(" High: ");
  Serial.println(m_irHigh.farRight);
}

/*!
 * \brief Sets the IR calibration min and max to the provided values. Useful if you have already determined the lightest and darkest features
 * 
 * \param low - The desired lowest values for each sensor
 * \param high - The desired highest value for each sensor
 */
void DB1::setIRCalibration(DB1_IRArray low, DB1_IRArray high) {
  m_irLow = low;
  m_irHigh = high;
}

/*!
 * \brief Calibrates the VEML6040 colour sensor. The DB1 will turn on the white LED and take an average high/white reading then turn off the LED and take an average low/black reading. Make sure you place the DB1 on a white surface
 */
void DB1::calibrateColourSensor() {
  bool currentLightSetting = m_currentSettings.whiteLightOn;
  VEML6040_Colour result;
  setWhiteLight(true);
  for (int i = 0; i < COLOUR_CALIBRATION_COUNT; i++) {
    VEML6040_Colour reading = m_colourSensor.getColour();
    result.red += reading.red;
    result.green += reading.green;
    result.blue += reading.blue;
    result.white += reading.white;

    delay(COLOUR_CALIBRATION_DELAY_MS);
  }
  m_colourHigh.red = result.red / COLOUR_CALIBRATION_COUNT;
  m_colourHigh.green = result.green / COLOUR_CALIBRATION_COUNT;
  m_colourHigh.blue = result.blue / COLOUR_CALIBRATION_COUNT;
  m_colourHigh.white = result.white / COLOUR_CALIBRATION_COUNT;

  setWhiteLight(false);
  result = {0};
  for (int i = 0; i < COLOUR_CALIBRATION_COUNT; i++) {
    VEML6040_Colour reading = m_colourSensor.getColour();
    result.red += reading.red;
    result.green += reading.green;
    result.blue += reading.blue;
    result.white += reading.white;
    delay(COLOUR_CALIBRATION_DELAY_MS);
  }

  m_colourLow.red = result.red / COLOUR_CALIBRATION_COUNT;
  m_colourLow.green = result.green / COLOUR_CALIBRATION_COUNT;
  m_colourLow.blue = result.blue / COLOUR_CALIBRATION_COUNT;
  m_colourLow.white = result.white / COLOUR_CALIBRATION_COUNT;

  setWhiteLight(currentLightSetting);
}

/*!
 * \brief Sets the colour calibration to the provided high/white and low/black readings
 * 
 * \param low - The desired low/black value
 * \param high - The desired high/white value
 */
void DB1::setColourCalibration(VEML6040_Colour low, VEML6040_Colour high) {
  m_colourLow = low;
  m_colourHigh = high;
}

/*!
 * \brief Sets the bottom side RGB LEDs to the provided colours
 * 
 * \param lights - A DB1_Lights struct containing the desired colours
 */
void DB1::setLights(DB1_Lights lights) {
  m_lights.clear();
  for (int i = 0; i < LIGHT_COUNT; i++) {
    m_lights.setPixelColor(i, lights.colours[i].red, lights.colours[i].green, lights.colours[i].blue);
  }
  m_lights.show();
  m_currentLights = lights;
}

/*!
 * \brief Sets the top side RGB LED to the provided colour
 * 
 * \param light - A DB1_Colour containing the desired colour
 */
void DB1::setTopLight(DB1_Colour light) {
  m_currentTopLight = light;
  m_topLight.clear();
  m_topLight.setPixelColor(0, m_currentTopLight.red, m_currentTopLight.green, m_currentTopLight.blue);
  m_topLight.show();
}

/*!
 * \brief Reads the current battery level
 * 
 * \param lights - (Default: true) If true, updates the battery LEDs to reflect current level
 * \return The remaining battery percentage
 */
float DB1::updateBatteryLevel(bool lights) {
  int battLevel = analogRead(V_DIV_BATT);

  float voltage = battLevel;
  voltage /= 1024.0f;
  voltage *= 8.4;

  float percentage = mapf(voltage, BATT_VOLT_LOW, BATT_VOLT_HIGH, 0, 100);

  digitalWrite(BATT_LVL1, LOW);
  digitalWrite(BATT_LVL2, LOW);
  digitalWrite(BATT_LVL3, LOW);
  digitalWrite(BATT_LVL4, LOW);

  if (lights) {
    if (percentage > 10)
      digitalWrite(BATT_LVL1, HIGH);
    if (percentage > 25)
      digitalWrite(BATT_LVL2, HIGH);
    if (percentage > 50)
      digitalWrite(BATT_LVL3, HIGH);
    if (percentage > 75)
      digitalWrite(BATT_LVL4, HIGH);
  }
  return percentage;
}

/*!
 * \brief Updates the state of the pen
 * 
 * \param down - If true, sets the pen to the down position. If false, sets the pen to the up position
 */
void DB1::setPen(bool down) {
  if (down)
    m_penLift.write(m_currentSettings.servo.penDownPosition);
  else
    m_penLift.write(m_currentSettings.servo.penUpPosition);
}

/*!
 * \brief Sets the poistion of the pen to an arbitrary point 
 * 
 * \param pos - A position between 0.0 - 1.0 where 0.0 is all the way up and 1.0 is all the way down
 */
void DB1::setPenServo(double pos) {
  double newPos = mapf(pos, 0.0, 1.0, m_currentSettings.servo.penUpPosition, m_currentSettings.servo.penDownPosition);
  m_penLift.write(newPos);
}

/*!
 * \brief Sets the speed of a specific motor
 * 
 * \param motor - The motor to set the speed of. Valid values are 1 or 2
 * \param speed - The speed to set. Valid values are -1.0 to 1.0 where 1.0 is full speed forward, -1.0 is full spped backwards and 0.0 is no movement
 */
void DB1::setMotorSpeed(int motor, double speed) {
  bool direction = (speed > 0);
  double pwmVal = abs(speed) * 65535.0;

  if (motor == 1) {
    if (direction) {
      digitalWrite(M1_DIR_A, HIGH);
      digitalWrite(M1_DIR_B, LOW);
    }
    else
    {
      digitalWrite(M1_DIR_A, LOW);
      digitalWrite(M1_DIR_B, HIGH);
    }
    analogWrite(M1_PWM, (int)pwmVal);
  }
  else if (motor == 2) {
    if (direction)
    {
      digitalWrite(M2_DIR_A, LOW);
      digitalWrite(M2_DIR_B, HIGH);
    }
    else {
      digitalWrite(M2_DIR_A, HIGH);
      digitalWrite(M2_DIR_B, LOW);
    }
    analogWrite(M2_PWM, (int)pwmVal);
  }
}

/*!
 * \brief The change in the Motor 1 Encoder since the last call to this method
 * 
 * \return The change in the Motor 1 Encoder since the last call to this method
 */
long DB1::getM1EncoderDelta() {
  long delta = m_m1En - m_lastM1En;
  m_lastM1En = m_m1En;

  return delta;
}

/*!
 * \brief The change in the Motor 2 Encoder since the last call to this method
 * 
 * \return The change in the Motor 2 Encoder since the last call to this method
 */
long DB1::getM2EncoderDelta() {
  long delta = m_m2En - m_lastM2En;
  m_lastM2En = m_m2En;

  return delta;
}

/*!
 * \brief Reads the current colour from the VEML6040 colour sensor
 * 
 * \param calibrated - (Default: true) If true, the returned Colour will be corrected by the current calibration values
 * \return The read colour value 
 */
VEML6040_Colour DB1::readColour(bool calibrated) {
  if (calibrated) {
    VEML6040_Colour reading = m_colourSensor.getColour();
    reading.red = constrain(mapf(reading.red, 0, m_colourHigh.red, 0, 255), 0, 255);
    reading.green = constrain(mapf(reading.green, 0, m_colourHigh.green, 0, 255), 0, 255);
    reading.blue = constrain(mapf(reading.blue, 0, m_colourHigh.blue, 0, 255), 0, 255);
    reading.white = constrain(mapf(reading.white, 0, m_colourHigh.white, 0, 255), 0, 255);

    return reading;
  }
  return m_colourSensor.getColour();
}

/*!
 * \brief Reads the current values from the IR line detectors
 * 
 * \param calibrated - (Default: true) If true, the returned values will be corrected by the current calibration values
 * \return The read IR values 
 */
DB1_IRArray DB1::readIRSensors(bool calibrated) {
  DB1_IRArray result; // = {0};
  int centre = 0;
  int left = 0;
  int right = 0;
  int farLeft = 0;
  int farRight = 0;

  for (int i = 0; i < m_currentSettings.irReadCount; i++) {
    centre += analogRead(LINE1);
    right += analogRead(LINE2);
    left += analogRead(LINE3);
    farRight += analogRead(LINE4);
    farLeft += analogRead(LINE5);
  }

  result.centre = centre / m_currentSettings.irReadCount;
  result.right = right / m_currentSettings.irReadCount;
  result.left = left / m_currentSettings.irReadCount;
  result.farRight = farRight / m_currentSettings.irReadCount;
  result.farLeft = farLeft / m_currentSettings.irReadCount;

  if (calibrated) {
    result.centre = constrain(map(result.centre, m_irLow.centre, m_irHigh.centre, 0, 255), 0, 255);
    result.right = constrain(map(result.right, m_irLow.right, m_irHigh.right, 0, 255), 0, 255);
    result.left = constrain(map(result.left, m_irLow.left, m_irHigh.left, 0, 255), 0, 255);
    result.farRight = constrain(map(result.farRight, m_irLow.farRight, m_irHigh.farRight, 0, 255), 0, 255);
    result.farLeft = constrain(map(result.farLeft, m_irLow.farLeft, m_irHigh.farLeft, 0, 255), 0, 255);
  }

  return result;
}

/*!
 * \brief Reads the current range from a specific VL53L0X Time of Flight Sensor
 * 
 * \param location - Which Time of Flight Sensor to read. Valid values are:\n TOF_LEFT\n TOF_CENTRE\n TOF_RIGHT
 * \return The read range in millimetres
 */
int DB1::readToFSensor(DB1_ToFLocation location) {
  return m_tofs[location].readRangeContinuousMillimeters();
}

/*!
 * \brief Enables monitoring of high g impacts on the IMU, calls the callback when one occurs
 * 
 * \param callback - A function pointer that is called when the bump occurs. This fuction is run in an interrupt, avoid performing any intensive operations in it
 * \param threshold - (Default: 1000) The threshold in milligs above which the callback will be run
 */
void DB1::enableBumpInterrupt(DB1_BumpInt_t callback, uint32_t threshold) {
  m_bumpCallback = callback;
  m_bumpThreshold = threshold;
}

/*!
 * \brief Disables the high g monitoring of the IMU
 */
void DB1::disableBumpInterrupt() {
  m_bumpCallback = NULL;
}