#include "Drawbotic_DB1.h"

//------- Static Methods -------//
DB1* DB1::s_instance = NULL;

const DB1_Settings DB1::s_defaultSettings = {
    //{ BMX160_ACCEL_RATE_25HZ, BMX160_ACCEL_RANGE_2G, 
    //  BMX160_GYRO_RATE_25HZ, BMX160_GYRO_RANGE_250_DPS }, //IMU Defaults
    { SERVO_PWM, SERVO_UP_DEFAULT, SERVO_DOWN_DEFAULT },      //Servo defaults
    { TOF_TIMEOUT_DEFAULT, TOF_SIGLIM_DEFAULT, 
      TOF_TIMING_BUDGET_DEFAULT, TOF_PRE_PCLKS_DEFAULT, 
      TOF_FIN_PCLKS_DEFAULT },                            //ToF defaults
    VEML6040_IT_40MS,                                     //Default colour sensor int time
    true,                                                 //Use encoders
    false,                                                //White Light default
    4,                                                    //IR Read Count
};

void DB1::m1EncoderCallBack()
{
    if(s_instance)
    {
        int m1EnAState = digitalRead(M1_E_A);

        if(m1EnAState != s_instance->m_m1EnALastState && m1EnAState == 1)
        {
            if(digitalRead(M1_E_B) != m1EnAState)
            {
                s_instance->m_m1En--;
            }
            else
            {
                s_instance->m_m1En++;
            }
        }
        
        s_instance->m_m1EnALastState = m1EnAState;
    }
}

void DB1::m2EncoderCallBack()
{
    if(s_instance)
    {
        int m2EnAState = digitalRead(M2_E_A);

        if(m2EnAState != s_instance->m_m2EnALastState && m2EnAState == 1)
        {
            if(digitalRead(M2_E_B) != m2EnAState)
            {
                s_instance->m_m2En++;
            }
            else
            {
                s_instance->m_m2En--;
            }
        }
        
        s_instance->m_m2EnALastState = m2EnAState;
    }
}

//------- Public Methods -------//
DB1::DB1() : 
    m_lights(LIGHT_COUNT, RGB_DOUT, NEO_GRB + NEO_KHZ800),
    m_topLight(1, STAT_DOUT, NEO_GRB + NEO_KHZ800)
{
    for(int i = 0; i < LIGHT_COUNT; i++)
    {
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

    m_currentSettings = GetDefaultSettings();
    //m_imu = BMX160::GetInstance();

    s_instance = this;
}

bool DB1::Initialise()
{
    return Initialise(DB1::GetDefaultSettings());
}

bool DB1::Initialise(DB1_Settings settings)
{
    Wire.begin();
    analogWriteResolution(16);
    
    //Set up motor driver
    SetupMotors(settings.useEncoders);
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
    
    //Setup white LED
    pinMode(LED_EN, OUTPUT);
    SetWhiteLight(settings.whiteLightOn);
    m_currentSettings.whiteLightOn = settings.whiteLightOn;
    Serial.println("White Done");
    
    //Setup servo
    m_penLift.attach(settings.servo.pin);
    Serial.println("Servo Done");
    
    //Turn off ToFs
    for(int i = 0; i < TOF_COUNT; i++)
    {
        pinMode(TOF_EN_PINS[i], OUTPUT);
        digitalWrite(TOF_EN_PINS[i], LOW);
    }
    Serial.println("Turn off ToFs");

    //Setup tof sensors
    for(int i = 0; i < TOF_COUNT; i++)
    {
        digitalWrite(TOF_EN_PINS[i], HIGH);
        Serial.print("Turnning on pin ");
        Serial.println(TOF_EN_PINS[i]);
        delay(TOF_BOOT_DELAY_MS);
        m_tofs[i] = VL53L0X();
        m_tofs[i].setAddress(TOF_ADDRESSES[i]);
        SetupToFSensor(i, settings.tof);
        Serial.print("Set up complete for ToF 0x");
        Serial.println(TOF_ADDRESSES[i], HEX);
    }
    Serial.println("Tofs Done");
   
    SetupColourSensor(settings.colourIntTime);
   
    //Setup IMU
    //SetupIMU(settings.imu);
    //Serial.println("IMU Done");
   
    //Setup neopixels
    m_lights.begin();
    SetLights(m_currentLights);
    m_lights.show();  
    
    m_topLight.begin();
    SetTopLight(m_currentTopLight);
    m_topLight.show();

    Serial.println("RGB Done");
    
    return true;
}

void DB1::SetWhiteLight(bool on)
{
    m_currentSettings.whiteLightOn = on;
    digitalWrite(LED_EN, on);
}

/*void DB1::SetupIMU(DB1_IMUSettings settings)
{
    m_imu->Init(true);
    m_imu->SetAccelRange(settings.accelRange);
    m_imu->SetAccelRate(settings.accelRate);
    m_imu->SetGyroRange(settings.gyroRange);
    m_imu->SetGyroRate(settings.gyroRate);

    m_currentSettings.imu = settings;
}*/

void DB1::SetupColourSensor(VEML6040_IntegrationTime colourIntTime)
{
    m_colourSensor.begin();
    m_colourSensor.setConfig(colourIntTime);
    m_currentSettings.colourIntTime = colourIntTime;
}

void DB1::SetupServo(DB1_ServoSettings settings)
{
    if(!m_penLift.attached())
        m_penLift.attach(settings.pin);
        
    m_currentSettings.servo = settings;
}

void DB1::SetupToFSensor(int index, DB1_ToFSettings settings)
{
    m_tofs[index].init();
    m_tofs[index].setTimeout(settings.timeout);
    m_tofs[index].setSignalRateLimit(settings.signalRateLimit);
    m_tofs[index].setMeasurementTimingBudget(settings.measurementTimingBudget);
    m_tofs[index].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, settings.prePclks);
    m_tofs[index].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, settings.finalPclks);
    m_tofs[index].startContinuous();

    m_currentSettings.tof = settings;
}

void DB1::SetupMotors(bool encoders)
{
    pinMode(M1_DIR_A, OUTPUT);
    pinMode(M1_DIR_B, OUTPUT);
    pinMode(M2_DIR_A, OUTPUT);
    pinMode(M2_DIR_B, OUTPUT);

    pinMode(M1_E_A, INPUT);
    pinMode(M1_E_B, INPUT);
    pinMode(M2_E_A, INPUT);
    pinMode(M2_E_B, INPUT);

    if(encoders)
    {
        attachInterrupt(M1_E_A, m1EncoderCallBack, CHANGE);
        attachInterrupt(M1_E_B, m1EncoderCallBack, CHANGE);
        attachInterrupt(M2_E_A, m2EncoderCallBack, CHANGE);
        attachInterrupt(M2_E_B, m2EncoderCallBack, CHANGE);
    }
    else
    {
        detachInterrupt(M1_E_A);
        detachInterrupt(M1_E_B);
        detachInterrupt(M2_E_A);
        detachInterrupt(M2_E_B);
    }
    
    m_currentSettings.useEncoders = encoders;
}


/*void DB1::CalibrateIMU()
{
    m_imu->BeginFOC(0, 0, 1);
}*/

void DB1::CalibrateIRArray()
{
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

    SetMotorSpeed(1, 0.1);
    SetMotorSpeed(2, -0.1);
    for(int i = 0; i < IR_CALIBRATION_COUNT; i++)
    {
        DB1_IRArray ir = ReadIRSensors(false);
        if(ir.centre < m_irLow.centre)
            m_irLow.centre = ir.centre;
        else if(ir.centre > m_irHigh.centre)
            m_irHigh.centre = ir.centre;

        if(ir.left < m_irLow.left)
            m_irLow.left = ir.left;
        else if(ir.left > m_irHigh.left)
            m_irHigh.left = ir.left;

        if(ir.right < m_irLow.right)
            m_irLow.right = ir.right;
        else if(ir.right > m_irHigh.right)
            m_irHigh.right = ir.right;

        if(ir.farLeft < m_irLow.farLeft)
            m_irLow.farLeft = ir.farLeft;
        else if(ir.farLeft > m_irHigh.farLeft)
            m_irHigh.farLeft = ir.farLeft;

        if(ir.farRight < m_irLow.farRight)
            m_irLow.farRight = ir.farRight;
        else if(ir.farRight > m_irHigh.farRight)
            m_irHigh.farRight = ir.farRight;

        delay(IR_CALIBRATION_DELAY_MS);
    }
    SetMotorSpeed(1, 0.0);
    SetMotorSpeed(2, 0.0);

    Serial.println("IR Calibration Complete:");
    Serial.print("Far Left - Low: "); Serial.print(m_irLow.farLeft); Serial.print(" High: "); Serial.println(m_irHigh.farLeft);
    Serial.print("Left - Low: "); Serial.print(m_irLow.left); Serial.print(" High: "); Serial.println(m_irHigh.left);
    Serial.print("Centre - Low: "); Serial.print(m_irLow.centre); Serial.print(" High: "); Serial.println(m_irHigh.centre);
    Serial.print("Right - Low: "); Serial.print(m_irLow.right); Serial.print(" High: "); Serial.println(m_irHigh.right);
    Serial.print("Far Right - Low: "); Serial.print(m_irLow.farRight); Serial.print(" High: "); Serial.println(m_irHigh.farRight);
}

void DB1::SetIRCalibration(DB1_IRArray low, DB1_IRArray high)
{
    m_irLow = low;
    m_irHigh = high;
}

void DB1::CalibrateColourSensor()
{
    VEML6040_Colour result;

    for(int i = 0; i < COLOUR_CALIBRATION_COUNT; i++)
    {
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
}

void DB1::SetLights(DB1_Lights lights)
{
    m_lights.clear();
    for(int i = 0; i < LIGHT_COUNT; i++)
    {
        m_lights.setPixelColor(i, lights.colours[i].red, lights.colours[i].green, lights.colours[i].blue);
    }
    m_lights.show();
    m_currentLights = lights;
}

void DB1::SetTopLight(DB1_Colour light)
{
    m_topLight.setPixelColor(0, m_currentTopLight.red, m_currentTopLight.green, m_currentTopLight.blue);
    m_topLight.show();
    m_currentTopLight = light;
}

float DB1::UpdateBatteryLevel(bool lights)
{
    int battLevel = analogRead(V_DIV_BATT);

    float voltage = battLevel;
    voltage /= 1024.0f;
    voltage *= 8.4;
    
    float percentage = mapf(voltage, BATT_VOLT_LOW, BATT_VOLT_HIGH, 0, 100);
    
    digitalWrite(BATT_LVL1, LOW);
    digitalWrite(BATT_LVL2, LOW);
    digitalWrite(BATT_LVL3, LOW);
    digitalWrite(BATT_LVL4, LOW);

    if(lights)
    {
        if(percentage > 10)
            digitalWrite(BATT_LVL1, HIGH);
        if(percentage > 25)
            digitalWrite(BATT_LVL2, HIGH);
        if(percentage > 50)
            digitalWrite(BATT_LVL3, HIGH);
        if(percentage > 75)
            digitalWrite(BATT_LVL4, HIGH);
    }
    return percentage;
}

void DB1::SetPenUp(bool up)
{
    if(up)
    {
        m_penLift.write(m_currentSettings.servo.penUpPosition);
    }
    else
    {
        m_penLift.write(m_currentSettings.servo.penDownPosition);
    }
}

void DB1::SetPenServo(double pos)
{
    double newPos = mapf(pos, 0.0, 1.0, m_currentSettings.servo.penDownPosition, m_currentSettings.servo.penUpPosition);
    m_penLift.write(newPos);
}

void DB1::SetMotorSpeed(int motor, double speed)
{
    bool direction = (speed > 0);
    double pwmVal = abs(speed) * 65535.0;

    if(motor == 1)
    {
        if(direction)
        {
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
    else if(motor == 2)
    {
        if(direction)
        {
            digitalWrite(M2_DIR_A, LOW);
            digitalWrite(M2_DIR_B, HIGH);
        }
        else
        {
            digitalWrite(M2_DIR_A, HIGH);
            digitalWrite(M2_DIR_B, LOW);
        }
        analogWrite(M2_PWM, (int)pwmVal);
    }
}

long DB1::GetM1EncoderDelta()
{
    long delta = m_m1En - m_lastM1En;
    m_lastM1En = m_m1En;

    return delta;
}

long DB1::GetM2EncoderDelta()
{
    long delta = m_m2En - m_lastM2En;
    m_lastM2En = m_m2En;

    return delta;
}

VEML6040_Colour DB1::ReadColour(bool calibrated)
{
    if(calibrated)
    {
        VEML6040_Colour reading = m_colourSensor.getColour();
        reading.red = constrain(mapf(reading.red, 0, m_colourHigh.red, 0, 255), 0, 255);
        reading.green = constrain(mapf(reading.green, 0, m_colourHigh.green, 0, 255), 0, 255);
        reading.blue = constrain(mapf(reading.blue, 0, m_colourHigh.blue, 0, 255), 0, 255);
        reading.white = constrain(mapf(reading.white, 0, m_colourHigh.white, 0, 255), 0, 255);

        return reading;
    }
    return m_colourSensor.getColour();
}

DB1_IRArray DB1::ReadIRSensors(bool calibrated)
{
    DB1_IRArray result;// = {0};
    int centre = 0;
    int left = 0;
    int right = 0;
    int farLeft = 0;
    int farRight = 0;

    for(int i = 0; i < m_currentSettings.irReadCount; i++)
    {
        centre   += analogRead(LINE1);
        right    += analogRead(LINE2);
        left     += analogRead(LINE3);
        farRight += analogRead(LINE4);
        farLeft  += analogRead(LINE5);
    }

    result.centre   = centre / m_currentSettings.irReadCount;
    result.right    = right / m_currentSettings.irReadCount;
    result.left     = left / m_currentSettings.irReadCount;
    result.farRight = farRight / m_currentSettings.irReadCount;
    result.farLeft  = farLeft / m_currentSettings.irReadCount;

    if(calibrated)
    {
        result.centre   = constrain(map(result.centre,   m_irLow.centre,   m_irHigh.centre,   0, 255), 0, 255);
        result.right    = constrain(map(result.right,    m_irLow.right,    m_irHigh.right,    0, 255), 0, 255);
        result.left     = constrain(map(result.left,     m_irLow.left,     m_irHigh.left,     0, 255), 0, 255);
        result.farRight = constrain(map(result.farRight, m_irLow.farRight, m_irHigh.farRight, 0, 255), 0, 255);
        result.farLeft  = constrain(map(result.farLeft,  m_irLow.farLeft,  m_irHigh.farLeft,  0, 255), 0, 255);
    }

    return result;
}

int DB1::ReadToFSensor(DB1_ToFLocation location)
{
    return m_tofs[location].readRangeContinuousMillimeters();
}

/*DB1_Motion DB1::ReadIMU()
{
    BMX160_IMUReading reading = m_imu->ReadIMU();

    DB1_Motion motion;
    motion.accel.x = reading.accel.x;
    motion.accel.y = reading.accel.y;
    motion.accel.z = reading.accel.z;
    motion.gyro.x = reading.gyro.x;
    motion.gyro.y = reading.gyro.y;
    motion.gyro.z = reading.gyro.z;
    motion.mag.x = reading.mag.x;
    motion.mag.y = reading.mag.y;
    motion.mag.z = reading.mag.z;
    motion.heading = reading.heading;

    return motion;
}*/

/*void DB1::EnableBumpInterrupt(uint8_t threshold, uint8_t duration, BMX160_InterruptPin pin)
{
    m_imu->AddHighGInterrupt(pin, threshold, duration, BMX160_HIGH_HY_1, BMX160_XYZ_AXES);
}

void DB1::DisableBumpInterrupt()
{
    m_imu->RemoveInterrupt(BMX160_HIGH_G_INT);
}

void DB1::EnableIMUReadyInterrupt(BMX160_InterruptPin pin)
{
    m_imu->AddDataReadyInterrupt(pin);
}

void DB1::DisableIMUReadyInterrupt()
{
    m_imu->RemoveInterrupt(BMX160_DATAREADY_INT);
}*/