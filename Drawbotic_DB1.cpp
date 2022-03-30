#include "Drawbotic_DB1.h"

//------- Static Methods -------//
DB1* DB1::s_instance = NULL;

const DB1_Settings DB1::s_defaultSettings = {
    { BMX160_ACCEL_RATE_1600HZ, BMX160_ACCEL_RANGE_2G, 
      BMX160_GYRO_RATE_1600HZ, BMX160_GYRO_RANGE_1000_DPS },    //IMU Defaults
    { TCS34725_GAIN_4X, TCS34725_INTEGRATIONTIME_24MS },        //Colour sensor defaults
    { S_PWM, SERVO_UP_DEFAULT, SERVO_DOWN_DEFAULT },            //Servo defaults
    { TOF_TIMEOUT_DEFAULT, TOF_SIGLIM_DEFAULT, 
      TOF_TIMING_BUDGET_DEFAULT, TOF_PRE_PCLKS_DEFAULT, 
      TOF_FIN_PCLKS_DEFAULT },                                  //ToF defaults
    true,                                                       //Use encoders
    false,                                                      //m1 flipped
    false,                                                      //m2 flipped
    true,                                                       //White Light default
    0                                                           //IR Dim default
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
    m_lights(LIGHT_COUNT, RGB_DOUT, NEO_GRB + NEO_KHZ800)
{
    for(int i = 0; i < LIGHT_COUNT; i++)
    {
        m_currentLights.colours[i].red = 0;
        m_currentLights.colours[i].green = 0;
        m_currentLights.colours[i].blue = 0;
    }
    
    for(int i = 0; i < IR_COUNT; i++)
    {
        m_irHigh[i] = 0;
        m_irLow[i] = 0;
    }
    
    m_m1En = 0;
    m_m2En = 0;
    m_lastM1En = 0;
    m_lastM2En = 0;

    m_currentSettings = GetDefaultSettings();
    m_imu = BMX160::GetInstance();

    s_instance = this;
}

bool DB1::Initialise()
{
    Initialise(DB1::GetDefaultSettings());
}

bool DB1::Initialise(DB1_Settings settings)
{
    Wire.begin();
    analogWriteResolution(16);
    
    //Set up motor driver
    SetupMotors(settings.useEncoders);
    Serial.println("Motors Done");
    
    //Setup IR LEDs
    pinMode(IR_CTRL, OUTPUT);
    digitalWrite(IR_CTRL, HIGH);
    SetIRDimLevel(settings.irDimLevel);
    m_currentSettings.irDimLevel = settings.irDimLevel;
    Serial.println("IR Done");
    
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);
    
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
    
    //Turn off TCS
    pinMode(TCS_EN, OUTPUT);
    digitalWrite(TCS_EN, LOW);
    Serial.println("Turn off colour");
    
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
   
    //Setup colour sensor
    digitalWrite(TCS_EN, HIGH);
    delay(TCS_BOOT_DELAY_MS);
    SetupColourSensor(settings.colourSensor);
    Serial.println("Colour Done");
   
    //Setup IMU
    SetupIMU(settings.imu);
    Serial.println("IMU Done");
   
    //Setup neopixels
    m_lights.begin();
    SetLights(m_currentLights);
    m_lights.show();  
    Serial.println("RGB Done");
}

void DB1::SetWhiteLight(bool on)
{
    m_currentSettings.whiteLightOn = on;
    digitalWrite(LED_EN, on);
}

void DB1::SetIRDimLevel(int level)
{
    //Reset both LED drivers
    digitalWrite(IR_CTRL, LOW);
    delay(10);

    if(level > 32)
        level = 32;
    else if(level < 0)
        level = 0;

    //Count to the right pulses
    for(int i = 0; i <= level; i++)
    {
        digitalWrite(IR_CTRL, LOW);
        delayMicroseconds(10);
        digitalWrite(IR_CTRL, HIGH);
        delayMicroseconds(10);
    }
    m_currentSettings.irDimLevel = level;
}

void DB1::SetupIMU(DB1_IMUSettings settings)
{
    m_imu->Init(true);
    m_imu->SetAccelRange(settings.accelRange);
    m_imu->SetAccelRate(settings.accelRate);
    m_imu->SetGyroRange(settings.gyroRange);
    m_imu->SetGyroRate(settings.gyroRate);

    m_currentSettings.imu = settings;
}

void DB1::SetupColourSensor(DB1_ColourSettings settings)
{
    m_colourTCS.begin();
    m_colourTCS.setGain(settings.gain);
    m_colourTCS.setIntegrationTime(settings.intergrationTime);

    m_currentSettings.colourSensor = settings;
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
    pinMode(M1_DIRA, OUTPUT);
    pinMode(M1_DIRB, OUTPUT);
    pinMode(M2_DIRA, OUTPUT);
    pinMode(M2_DIRB, OUTPUT);

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


void DB1::CalibrateIMU()
{
    m_imu->BeginFOC(0, 0, 1);
}

void DB1::CalibrateIRArray()
{
    for(int i = 0; i < IR_COUNT; i++)
    {
        m_irLow[i] = 1024;
        m_irHigh[i] = 0;
    }

    SetMotorSpeed(1, 0.1);
    SetMotorSpeed(2, 0.1);
    for(int i = 0; i < IR_CALIBRATION_COUNT; i++)
    {
        DB1_IRArray ir = ReadIRSensors(false);
        if(ir.centre < m_irLow[0])
            m_irLow[0] = ir.centre;
        else if(ir.centre > m_irHigh[0])
            m_irHigh[0] = ir.centre;

        if(ir.left < m_irLow[1])
            m_irLow[1] = ir.left;
        else if(ir.left > m_irHigh[1])
            m_irHigh[1] = ir.left;

        if(ir.right < m_irLow[2])
            m_irLow[2] = ir.centre;
        else if(ir.right > m_irHigh[2])
            m_irHigh[2] = ir.right;

        if(ir.farLeft < m_irLow[3])
            m_irLow[3] = ir.farLeft;
        else if(ir.farLeft > m_irHigh[3])
            m_irHigh[3] = ir.farLeft;

        if(ir.farRight < m_irLow[4])
            m_irLow[4] = ir.farRight;
        else if(ir.farRight > m_irHigh[4])
            m_irHigh[4] = ir.farRight;

        delay(IR_CALIBRATION_DELAY_MS);
    }
    SetMotorSpeed(1, 0.0);
    SetMotorSpeed(2, 0.0);
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

float DB1::UpdateBatteryLevel(bool lights)
{
    int battLevel = analogRead(V_DIV_BATT);

    float voltage = battLevel;
    voltage /= 1024.0f;
    voltage *= 8.4;
    
    float percentage = mapf(voltage, 3, 8.4, 0, 100);
    
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
    bool direction = false;
    if(speed > 0)
        direction = true;
    double pwmVal = abs(speed) * 65535.0;

    if(motor == 1)
    {
        if(direction)
        {
            digitalWrite(M1_DIRA, HIGH);
            digitalWrite(M1_DIRB, LOW);
        }
        else
        {
            digitalWrite(M1_DIRA, LOW);
            digitalWrite(M1_DIRB, HIGH);
        }
        analogWrite(M1_PWM, (int)pwmVal);
    }
    else if(motor == 2)
    {
        if(direction)
        {
            digitalWrite(M2_DIRA, LOW);
            digitalWrite(M2_DIRB, HIGH);
        }
        else
        {
            digitalWrite(M2_DIRA, HIGH);
            digitalWrite(M2_DIRB, LOW);
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

DB1_Colour DB1::ReadColour()
{
    float r, g, b;
    m_colourTCS.getRGB(&r, &g, &b);

    DB1_Colour colour = { (uint8_t)r, (uint8_t)g, (uint8_t)b };
    return colour;
}

DB1_IRArray DB1::ReadIRSensors(bool calibrated)
{
    int ir1 = map(analogRead(IR1), 0, 1023, 0, 255);
    int ir2 = map(analogRead(IR2), 0, 1023, 0, 255);
    int ir3 = map(analogRead(IR3), 0, 1023, 0, 255);
    int ir4 = map(analogRead(IR4), 0, 1023, 0, 255);
    int ir5 = map(analogRead(IR5), 0, 1023, 0, 255);

    if(calibrated)
    {
        ir1 = constrain(map(ir1, m_irLow[0], m_irHigh[0], 0, 255), 0, 255);
        ir2 = constrain(map(ir2, m_irLow[1], m_irHigh[1], 0, 255), 0, 255);
        ir3 = constrain(map(ir3, m_irLow[2], m_irHigh[2], 0, 255), 0, 255);
        ir4 = constrain(map(ir4, m_irLow[3], m_irHigh[3], 0, 255), 0, 255);
        ir5 = constrain(map(ir5, m_irLow[4], m_irHigh[4], 0, 255), 0, 255);
    }

    DB1_IRArray result;
    result.farLeft = ir5;
    result.left = ir3;
    result.centre = ir1;
    result.right = ir2;
    result.farRight = ir4;
    return result;
}

int DB1::ReadToFSensor(DB1_ToFLocation location)
{
    return m_tofs[location].readRangeContinuousMillimeters();
}

DB1_Motion DB1::ReadIMU()
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

    return motion;
}

void DB1::EnableBumpInterrupt(uint8_t threshold, uint8_t duration, BMX160_InterruptPin pin)
{
    m_imu->AddHighGInterrupt(pin, threshold, duration, BMX160_HIGH_HY_1, BMX160_XYZ_AXES);
}

void DB1::DisableBumpInterrupt()
{
    m_imu->RemoveInterrupt(BMX160_HIGH_G_INT);
}