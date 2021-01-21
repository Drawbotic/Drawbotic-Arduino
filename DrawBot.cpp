#include "DrawBot.h"

//------- Static Methods -------//
DrawBot* DrawBot::s_instance = NULL;

const DrawBot_Settings DrawBot::s_defaultSettings = {
    { BMX160_ACCEL_RATE_1600HZ, BMX160_ACCEL_RANGE_2G, 
      BMX160_GYRO_RATE_1600HZ, BMX160_GYRO_RANGE_1000_DPS },    //IMU Defaults
    { TCS34725_GAIN_4X, TCS34725_INTEGRATIONTIME_24MS },        //Colour sensor defaults
    { S_PWM, 25, 90 },                                          //Servo defaults
    { 500, 0.25f, 33000, 14, 10 },                              //ToF defaults
    true,                                                       //Motor defaults
    true,                                                       //White Light default
    0                                                           //IR Dim default
};

void DrawBot::m1EncoderCallBack()
{
    if(s_instance)
    {
        if(digitalRead(M1_E_A) && !digitalRead(M1_E_B))
        {
            s_instance->m_m1En++;
        }
        else if(digitalRead(M1_E_A) && digitalRead(M1_E_B))
        {
            s_instance->m_m1En--;
        }
    }
}

void DrawBot::m2EncoderCallBack()
{
    if(s_instance)
    {
        if(digitalRead(M2_E_A) && !digitalRead(M2_E_B))
        {
            s_instance->m_m2En++;
        }
        else if(digitalRead(M2_E_A) && digitalRead(M2_E_B))
        {
            s_instance->m_m2En--;
        }
    }
}

//------- Public Methods -------//
DrawBot::DrawBot() : 
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

bool DrawBot::Initialise()
{
    Initialise(DrawBot::GetDefaultSettings());
}

bool DrawBot::Initialise(DrawBot_Settings settings)
{
    Wire.begin();
    analogWriteResolution(16);
    
    //Set up motor driver
    SetupMotors(settings.useEncoders);
    
    //Setup IR LEDs
    pinMode(IR_CTRL, OUTPUT);
    digitalWrite(IR_CTRL, HIGH);
    SetIRDimLevel(settings.irDimLevel);
    m_currentSettings.irDimLevel = settings.irDimLevel;
    
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
    
    //Setup servo
    m_penLift.attach(settings.servo.pin);
    
    //Turn off TCS
    pinMode(TCS_EN, OUTPUT);
    digitalWrite(TCS_EN, LOW);
    
    //Turn off ToFs
    for(int i = 0; i < TOF_COUNT; i++)
    {
        pinMode(TOF_EN_PINS[i], OUTPUT);
        digitalWrite(TOF_EN_PINS[i], LOW);
    }
    
    //Setup tof sensors
    for(int i = 0; i < TOF_COUNT; i++)
    {
        digitalWrite(TOF_EN_PINS[i], HIGH);
        delay(TOF_BOOT_DELAY_MS);
        m_tofs[i] = VL53L0X();
        m_tofs[i].setAddress(TOF_ADDRESSES[i]);
        SetupToFSensor(i, settings.tof);
    }
   
    //Setup colour sensor
    digitalWrite(TCS_EN, HIGH);
    delay(TCS_BOOT_DELAY_MS);
    SetupColourSensor(settings.colourSensor);
   
    //Setup IMU
    SetupIMU(settings.imu);
   
    //Setup neopixels
    m_lights.begin();
    SetLights(m_currentLights);
    m_lights.show();  
}

void DrawBot::SetWhiteLight(bool on)
{
    m_currentSettings.whiteLightOn = on;
    digitalWrite(LED_EN, on);
}

void DrawBot::SetIRDimLevel(int level)
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

void DrawBot::SetupIMU(DrawBot_IMUSettings settings)
{
    m_imu->Init(true);
    m_imu->SetAccelRange(settings.accelRange);
    m_imu->SetAccelRate(settings.accelRate);
    m_imu->SetGyroRange(settings.gyroRange);
    m_imu->SetGyroRate(settings.gyroRate);

    m_currentSettings.imu = settings;
}

void DrawBot::SetupColourSensor(DrawBot_ColourSettings settings)
{
    m_colourTCS.begin();
    m_colourTCS.setGain(settings.gain);
    m_colourTCS.setIntegrationTime(settings.intergrationTime);

    m_currentSettings.colourSensor = settings;
}

void DrawBot::SetupServo(DrawBot_ServoSettings settings)
{
    if(!m_penLift.attached())
        m_penLift.attach(settings.pin);
        
    m_currentSettings.servo = settings;
}

void DrawBot::SetupToFSensor(int index, DrawBot_ToFSettings settings)
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

void DrawBot::SetupMotors(bool encoders)
{
    pinMode(M1_DIRA, OUTPUT);
    pinMode(M1_DIRB, OUTPUT);
    pinMode(M2_DIRA, OUTPUT);
    pinMode(M2_DIRB, OUTPUT);

    if(encoders)
    {
        attachInterrupt(M1_E_A, m1EncoderCallBack, RISING);
        attachInterrupt(M2_E_A, m2EncoderCallBack, RISING);
    }
    else
    {
        detachInterrupt(M1_E_A);
        detachInterrupt(M2_E_A);
    }
    
    m_currentSettings.useEncoders = encoders;
}


void DrawBot::CalibrateIMU()
{
    m_imu->BeginFOC(0, 0, 1);
}

void DrawBot::CalibrateIRArray()
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
        DrawBot_IRArray ir = ReadIRSensors(false);
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

void DrawBot::SetLights(DrawBot_Lights lights)
{
    m_lights.clear();
    for(int i = 0; i < LIGHT_COUNT; i++)
    {
        m_lights.setPixelColor(i, lights.colours[i].red, lights.colours[i].green, lights.colours[i].blue);
    }
    m_lights.show();
    m_currentLights = lights;
}

float DrawBot::UpdateBatteryLevel(bool lights)
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

void DrawBot::SetPenUp(bool up)
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

void DrawBot::SetPenServo(double pos)
{
    double newPos = mapf(pos, 0.0, 1.0, m_currentSettings.servo.penDownPosition, m_currentSettings.servo.penUpPosition);
    m_penLift.write(newPos);
}

void DrawBot::SetMotorSpeed(int motor, double speed)
{
    bool direction = false;
    if(speed > 0)
        direction = true;
    double pwmVal = abs(speed) * 65535.0;

    if(motor == 1)
    {
        if(direction)
        {
            digitalWrite(M1_DIRA, LOW);
            digitalWrite(M1_DIRB, HIGH);
        }
        else
        {
            digitalWrite(M1_DIRA, HIGH);
            digitalWrite(M1_DIRB, LOW);
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

long DrawBot::GetM1EncoderDelta()
{
    long delta = m_m1En - m_lastM1En;
    m_lastM1En = m_m1En;

    return delta;
}

long DrawBot::GetM2EncoderDelta()
{
    long delta = m_m2En - m_lastM2En;
    m_lastM2En = m_m2En;

    return delta;
}

DrawBot_Colour DrawBot::ReadColour()
{
    float r, g, b;
    m_colourTCS.getRGB(&r, &g, &b);

    DrawBot_Colour colour = { (uint8_t)r, (uint8_t)g, (uint8_t)b };
    return colour;
}

DrawBot_IRArray DrawBot::ReadIRSensors(bool calibrated)
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

    DrawBot_IRArray result;
    result.farLeft = ir5;
    result.left = ir3;
    result.centre = ir1;
    result.right = ir2;
    result.farRight = ir4;
    return result;
}

int DrawBot::ReadToFSensor(DrawBot_ToFLocation location)
{
    return m_tofs[location].readRangeContinuousMillimeters();
}

DrawBot_Motion DrawBot::ReadIMU()
{
    BMX160_IMUReading reading = m_imu->ReadIMU();

    DrawBot_Motion motion;
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

void DrawBot::EnableBumpInterrupt(uint8_t threshold, uint8_t duration, BMX160_InterruptPin pin)
{
    m_imu->AddHighGInterrupt(pin, threshold, duration, BMX160_HIGH_HY_1, BMX160_XYZ_AXES);
}

void DrawBot::DisableBumpInterrupt()
{
    m_imu->RemoveInterrupt(BMX160_HIGH_G_INT);
}