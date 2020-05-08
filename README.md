# DrawBot API
## Introduction
An Arduino based library for the DrawBot board made in SensiLab

---
## Required Arduino Libraries
* The SensiLab BMX160-Arduino 9DoF IMU library
* Adafruit_NeoPixel library - https://github.com/adafruit/Adafruit_NeoPixel
* Adafruit_TCS34725 Colour Sensor library - https://github.com/adafruit/Adafruit_TCS34725
* Pololu VL53L0X ToF Sensor Library - https://github.com/pololu/vl53l0x-arduino
* SensiLab DrawBot V3 board definition

---
## Usage Guide
### Initialisation
To initialise the DrawBot with the default settings:
``` c++
#include <DrawBot.h>
DrawBot bot;

bot.Initialise();
```
The various settings of the DrawBot are stored in an instance of the DrawBot_Setting struct which can be passed to the Initialise method. This struct is defined as the following:

``` c++
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
```

Given the large number of settings it is easiest to use the default settings as a starting point and modify anything that is needed. For example, if you wanted to use the default settings but a higher gain on the colour sensor:

``` c++
DrawBot_Settings settings = DrawBot::GetDefaultSettings();
settings.colour.gain = TCS34725_GAIN_16X;

DrawBot bot;

bot.Initialise(settings);
```
The default settings are as follows:
``` c++
const DrawBot_Settings DrawBot::s_defaultSettings = {
    { BMX160_ACCEL_RATE_1600HZ, BMX160_ACCEL_RANGE_2G, 
      BMX160_GYRO_RATE_1600HZ, BMX160_GYRO_RANGE_1000_DPS },    //IMU Defaults
    { TCS34725_GAIN_4X, TCS34725_INTEGRATIONTIME_24MS },        //Colour sensor defaults
    { S2_PWM, 50.0, 120.0 },                                    //Servo defaults
    { 500, 0.25f, 33, 14, 10 },                                 //ToF defaults
    { true, true },                                             //Motor defaults
    true,                                                       //White Light default
    0                                                           //IR Dim default
};
```
After the bot has been initialised, single sensor settings can be modified using the following methods:
``` c++
void SetWhiteLight(bool on);
void SetIRDimLevel(int level);
void SetupIMU(DrawBot_IMUSettings settings);
void SetupColourSensor(DrawBot_ColourSettings settings);
void SetupToFSensor(int number, DrawBot_ToFSettings settings);
void SetupMotors(DrawBot_MotorSettings settings);
```
---
### Sensor Reading
To read the IMU:
``` c++
DrawBot_Motion reading = bot.ReadIMU();
```
The DrawBot_Motion struct is defined as follows: 
``` c++
struct DrawBot_Vector3
{
    double x, y, z;
};

struct DrawBot_Motion
{
    DrawBot_Vector3 accel;
    DrawBot_Vector3 gyro;
    DrawBot_Vector3 mag;
};
```
The IMU can also generate an interrupt based on bump sensing. Setting up bump detection is as follows:
``` c++
void bumpMethod() 
{ 
    //code here 
}
//Sets up the default bump interrupt, threshold = 20, duration = 10, using the first BMX16 Int pin
bot.EnableBumpInterrupt();
attachInterrupt(BMX_INT_1, bumpMethod, RISING);

//Alternatively threshold, duration and pin can be defined
bot.EnableBumpInterrupt(100, 10, BMX160_INT_PIN_2)
attachInterrupt(BMX_INT_2, bumpMethod, RISING);

```
Threshold is measured in milli-gs and is the amount required to trigger the bump, duration is in milliseconds in is the time that the imu must be over the threshold in order to trigger in the interrupt

---
To read the Colour Sensor:
``` c++
DrawBot_Colour reading = bot.ReadColour();
```
The DrawBot_Colour struct is defined as follows:
``` c++
struct DrawBot_Colour
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};
```
---
To read the IR line sensors:
``` c++
DrawBot_IRArray reading = bot.ReadIRSensors();      //Will read sensors and apply calibration values
DrawBot_IRArray reading = bot.ReadIRSensors(false); //Will read raw values without calibration
```
The DrawBot_IRArray struct is defined as follows:
``` c++
struct DrawBot_IRArray
{
    int centre;
    int left;
    int right;
    int farLeft;
    int farRight;
};
```
When reading calibrated the results will be between 0-255 where 0 is the whitest seen during calibration and 255 is the blackest. When reading uncalibrated the values will be between 0-1023.
The IR emitters in the array can also be dimmed if too much IR light is present:
``` c++
bot.SetIRDimLevel(20);
```
The IR dimmer has 32 levels of dimming. The parameter value should be between 0-31.

---
To read the Time of Flight sensors:
``` c++
int readingLeft = bot.ReadToFSensor(TOF_LEFT);
int readingCenter = bot.ReadToFSensor(TOF_CENTRE);
int readingRight = bot.ReadToFSensor(TOF_RIGHT);
```
Valid parameter values are defined by the DrawBot_ToFLocation enum:
``` c++
enum DrawBot_ToFLocation
{
    TOF_LEFT,
    TOF_CENTRE,
    TOF_RIGHT
};
```
The values returned are in millimetres.

---
To read the motor encoders:
``` c++
long m1Reading = bot.GetM1Encoder();
long m2Reading = bot.GetM2Encoder();
```
Values are increased when encoders spin in one direction and decreased when spun in the other direction.

---

### Setting the RGB lights
The RGB LEDs can be setting like so:
``` c++
DrawBot_Lights newLights;
//Zero out the light colours
for(int i = 0; i < LIGHT_COUNT; i++)
    newLights.colours[i] = { 0, 0, 0 }
//Set the first light to red
newLights.colours[0] = { 255, 0, 0 };
//Set the second light to purple
newLights.colours[1] = { 255, 0, 255 };
//Set the third light to green
newLights.colours[2] = { 0, 255, 0 }
//... etc.

bot.SetLights(newLights);
```
You can also read and modify the current lights (without having to recreate the whole light struct):
``` c++
DrawBot_Light currentLights = bot.GetCurrentLights();
//Turn off the last light 
currentLights.colours[7].red = 0;
currentLights.colours[7].green = 0;
currentLights.colours[7].blue = 0;

bot.SetLights(currentLights);
```
---
### Updating the Battery Level LEDs
The battery level LEDs can be set by calling the following:
``` c++
float percentage = bot.UpdateBatteryLevel();
```
The UpdateBatteryLevel method also returns the current battery level percentage.

---
### Running the Motors
The motors can be driven as follows:
``` c++
bot.SetMotorSpeed(1, 1.0);  //Set motor 1 to full speed
bot.SetMotorSpeed(2, -1.0); //Set motor 2 to full speed in the other direction
bot.SetMotorSpeed(1, 0.0);  //Set motor 1 to off
bot.SetMotorSpeed(2, 0.5);  //Set motor 2 to half speed
```
The first parameter identifies the motor (1 or 2), the second parameter is the desired speed. The speed should be a number between -1.0 and 1.0.

---
### Pen Servo
The pen mechanism can be controlled as follows:
``` c++
bot.RaisePen();
bot.LowerPen();
```
The two positions that the servo moves to are defined in the servo settings struct.

---
### Calibrating the IMU
IMU calibration is coming soon

---
### Calibrating the IR sensor array
The IR Array can be calibrated like so:
``` c++
bot.CalibrateIRArray();
```
Place a black and white calibration pattern under the DrawBot. The DrawBot will spin for around 10 seconds measuring the pattern and saving the lows and highs for each sensor.

---
### © SensiLab 2019