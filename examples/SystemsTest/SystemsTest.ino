#include <Drawbotic_DB1.h>

DB1 bot;

void setup()
{
    Serial.begin(9600);
    bot.Initialise();
    
    bot.SetWhiteLight(false);
    
    bot.SetPenUp(false);
    
    //bot.CalibrateIRArray();       //Uncomment if you want the robot to calibrate the IR sensors. Robot will spin on spot for a few seconds!
    //bot.CalibrateColourSensor();  //Uncomment to calibrate the colour sensor. Make sure the robot is on a white surface.
}


void loop()
{
    //Uncomment each method to test different functionality

    //I2C_Loop();
    //Colour_Loop();
    //IRCali_Loop();
    //IRRaw_Loop();
    //ToF_Loop();
    //Motors_Loop();
    //RGB_Loop();
    //Pen_Loop();
    Battery_Loop();
}

void Battery_Loop()
{
    Serial.println(bot.UpdateBatteryLevel());

    delay(1000);
}

void Pen_Loop()
{
    bot.SetPenUp(true);
    delay(2000);
    bot.SetPenUp(false);
    delay(2000);
}

void RGB_Loop()
{
    DB1_Lights newLights;

    //Set all lights to red
    for(int i = 0; i < LIGHT_COUNT; i++)
        newLights.colours[i] = {255, 0, 0};
    
    bot.SetLights(newLights);
    delay(500);

    //Set all lights to yellow
    for(int i = 0; i < LIGHT_COUNT; i++)
        newLights.colours[i] = {255, 255, 0};
    
    bot.SetLights(newLights);
    delay(500);

    //Set all lights to green
    for(int i = 0; i < LIGHT_COUNT; i++)
        newLights.colours[i] = {0, 255, 0};
    
    bot.SetLights(newLights);
    delay(500);

    //Set all lights to cyan
    for(int i = 0; i < LIGHT_COUNT; i++)
        newLights.colours[i] = {0, 255, 255};
    
    bot.SetLights(newLights);
    delay(500);

    //Set all lights to blue
    for(int i = 0; i < LIGHT_COUNT; i++)
        newLights.colours[i] = {0, 0, 255};
    
    bot.SetLights(newLights);
    delay(500);

    //Set all lights to purple
    for(int i = 0; i < LIGHT_COUNT; i++)
        newLights.colours[i] = {255, 0, 255};
    
    bot.SetLights(newLights);
    delay(500);
}

void Motors_Loop()
{
    //Set motor speed
    bot.SetMotorSpeed(1, 0.1);
    bot.SetMotorSpeed(2, -0.1);

    //Print encoder values
    Serial.print("M1:\t"); Serial.print(bot.GetM1Encoder());
    Serial.print("\tM2:\t"); Serial.println(bot.GetM2Encoder());

    delay(50);
}

void ToF_Loop()
{
    //Read and Print all three Time of Flight sensors
    Serial.print(bot.ReadToFSensor(TOF_LEFT)); Serial.print("\t\t");
    Serial.print(bot.ReadToFSensor(TOF_CENTRE)); Serial.print("\t\t");
    Serial.println(bot.ReadToFSensor(TOF_RIGHT));

    delay(50);
}

void IRCali_Loop()
{
    //Read calibrated values of IR sensors
    DB1_IRArray reading = bot.ReadIRSensors();

    //Print each value
    Serial.print(reading.farLeft); Serial.print("\t\t");
    Serial.print(reading.left); Serial.print("\t");
    Serial.print(reading.centre); Serial.print("\t");
    Serial.print(reading.right); Serial.print("\t\t");
    Serial.println(reading.farRight);

    delay(50);
}

void IRRaw_Loop()
{
    //Read raw values of tIR sensors
    DB1_IRArray reading = bot.ReadIRSensors(false);

    //Print each value
    Serial.print(reading.farLeft); Serial.print("\t\t");
    Serial.print(reading.left); Serial.print("\t");
    Serial.print(reading.centre); Serial.print("\t");
    Serial.print(reading.right); Serial.print("\t\t");
    Serial.println(reading.farRight);

    delay(50);
}

void Colour_Loop()
{
    bot.SetWhiteLight(true);
    VEML6040_Colour reading = bot.ReadColour(true);

    Serial.print("R:\t"); Serial.print(reading.red); 
    Serial.print("\tG:\t"); Serial.print(reading.green); 
    Serial.print("\tB:\t"); Serial.println(reading.blue);

    DB1_Colour lightColour = { (uint8_t)reading.red, (uint8_t)reading.green, (uint8_t)reading.blue };
    bot.SetTopLight(lightColour);
    delay(50);
}

void I2C_Loop()
{
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ ) 
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.

        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16) 
            Serial.print("0");
            Serial.print(address,HEX);
            Serial.println("  !");
            nDevices++;
        }
        else if (error==4) 
        {
            Serial.print("Unknown error at address 0x");
            if (address<16) 
            Serial.print("0");
            Serial.println(address,HEX);
        }
        //delay(10); 
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
    //while(1){}

    delay(1000);           // wait 5 seconds for next scan
}
