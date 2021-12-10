#include <Drawbotic_DB1.h>
#include "HSV2RGB.h"

DB1 bot;

//Used for delta time calc
int lastMS = 0;

//Hue value for LEDs
double hue = 0;

void setup() 
{
    Serial.begin(9600);
    bot.Initialise();
    bot.SetWhiteLight(false);
}

void loop() 
{
    int currentMS = millis();
    int deltaMS = currentMS - lastMS;
    lastMS = currentMS;

    //slowly increase the hue value
    hue += 0.1 * (deltaMS / 1000.0);
    //wrap around if needed
    if(hue > 1.0)
        hue = 0;

    //New light colour is based on hue value
    rgb lightColour = hsv2rgb({hue * 360.0, 1.0, 1.0});

    //Update the lights with new colour
    DB1_Lights newLights;
    for(int i = 0; i < LIGHT_COUNT; i++)
        newLights.colours[i] = { (int)(255 * lightColour.r), (int)(255 * lightColour.g), (int)(255 * lightColour.b) };
    bot.SetLights(newLights);

    bot.UpdateBatteryLevel();
}