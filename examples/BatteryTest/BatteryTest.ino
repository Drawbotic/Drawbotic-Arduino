#include <Drawbotic_DB1.h>

#define PEN_RATE_MS 2000

DB1 bot;

int penTimebankMS = 0;
long lastMillis = 0;
bool penState = false;

void setup()
{
    Serial.begin(9600);
    bot.init();

    DB1_Lights whiteLights;
    for(int i = 0; i < LIGHT_COUNT; i++)
        whiteLights.colours[i] = {255, 255, 255};
    bot.setLights(whiteLights);

    bot.setMotorSpeed(1, 0.5);
    bot.setMotorSpeed(2, -0.5);
}

void loop()
{
    long currentMillis = millis();
    long deltaMS = currentMillis - lastMillis;
    lastMillis = currentMillis;
    penTimebankMS += deltaMS;
    if(penTimebankMS > PEN_RATE_MS)
    {
        penState = !penState;
        bot.setPen(penState);
    }

    Serial.print("Up time: ");
    Serial.print(currentMillis/1000.0);
    Serial.println("s");

    DB1_IRArray irReading = bot.readIRSensors(false);
    Serial.print("IR: ");
    Serial.print(irReading.farLeft); Serial.print("\t\t");
    Serial.print(irReading.left); Serial.print("\t");
    Serial.print(irReading.centre); Serial.print("\t");
    Serial.print(irReading.right); Serial.print("\t\t");
    Serial.println(irReading.farRight);

    Serial.print("ToF: ");
    Serial.print(bot.readToFSensor(TOF_LEFT)); Serial.print("\t\t");
    Serial.print(bot.readToFSensor(TOF_CENTRE)); Serial.print("\t\t");
    Serial.println(bot.readToFSensor(TOF_RIGHT));

    DB1_Orientation orientation = bot.getOrientation();
    Serial.print("IMU: ");
    Serial.print("Heading: "); Serial.print(orientation.heading);
    Serial.print("\tPitch: "); Serial.print(orientation.pitch);
    Serial.print("\tRoll: "); Serial.print(orientation.roll);

    VEML6040_Colour colour = bot.readColour();
    Serial.println("Colour: ");
    Serial.print("R:\t"); Serial.print(colour.red); 
    Serial.print("\tG:\t"); Serial.print(colour.green); 
    Serial.print("\tB:\t"); Serial.println(colour.blue);

    Serial.print("Battery: ");
    Serial.print(bot.updateBatteryLevel());
    Serial.println("%");
    delay(1000);
}