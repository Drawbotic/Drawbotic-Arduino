#include <Drawbotic_DB1.h>

#define PEN_RATE_MS 2000

DB1 bot;

void setup()
{
    Serial.begin(9600);
    bot.Initialise();
    
    bot.SetWhiteLight(true);
    
    bot.SetPenUp(false);

    DB1_Lights whiteLights;
    for(int i = 0; i < LIGHT_COUNT; i++)
        whiteLights.colours[i] = {255, 255, 255};
    bot.SetLights(whiteLights);

    bot.SetMotorSpeed(1, 0.5);
    bot.SetMotorSpeed(2, -0.5);
}

int penTimebankMS = 0;
long lastMillis = 0;
bool penState = false;

void loop()
{
    long currentMillis = millis();
    long deltaMS = currentMillis - lastMillis;
    lastMillis = currentMillis;
    penTimebankMS += deltaMS;
    if(penTimebankMS > PEN_RATE_MS)
    {
        penState = !penState;
        bot.SetPenUp(penState);
    }

    Serial.print("Up time: ");
    Serial.print(currentMillis/1000.0);
    Serial.println("s");

    DB1_IRArray irReading = bot.ReadIRSensors(false);
    Serial.print("IR: ");
    Serial.print(irReading.farLeft); Serial.print("\t\t");
    Serial.print(irReading.left); Serial.print("\t");
    Serial.print(irReading.centre); Serial.print("\t");
    Serial.print(irReading.right); Serial.print("\t\t");
    Serial.println(irReading.farRight);

    Serial.print("ToF: ");
    Serial.print(bot.ReadToFSensor(TOF_LEFT)); Serial.print("\t\t");
    Serial.print(bot.ReadToFSensor(TOF_CENTRE)); Serial.print("\t\t");
    Serial.println(bot.ReadToFSensor(TOF_RIGHT));

    /*DB1_Motion imuReading = bot.ReadIMU();
    Serial.print("IMU: ");
    Serial.print("AX: "); Serial.print(imuReading.accel.x);
    Serial.print("\tAY: "); Serial.print(imuReading.accel.y);
    Serial.print("\tAZ: "); Serial.print(imuReading.accel.z);
    Serial.print("\tGX: "); Serial.print(imuReading.gyro.x);
    Serial.print("\tGY: "); Serial.print(imuReading.gyro.y);
    Serial.print("\tGZ: "); Serial.print(imuReading.gyro.z);
    Serial.print("\tMX: "); Serial.print(imuReading.mag.x);
    Serial.print("\tMY: "); Serial.print(imuReading.mag.y);
    Serial.print("\tMZ: "); Serial.println(imuReading.mag.z);*/

    VEML6040_Colour cReading = bot.ReadColour();
    Serial.println("Colour: ");
    Serial.print("R:\t"); Serial.print(cReading.red); 
    Serial.print("\tG:\t"); Serial.print(cReading.green); 
    Serial.print("\tB:\t"); Serial.println(cReading.blue);

    Serial.print("Battery: ");
    Serial.print(bot.UpdateBatteryLevel());
    Serial.println("%");
    delay(1000);
}