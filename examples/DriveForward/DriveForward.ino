#include <Drawbotic_DB1.h>

DB1 bot;

double kp = 0.001;      //Determines the "strength" of the motor correction.
double power = 0.2;     //The forward speed
double followPower = 0.2;  //The modified power for the second motor

void setup()
{
    Serial.begin(9600);
    bot.init();
}


void loop()
{
    //Find the difference between the two encoders since last read
    double error = bot.getM1EncoderDelta() - bot.getM2EncoderDelta();

    //Calculate the second motor power based on the error value and the correction "strength" factor
    followPower += (error * kp);

    //The first motor is set to the target speed. The second is set to the corrected follow speed
    bot.setMotorSpeed(1, power);
    bot.setMotorSpeed(2, followPower);

    //Update the battery lights on the DB-1
    bot.updateBatteryLevel();
}
