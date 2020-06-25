#include <DrawBot.h>

DrawBot bot;

double kp = 0.001;      //Determines the "strength" of the motor correction.
double power = 0.2;     //The forward speed
double slavePower = 0;  //The modified power for the second motor

void setup()
{
    Serial.begin(9600);
    bot.Initialise();
    
    bot.SetWhiteLight(false);
    
    bot.SetPenUp(false);
}


void loop()
{
    //Find the difference between the two encoders since last read
    double error = bot.GetM1EncoderDelta() - bot.GetM2EncoderDelta();

    //Calculate the second motor power based on the error value and the correction "strength" factor
    slavePower += error * kp;

    Serial.println(slavePower);

    bot.SetMotorSpeed(1, power);
    bot.SetMotorSpeed(2, slavePower * -1);  //Negate the speed to deal with mirrored motors

    bot.UpdateBatteryLevel();
}
