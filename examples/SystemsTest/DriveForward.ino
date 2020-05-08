#include <DrawBot.h>

DrawBot bot;

double kp = 0.001;      //Determines the "strength" of the motor correction.
double power = 0.2;     //The forward speed
double slavePower = 0;  //The modified power for the second motor

void bumpInt()
{
    Serial.println("Bump!");
}

void setup()
{
    Serial.begin(9600);
    bot.Initialise();
    
    bot.SetWhiteLight(false);
    bot.SetIRDimLevel(0);   //0 is the highest

    bot.EnableBumpInterrupt();
    attachInterrupt(BMX_INT_1, bumpInt, RISING);
    
    bot.LowerPen();
}


void loop()
{
    //Find the difference between the two encoders since last read
    double error = bot.GetM1EncoderDelta() - bot.GetM2EncoderDelta();

    //Calculate the second motor power based on the error value and the correction "strength" factor
    slavePower += error * kp;
    bot.SetMotorSpeed(1, power);
    bot.SetMotorSpeed(2, slavePower * -1);  //Negate the speed to deal with mirrored motors

    bot.UpdateBatteryLevel();
}
