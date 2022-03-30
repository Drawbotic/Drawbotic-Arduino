#include <Drawbotic_DB1.h>

DB1 bot;

double kp = 0.001;      //Determines the "strength" of the motor correction.
double power = 0.2;     //The forward speed
double followPower = 0;  //The modified power for the second motor

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
    int m1 = bot.GetM1Encoder();
    int m2 = bot.GetM2Encoder();
    int m1Delta = bot.GetM1EncoderDelta();
    int m2Delta = bot.GetM2EncoderDelta();
    double error = m1Delta - m2Delta;

    //Calculate the second motor power based on the error value and the correction "strength" factor
    followPower += (error * kp);
    Serial.print("M1:\t"); Serial.print(m1);
    Serial.print("\tM2:\t"); Serial.print(m2);
    Serial.print("\tM1 D: ");
    Serial.print(m1Delta);
    Serial.print("\tM2 D: ");
    Serial.print(m2Delta);
    Serial.print("\terror: ");
    Serial.print(error);
    Serial.print("\tM2 Power: ");
    Serial.println(followPower);

    bot.SetMotorSpeed(1, power);
    bot.SetMotorSpeed(2, followPower);  //Negate the speed to deal with mirrored motors

    bot.UpdateBatteryLevel();
    delay(100);
}
