#include <Drawbotic_DB1.h>

#define TOF_TH 80   //Distance from ToF sensor that triggers the bot to avoid

DB1 bot;

double kp = 0.001;      //Determines the "strength" of the motor correction.
double power = 0.5;     //The forward speed
double followPower = 0.5;  //The modified power for the second motor

void setup()
{
    Serial.begin(9600);
    bot.Initialise();
    
    bot.SetWhiteLight(false);
    
    bot.SetPenUp(false);
}

void loop()
{
    //Read ToF sensors before anything else
    int tofLeft = bot.ReadToFSensor(TOF_LEFT);
    int tofCentre = bot.ReadToFSensor(TOF_CENTRE);
    int tofRight = bot.ReadToFSensor(TOF_RIGHT);

    if(tofCentre < TOF_TH)  //If the centre ToF is triggered
    {
        if(tofRight >  tofLeft) //If there is more space to the right...
        {
            //...turn right
            OnSpotTurn(-0.9);
        }
        else if(tofLeft > tofRight) //If there is more space to the left...
        {
            //...turn left
            OnSpotTurn(0.9);
        }
        else
        {
            //Turn a random direction
            if(random(0,2) == 1)
            {
                OnSpotTurn(-0.9);
            }
            else
            {
                OnSpotTurn(0.9);
            }
        }
        //Keep turning until clear
        while(bot.ReadToFSensor(TOF_CENTRE) < TOF_TH){}
    }
    else if(tofLeft < TOF_TH) //If the left ToF is triggered
    {
        //Turn right
        OnSpotTurn(-0.9);
        //Keep turning until clear
        while(bot.ReadToFSensor(TOF_LEFT) < TOF_TH){}
    }
    else if(tofRight < TOF_TH)  //If the right ToF is triggered
    {
        //Turn left
        OnSpotTurn(0.9);
        //Keep turning until clear
        while(bot.ReadToFSensor(TOF_RIGHT) < TOF_TH){}
    }
    else    //If no ToF sensors are triggered, then drive forward
    {
        //Find the difference between the two encoders since last read
        double error = bot.GetM1EncoderDelta() - bot.GetM2EncoderDelta();
    
        //Calculate the second motor power based on the error value and the correction "strength" factor
        followPower += error * kp;
        bot.SetMotorSpeed(1, power);
        bot.SetMotorSpeed(2, followPower);  //Negate the speed to deal with mirrored motors
    }
    
    bot.UpdateBatteryLevel();   //Update the battery level
}

void OnSpotTurn(double power)
{
    Turn(power, 1.0);
}

void Turn(double power, double turnAmount)
{
    bot.SetMotorSpeed(1, power);
    bot.SetMotorSpeed(2, power * turnAmount);
}