#include <Drawbotic_DB1.h>

#define TOF_TH 80   //Distance from ToF sensor that triggers the bot to avoid

double kp = 0.001;      //Determines the "strength" of the motor correction.
double power = 0.2;     //The forward speed
double followPower = 0.2;  //The modified power for the second motor

void setup() {
  Serial.begin(9600);
  DB1.init();
}

void loop() {
  //Read ToF sensors before anything else
  int tofLeft = DB1.getToFSensor(DB1_TOF_LEFT);
  int tofCentre = DB1.getToFSensor(DB1_TOF_CENTRE);
  int tofRight = DB1.getToFSensor(DB1_TOF_RIGHT);

  if(tofCentre < TOF_TH) { //If the centre ToF is triggered
    if(tofRight >  tofLeft) { //If there is more space to the right...
      //...turn right
      onSpotTurn(-0.2);
    }
    else { //If there is more space to the left...
      //...turn left
      onSpotTurn(0.2);
    }
    //Keep turning until clear
    while(DB1.getToFSensor(DB1_TOF_CENTRE) < TOF_TH){}
  }
  else if(tofLeft < TOF_TH) { //If the left ToF is triggered
    //Turn right
    onSpotTurn(-0.2);
    //Keep turning until clear
    while(DB1.getToFSensor(DB1_TOF_LEFT) < TOF_TH){}
  }
  else if(tofRight < TOF_TH) { //If the right ToF is triggered
    //Turn left
    onSpotTurn(0.2);
    //Keep turning until clear
    while(DB1.getToFSensor(DB1_TOF_RIGHT) < TOF_TH){}
  }
  else { //If no ToF sensors are triggered, then drive forward
    //Find the difference between the two encoders since last read
    double error = DB1.getM1EncoderDelta() - DB1.getM2EncoderDelta();

    //Calculate the second motor power based on the error value and the correction "strength" factor
    followPower += error * kp;
    DB1.setMotorSpeed(DB1_M1, power);
    DB1.setMotorSpeed(DB1_M2, followPower);  //Negate the speed to deal with mirrored motors
  }
  
  DB1.updateBatteryLevel();   //Update the battery level
}

//Power is a number between -1 and 1 determining the speed of the turn. Positive values are right turns, negative are left
void onSpotTurn(double power) {
  turn(power, 1.0);
}

void turn(double power, double turnAmount) {
  DB1.setMotorSpeed(DB1_M1, power);
  DB1.setMotorSpeed(DB1_M2, -power * turnAmount);
}