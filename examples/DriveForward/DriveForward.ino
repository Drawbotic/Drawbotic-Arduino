#include <Drawbotic_DB1.h>

double kp = 0.001;      //Determines the "strength" of the motor correction.
double power = 0.2;     //The forward speed
double followPower = 0.2;  //The modified power for the second motor

void setup() {
  Serial.begin(9600);
  DB1.init();
}


void loop() {
  //Find the difference between the two encoders since last read
  double error = DB1.getM1EncoderDelta() - DB1.getM2EncoderDelta();

  //Calculate the second motor power based on the error value and the correction "strength" factor
  followPower += (error * kp);

  //The first motor is set to the target speed. The second is set to the corrected follow speed
  DB1.setMotorSpeed(1, power);
  DB1.setMotorSpeed(2, followPower);

  //Update the battery lights on the DB-1
  DB1.updateBatteryLevel();
}
