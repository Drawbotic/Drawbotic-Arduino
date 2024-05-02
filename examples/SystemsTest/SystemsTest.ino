#include <Drawbotic_DB1.h>

void setup() {
  Serial.begin(9600);
  //while(!Serial){}              //Uncomment to block until Serial is connected, useful for debugging
  DB1.init();
  
  //DB1.calibrateIRArray();       //Uncomment if you want the robot to calibrate the IR sensors. Robot will spin on the spot for a few seconds!
  //DB1.calibrateColourSensor();  //Uncomment to calibrate the colour sensor. Make sure the robot is on a white surface.
}


void loop() {
  //Uncomment each method to test different functionality

  //I2C_Loop();     //This lists all of the devices on the I2C bus you should see: 0x10, 0x1A, 0x1B, 0x1C, 0x4A
  //IMU_Loop();     //This will output the orientation and acceleration values from the IMU
  //Colour_Loop();  //This will output the RBG value from the colour sensor and update the top Neopixel to an approximation of that colour
  //IRCali_Loop();  //This will output the calibrated values of the IR line sensors (calibration must be run before hand for them to be meaningful)
  //IRRaw_Loop();   //This will output the raw (uncalibrated) values of the IR line sensors
  //ToF_Loop();     //This will output the distances measured by the time of flight sensors
  //Motors_Loop();  //This will run both motors in opposite directions at 1/10th power, the motor encoder values will also be output
  //RGB_Loop();     //This will cycle the bottom RGB LEDs through a series of colours
  //Pen_Loop();     //This will move the pen up and down every 2 seconds
  //Sensor_Loop();  //This will output the data of all I2C sensors (ToF, Colour, IMU) at once to test data throughput
  
  //Calling DB1.updateBatteryLevel will update the battery guage LEDs on the top of the DB1, the returned percentage value can optionally be printed out
  float batt_lvl = DB1.updateBatteryLevel();
  //Serial.print("Battery: "); Serial.print(batt_lvl); Serial.println("%");
}

void Pen_Loop() {
  DB1.setPen(true);
  delay(2000);
  DB1.setPen(false);
  delay(2000);
}

void RGB_Loop() {
  db1_lights_t newLights;

  //Set all lights to red
  for(int i = 0; i < DB1_LIGHT_COUNT; i++)
    newLights.led[i] = {255, 0, 0};
  
  DB1.setLights(newLights);
  delay(500);

  //Set all lights to yellow
  for(int i = 0; i < DB1_LIGHT_COUNT; i++)
    newLights.led[i] = {255, 255, 0};
  
  DB1.setLights(newLights);
  delay(500);

  //Set all lights to green
  for(int i = 0; i < DB1_LIGHT_COUNT; i++)
    newLights.led[i] = {0, 255, 0};
  
  DB1.setLights(newLights);
  delay(500);

  //Set all lights to cyan
  for(int i = 0; i < DB1_LIGHT_COUNT; i++)
    newLights.led[i] = {0, 255, 255};
  
  DB1.setLights(newLights);
  delay(500);

  //Set all lights to blue
  for(int i = 0; i < DB1_LIGHT_COUNT; i++)
    newLights.led[i] = {0, 0, 255};
  
  DB1.setLights(newLights);
  delay(500);

  //Set all lights to purple
  for(int i = 0; i < DB1_LIGHT_COUNT; i++)
    newLights.led[i] = {255, 0, 255};
  
  DB1.setLights(newLights);
  delay(500);
}

void Motors_Loop() {
  //Set motor speed
  DB1.setMotorSpeed(DB1_M1, 0.1);
  DB1.setMotorSpeed(DB1_M2, -0.1);

  //Print encoder values
  Serial.print("M1:\t"); Serial.print(DB1.getM1Encoder());
  Serial.print("\tM2:\t"); Serial.println(DB1.getM2Encoder());

  delay(50);
}

void ToF_Loop() {
  //Read and Print all three Time of Flight sensors
  Serial.print(DB1.getToFSensor(DB1_TOF_LEFT)); Serial.print("\t\t");
  Serial.print(DB1.getToFSensor(DB1_TOF_CENTRE)); Serial.print("\t\t");
  Serial.println(DB1.getToFSensor(DB1_TOF_RIGHT));

  delay(50);
}

void IRCali_Loop() {
  //Read calibrated values of IR sensors
  db1_ir_array_t reading = DB1.getIRSensors(true);

  //Print each value
  Serial.print(reading.far_left); Serial.print("\t\t");
  Serial.print(reading.left); Serial.print("\t");
  Serial.print(reading.centre); Serial.print("\t");
  Serial.print(reading.right); Serial.print("\t\t");
  Serial.println(reading.far_right);

  delay(50);
}

void IRRaw_Loop() {
  //Read raw values of IR sensors
  db1_ir_array_t reading = DB1.getIRSensors(false);

  //Print each value
  Serial.print(reading.far_left); Serial.print("\t\t");
  Serial.print(reading.left); Serial.print("\t");
  Serial.print(reading.centre); Serial.print("\t");
  Serial.print(reading.right); Serial.print("\t\t");
  Serial.println(reading.far_right);

  delay(50);
}

void Colour_Loop() {
  DB1.setWhiteLight(true);
  db1_colour_reading_t reading = DB1.getColour(true);

  Serial.print("R:\t"); Serial.print(reading.r); 
  Serial.print("\tG:\t"); Serial.print(reading.g); 
  Serial.print("\tB:\t"); Serial.print(reading.b);
  Serial.print("\tW:\t"); Serial.println(reading.w);

  db1_colour_t lightColour = { (uint8_t)reading.r, (uint8_t)reading.g, (uint8_t)reading.b };
  DB1.setTopLight(lightColour);
  delay(50);
}

void IMU_Loop() {
  db1_orientation_t o = DB1.getOrientation();
  db1_vector_t a = DB1.getAcceleration();

  Serial.print("Heading:\t"); Serial.print(o.heading);
  Serial.print("\tPitch:\t"); Serial.print(o.pitch);
  Serial.print("\tRoll:\t"); Serial.print(o.roll);
  Serial.print("\tX:\t"); Serial.print(a.x);
  Serial.print("\tY:\t"); Serial.print(a.y);
  Serial.print("\tZ:\t"); Serial.print(a.z);
  Serial.print("\tBump:\t"); Serial.println(DB1.wasBumped());

  delay(10);
}

void Sensor_Loop() {
  db1_orientation_t o = DB1.getOrientation();
  db1_vector_t a = DB1.getAcceleration();
  db1_colour_reading_t c = DB1.getColour(false);

  Serial.print(DB1.getToFSensor(DB1_TOF_LEFT)); Serial.print("\t");
  Serial.print(DB1.getToFSensor(DB1_TOF_CENTRE)); Serial.print("\t");
  Serial.print(DB1.getToFSensor(DB1_TOF_RIGHT));

  Serial.print("\tHeading:\t"); Serial.print(o.heading);
  Serial.print("\tPitch:\t"); Serial.print(o.pitch);
  Serial.print("\tRoll:\t"); Serial.print(o.roll);
  Serial.print("\tX:\t"); Serial.print(a.x);
  Serial.print("\tY:\t"); Serial.print(a.y);
  Serial.print("\tZ:\t"); Serial.print(a.z);

  Serial.print("\tR:\t"); Serial.print(c.r); 
  Serial.print("\tG:\t"); Serial.print(c.g); 
  Serial.print("\tB:\t"); Serial.print(c.b);
  Serial.print("\tW:\t"); Serial.println(c.w);

  delay(100);
}

void I2C_Loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning I2C...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
      Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
      Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(1000);           // wait 1 second for next scan
}