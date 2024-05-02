/*
 * This sketch is designed to test the battery usage of the DB1
 * It runs the motors 50% speed, the pen servo toggled every 2 seconds,
 * the RGB LEDs are set to full brightness and all sensors are read and
 * logged to the Serial terminal.
 * 
 * It should be a good indication of typical-to-high usage of the battery
 * The total uptime in seconds is also logged to the Serial terminal.
 */

#include <Drawbotic_DB1.h>

#define PEN_RATE_MS 2000

int penTimebankMS = 0;
long lastMillis = 0;
bool penState = false;

void setup() {
  Serial.begin(9600);
  DB1.init();

  db1_lights_t whiteLights;
  for(int i = 0; i < DB1_LIGHT_COUNT; i++)
    whiteLights.led[i] = {255, 255, 255};
  DB1.setLights(whiteLights);

  DB1.setMotorSpeed(DB1_M1, 0.5);
  DB1.setMotorSpeed(DB1_M2, -0.5);
}

void loop() {
  long currentMillis = millis();
  long deltaMS = currentMillis - lastMillis;
  lastMillis = currentMillis;
  penTimebankMS += deltaMS;
  if(penTimebankMS > PEN_RATE_MS) {
    penState = !penState;
    DB1.setPen(penState);
  }

  Serial.print("Up time: ");
  Serial.print(currentMillis/1000.0);
  Serial.println("s");

  db1_ir_array_t irReading = DB1.getIRSensors(false);
  Serial.print("IR: ");
  Serial.print(irReading.far_left); Serial.print("\t\t");
  Serial.print(irReading.left); Serial.print("\t");
  Serial.print(irReading.centre); Serial.print("\t");
  Serial.print(irReading.right); Serial.print("\t\t");
  Serial.println(irReading.far_right);

  Serial.print("ToF: ");
  Serial.print(DB1.getToFSensor(DB1_TOF_LEFT)); Serial.print("\t\t");
  Serial.print(DB1.getToFSensor(DB1_TOF_CENTRE)); Serial.print("\t\t");
  Serial.println(DB1.getToFSensor(DB1_TOF_RIGHT));

  db1_orientation_t orientation = DB1.getOrientation();
  Serial.print("IMU: ");
  Serial.print("Heading: "); Serial.print(orientation.heading);
  Serial.print("\tPitch: "); Serial.print(orientation.pitch);
  Serial.print("\tRoll: "); Serial.println(orientation.roll);

  db1_colour_reading_t colour = DB1.getColour(false);
  Serial.println("Colour: ");
  Serial.print("R:\t"); Serial.print(colour.r); 
  Serial.print("\tG:\t"); Serial.print(colour.g); 
  Serial.print("\tB:\t"); Serial.println(colour.b);

  Serial.print("Battery: ");
  Serial.print(DB1.updateBatteryLevel());
  Serial.println("%");
  delay(1000);
}