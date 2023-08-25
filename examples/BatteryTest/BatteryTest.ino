#include <Drawbotic_DB1.h>

#define PEN_RATE_MS 2000

int penTimebankMS = 0;
long lastMillis = 0;
bool penState = false;

void setup() {
  Serial.begin(9600);
  DB1.init();

  DB1_Lights whiteLights;
  for(int i = 0; i < LIGHT_COUNT; i++)
    whiteLights.colours[i] = {255, 255, 255};
  DB1.setLights(whiteLights);

  DB1.setMotorSpeed(1, 0.5);
  DB1.setMotorSpeed(2, -0.5);
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

  DB1_IRArray irReading = DB1.getIRSensors(false);
  Serial.print("IR: ");
  Serial.print(irReading.farLeft); Serial.print("\t\t");
  Serial.print(irReading.left); Serial.print("\t");
  Serial.print(irReading.centre); Serial.print("\t");
  Serial.print(irReading.right); Serial.print("\t\t");
  Serial.println(irReading.farRight);

  Serial.print("ToF: ");
  Serial.print(DB1.getToFSensor(TOF_LEFT)); Serial.print("\t\t");
  Serial.print(DB1.getToFSensor(TOF_CENTRE)); Serial.print("\t\t");
  Serial.println(DB1.getToFSensor(TOF_RIGHT));

  DB1_Orientation orientation = DB1.getOrientation();
  Serial.print("IMU: ");
  Serial.print("Heading: "); Serial.print(orientation.heading);
  Serial.print("\tPitch: "); Serial.print(orientation.pitch);
  Serial.print("\tRoll: "); Serial.print(orientation.roll);

  VEML6040_Colour colour = DB1.getColour();
  Serial.println("Colour: ");
  Serial.print("R:\t"); Serial.print(colour.red); 
  Serial.print("\tG:\t"); Serial.print(colour.green); 
  Serial.print("\tB:\t"); Serial.println(colour.blue);

  Serial.print("Battery: ");
  Serial.print(DB1.updateBatteryLevel());
  Serial.println("%");
  delay(1000);
}