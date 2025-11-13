#include <Encoder.h>
#include "variable.h"
#include "CustomDS.h"
#include "SerialReader.h"
#include <HX711_ADC.h>


void setup() {
  Serial.begin(115200);
  bt.begin(115200);

  SerialUSB.print("MARS Starting: ");

  // Set the read and write resolutions
  analogReadResolution(12);
  analogWriteResolution(PWMRESOLN);
  
  // Set up stuff.
  deviceSetUp();
  // SerialUSB.println("Started.");

  // Initialize variables.
  // Always use the setLimb function to change the limb.
  setLimb(NOLIMB);

  // Device button settiongs
  readStream.begin(readHandleIncomingMessage, 100);

  // Set targets to invalid value 999.
  target = INVALID_TARGET;
  desired.add(INVALID_TARGET);

  // Initialize variable.
  streamType = DIAGNOSTICS; //SENSORSTREAM;
  stream = true;
  ctrlType = NONE;
  calib = NOCALIB;
  deviceError.num = 0x0000;
  // No fatal error.
  fatalError = false;

  // Reset packet number and run time.
  packetNumber.num = 0;
  startTime = millis();
  runTime.num = 0;
  delTime = 1.0;

  // Last received heart beat time
  lastRxdHeartbeat = millis();
  //Initialize the Laser pin 28
  pinMode(LASER_PIN, OUTPUT);
  // Set the values of pins 40 and 41
  pinMode(SAFETY_PIN, OUTPUT);
  // SAFETY_PIN and LASER_PIN are set to high when the robot is ready.
  digitalWrite(SAFETY_PIN, LOW);
  digitalWrite(LASER_PIN,HIGH);
  // Ready message.
  SerialUSB.print(fwVersion);
  SerialUSB.print(" | ");
  SerialUSB.print(deviceId);
  SerialUSB.print(" | ");
  SerialUSB.print(compileDate);
  SerialUSB.println(" | Ready.");
}

void loop() {
  // Check for fatal error.
  fatalError = fatalError || (deviceError.num > NOHEARTBEAT) ? true : false;
  
  // Check heartbeat
  checkHeartbeat();
  
  // Read and update sensor values.
  updateSensorData();

  // Handle errors.
  handleErrors();

  // Update control
  if (~fatalError) updateControlLaw();

  // Send data out.
  writeSensorStream();

  // Update packet number and runtime.
  packetNumber.num += 1;
  currMilliCount = millis();
  runTime.num = currMilliCount - startTime;
  delTime = (currMilliCount - prevMilliCount) / 1000.0;
  prevMilliCount = currMilliCount;

  // Read handle USB Serial commands.
  handleSerialUSBCommands();
}
