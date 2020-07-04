/*------------------------------------------------------------------------------

  GarminLidarliteV4 Arduino Library


  Use this sketch to configure I2C addresses.

  *** NOTE ***
  The LIDAR-Lite v4 LED is strictly a 3.3V system. The Arduino Due is a
  3.3V system and is recommended for use with the LIDAR-Lite v4 LED.
  Care MUST be taken if connecting to a 5V system such as the Arduino Uno.
  See comment block in the setup() function for details on I2C connections.
  It is recommended to use a voltage level-shifter if connecting the GPIO
  pins to any 5V system I/O.

  Connections:
  LIDAR-Lite 5 VDC   (pin 1) to Arduino 5V
  LIDAR-Lite Ground  (pin 2) to Arduino GND
  LIDAR-Lite I2C SDA (pin 3) to Arduino SDA
  LIDAR-Lite I2C SCL (pin 4) to Arduino SCL

  Optional connections to utilize GPIO triggering:
  LIDAR-Lite GPIOA   (pin 5) to Arduino Digital 2
  LIDAR-Lite GPIOB   (pin 6) to Arduino Digital 3

  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5V
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information

  ------------------------------------------------------------------------------*/
#include <Wire.h>
#include "GarminLidarliteV4.h"

#define SDA_PIN 4
#define SCL_PIN 5

GarminLidarliteV4 lidar4;

uint8_t i2c_lidarAddress = 0x0; // reference address you want change or add a seconday
uint8_t i2c_newAddress = 0x0;  // new address
uint8_t i2c_addressResponseControl = 2;

bool success = false;


void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN);

  digitalWrite(SDA_PIN, LOW);
  digitalWrite(SCL_PIN, LOW);

  Serial.println("--------------------------------------");
  Serial.println("LIDARLite_v4 Configure I2C addresses");
  Serial.println("Change or create a secondary address.");
  Serial.println("--------------------------------------");
  scanner();

  Serial.println("[1]. Please enter the address you want to configure in hexadecimal");
  Serial.println("!!!without the 0x prefix. e.g 5B !!!");
  while (Serial.available() == 0);
  if (Serial.available()) {

    String stringBuffer = Serial.readString();

    if (stringBuffer.length() != 4  ) {
      Serial.println("You muss enter a valid existing address (two digits). System restart...");
      delay(10000);
      ESP.restart();

    }

    uint8_t address = 0;
    char charBuffer[20];
    stringBuffer.toCharArray(charBuffer, 10);
    uint8_t success = std::sscanf(charBuffer, "%x", &address);

    i2c_lidarAddress = address;
    Serial.print("0x");
    Serial.print(i2c_lidarAddress, HEX);


    Wire.beginTransmission(i2c_lidarAddress);
    if (Wire.endTransmission() == 0) {
      Serial.print(" ok");
      Serial.println("");
    }
    else {
      Serial.println(" ERROR: Address could not be found. System restart...");
      delay(10000);
      ESP.restart();
    }

  }
  Serial.println("");
  Serial.println("[2]. Please enter a none existing  NEW adress in hexadecimal from 0x08 to 0x77");
  Serial.println("!!!without the 0x prefix. e.g 5B !!!");
  while (Serial.available() == 0);
  if (Serial.available()) {

    String stringBuffer = Serial.readString();

    if (stringBuffer.length() != 4  ) {
      Serial.println("You muss enter a valid address (two digits). System restart...");
      delay(10000);
      ESP.restart();

    }

    uint8_t address = 0;
    char charBuffer[20];
    stringBuffer.toCharArray(charBuffer, 10);
    uint8_t success = std::sscanf(charBuffer, "%x", &address);
    i2c_newAddress = address;
    Serial.print("0x");
    Serial.print(i2c_newAddress, HEX);


    Wire.beginTransmission(i2c_newAddress);
    if (Wire.endTransmission() != 0) {
      Serial.print(" ok");
      Serial.println("");
    }
    else {
      Serial.println(" ERROR: Address already exists on device. System restart...");
      delay(10000);
      ESP.restart();
    }

  }
  Serial.println("");
  Serial.println("[3]. Default address response control:");
  Serial.println("0 = Use the default address only (0x62)");
  Serial.println("1 = Use the secondary address only");
  Serial.println("2 = Use both addresses");
  Serial.println("");
  Serial.println("Please enter an option 0, 1 or 2");
  while (Serial.available() == 0);
  if (Serial.available()) {

    String stringBuffer = Serial.readString();

    uint8_t option = 0;
    char charBuffer[20];
    stringBuffer.toCharArray(charBuffer, 10);
    uint8_t success = std::sscanf(charBuffer, "%d", &option);

    if (option < 0 &&  option > 2) {
      Serial.println(" ERROR: invalid option ( enter 0, 1 or 2 ). System restart...");
      delay(10000);
      ESP.restart();
    }

    i2c_addressResponseControl = option;
    Serial.println("");
    Serial.print(option);
    Serial.println(" ok");
  }


  Serial.println("");
  Serial.println("Press ENTER to confirm changing I2C address:");
  Serial.println("System will restart after completion.");

  while (Serial.available() == 0);
  if (Serial.available()) {
    uint8_t newAddress = 0;
    String stringBuffer = Serial.readString();
  }

  // Change I2C address config;
  success = lidar4.setI2Caddr(i2c_newAddress, i2c_addressResponseControl, i2c_lidarAddress );

  if (success) {
    Serial.println("--------------------------------");
    Serial.println("Address successfull changed!");
    Serial.println("--------------------------------");

  }
  else
  {
    Serial.println("--------------------------------");
    Serial.println("Error detected. evetualy the address could not be set!");
    Serial.println("--------------------------------");

  }

  scanner();

  Serial.println("Please power off your device");
  Serial.println("wait 10 secs and power on the device again, to verify address persistence!");

}

void scanner()
{

  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  return; delay(5000);
}

void loop() {


  //if (!success) return;
  //scanner();

}
