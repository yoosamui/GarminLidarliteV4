/*------------------------------------------------------------------------------

  GarminLidarliteV4 Arduino Library

  This example shows how to read the temerature from LIDAR-Lite v4 LED sensor.

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

#include <stdint.h>
#include <Wire.h>
#include "GarminLidarliteV4.h"

GarminLidarliteV4 lidar4;

void setup()
{
  // Initialize Arduino serial port (for display of ASCII output to PC)
  Serial.begin(115200);
  delay(500);

  // Initialize Arduino I2C (for communication to LidarLite)
  Wire.begin();

  // ----------------------------------------------------------------------
  // The LIDAR-Lite v4 LED is strictly a 3.3V system. The Arduino Due is a
  // 3.3V system and is recommended for use with the LIDAR-Lite v4 LED.
  // Care MUST be taken if connecting to a 5V system such as the Arduino Uno.
  //
  // I2C is a two wire communications bus that requires a pull-up resistor
  // on each signal. In the Arduino microcontrollers the Wire.begin()
  // function (called above) turns on pull-up resistors that pull the I2C
  // signals up to the system voltage rail. The Arduino Uno is a 5V system
  // and using the Uno's internal pull-ups risks damage to the LLv4.
  //
  // The two digitalWrite() functions (below) turn off the micro's internal
  // pull-up resistors. This protects the LLv4 from damage via overvoltage
  // but requires external pullups to 3.3V for the I2C signals.
  //
  // External pull-ups are NOT present on the Arduino Uno and must be added
  // manually to the I2C signals. 3.3V is available on pin 2 of the 6pin
  // "POWER" connector and can be used for this purpose. See the Uno
  // schematic for details:
  // https://www.arduino.cc/en/uploads/Main/arduino-uno-schematic.pdf
  //
  // External 1.5k ohm pull-ups to 3.3V are already present on the
  // Arduino Due. If using the Due no further action is required
  // ----------------------------------------------------------------------
  digitalWrite(SCL, LOW);
  digitalWrite(SDA, LOW);

  // ----------------------------------------------------------------------
  // Optional GPIO pin assignments for measurement triggering & monitoring
  // ----------------------------------------------------------------------
  pinMode(SCL, OUTPUT);
  pinMode(SDA, OUTPUT);

  // ----------------------------------------------------------------------
  // Optionally configure the LidarLite parameters to lend itself to
  // various modes of operation by altering 'configure' input integer.
  // See GarminLidarlineV4.cpp for details.
  // ----------------------------------------------------------------------
  lidar4.configure(0);
}

void loop()
{
  Serial.print("Board celsius temperature: ");
  Serial.println(lidar4.readTemperature());
  delay(1000);

}
