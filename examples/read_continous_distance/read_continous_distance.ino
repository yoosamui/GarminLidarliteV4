/*------------------------------------------------------------------------------

  GarminLidarliteV4 Arduino Library

  This example shows methods for running the LIDAR-Lite v4 LED and read distance continuous.

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

#define FAST_I2C
#define LIDAR_ADDR LIDARLITE_ADDR_DEFAULT //(0x62)
GarminLidarliteV4 lidar4;

void setup()
{
  // Initialize Arduino serial port (for display of ASCII output to PC)
  Serial.begin(115200);
  delay(500);

  // Initialize Arduino I2C (for communication to LidarLite)
  Wire.begin();
#ifdef FAST_I2C
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
#endif

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
  lidar4.configure(0, LIDAR_ADDR);
}


void loop()
{
  uint16_t distance;
  uint8_t  newDistance;

  newDistance = lidar4.distanceContinuous(&distance,LIDAR_ADDR);
  if (newDistance) {
    Serial.print("distance ");
    Serial.print(distance);
    Serial.println(" cm.");
  }

  delay(1);
}



