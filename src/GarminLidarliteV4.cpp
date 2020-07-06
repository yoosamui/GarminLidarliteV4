/*------------------------------------------------------------------------------

  GarminLidarliteV4 arduino Library based on GarminLidarliteV4 Arduino Library
  GarminLidarliteV4.cpp

  This library provides quick access to all the basic functions of LIDAR-Lite
  via the Arduino interface. Additionally, it can provide a user of any
  platform with a template for their own application code.

  Copyright (c) 2019 Garmin Ltd. or its subsidiaries.
  Modify by Juan R. González

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

------------------------------------------------------------------------------*/

// clang-format off

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include <stdint.h>
#include "GarminLidarliteV4.h"

// clang-format on

//#define DEBUGME

void GarminLidarliteV4::notifyError(byte reg, uint8_t mode)
{
#ifdef DEBUGME
    if (!Serial.available()) return;

    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    if (mode) {
        Serial.print("ERROR: write in to register 0x");

    } else {
        Serial.print("ERROR: read from register 0x");
    }

    Serial.println(reg, HEX);
#endif
}

void GarminLidarliteV4::notifyWriteError(byte reg)
{
    notifyError(reg, 1);
}

void GarminLidarliteV4::notifyReadError(byte reg)
{
    notifyError(reg, 0);
}

void GarminLidarliteV4::printVersion(uint8_t address)
{
    char version[512];
    uint8_t dataBytes[12];
    uint8_t *nrfVerString;
    uint16_t *lrfVersion;
    uint8_t *hwVersion;
    uint8_t i;

    read(0x30, dataBytes, 11, 0x62);
    nrfVerString = dataBytes;

    nrfVerString = dataBytes;
    Serial.print("nRF Software Version  - ");
    for (i = 0; i < 11; i++) {
        Serial.write(nrfVerString[i]);
    }
    Serial.println("");

    //===========================================
    // Print LRF Firmware Version
    //===========================================
    read(0x72, dataBytes, 2, 0x62);
    lrfVersion = (uint16_t *)dataBytes;
    Serial.print("LRF Firmware Version  - v");
    Serial.print((*lrfVersion) / 100);
    Serial.print(".");
    Serial.print((*lrfVersion) % 100);
    Serial.println("");

    //===========================================
    // Print Hardware Version
    //===========================================
    read(0xE1, dataBytes, 1, 0x62);
    hwVersion = dataBytes;
    Serial.print("Hardware Version      - ");
    switch (*hwVersion) {
        case 16:
            Serial.println("RevA");
            break;
        default:
            Serial.println("????");
            break;
    }
}

// ----------------------------------------------------------------------
// ** One method of increasing the measurement speed of the
//    LIDAR-Lite v4 LED is to adjust the number of acquisitions taken
//    per measurement from the default of 20. For max speed you
//    can set  this value to 0x00.
// ** Note that when reducing the number of acquisitions taken per
//    measurement from the default, repeatability of measurements is
//    reduced.
// ----------------------------------------------------------------------
bool GarminLidarliteV4::setAccuracyMode(uint8_t value, uint8_t address)
{
    uint8_t dataByte = value;
    return write(0xEB, &dataByte, 1, address);
}

int GarminLidarliteV4::readTemperature(uint8_t address)
{
    uint8_t dataByte;
    read(BOARD_TEMPERATURE, &dataByte, 1, address);
    return dataByte;
}

/*------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Maximum range. Uses maximum acquisition count.
    1: Balanced performance.
    2: Short range, high speed. Reduces maximum acquisition count.
    3: Mid range, higher speed. Turns on quick termination
         detection for faster measurements at short range (with decreased
         accuracy)
    4: Maximum range, higher speed on short range targets. Turns on quick
         termination detection for faster measurements at short range (with
         decreased accuracy)
    5: Very short range, higher speed, high error. Reduces maximum
         acquisition count to a minimum for faster rep rates on very
         close targets with high error.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void GarminLidarliteV4::configure(uint8_t configuration, uint8_t lidarliteAddress)
{
    uint8_t sigCountMax;
    uint8_t acqConfigReg;

    switch (configuration) {
        case 0:  // Default mode - Maximum range
            sigCountMax = 0xff;
            acqConfigReg = 0x08;
            break;

        case 1:  // Balanced performance
            sigCountMax = 0x80;
            acqConfigReg = 0x08;
            break;

        case 2:  // Short range, high speed
            sigCountMax = 0x18;
            acqConfigReg = 0x00;
            break;

        case 3:  // Mid range, higher speed on short range targets
            sigCountMax = 0x80;
            acqConfigReg = 0x00;
            break;

        case 4:  // Maximum range, higher speed on short range targets
            sigCountMax = 0xff;
            acqConfigReg = 0x00;
            break;

        case 5:  // Very short range, higher speed, high error
            sigCountMax = 0x04;
            acqConfigReg = 0x00;
            break;
    }

    if (!write(ACQUISITION_COUNT, &sigCountMax, 1, lidarliteAddress))
        notifyWriteError(ACQUISITION_COUNT);

    if (!write(QUICK_TERMINATION, &acqConfigReg, 1, lidarliteAddress))
        notifyWriteError(QUICK_TERMINATION);

} /* GarminLidarliteV4::configure */

//---------------------------------------------------------------------
// Read Continuous Distance Measurements
//
// The most recent distance measurement can always be read from
// device registers. Polling for the BUSY flag in the STATUS
// register can alert the user that the distance measurement is new
// and that the next measurement can be initiated. If the device is
// BUSY this function does nothing and returns 0. If the device is
// NOT BUSY this function triggers the next measurement, reads the
// distance data from the previous measurement, and returns 1.
//---------------------------------------------------------------------
uint8_t GarminLidarliteV4::distanceContinuous(uint16_t *distance, uint8_t address)
{
    uint8_t newDistance = 0;
    //    Serial.println(address, HEX);
    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
    if (getBusyFlag(address) == 0) {
        // Trigger the next range measurement
        takeRange(address);

        // Read new distance data from device registers
        *distance = readDistance(address);

        // Report to calling function that we have new data
        newDistance = 1;
    }

    return newDistance;
}

//---------------------------------------------------------------------
// Read Single Distance Measurement
//
// This is the simplest form of taking a measurement. This is a
// blocking function as it will not return until a range has been
// taken and a new distance measurement can be read.
//---------------------------------------------------------------------
uint8_t GarminLidarliteV4::distanceSingle(uint16_t *distance, uint8_t lidarliteAddress)
{
    // 1. Trigger range measurement.
    takeRange(lidarliteAddress);

    // 2. Wait for busyFlag to indicate device is idle.
    waitForBusy(lidarliteAddress);

    // 3. Read new distance data from device registers
    *distance = readDistance(lidarliteAddress);

    return 1;
}

/*------------------------------------------------------------------------------
Enable or sidabe Flash storage


After change the I2C address the adress will persist in the device.

parameters

  ------------------------------------------------------------------------------
  enable: enable or disable the flash storage

------------------------------------------------------------------------------*/
void GarminLidarliteV4::enableFlash(bool enable, uint8_t lidarliteAddress)
{
    uint8_t dataByte = enable ? 0x11 : 0x00;

    if (!write(ENABLE_FLASH_STORAGE, &dataByte, 1, lidarliteAddress))
        notifyWriteError(ENABLE_FLASH_STORAGE);

    // inportand! Give LIDAR time to write the register
    delay(100);
}

// ----------------------------------------------------------------------
// ** To utilize the low power capabilities of the LIDAR-Lite v4 LED,
//    turn off high accuracy mode and then switch on asynchronous mode.
// ** Note that when defeating high accuracy mode, repeatability
//    of measurements is reduced.
// ** Note that while in asynchronous mode, minimum time between
//    measurements is 100 milliseconds.
// ----------------------------------------------------------------------
void GarminLidarliteV4::enableLowpower(bool enable, uint8_t lidarliteAddress)
{
    uint8_t dataByte = 0x00;
    write(0xEB, &dataByte, 1, lidarliteAddress);  // Turn off high accuracy mode

    if (enable) {
        write(0xE2, &dataByte, 1, lidarliteAddress);  // Turn on asynchronous mode
    } else {
        dataByte = 0x01;
        write(0xE2, &dataByte, 1, lidarliteAddress);  // Turn on synchronous mode

        dataByte = 0x14;
        write(0xEB, &dataByte, 1, lidarliteAddress);  // Turn on high accuracy mode
    }
}

bool GarminLidarliteV4::useBothAddresses()
{
    uint8_t dataByte = 0x02;
    if (!write(I2C_CONFIG, &dataByte, 1)) {
        notifyWriteError(I2C_CONFIG);
        return false;
    }

    return true;
}
/*------------------------------------------------------------------------------
  Set I2C Address

  Set Alternate I2C Device Address. See Operation Manual for additional info.

  Parameters
  ------------------------------------------------------------------------------
  newAddress: desired secondary I2C device address
  disableDefault: a non-zero value here means the default 0x62 I2C device
    address will be disabled.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
bool GarminLidarliteV4::setI2Caddr(uint8_t newAddress, uint8_t addressResponseControl,
                                   uint8_t lidarliteAddress)
{
    // Check if address is within range
    if (newAddress < 0x08 || newAddress > 0x77) {
        return false;
    }

    // Check if address is within range
    if (lidarliteAddress < 0x08 || lidarliteAddress > 0x77) {
        return false;
    }
    // Check if addressResponseControl is within range
    if (addressResponseControl > 2) {
        return false;
    }

    uint8_t dataBytes[5] = {0};  // all elements 0

    if (newAddress == LIDARLITE_ADDR_DEFAULT) {
        dataBytes[0] = 0x00;  // set bit to enable default address
        if (!write(I2C_CONFIG, dataBytes, 1, newAddress)) {
            notifyWriteError(I2C_CONFIG);
            return false;
        }
        return true;
    }

    // Read 4-byte device serial number
    if (!read(UNIT_ID_0, dataBytes, 4, lidarliteAddress)) {
        notifyReadError(UNIT_ID_0);
        return false;
    }

    // Append the desired I2C address to the end of the serial number byte array
    dataBytes[4] = newAddress;

    // Before we configure (change) the I2C address, first we enable the flash storage
    enableFlash(true);

    // Write the serial number and new address in one 5-byte transaction
    if (!write(UNIT_ID_0, dataBytes, 5, lidarliteAddress)) {
        enableFlash(false);
        notifyWriteError(UNIT_ID_0);
        return false;
    }
    // Give LIDAR the time to record new address to eeprom
    delay(100);

    // Disable flash storage
    enableFlash(false);

    dataBytes[0] = addressResponseControl;
    if (!write(I2C_CONFIG, dataBytes, 1, newAddress)) {
        notifyWriteError(I2C_CONFIG);
        return false;
    }

    return true;

} /* GarminLidarliteV4::setI2Caddr */

/*------------------------------------------------------------------------------
  Take Range

  Initiate a distance measurement by writing to register 0x00.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void GarminLidarliteV4::takeRange(uint8_t lidarliteAddress)
{
    uint8_t dataByte = 0x04;

    write(0x00, &dataByte, 1, lidarliteAddress);
} /* GarminLidarliteV4::takeRange */

/*------------------------------------------------------------------------------
  Wait for Busy Flag

  Blocking function to wait until the Lidar Lite's internal busy flag goes low

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void GarminLidarliteV4::waitForBusy(uint8_t lidarliteAddress)
{
    uint8_t busyFlag;

    do {
        busyFlag = getBusyFlag(lidarliteAddress);
    } while (busyFlag);

} /* GarminLidarliteV4::waitForBusy */

/*------------------------------------------------------------------------------
  Get Busy Flag

  Read BUSY flag from device registers. Function will return 0x00 if not busy.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
uint8_t GarminLidarliteV4::getBusyFlag(uint8_t lidarliteAddress)
{
    uint8_t statusByte = 0;
    uint8_t busyFlag;  // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    read(0x01, &statusByte, 1, lidarliteAddress);

    // STATUS bit 0 is busyFlag
    busyFlag = statusByte & 0x01;

    return busyFlag;
} /* GarminLidarliteV4::getBusyFlag */

/*------------------------------------------------------------------------------
  Take Range using Trigger / Monitor Pins

  Initiate a distance measurement by toggling the trigger pin

  Parameters
  ------------------------------------------------------------------------------
  triggerPin: digital output pin connected to trigger input of LIDAR-Lite
  monitorPin: digital input pin connected to monitor output of LIDAR-Lite
------------------------------------------------------------------------------*/
void GarminLidarliteV4::takeRangeGpio(uint8_t triggerPin, uint8_t monitorPin)
{
    uint8_t busyFlag;

    if (digitalRead(triggerPin))
        digitalWrite(triggerPin, LOW);
    else
        digitalWrite(triggerPin, HIGH);

    // When LLv4 receives trigger command it will drive monitor pin low.
    // Wait for LLv4 to acknowledge receipt of command before moving on.
    do {
        busyFlag = getBusyFlagGpio(monitorPin);
    } while (!busyFlag);
} /* GarminLidarliteV4::takeRangeGpio */

/*------------------------------------------------------------------------------
  Wait for Busy Flag using Trigger / Monitor Pins

  Blocking function to wait until the Lidar Lite's internal busy flag goes low

  Parameters
  ------------------------------------------------------------------------------
  monitorPin: digital input pin connected to monitor output of LIDAR-Lite
------------------------------------------------------------------------------*/
void GarminLidarliteV4::waitForBusyGpio(uint8_t monitorPin)
{
    uint8_t busyFlag;

    do {
        busyFlag = getBusyFlagGpio(monitorPin);
    } while (busyFlag);

} /* GarminLidarliteV4::waitForBusyGpio */

/*------------------------------------------------------------------------------
  Get Busy Flag using Trigger / Monitor Pins

  Check BUSY status via Monitor pin. Function will return 0x00 if not busy.

  Parameters
  ------------------------------------------------------------------------------
  monitorPin: digital input pin connected to monitor output of LIDAR-Lite
------------------------------------------------------------------------------*/
uint8_t GarminLidarliteV4::getBusyFlagGpio(uint8_t monitorPin)
{
    uint8_t busyFlag;  // busyFlag monitors when the device is done with a measurement

    // Check busy flag via monitor pin
    if (digitalRead(monitorPin))
        busyFlag = 1;
    else
        busyFlag = 0;

    return busyFlag;
} /* GarminLidarliteV4::getBusyFlagGpio */

/*------------------------------------------------------------------------------
  Read Distance

  Read and return the result of the most recent distance measurement.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
uint16_t GarminLidarliteV4::readDistance(uint8_t lidarliteAddress)
{
    uint16_t distance;
    uint8_t *dataBytes = (uint8_t *)&distance;

    // Read two bytes from registers 0x10 and 0x11
    read(0x10, dataBytes, 2, lidarliteAddress);

    return (distance);
} /* GarminLidarliteV4::readDistance */

/*------------------------------------------------------------------------------
  Write

  Perform I2C write to device. The I2C peripheral in the LidarLite v4
  will receive multiple bytes in one I2C transmission. The first byte is
  always the register address. The the bytes that follow will be written
  into the specified register address first and then the internal address
  in the Lidar Lite will be auto-incremented for all following bytes.

  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to write to
  dataBytes: pointer to array of bytes to write
  numBytes:  number of bytes in 'dataBytes' array to write
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
bool GarminLidarliteV4::write(uint8_t regAddr, uint8_t *dataBytes, uint8_t numBytes,
                              uint8_t lidarliteAddress)
{
    int nackCatcher;

    Wire.beginTransmission((int)lidarliteAddress);

    // First byte of every write sets the LidarLite's internal register address pointer
    Wire.write((int)regAddr);

    // Subsequent bytes are data writes
    Wire.write(dataBytes, (int)numBytes);

    // A nack means the device is not responding. Report the error over serial.
    nackCatcher = Wire.endTransmission();
    if (nackCatcher != 0) {
        // handle nack issues in here
        /*
            0:success
            1:data too long to fit in transmit buffer
            2:received NACK on transmit of address
            3:received NACK on transmit of data
            4:other error
        */

        return false;
    }

    return true;

} /* GarminLidarliteV4::write */

/*------------------------------------------------------------------------------
  Read

  Perform I2C read from device.  The I2C peripheral in the LidarLite v3 HP
  will send multiple bytes in one I2C transmission. The register address must
  be set up by a previous I2C write. The bytes that follow will be read
  from the specified register address first and then the internal address
  pointer in the Lidar Lite will be auto-incremented for following bytes.

  Will detect an unresponsive device and report the error over serial.

  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to write to
  dataBytes: pointer to array of bytes to write
  numBytes:  number of bytes in 'dataBytes' array to write
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
bool GarminLidarliteV4::read(uint8_t regAddr, uint8_t *dataBytes, uint8_t numBytes,
                             uint8_t lidarliteAddress)
{
    uint16_t i = 0;
    int nackCatcher = 0;

    // Set the internal register address pointer in the Lidar Lite
    Wire.beginTransmission((int)lidarliteAddress);
    Wire.write((int)regAddr);  // Set the register to be read

    // A nack means the device is not responding, report the error over serial
    nackCatcher = Wire.endTransmission(false);  // false means perform repeated start
    if (nackCatcher != 0) {
        // handle nack issues in here
        return false;
    }

    // Perform read, save in dataBytes array
    Wire.requestFrom((int)lidarliteAddress, (int)numBytes);
    if ((int)numBytes <= Wire.available()) {
        while (i < numBytes) {
            dataBytes[i] = (uint8_t)Wire.read();
            i++;
        }
    }
    return true;

} /* GarminLidarliteV4::read */

/*------------------------------------------------------------------------------
  Correlation Record Read

  The correlation record used to calculate distance can be read from the device.
  It has a bipolar wave shape, transitioning from a positive going portion to a
  roughly symmetrical negative going pulse. The point where the signal crosses
  zero represents the effective delay for the reference and return signals.

  Process
  ------------------------------------------------------------------------------
  1.  Take a distance reading (there is no correlation record without at least
      one distance reading being taken)
  2.  For as many points as you want to read from the record (max is 192) read
      the two byte signed correlation data point from 0x52

  Parameters
  ------------------------------------------------------------------------------
  correlationArray: pointer to memory location to store the correlation record
                    ** Two bytes for every correlation value must be
                       allocated by calling function
  numberOfReadings: Default = 192. Maximum = 192
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void GarminLidarliteV4::correlationRecordRead(int16_t *correlationArray, uint8_t numberOfReadings,
                                              uint8_t lidarliteAddress)
{
    uint8_t i;
    int16_t correlationValue;
    uint8_t *dataBytes = (uint8_t *)&correlationValue;

    for (i = 0; i < numberOfReadings; i++) {
        read(0x52, dataBytes, 2, lidarliteAddress);
        correlationArray[i] = correlationValue;
    }
} /* GarminLidarliteV4::correlationRecordRead */

