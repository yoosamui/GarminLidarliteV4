/*------------------------------------------------------------------------------

  GarminLidarliteV4 arduino Library based on LIDARLite_v4LED Arduino Library
  GarminLidarliteV4.h

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
#ifndef GARMINLIDARLITEV4_h
#define GARMINLIDARLITEV4_h

#define LIDARLITE_ADDR_DEFAULT 0x62

#include <Arduino.h>
#include <stdint.h>

class GarminLidarliteV4
{
  public:
    void configure(uint8_t configuration = 0, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    bool setI2Caddr(uint8_t newAddress, uint8_t addressResponseControl,
                    uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    uint16_t readDistance(uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    void waitForBusy(uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    uint8_t getBusyFlag(uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    void takeRange(uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    void waitForBusyGpio(uint8_t monitorPin);
    uint8_t getBusyFlagGpio(uint8_t monitorPin);
    void takeRangeGpio(uint8_t triggerPin, uint8_t monitorPin);

    bool write(uint8_t regAddr, uint8_t* dataBytes, uint8_t numBytes,
               uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    bool read(uint8_t regAddr, uint8_t* dataBytes, uint8_t numBytes,
              uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    void correlationRecordRead(int16_t* correlationArray, uint8_t numberOfReadings = 192,
                               uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    bool useBothAddresses();
    uint8_t distanceContinuous(uint16_t* distance,
                               uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    uint8_t distanceSingle(uint16_t* distance, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    int readTemperature(uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    void printVersion(uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    bool setAccuracyMode(uint8_t value, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    void enableLowpower(bool enable, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

  private:
    enum {
        ACQ_COMMANDS = 0x00,
        STATUS = 0x01,
        ACQUISITION_COUNT = 0x05,
        FULL_DELAY_LOW = 0x10,
        FULL_DELAY_HIGH = 0x11,
        UNIT_ID_0 = 0x16,
        UNIT_ID_1 = 0x17,
        UNIT_ID_2 = 0x18,
        UNIT_ID_3 = 0x19,
        I2C_SEC_ADDR = 0x1A,
        I2C_CONFIG = 0x1B,
        DETECTION_SENSITIVITY = 0x1C,
        LIB_VERSION = 0x30,
        CORR_DATA = 0x52,
        CP_VER_LO = 0x72,
        BOARD_TEMPERATURE = 0xE0,
        HARDWARE_VERSION = 0xE1,
        POWER_MODE = 0xE2,
        MEASUREMENT_INTERVAL = 0xE3,
        FACTORY_RESET = 0xE4,
        QUICK_TERMINATION = 0xE5,
        START_BOOTLOADER = 0xE6,
        ENABLE_FLASH_STORAGE = 0xEA,
        HIGH_ACCURACY_MODE = 0xEB,
        SOC_TEMPERATURE = 0xEC,
    };

    void enableFlash(bool enable, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
    void notifyError(byte reg, uint8_t mode);
    void notifyReadError(byte reg);
    void notifyWriteError(byte reg);
};

#endif
