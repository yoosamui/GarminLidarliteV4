# GarminLidarliteV4 Arduino Library

* [Product Page: LIDAR-Lite v4 LED](https://buy.garmin.com/en-US/US/p/610275) - See product page for operating manual
* [Product Support](https://support.garmin.com) - Please direct any support issues to Garmin's Support Team. They're smart, helpful, and super nice!

This library is based on the LIDAR-Lite Arduino Library. It provides quick access to basic functions of LIDAR-Lite v4
via the Arduino interface. Additionally, it can provide a user of any
platform with a template for their own application code.

For detailed specifications, pinout, and connection diagrams, see the manuals linked at the above product pages.
LIDAR-Lite Arduino Library (https://github.com/garmin/LIDARLite_Arduino_Library)

***A Note on Compatibility:*** *This library supports only the LIDAR-Lite v4. Support is not provided.

## Installation instructions
To install, download this repository and place in your Arduino libraries folder or use the Arduino Library Manager. If you need help, follow the instructions here: [http://arduino.cc/en/Guide/Libraries](http://arduino.cc/en/Guide/Libraries).

## Example Sketches
### change_address
Use this sketch to configure I2C addresses.

### fast
This example shows how to take multiple distance measurements with a LIDAR-Lite v4 LED 
with a high rep rate using  the I2C port to trigger measurements.

### read_continous_distance
This example shows methods for running the LIDAR-Lite v4 LED and read distance continuous.

### read_continous_distance_two_lidars
This example shows methods for running two LIDAR-Lite v4 LED and read distance continuous.

### temperature
This example shows how to read the temerature from LIDAR-Lite v4 LED sensor.

### version
This example shows how to read the temerature from LIDAR-Lite v4 LED sensor.

## Version History

* [1.0.0](https://github.com/yoosamui/GarminLidarliteV4) - Initial release

## Raspberry Pi Library for LIDAR-Lite v3
* [LIDAR-Lite v3 on Pi](https://github.com/garmin/LIDARLite_RaspberryPi_Library/) - Basic support available for Raspberry Pi 3B+

## ANT Library for LIDAR-Lite v4 LED
* [LIDAR-Lite_ANT_Library](https://github.com/garmin/LIDARLite_ANT_Library/) - Developer library for the nRF52840 powered LIDAR-Lite v4 LED

## License
Copyright (c) 2018 Garmin Ltd. or its subsidiaries. Distributed under the Apache 2.0 License.
Maintainer Juan R. Gonzalez
See [LICENSE](LICENSE) for further details.
