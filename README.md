# ULP I2C BMP280 application

Implementation of ESP-32 ULP I2C application which reads a sensor (BMP-280) over I2C and wakes up the main
processor after a significant change of the measured values.

Todo :
* Fix pressure sensitivity

## I2C bit banged support

Note that this example uses a bit-banged I2C implementation, because the hardware ULP I2C support cannot read 16 bit values.

## Credits

Modified from https://github.com/tomtor/ulp-i2c
