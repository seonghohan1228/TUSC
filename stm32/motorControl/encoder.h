// encoder.h

#ifndef ENCODER_H
#define ENCODER_H

// AS5600 I2C address
uint8_t AS5600_ADDR = 0x36;
uint8_t ANGLE_ADDR = 0x0E;
uint8_t ZPOSH_ADDR = 0x01;
uint8_t ZPOSL_ADDR = 0x02;
uint8_t STATUS_ADDR = 0x0B;

const char COM_FAIL = -1;
const char COM_SUCCESS = 1;


float readDegreeAngle(TwoWire &wire);
bool setZeroPosition(TwoWire &wire, unsigned int zeroPosition);
uint8_t readStatus(TwoWire &wire);


#endif