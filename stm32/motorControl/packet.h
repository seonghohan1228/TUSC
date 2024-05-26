// packet.h

#ifndef PACKET_H
#define PACKET_H

// Packet info
const byte START_BYTE = 0x02;
const byte END_BYTE = 0x03;
const int PACKET_SIZE = 9;

const uint8_t OFF = 0;
const uint8_t ON = 1;
const uint8_t DISABLED = 0;
const uint8_t ENABLED = 1;

uint8_t calculateChecksum(uint8_t *data, uint8_t length);
bool packetIsValid(uint8_t *packet);

#endif

