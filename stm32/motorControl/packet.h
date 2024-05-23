// packet.h

#ifndef PACKET_H
#define PACKET_H

// Packet info
const byte START_BYTE = 0x02;
const byte END_BYTE = 0x03;
const int PACKET_SIZE = 9;


uint8_t calculateChecksum(uint8_t *data, uint8_t length);
bool packetIsValid(uint8_t *packet);

#endif

