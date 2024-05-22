// packet.h

#ifndef PACKET_H
#define PACKET_H

// Packet info
const byte START_BYTE = 0x02;
const byte END_BYTE = 0x03;
const int PACKET_SIZE = 9;

struct Payload
{
  uint8_t mode;
  uint8_t gear;
  int16_t speed_L;
  int16_t speed_R;
};

struct Packet
{
  uint8_t startByte;
  Payload *payload;
  uint8_t checksum;
  uint8_t endByte;
};

uint8_t calculateChecksum(uint8_t *data, uint8_t length);
struct Packet *arrayToPacket(uint8_t *arr);
bool packetIsValid(struct Packet *packet);

#endif