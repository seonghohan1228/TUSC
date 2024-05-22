// packet.ino

#include "packet.h"

uint8_t calculateChecksum(uint8_t *data, uint8_t length)
{
  uint8_t checksum = 0;
  for (int i = 0; i < length; i++)
  {
    checksum += data[i];
  }
  return checksum % 256;
}


void arrayToPacket(uint8_t *arr, struct Packet *packet)
{
  packet->startByte = arr[0];
  packet->payload->mode = arr[1];
  packet->payload->gear = arr[2];
  packet->payload->speed_L = (arr[3] << 8) | arr[4];  // Higher byte | Lower byte
  packet->payload->speed_R = (arr[5] << 8) | arr[6];
  packet->checksum = arr[7];
  packet->endByte = arr[8];
}


bool packetIsValid(struct Packet *packet)
{
  // Verify start and end byte
  if (packet->startByte != START_BYTE || packet->endByte != END_BYTE)
    return false;

  // Verify checksum
  uint8_t checksum = calculateChecksum((uint8_t *)packet->payload, sizeof(struct Payload));
  if (packet->checksum != checksum)
    return false;

  // Packet is valid
  return true;
}

