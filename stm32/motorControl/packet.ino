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


bool packetIsValid(uint8_t *packet)
{
  // Verify start and end byte
  if (packet[0] != START_BYTE || packet[PACKET_SIZE - 1] != END_BYTE)
    return false;

  // Verify checksum
  uint8_t payload[] = {packet[1], packet[2], packet[3], packet[4], packet[5]};
  uint8_t checksum = calculateChecksum(payload, 5);
  if (packet[PACKET_SIZE - 2] != checksum)
    return false;

  // Packet is valid
  return true;
}

