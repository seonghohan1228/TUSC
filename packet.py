import struct


def calculate_checksum(data):
    return sum(data) % 256


class Packet:
    # Define start and end bytes
    START_BYTE = 0x02
    END_BYTE = 0x03
    LENGTH = 7  # bytes
    PAYLOAD_STRUCTURE = ">Bhh"
    PACKET_STRUCTURE = ">B5sBB"

    def __init__(self, serial_connection):
        self.serial_connection = serial_connection
    
    def create(self, mode, speed_L, speed_R):
        self.mode = mode
        self.speed_L = speed_L
        self.speed_R = speed_R

    def send(self):
        # Convert speeds to 2-byte signed integers
        self.payload = struct.pack(self.PAYLOAD_STRUCTURE, self.mode, self.speed_L, self.speed_R)
        self.checksum = calculate_checksum(self.payload)
        self.packet = struct.pack(self.PACKET_STRUCTURE, self.START_BYTE, self.payload, self.checksum, self.END_BYTE)
        self.serial_connection.write(self.packet)
    
    def print(self):
        print(f"Sended: {self.mode}\t{self.speed_L}\t{self.speed_R}")

