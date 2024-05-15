import struct


def calculate_checksum(data):
    return sum(data) % 256


class Packet:
    # Define start and end bytes
    START_BYTE = 0x02
    END_BYTE = 0x03
    LENGTH = 10  # bytes
    PAYLOAD_STRUCTURE = ">BBhh"
    PACKET_STRUCTURE = ">B7sBB"


class OutgoingPacket(Packet):
    def __init__(self, serial_connection):
        self.serial_connection = serial_connection
    
    def create(self, mode, gear, speed_L, speed_R):
        self.mode = mode
        self.gear = gear
        self.speed_L = speed_L
        self.speed_R = speed_R

    def send(self):
        # Convert speeds to 2-byte signed integers
        self.payload = struct.pack(self.PAYLOAD_STRUCTURE, self.mode, self.gear, self.speed_L, self.speed_R)
        self.checksum = calculate_checksum(self.payload)
        self.packet = struct.pack(self.PACKET_STRUCTURE, self.START_BYTE, self.payload, self.checksum, self.END_BYTE)
        self.serial_connection.write(self.packet)
    
    def print(self):
        print(f"Sended: {self.mode}\t{self.gear}\t{self.speed_L}\t{self.speed_R}")


class IncomingPacket(Packet):
    def __init__(self, serial_connection):
        self.serial_connection = serial_connection
    
    def receive(self):
        self.bytes = self.serial_connection.read_until(bytes([self.END_BYTE]))

    def is_valid(self):
        if len(self.bytes) != 10:
            return False
        self.start_byte, self.mode, self.gear, self.speed_L, self.speed_R, self.checksum, self.end_byte = struct.unpack(self.PACKET_STRUCTURE, self.bytes)
        
        # Check start and end byte
        if self.start_byte != self.START_BYTE or self.end_byte != self.END_BYTE:
            return False
        
        # Check payload
        payload = self.bytes[1:8]
        if calculate_checksum(payload) != self.checksum:
            return False
        
        return True

    def print(self):
        print(f"Received: {self.mode}\t{self.gear}\t{self.speed_L}\t{self.speed_R}")


