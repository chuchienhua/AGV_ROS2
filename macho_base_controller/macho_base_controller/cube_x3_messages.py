import struct
def calculateCRC16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder='little') 

class CubeX3FrameHeader:
    def __init__(self, opcode, length):
        self.sof = 0x23
        self.id = 0x01
        self.opcode = opcode
        self.length = length
        self.control_mode = 0x02
        self.checksum = 0
    
    def to_bytes(self):
        header_bytes =  bytes([self.sof, self.id, self.opcode, self.length, self.control_mode])
        checksum_bytes = calculateCRC16(header_bytes)
        return header_bytes + checksum_bytes

class CubeX3ExtAckermannCtrlMsg:
    def __init__(self, velocity, radius, heartbeat):
        self.header = CubeX3FrameHeader(opcode = 0x24, length=0x05)
        self.velocity = velocity
        self.radius = radius
        self.heartbeat = heartbeat
        self.checksum = 0
    
    def to_bytes(self):
        header_bytes =  bytes([self.header.sof,self.header.id, self.header.opcode, self.header.length, *struct.pack('>hh', self.velocity, self.radius), self.heartbeat])
        checksum_bytes = calculateCRC16(header_bytes)
        return header_bytes + checksum_bytes

