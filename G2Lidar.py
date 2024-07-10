import serial
import struct
import time
import math
import msvcrt

LIDAR_CMD_SYNC_BYTE = 0xA5
LIDAR_CMD_STOP = 0x65
LIDAR_CMD_SCAN = 0x60
LIDAR_SCAN_FREQ = 0x0D
LIDAR_SCAN_FREQ_RED_01 = 0x0A
LIDAR_SCAN_FREQ_INC_01 = 0x09

LIDAR_ANS_TYPE_DEVINFO = 0x4
LIDAR_ANS_TYPE_DEVHEALTH = 0x6
LIDAR_ANS_TYPE_GETFREQ = 0x4
LIDAR_ANS_SYNC_BYTE1 = 0xA5
LIDAR_ANS_SYNC_BYTE2 = 0x5A
LIDAR_ANS_TYPE_MEASUREMENT = 0x81

LIDAR_CMD_GET_DEVICE_HEALTH = 0x92
LIDAR_CMD_GET_DEVICE_INFO = 0x90

PackageSampleBytes = 2
PackageSampleMaxLngth = 0x80
Node_Default_Quality = (10 << 2)
Node_Sync = 1
Node_NotSync = 2
PackagePaidBytes = 10
PH = 0x55AA

LIDAR_RESP_MEASUREMENT_SYNCBIT = (0x1 << 0)
LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT = 2
LIDAR_RESP_MEASUREMENT_CHECKBIT = (0x1 << 0)
LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT = 1
LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT = 8

class G2Lidar:
    SERIAL_BAUDRATE = 230400
    DEFAULT_TIMEOUT = 500

    def __init__(self):
        self._serial = None
        self.distanceList = [0] * 360
        self.lightList = [0] * 360
        self.temp = [0] * 360

    def begin(self, port):
        self._serial = serial.Serial(port, self.SERIAL_BAUDRATE, timeout=1)
        return self._serial.is_open

    def isOpen(self):
        return self._serial.is_open if self._serial else False

    def sendCommand(self, cmd, payload=None):
        header = struct.pack('BB', LIDAR_CMD_SYNC_BYTE, cmd & 0xff)
        #print(header.hex())
        self._serial.write(header)
        if payload:
            self._serial.write(payload)
        return True

    def waitResponseHeader(self, timeout=DEFAULT_TIMEOUT):
        start_time = time.time()
        header = bytearray(7)
        recv_pos = 0

        while (time.time() - start_time) < timeout / 1000.0:
            if self._serial.in_waiting > 0:
                current_byte = self._serial.read(1)
                if recv_pos == 0 and current_byte != bytes([LIDAR_ANS_SYNC_BYTE1]):
                    continue
                if recv_pos == 1 and current_byte != bytes([LIDAR_ANS_SYNC_BYTE2]):
                    recv_pos = 0
                    continue
                header[recv_pos] = current_byte[0]
                recv_pos += 1
                if recv_pos == len(header):
                    return header
        return None

    def getHealth(self, timeout=DEFAULT_TIMEOUT):
        self.sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH)
        header = self.waitResponseHeader(timeout)
        if not header or header[6] != LIDAR_ANS_TYPE_DEVHEALTH:
            return None
        health = self._serial.read(3)
        return health #struct.unpack('BHB', health[:5])

    def getDeviceInfo(self, timeout=DEFAULT_TIMEOUT):
        self.sendCommand(LIDAR_CMD_GET_DEVICE_INFO)
        header = self.waitResponseHeader(timeout)
        if not header or header[6] != LIDAR_ANS_TYPE_DEVINFO:
            return None
        info = self._serial.read(20)
        return info #struct.unpack('BHB16s', info)

    def stop(self):
        return self.sendCommand(LIDAR_CMD_STOP)

    def getScanFreq(self, timeout=DEFAULT_TIMEOUT):
        self.sendCommand(LIDAR_SCAN_FREQ)
        header = self.waitResponseHeader(timeout)
        if not header or header[6] != LIDAR_ANS_TYPE_GETFREQ:
            return None
        freq = self._serial.read(4)
        return freq #struct.unpack('I', freq)[0]

    def increaseFreq(self, timeout=DEFAULT_TIMEOUT):
        self.sendCommand(LIDAR_SCAN_FREQ_INC_01)
        header = self.waitResponseHeader(timeout)
        if not header or header[6] != LIDAR_ANS_TYPE_GETFREQ:
            return None
        freq = self._serial.read(4)
        return freq #struct.unpack('I', freq)[0]

    def decreaseFreq(self, timeout=DEFAULT_TIMEOUT):
        self.sendCommand(LIDAR_SCAN_FREQ_RED_01)
        header = self.waitResponseHeader(timeout)
        if not header or header[6] != LIDAR_ANS_TYPE_GETFREQ:
            return None
        freq = self._serial.read(4)
        return struct.unpack('I', freq)[0]

    def startScan(self, timeout=DEFAULT_TIMEOUT * 2):
        #self.stop()
        self.sendCommand(LIDAR_CMD_SCAN)
        header = self.waitResponseHeader(timeout)
        if not header or header[6] != LIDAR_ANS_TYPE_MEASUREMENT:
            return None
        return True

    def waitScanDot(self, timeout=DEFAULT_TIMEOUT):
        start_time = time.time()
        scan_response = bytearray(200)
        recv_pos = 0
        while (time.time() - start_time) < timeout / 1000.0:
            if self._serial.in_waiting > 0:
                current_byte = self._serial.read(1)
                scan_response[recv_pos] = current_byte[0]
                recv_pos += 1
                if recv_pos == len(scan_response):
                    break
        return scan_response
    
    def waitScanData(self, timeout=DEFAULT_TIMEOUT):
        start_time = time.time()
        scan_response = bytearray()
        recv_pos = 0

        while (time.time() - start_time) < timeout / 1000.0:
            if self._serial.in_waiting > 0:
                data = self._serial.read(600).split(b"\xaa\x55")
                #data_dict = {i: b"\xaa\x55" + chunk for i, chunk in enumerate(data) if chunk}
                data_dict = {i: PH.to_bytes(2, 'little') + chunk for i, chunk in enumerate(data) if chunk}

        if len(data_dict) > 0:
            #print(f"max data: {len(data_dict)}, max index: {max(data_dict.keys())}")
            return data_dict
        else:
            #print("No data received")
            return None

    def scanData(self, timeout=DEFAULT_TIMEOUT):
        start_time = time.time()
        read_data = bytearray()

        while (time.time() - start_time) < timeout / 1000.0:
            if self._serial.in_waiting > 0:
                temp = self._serial.read(1)
                if temp == b"\xaa" and len(read_data) == 0:
                    read_data += temp
                    continue
                if temp == b"\x55" and len(read_data) == 1:
                    if read_data[-1] == b"\xaa":
                        read_data += temp
                        continue
                    else:
                        read_data += temp
                        continue
                if len(read_data) == 3:
                    if temp == 0:
                        read_data = bytearray()
                        continue
                    read_data += temp
                    read_data += self._serial.read(int(temp[0]) * 3 + 6)
                    break
                else:
                    if len(read_data) != 0:
                        read_data += temp
                    continue
                
        return read_data
    
    def process_lidar_sizeCheck(self, data):
        if(len(data) < 10):
            #print(f"data length is insufficient: {len(data)}")
            return False
        dataSize = data[3]
        if len(data) == dataSize * 3 + 10:
            return True
        else:
            #print(f"data length is incorrect: {len(data)} {dataSize * 3 + 10}")
            return False
            
    def process_lidar_sumCheck(self, data):
        if(len(data) < 10):
            #print(f"data length is insufficient: {len(data)}")
            return False
        dataSize = data[3]
        #if len(data) < dataSize * 3 + 10:
        if len(data) != dataSize * 3 + 10:
            #print(f"data length is incorrect: {len(data)} {dataSize * 3 + 10}")
            return False
        
        cs = (data[9] << 8) | data[8]
        #checksum = 0x55AA
        checkSum = PH
        for i in range(0, dataSize * 3, 3):
            checkSum ^= data[10 + i]
            checkSum ^= ((data[10 + i + 2] << 8) | data[10 + i + 1])
        #checkSum ^= (data[3] << 8 | (data[2] & 0x01))
        checkSum ^= (data[3] << 8 | data[2])
        checkSum ^= (data[5] << 8 | data[4])
        checkSum ^= (data[7] << 8 | data[6])
        if checkSum == cs:
            return True
        else:
            #print(f"data sum is incorrect: {checkSum} {cs} {dataSize}")
            return False

    # flag = True: return angle, distance
    # flag = False: return x, y
    def process_lidar_data(self, data, flag=False):
        CT = data[2] & 0x01

        LSN = data[3]
        FSA = (data[5] << 8 | data[4])
        LSA = (data[7] << 8 | data[6])
        row = 0
        Distance = [0] * LSN
        Intensity = [0] * LSN

        for i in range(0, 3 * LSN, 3):
            distance_data = (data[10 + i + 2] << 8 | data[10 + i + 1])
            Distance[i // 3] = distance_data >> 2

        Angle = [0] * LSN
        Angle_FSA = (FSA >> 1) / 64
        Angle_LSA = (LSA >> 1) / 64
        Angle_Diff = (Angle_LSA - Angle_FSA)

        X_value = [0] * LSN
        Y_value = [0] * LSN
        if Angle_Diff < 0:
            Angle_Diff += 360
        for i in range(LSN):
            try:
                Angle[i] = i * Angle_Diff / (LSN - 1) + Angle_FSA
                if Distance[i] > 0:
                    AngCorrect = math.atan(21.8 * (155.3 - Distance[i]) / (155.3 * Distance[i]))
                    if flag:
                        Angle[i] += AngCorrect * 180 / math.pi
                        if Angle[i] >= 360:
                            Angle[i] -= 360
                    else:
                        X_value[i] = Distance[i] * math.cos(math.radians(Angle[i]))
                        Y_value[i] = Distance[i] * math.sin(math.radians(Angle[i]))
                        
            except Exception as e:
                #print(f"Error: {e} {i}")
                continue
        if flag:
            #angle = [math.radians(a) for a in Angle]
            angle_distance_pairs = [(a, d) for a, d in zip(Angle, Distance) if d > 0]
            angle, Distance = zip(*angle_distance_pairs) if angle_distance_pairs else ([], [])
            return angle, Distance        
        else:
            return X_value, Y_value



# Usage example
if __name__ == "__main__":

    lidar = G2Lidar()
    if lidar.begin('COM5'):
        print("Lidar connected")
        print("Start Scan")
        lidar.startScan()

        _ = lidar.waitScanData()
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8')
                if key.lower() == 'q':
                    lidar.stop()
                    print("Exiting...")
                    break
            scan_data = lidar.scanData()
            if scan_data is not None:
                if lidar.process_lidar_sumCheck(scan_data):
                    angle, distance = lidar.process_lidar_data(scan_data,True)
                    if angle is not None and distance is not None:
                        print(f"angle: {angle}, distance: {distance}")

