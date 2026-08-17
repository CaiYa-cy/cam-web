"""串口通信模块 - 基于帧头长度帧尾 + CRC16 校验的串口收发（MaixCAM2版本）"""
from maix import uart
import time

# 协议常量
FRAME_HEAD = 0x7E
FRAME_TAIL = 0x7F
MAX_PAYLOAD_LEN = 65535  # LEN 2 字节（大端），负载上限 64KB 以内
RECEIVE_TIMEOUT_MS = 2000
READ_CHUNK_SIZE = 64


def _crc16_modbus(data):
    """CRC-16/MODBUS：多项式 0x8005（反向 0xA001），初值 0xFFFF。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class CamComm:
    def __init__(self, device="/dev/ttyS1", baudrate=115200,
                 timeout_ms=RECEIVE_TIMEOUT_MS):
        self.serial = uart.UART(device, baudrate)
        self.timeout_ms = timeout_ms
        self._reset_state()

    def _reset_state(self):
        self.state = 0  # 0=等帧头,1=等长度高字节,2=等长度低字节,3=收数据,4=收CRC高字节,5=收CRC低字节,6=等帧尾
        self.length = 0
        self.data = bytearray()
        self.count = 0
        self.crc_high = 0
        self.crc_low = 0
        self.crc_ok = False
        self._frame_queue = []

    def _send(self, data):
        if isinstance(data, str):
            data = data.encode("utf-8")
        if len(data) > MAX_PAYLOAD_LEN:
            print("警告: 数据长度 %d 超过最大负载 %d，将被截断" % (len(data), MAX_PAYLOAD_LEN))
            data = data[:MAX_PAYLOAD_LEN]
        crc = _crc16_modbus(data)
        packet = bytes([FRAME_HEAD, (len(data) >> 8) & 0xFF, len(data) & 0xFF]) + data
        packet += bytes([(crc >> 8) & 0xFF, crc & 0xFF, FRAME_TAIL])
        try:
            written = self.serial.write(packet)
            return written > 0
        except Exception as e:
            print("串口发送失败: %s" % e)
            return False

    def _receive(self, timeout=None):
        if timeout is None:
            timeout = self.timeout_ms

        if self._frame_queue:
            return self._frame_queue.pop(0)

        try:
            rx_data = self.serial.read(timeout=timeout)
        except Exception as e:
            print("串口读取异常: %s" % e)
            self._reset_state()
            return None

        if not rx_data:
            if self.state != 0:
                self._reset_state()
            return None

        for byte in rx_data:
            if self.state == 0:
                if byte == FRAME_HEAD:
                    self.state = 1
                    self.length = 0
                    self.data = bytearray()
                    self.count = 0
            elif self.state == 1:
                self.length = byte
                self.state = 2
            elif self.state == 2:
                self.length = (self.length << 8) | byte
                if self.length > 0:
                    self.state = 3
                else:
                    self.state = 4
            elif self.state == 3:
                self.data.append(byte)
                self.count += 1
                if self.count >= self.length:
                    self.state = 4
            elif self.state == 4:
                self.crc_high = byte
                self.state = 5
            elif self.state == 5:
                self.crc_low = byte
                rx_crc = (self.crc_high << 8) | self.crc_low
                self.crc_ok = (_crc16_modbus(bytes(self.data)) == rx_crc)
                self.state = 6
            elif self.state == 6:
                if byte == FRAME_TAIL:
                    self.state = 0
                    if not self.crc_ok:
                        print("CRC校验失败，数据可能损坏")
                        self.data = bytearray()
                    else:
                        try:
                            frame = self.data.decode("utf-8")
                            self._frame_queue.append(frame)
                        except UnicodeDecodeError:
                            print("解码失败，数据可能损坏")
                            self.data = bytearray()
                else:
                    print("帧尾错误: 期望 0x7F, 收到 0x%02X" % byte)
                    self.state = 0
                    self.length = 0
                    self.data = bytearray()
                    self.count = 0

        if self._frame_queue:
            return self._frame_queue.pop(0)
        return None

    def send_track(self, value):
        return self._send("T:0x%02X" % value)

    def send_number(self, value):
        return self._send("N:%d" % value)

    def send_string(self, text):
        return self._send(text)

    def reset(self):
        self._reset_state()

    def receive(self, timeout=None):
        return self._receive(timeout)

    def put_back(self, cmd):
        """Push a decoded command back to the head of the receive queue."""
        self._frame_queue.insert(0, cmd)
