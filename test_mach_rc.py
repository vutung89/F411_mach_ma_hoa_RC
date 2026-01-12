"""
    @file test_rc_control.py
    @brief Thử nghiệm gửi lệnh RC qua cổng serial bất đồng bộ.
    @author VuTung
    @date 2025-07-29
    @version 2.0
    @details:
    - Mục đích của tập tin này là để kiểm tra việc gửi 
    lệnh RC và nhận phản hồi từ bộ điều khiển RC thông qua cổng serial.
    - Sử dụng thư viện serial_asyncio để thực hiện các thao tác bất đồng 
    bộ với cổng serial.
    @usage:
    - Chạy tập tin này để kết nối tới cổng RC, gửi lệnh và nhận dữ liệu RC channels.
    - Có thể điều chỉnh các giá trị RC (pitch, roll, throttle, yaw) trong 
    hàm main để kiểm tra các lệnh khác nhau
    - Có thể thay đổi TIME_DELAY để điều chỉnh thời gian chờ giữa các lệnh.
    - Nhấn Ctrl+C để dừng chương trình.
    @Mode RC_control:
    - Tay điều khiển SIYI MK15
        1. Chế độ Auto RC_control pitch, roll, throttle, yaw
            - Đồng thời: Gạt Switch bên phải sang nấc dưới cùng + nhấn Button A sáng đèn
        2. Chế độ Manual RC_control pitch, roll, throttle, yaw
            - Một trong hai, hoặc cả hai: Gạt Switch bên phải sang nấc giữa/ trên cùng / nhấn Button A tắt đèn
        3. Chế độ Ngắt gửi Sbus_out vào Px4
            - Nhấn Button B sáng đèn
        4. Chế độ gửi Sbus_out vào Px4
            - Nhấn Button B tắt đèn 
    @note:
    - Đảm bảo rằng cổng serial được kết nối đúng và thiết bị đã sẵn sàng nhận lệnh.
    - Lưu ý rằng cổng serial và baudrate cần được cấu hình đúng với thiết bị của bạn
    - Thời gian chờ giữa các lệnh có thể điều chỉnh thông qua biến TIME_DELAY.
    - Nếu gặp lỗi kết nối, hãy kiểm tra lại cổng serial và thiết bị.
    - Đảm bảo rằng thư viện crc đã được cài đặt, mã crc của máy tính nhúng và mạch giống nhau để tính toán CRC16.
"""
# -----------------------------------------------------------------------------------------------------
import asyncio
import serial_asyncio
import logging
from datetime import datetime

# ------------------------------------------------------------------------------------------------------
# LOGGING CONFIGURATION
# ------------------------------------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,  # Đặt mức ghi nhật ký tối thiểu là INFO
    # level=logging.DEBUG,  # Đặt mức ghi nhật ký tối thiểu là INFO
    # format='%(asctime)s - %(levelname)s - %(message)s',
    format='[%(levelname)s]:  %(message)s',
    
    handlers=[
        logging.StreamHandler(),  # Ghi nhật ký ra console
        # logging.FileHandler(f'csv/uav_log_{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.log') # Bỏ comment để ghi ra file
    ]
)
# ------------------------------------------------------------------------------------------------------
# PARAMETERS
# ------------------------------------------------------------------------------------------------------
# PowerShell: [System.IO.Ports.SerialPort]::GetPortNames()
MACH_GHI_DE_RC_PORT = 'COM11'
# MACH_GHI_DE_RC_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
TIME_DELAY = 1  # Thời gian chờ giữa các lệnh
# RC Control Commmand
RC_PITCH = 0.3      # Giá trị RC pitch mặc định (-1 đến 1)
RC_ROLL = -0.25     # Giá trị RC roll mặc định (-1 đến 1)
RC_YAW = 0.15       # Giá trị RC yaw mặc định (-1 đến 1)
RC_THROTTLE = 0.5   # Giá trị RC throttle mặc định (0 đến 1)

#------------------------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------------------------
# CRC16 Calculation
# ------------------------------------------------------------------------------------------------------
# Lookup tables for the high and low byte of the CRC
aucCRCHi = [
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
]

aucCRCLo = [
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
]

def calculate_crc16(data):
    ucCRCHi = 0xFF
    ucCRCLo = 0xFF
    
    for byte in data:
        iIndex = ucCRCLo ^ byte
        ucCRCLo = ucCRCHi ^ aucCRCHi[iIndex]
        ucCRCHi = aucCRCLo[iIndex]
    
    return (ucCRCHi << 8) | ucCRCLo
# ------------------------------------------------------------------------------------------------------
# MACH GHI DE RC
# ------------------------------------------------------------------------------------------------------
class AsyncSerialRC:
    """
    Một lớp để quản lý kết nối và gửi lệnh RC qua cổng serial bất đồng bộ.
    """
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.reader = None
        self.writer = None
        self.p = bytearray(10)

    async def connect(self):
        """Mở kết nối serial bất đồng bộ."""
        try:
            self.reader, self.writer = await serial_asyncio.open_serial_connection(
                url=self.port, baudrate=self.baudrate)
            logging.info(f"Đã kết nối thành công tới cổng RC: {self.port}")
            return True
        except Exception as e:
            logging.warning(f"Lỗi khi kết nối cổng RC {self.port}: {e}")
            return False

    async def close(self):
        """Đóng kết nối serial."""
        if self.writer:
            self.writer.close()
            await asyncio.sleep(0.5)
            logging.info("Đã đóng kết nối cổng RC.")

    async def request_rc_channels(self):
        """Chuẩn bị và gửi lệnh RC một cách bất đồng bộ."""
        if not self.writer:
            logging.warning("Lỗi: Chưa kết nối tới cổng RC để gửi lệnh.")
            return
        
        self.p[0] = 0xAA
        self.p[1] = 0xAF
        self.p[2] = 0x00
        self.p[3] = 0x00
        self.p[4] = 0x00
        self.p[5] = 0x00
        self.p[6] = 0x00
        self.p[7] = 0x00
        
        CRCCal = calculate_crc16(self.p[:8])
        self.p[8] = CRCCal & 0xFF
        self.p[9] = (CRCCal >> 8) & 0xFF
        
        try:
            # Gửi dữ liệu đi
            self.writer.write(self.p)
            await self.writer.drain()  # Chờ cho đến khi buffer được gửi hết
            
            # Đọc 40 byte từ UART với timeout dài hơn (100ms)
            data = await asyncio.wait_for(self.reader.readexactly(40), timeout=0.1)
            if data[0] == 0xAA and data[1] == 0xAF:
                ch_buf = data[2:38]  # 36 byte channel
                crc_recv = data[38] | (data[39] << 8)
                crc_calc = calculate_crc16(ch_buf)
                if crc_recv == crc_calc:
                    # Giải mã 18 channel (low-high byte)
                    rc_channels = [ch_buf[i] | (ch_buf[i+1] << 8) for i in range(0, 36, 2)]
                    logging.info("Nhận dữ liệu RC channels thành công.")
                    return rc_channels
                else:
                    logging.warning(f"CRC không khớp. Tính toán: 0x{crc_calc:04X}, Nhận: 0x{crc_recv:04X}")
            else:
                logging.warning(f"Frame không hợp lệ. Header: 0x{data[0]:02X} 0x{data[1]:02X}")
        except asyncio.TimeoutError:
            logging.info("Gửi request RC thất bại: Không nhận được phản hồi (timeout).")
        except Exception as e:
            logging.warning(f"Lỗi trong quá trình gửi/nhận lệnh RC: {e}")

    async def send_commands_async(self, pitch, roll, throttle, yaw):
        """Chuẩn bị và gửi lệnh RC một cách bất đồng bộ."""
        if not self.writer:
            logging.info("Lỗi: Chưa kết nối tới cổng RC để gửi lệnh.")
            return

        normalized_pitch = (pitch + 1) / 2
        normalized_roll = (roll + 1) / 2
        normalized_yaw = (yaw + 1) / 2
        scaled_pitch = int(normalized_pitch * 255)
        scaled_roll = int(normalized_roll * 255)
        scaled_throttle = int(throttle * 255)
        scaled_yaw = int(normalized_yaw * 255)
        
        self.p[0] = 0xAA
        self.p[1] = 0x55
        self.p[2] = scaled_roll
        self.p[3] = scaled_pitch
        self.p[4] = scaled_throttle
        self.p[5] = scaled_yaw
        self.p[6] = 0x7F
        self.p[7] = 2
        
        CRCCal = calculate_crc16(self.p[:8])
        self.p[8] = CRCCal & 0xFF
        self.p[9] = (CRCCal >> 8) & 0xFF
        
        try:
            # Gửi dữ liệu đi
            self.writer.write(self.p)
            await self.writer.drain()  # Chờ cho đến khi buffer được gửi hết
            
            # Đọc lại để xác nhận (với timeout)
            message = await asyncio.wait_for(self.reader.readexactly(10), timeout=0.1)
            
            if message == self.p:
                logging.info("Gửi lệnh RC thành công.")
            else:
                logging.warning("Gửi lệnh RC thất bại: Dữ liệu nhận lại không khớp.")
                logging.warning(f"Sent: {list(self.p)}")
                logging.warning(f"Received: {list(message)}")
                
        except asyncio.TimeoutError:
            logging.info("Gửi lệnh RC thất bại: Không nhận được phản hồi (timeout).")
        except Exception as e:
            logging.warning(f"Lỗi trong quá trình gửi/nhận lệnh RC: {e}")
# ------------------------------------------------------------------------------------------------------
async def connect_rc_controller(port, baudrate):
    """
    Tạo và kết nối bộ điều khiển RC qua serial.
    Trả về đối tượng AsyncSerialRC nếu thành công, ngược lại trả về None.
    """
    logging.info("Đang kết nối tới cổng RC...")
    rc_controller = AsyncSerialRC(port, baudrate)
    is_connected = await rc_controller.connect()
    
    if is_connected:
        return rc_controller
    else:
        return None
    
async def main():
    rc_controller = await connect_rc_controller(MACH_GHI_DE_RC_PORT, BAUDRATE)
    logging.info("Kết nối tới cổng RC thành công." if rc_controller else "Kết nối tới cổng RC thất bại.")
    # Nếu kết nối thành công, bắt đầu gửi lệnh RC
    if rc_controller:
        try:
            while True:
                #--------------------------------------------------------------------------------------
                #-------------------------------test RC value------------------------------------
                #--------------------------------------------------------------------------------------
                rc_pitch = RC_PITCH
                rc_roll = RC_ROLL
                rc_throttle = RC_THROTTLE
                rc_yaw = RC_YAW
                #--------------------------------------------------------------------------------------

                logging.info(f"RC command: P-R-Y-T: {rc_pitch}, {rc_roll}, {rc_yaw}, {rc_throttle}")
                await rc_controller.send_commands_async(rc_pitch, rc_roll, rc_throttle, rc_yaw)
                # # doc rc_channels
                # rc_channels = await rc_controller.request_rc_channels()
                # if rc_channels:
                #     logging.info("18 channel: %s", rc_channels)
                # else:
                #     logging.info("Không nhận được dữ liệu RC channels.")
                logging.info("--------------------------------------------------------------------\n")
                await asyncio.sleep(TIME_DELAY)  # Chờ trước khi gửi lệnh tiếp theo
        finally:
            await rc_controller.close()
            logging.info("Close RC_Port")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("\nĐã nhận tín hiệu Ctrl+C. kết thúc chương trình.")