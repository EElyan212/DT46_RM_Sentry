import serial
import struct
import time
from .modules.crc import *



def send_mock_data():
    # 连接到虚拟串口的一端
    port = "/tmp/ttyV0" 
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"模拟发送器已启动，连接至: {port}")
    except Exception as e:
        print(f"无法打开串口: {e}")
        return

    header = 0xA5          # 假设帧头是 0x5A (B)
    detect_color = 1       # 红色 (B)
    roll = 1.2             # (f)
    pitch = -0.5           # (f)
    yaw = 3.14             # (f)
    vx = 0.5               # (f)
    vy = 0.2               # (f)
    sentry_hp = 600        # (H)
    hero_hp = 1500         # (H)
    infantry_hp = 200      # (H)
    remain_time = 420      # (H)
    remain_bullet = 150    # (H)
    match_progress = 2     # (B)
    occupation = 0       
    while True:
        try:
            # --- 构造数据负载 (34 字节) ---
              # (B)

            # 打包成 39 字节的二进制流
            # 格式: <BBffffffHHHHHBB
            data_payload = struct.pack(
                "<BBfffffHHHHHBB",
                header, detect_color,
                roll, pitch, yaw,
                vx, vy, 
                sentry_hp, hero_hp, infantry_hp,
                remain_time, remain_bullet,
                match_progress, occupation
            )

            # --- 计算并附加 CRC (2 字节) ---
            crc_val = get_crc16_check_sum(data_payload)
            crc_bytes = struct.pack("<H", crc_val)

            # --- 发送完整包 (41 字节) ---
            full_packet = data_payload + crc_bytes
            ser.write(full_packet)
            print(full_packet)

            # if count % 10 == 0:
            #     print(f"已发送第 {count} 个数据包，校验和: {hex(crc_val)}")
            
            if yaw >= 100:
                yaw =0
            if vx >= 100:
                vx =0
            if vy >= 100:
                vy =0
            yaw += 0.1             # (f)
            vx += 0.1               # (f)
            vy += 0.1               # (f)
            
            # time.sleep(0.02)  # 50Hz 发送频率

        except KeyboardInterrupt:
            ser.close()
            break

def main():
    send_mock_data()

if __name__ == "__main__":
    main()