# 串口接收帧调试消息（仅测试用）
# 字段与下位机 53 字节固定长帧偏移一一对应
std_msgs/Header header

uint8 sof                 # 0  帧头，应为 0xA5
uint8 detect_color        # 1  0=红, 1=蓝
uint8 reserved            # 2  协议保留，应为 0
float32 roll              # 3  IMU 横滚 (度)
float32 pitch             # 7  IMU 俯仰 (度)
float32 yaw               # 11 IMU 偏航 (绝对角, 度)
float32 vx                # 15 底盘 x 指令速度 (云台系)
float32 vy                # 19 底盘 y 指令速度
uint16 self_sentry_hp     # 23 己方哨兵血量
uint16 self_hero_hp       # 25 己方英雄血量
uint16 self_infantry_hp   # 27 己方步兵血量
uint16 remain_time        # 29 剩余时间 (s)
uint16 remain_bullet      # 31 剩余允许发弹量
uint8 match_progress      # 33 比赛进度
uint8 occupation          # 34 占领状态
float32 bullet_speed      # 35 弹速 (m/s)
uint8 armor_id            # 39 最近一次受击装甲板 ID (0~15)
uint8 armor_reason        # 40 扣血原因 (0=弹丸命中,1=撞击,4=超射速...)
uint8 armor_hit_cnt       # 41 累计受击次数 (8 位回绕)
uint8 armor_hit_seq       # 42 受击事件序号，每次新受击 +1
float32 armor_angle_chassis  # 43 受击方向, 底盘系, 逆时针为正
float32 armor_angle_world    # 47 受击方向场地方位 (0~360)
uint16 crc_received       # 51 报文携带的 CRC
uint16 crc_calculated     # 本地计算的 CRC
bool crc_ok               # 两者是否一致
