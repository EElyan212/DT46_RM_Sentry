# 决策数据
std_msgs/Header header
uint16 self_sentry_hp     # 我方哨兵血量
uint16 self_heavy_hp      # 我方重装血量
uint16 self_infantry_hp   # 我方步兵血量
uint16 remain_time        # 剩余时间 300s
uint16 remain_bullet      # 剩余子弹
uint8 match_progress      # 比赛进程 （准备为1,开始为4）
uint8 occupation          # 中心增益点占领状态 (字节34 bit0-1): 0未占领 1己方 2对方 3双方
bool center_rfid          # 本车是否检测到中心增益点 RFID 卡 (字节34 bit2)
uint8 color               # 颜色
float32 yaw               # 角度
int32 can_fire
float32 bullet_speed