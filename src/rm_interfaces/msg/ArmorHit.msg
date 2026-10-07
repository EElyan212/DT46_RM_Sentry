# 受击反馈事件消息（serial_node 发布，仅在受击瞬间发布一次）
std_msgs/Header header

uint8 armor_id                # 受击装甲板 ID (0~15)
uint8 armor_reason            # 扣血原因 (0=弹丸命中, 1=撞击, 4=超射速...)
uint8 armor_hit_cnt           # 累计受击次数 (8 位回绕)
uint8 armor_hit_seq           # 受击事件序号，每次新受击 +1
float32 armor_angle_chassis   # 受击方向, 底盘系, 逆时针为正
float32 armor_angle_world     # 受击方向场地方位 (0~360)
