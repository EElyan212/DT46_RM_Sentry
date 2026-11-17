std_msgs/Header header

bool enabled                # 过洞检测是否启用；矩形参数非法或配置关闭时为 false
bool path_valid             
bool path_intersects        # 机器人前方全局路径是否经过洞矩形
bool approaching            # 机器人沿路径到入口的距离是否小于等于 pre_stop_distance
bool inside                 # 是否在隧洞内
float32 distance_to_entry   # 机器人沿路径到入口的弧长，单位米；无效时为 -1.0
float32 distance_since_exit # 机器人离开隧道后沿路径累计移动距离，单位米；无效时为 -1.0
int8 entry_y_sign           # 入洞方向在 y 上的符号：+1 沿 +y，-1 沿 -y，0 未知
