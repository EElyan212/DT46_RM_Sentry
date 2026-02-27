import time
import serial
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rm_interfaces.msg import Decision, Senddata, GimbalControl
from std_msgs.msg import Int32
import struct
from geometry_msgs.msg import Vector3Stamped, Twist
from .modules.crc import *
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ColorPrint():
    def __init__(self):
        self.PINK = "\033[38;5;218m"
        self.CYAN = "\033[96m"
        self.GREEN = "\033[32m"
        self.RED = "\033[31m"
        self.BLUE = "\033[34m"
        self.RESET = "\033[0m"

class SerialNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("启动Serial node !!!")

        self.send_datas = Senddata()
        self.lock = threading.Lock()
        # 新增：串口重连专用的锁和标志位
        self.serial_lock = threading.Lock()
        self.is_reconnecting = False
        #创建qos
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )
        #获取参数
        self.get_params()
        #创建决策消息发布者
        self.pub_uart_receive_decision = self.create_publisher(Decision, "/nav/decision", 10)
        #串建导航（底盘）消息接收者
        self.sub_uart_cmd = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        #创建自瞄（云台）消息订阅者
        self.sub_gimbal_control = self.create_subscription(GimbalControl, "/gimbal/control", self.gimbal_control_callback, 10)
        #创建小陀螺模式接收者
        self.sub_gimbal_mode = self.create_subscription(Int32, "/gimbal_mode", self.gimbal_mode_callback, 10)
        # 创建发布者 2: IMU 数据
        self.pub_uart_receive_imu = self.create_publisher(Vector3Stamped, '/imu/rpy', qos)
        # 创建变量
        self.tracking_color = 10

        self.color = ColorPrint()
        #初始化串口
        try:
            self.serial = serial.Serial(
                port = self.port_name,
                baudrate = self.baudrate,
                timeout = self.timeout,
                write_timeout = self.write_timeout,
                )
            if self.serial.is_open:
                self.get_logger().info(f"串口已打开: {self.port_name}")
                self.receive_thread = threading.Thread(target=self.receive_data,daemon=True)  
                # self.serial.setDaemon(True) 
                self.receive_thread.start()
                self.timer = self.create_timer(0.01, self.Send)
        except serial.SerialException as e:
            self.get_logger().error(f"创建串口时出错: {self.port_name} - {str(e)}")
            raise e    
            
    def get_params(self):
        """
        从 ROS 2 参数服务器声明并获取串口配置参数
        """
        # 1. 批量声明参数及其默认值
        # 这里的默认值仅在 YAML 文件未指定或命令行未提供参数时生效
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port_name', '/dev/ttyUSB0'),
                ('baudrate', 115200),
                ('timeout', 1.0),            # 对应 YAML 中的 timeout
                ('write_timeout', 1.0),      # 对应 YAML 中的 write_timeout
                ('flow_control', 'none'),
                ('parity', 'none'),
                ('stop_bits', '1')
            ]
        )

        # 2. 获取参数值并赋值给实例变量
        self.port_name = self.get_parameter("port_name").value
        self.baudrate = self.get_parameter("baudrate").value
        self.timeout = self.get_parameter("timeout").value
        self.write_timeout = self.get_parameter("write_timeout").value
        self.flow_control = self.get_parameter("flow_control").value
        self.parity = self.get_parameter("parity").value
        self.stop_bits = self.get_parameter("stop_bits").value

        # 3. 打印当前生效的配置信息，方便在终端确认 YAML 是否加载成功
        self.get_logger().info("-" * 30)
        self.get_logger().info("串口参数配置已加载:")
        self.get_logger().info(f"  端口号: {self.port_name}")
        self.get_logger().info(f"  波特率: {self.baudrate}")
        self.get_logger().info(f"  超时设置: Read={self.timeout}s, Write={self.write_timeout}s")
        self.get_logger().info(f"  校验/流控: Parity={self.parity}, Flow={self.flow_control}")
        self.get_logger().info("-" * 30)
    def cmd_vel_callback(self,msg):
        with self.lock:
            self.send_datas.linear_velocity_x = msg.linear.x
            self.send_datas.linear_velocity_y = msg.linear.y
            self.send_datas.angular_velocity_z = msg.angular.z
    def gimbal_mode_callback(self,msg):
        with self.lock:
            self.send_datas.gimbal_mode = msg.data
    def gimbal_control_callback(self,msg):
        with self.lock:
            self.send_datas.pitch = msg.pitch
            self.send_datas.yaw = msg.yaw
            self.send_datas.can_fire = msg.can_fire

    def receive_data(self):
        serial_receive_msg = Decision()
        serial_receive_msg.header.frame_id = 'serial_receive_frame'
        serial_receive_msg.color = 10

        # [确认配置]
        # 总长 16 = Header(1)+Color(1)+Roll(4)+Pitch(4)+Yaw(4) + CRC(2)
        packet_length = 16

        self.get_logger().info("接收数据线程已启动 (CRC16 Mode - 16 Bytes)")

        while rclpy.ok():
            if self.is_reconnecting:
                time.sleep(0.1)
                continue
            try:
                # 1. 查找帧头
                header = self.serial.read(1)
                if not header or header != b'\xA5':
                    continue

                # 2. 读取剩余数据 (15字节)
                remaining_data = self.serial.read(packet_length - 1)
                if len(remaining_data) != packet_length - 1:
                    self.get_logger().warn("数据包不完整")
                    continue

                # 3. 组合完整包
                full_packet = header + remaining_data

                # 4. CRC16 校验逻辑
                # 数据载荷：前14字节 (Header ~ Yaw)
                data_payload = full_packet[:-2]
                # 校验位：最后2字节
                checksum_bytes = full_packet[-2:]

                # 解析收到的校验值 (小端序 unsigned short)
                received_crc = struct.unpack('<H', checksum_bytes)[0]

                # 计算本地数据的 CRC16
                # 注意：确保 get_crc16_check_sum 算法与下位机一致 (通常是 CRC-CCITT)
                calculated_crc = get_crc16_check_sum(data_payload)

                if calculated_crc != received_crc:
                    # 校验失败时打印 Hex 用于调试
                    raw_hex = ' '.join([f'{b:02x}' for b in full_packet])
                    self.get_logger().warn(f"校验失败 | 收:{hex(received_crc)} 算:{hex(calculated_crc)} | Raw: {raw_hex}")
                    continue

                # 5. 数据解包 (14字节)
                # <BBfff: Header(1), Color(1), Roll(4), Pitch(4), Yaw(4)
                _, detect_color, roll, pitch, yaw = struct.unpack("<BBfff", data_payload)

                # 6. 发布 IMU 消息
                rpy_msg = Vector3Stamped()
                rpy_msg.header.stamp = self.get_clock().now().to_msg()
                rpy_msg.header.frame_id = 'imu_link'
                rpy_msg.vector.x = float(roll)
                rpy_msg.vector.y = float(-pitch)
                rpy_msg.vector.z = float(yaw)

                self.pub_uart_receive_imu.publish(rpy_msg)

                # 7. 处理 Decision 逻辑
                serial_receive_msg.header.stamp = self.get_clock().now().to_msg()

                if self.tracking_color != detect_color:
                    self.tracking_color = detect_color
                    serial_receive_msg.color = detect_color

                # 发布决策消息 (注意：match 字段已被移除)
                self.pub_uart_receive_decision.publish(serial_receive_msg)

            except (serial.SerialException, struct.error, ValueError) as e:
                self.get_logger().error(f"接收数据异常: {str(e)}")
                self.reopen_port()

    
    def Send(self):
        if self.is_reconnecting:
            return  # 如果正在重连，跳过本次发送

        try:
            with self.lock:
                linear_velocity_x = self.send_datas.linear_velocity_x
                linear_velocity_y = self.send_datas.linear_velocity_y
                gimbal_mode = self.send_datas.gimbal_mode
                yaw = self.send_datas.yaw
                pitch = self.send_datas.pitch
                can_fire = self.send_datas.can_fire
            # 数据打包
            #帧头，帧尾赋值
            # self.send_datas.header = b'\xA5'
            # self.send_datas.ender  = b'\x2b'
            header = 0xA5
            ender = 0x2b
            #帧头，x线速度，y线速度，小陀螺模式， 云台yaw，云台pitch，开火，帧尾
            data_payload = struct.pack(
                '<BffiffiB',
                header, 
                linear_velocity_x,
                linear_velocity_y, 
                gimbal_mode,
                yaw, 
                pitch, 
                can_fire, 
                ender
                )

            checksum = get_crc16_check_sum(data_payload)

            packet = data_payload + struct.pack("<H", checksum)

            self.serial.write(packet)
            print(header, linear_velocity_x,linear_velocity_y, gimbal_mode,yaw, pitch, can_fire, ender)
        except Exception as e:
            self.get_logger().error(f"发送数据时出错: {str(e)}")

    def reopen_port(self):
        # 使用锁保护状态标志的修改
        with self.serial_lock:
            if self.is_reconnecting:
                return  # 如果已经有其他线程在执行重连，直接退出
            self.is_reconnecting = True

        self.get_logger().warn("正在重连串口...")
        
        while rclpy.ok():
            try:
                if self.serial and self.serial.is_open:
                    self.serial.close()
                
                # 重新实例化串口对象，处理底层设备节点变更
                self.serial = serial.Serial(
                    port=self.port_name,
                    baudrate=self.baudrate,
                    timeout=1,
                    write_timeout=1,
                )
                self.get_logger().info("串口重连成功")
                break  # 重连成功，跳出循环
                
            except serial.SerialException as e:
                self.get_logger().error(f"串口重连失败，1秒后重试: {str(e)}")
                time.sleep(1)

        # 重连完成后，恢复标志位
        with self.serial_lock:
            self.is_reconnecting = False

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode("serial_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    