#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import sys

# 宇树 SDK 导入
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

class GO2ControllerNode(Node):
    def __init__(self):
        super().__init__('go2_controller')

        # 1. 核心变量预初始化 (防止定时器提前运行导致 AttributeError)
        self.sport_client = None
        self.current_waypoint = None
        self.waypoint_timestamp = None
        self.last_v = 0.0
        self.last_w = 0.0

        # 2. ========== 参数声明与读取 ==========
        self.declare_parameter('max_v', 1.0)          # 最大线速度 (m/s)
        self.declare_parameter('max_w', 1.5)          # 最大角速度 (rad/s)
        self.declare_parameter('control_rate', 30.0)  # 控制频率 (Hz)
        self.declare_parameter('waypoint_timeout', 1.0) # waypoint 超时时间 (s)
        self.declare_parameter('smoothing_alpha', 0.3)  # 速度平滑系数 (0~1)
        self.declare_parameter('vel_deadband', 0.05)   # 线速度死区 (m/s)
        self.declare_parameter('yaw_deadband', 0.05)   # 角速度死区 (rad/s)
        self.declare_parameter('min_v_cmd', 0.1)       # 最小有效线速度 (m/s)
        self.declare_parameter('min_w_cmd', 0.1)       # 最小有效角速度 (rad/s)
        self.declare_parameter('network_interface', 'enp4s0')  # 网络接口
        self.declare_parameter('waypoint_topic', '/flownav/waypoint')  # waypoint 订阅主题

        self.max_v = self.get_parameter('max_v').value
        self.max_w = self.get_parameter('max_w').value
        self.control_rate = self.get_parameter('control_rate').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.vel_deadband = self.get_parameter('vel_deadband').value
        self.yaw_deadband = self.get_parameter('yaw_deadband').value
        self.min_v_cmd = self.get_parameter('min_v_cmd').value
        self.min_w_cmd = self.get_parameter('min_w_cmd').value
        network_if = self.get_parameter('network_interface').value
        waypoint_topic = self.get_parameter('waypoint_topic').value

        # 3. ========== 初始化宇树 SDK ==========
        self.get_logger().info(f"正在初始化宇树 SDK，网卡: {network_if}")
        try:
            # 初始化底层通信通道
            ChannelFactoryInitialize(0, network_if)
            
            # 实例化运动控制客户端 (这一步之前漏掉了)
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(5.0)
            self.sport_client.Init()
            
            self.get_logger().info("宇树 SDK 客户端初始化成功")
        except Exception as e:
            self.get_logger().error(f"SDK 初始化异常: {e}")
            # 如果初始化失败，直接抛出错误退出，防止误动
            sys.exit(1)

        # 4. ========== 订阅与定时器 ==========
        self.sub = self.create_subscription(
            Float32MultiArray,
            waypoint_topic,
            self.waypoint_callback,
            10
        )
        
        # 将定时器放在最后启动，确保所有变量已就绪
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info("="*50)
        self.get_logger().info("GO2 运动控制器已就绪")
        self.get_logger().info(f"订阅话题: {waypoint_topic}")
        self.get_logger().info(f"限速: V={self.max_v}m/s, W={self.max_w}rad/s")
        self.get_logger().info("="*50)

    def waypoint_callback(self, msg: Float32MultiArray):
        """接收 waypoint 数据"""
        if len(msg.data) < 2:
            self.get_logger().warn("收到无效 waypoint：长度不足")
            return
        self.current_waypoint = np.array(msg.data[:2], dtype=np.float32)
        self.waypoint_timestamp = self.get_clock().now()

    def control_loop(self):
        """核心控制循环"""
        # 安全检查：如果 SDK 客户端没准备好，直接返回
        if self.sport_client is None:
            return

        # 1. 检查 waypoint 是否有效与超时
        if self.current_waypoint is None or self.waypoint_timestamp is None:
            self.send_stop()
            return

        now = self.get_clock().now()
        dt = (now - self.waypoint_timestamp).nanoseconds / 1e9
        if dt > self.waypoint_timeout:
            self.get_logger().warn(f"数据超时 ({dt:.2f}s)，执行刹车")
            self.current_waypoint = None
            self.send_stop()
            return

        # 2. 计算目标速度 (基于简单增益，control_dt 转换)
        dx, dy = self.current_waypoint
        control_dt = 1.0 / self.control_rate

        if abs(dx) < 1e-6:
            v = 0.0
            w = np.clip(np.sign(dy) * np.pi, -self.max_w, self.max_w)
        else:
            v = dx / control_dt
            w = np.arctan2(dy, dx) / control_dt

        # 3. 速度限幅与平滑
        v = np.clip(v, -self.max_v, self.max_v)
        w = np.clip(w, -self.max_w, self.max_w)

        v_smooth = self.smoothing_alpha * v + (1 - self.smoothing_alpha) * self.last_v
        w_smooth = self.smoothing_alpha * w + (1 - self.smoothing_alpha) * self.last_w

        # 4. 死区与最小指令处理
        if abs(v_smooth) < self.vel_deadband: v_smooth = 0.0
        if abs(w_smooth) < self.yaw_deadband: w_smooth = 0.0

        if 0.0 < abs(v_smooth) < self.min_v_cmd:
            v_smooth = np.sign(v_smooth) * self.min_v_cmd
        if 0.0 < abs(w_smooth) < self.min_w_cmd:
            w_smooth = np.sign(w_smooth) * self.min_w_cmd

        # 更新历史值
        self.last_v, self.last_w = v_smooth, w_smooth

        # 5. 执行指令
        try:
            self.sport_client.Move(float(v_smooth), 0.0, float(w_smooth))
        except Exception as e:
            self.get_logger().error(f"发送控制指令失败: {e}")

    def send_stop(self):
        """紧急停止运动"""
        if self.sport_client is not None:
            try:
                self.sport_client.Move(0.0, 0.0, 0.0)
            except:
                pass
        self.last_v = 0.0
        self.last_w = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = GO2ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n正在停止控制器...")
    finally:
        if rclpy.ok():
            node.send_stop()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
