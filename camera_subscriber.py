import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscriber_status={
            "node_init": False,
            "topic_subscribed": False,
            "callback_triggered": False,
            "image_converted": False,
            "image_cached": False
        }
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.last_callback_time = None
        self.topic_name = '/camera/camera/color/image_raw'

        self.subscriber_status["node_init"] = True
        self.get_logger().info("="*30)
        self.get_logger().info("Initializing Camera Subscriber node...")
        self.get_logger().info(f"Subscribing to topic: {self.topic_name}")
        self.get_logger().info("camera_subscriber_status: " + str(self.subscriber_status))
        self.get_logger().info("="*30)
        # self.get_logger().info("Camera Subscriber node initialized. Waiting for images...")


        self.qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        try:
            self.subscription = self.create_subscription(
                Image,
                self.topic_name,
                self.diagnostic_callback,
                self.qos_profile
            )
            self.subscriber_status["topic_subscribed"] = True
            self.get_logger().info(f"Successfully subscribed to topic: {self.topic_name}")
        except Exception as e:
            self.subscriber_status["topic_subscribed"] = False
            self.get_logger().error(f"Failed to subscribe to topic {self.topic_name}: {e}")
        

        self.diagnostic_timer = self.create_timer(2.0, self.periodic_diagnostic)

        # self.display_thread = threading.Thread(target=self.
    
    def diagnostic_callback(self, msg):
        self.subscriber_status["callback_triggered"] = True
        self.last_callback_time = time.time()
        self.get_logger().info(f"回调函数触发 at {self.last_callback_time}")

        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            if isinstance(self.latest_rgb, np.ndarray)and len(self.latest_rgb.shape) == 3:
                self.subscriber_status["image_converted"] = True
                self.subscriber_status["image_cached"] = True
                self.get_logger().info(f"Image converted and cached successfully, shape: {self.latest_rgb.shape}")
                # self.show_rgb()
            else:
                self.subscriber_status["image_converted"] = False
                self.subscriber_status["image_cached"] = False
                self.get_logger().error("Converted image is not a valid numpy array")
        except Exception as e:
            self.subscriber_status["image_converted"] = False
            self.subscriber_status["image_cached"] = False
            self.latest_rgb = None
            self.get_logger().error(f"Error converting image: {str(e)}")

    def show_rgb(self):
        if self.latest_rgb is not None:
            cv2.imshow("realsense_rgb", self.latest_rgb)
            cv2.waitKey(1)
        else:
            self.get_logger().warning("No image to display in show_rgb()")

    def periodic_diagnostic(self):
        current_time = time.time()
        self.get_logger().info("\n"+"="*30)
        self.get_logger().info("Camera Subscriber Diagnostic Report(every 2 seconds):")
        self.get_logger().info(f"Node Initialized: {self.subscriber_status['node_init']}")
        self.get_logger().info(f"Topic Subscribed: {self.subscriber_status['topic_subscribed']}")
        self.get_logger().info(f"Callback Triggered: {self.subscriber_status['callback_triggered']}")
        self.get_logger().info(f"Image Converted: {self.subscriber_status['image_converted']}")
        self.get_logger().info(f"Image Cached: {self.subscriber_status['image_cached']}")
        self.get_logger().info("="*30+"\n")


    def listener_callback(self, msg):
        self.get_logger().info('Received image message from topic')
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.get_logger().info(f"Image converted to OpenCV format successfully")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def get_rgb(self):
        if self.latest_rgb is not None:
            self.get_logger().debug("get_rgb called, returning latest image")
            return self.latest_rgb
        else:
            self.get_logger().warning("get_rgb()return None, 原因：" + str(self.subscriber_status))
            return None
        
def main(args=None):
    try:
        rclpy.init(args=args)
    except Exception as e:
        print(f"Failed to initialize rclpy: {str(e)}")
        return
    
    try:
        camera_subscriber = CameraSubscriber()
        # import threading
        # spin_thread = threading.Thread(target=rclpy.spin, args=(camera_subscriber,), daemon=True)
        # spin_thread.start()
        rclpy.spin(camera_subscriber)
        print("Camera Subscriber node is running. Press Ctrl+C to exit.")

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down Camera Subscriber node...")
    finally:
        if'camera_subscriber' in locals():
            camera_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()