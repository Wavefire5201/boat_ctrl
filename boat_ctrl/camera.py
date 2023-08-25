import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.create_subscription(Image, "/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw", self.callback, 10)
        
    def callback(self, data: Image):
        self.camera_output = CvBridge().imgmsg_to_cv2(data, "bgr8")
        # cv2.imshow("boat_output", self.camera_output)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    cam_sub = CameraSubscriber()
    rclpy.spin(cam_sub)
    cam_sub.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
        
        