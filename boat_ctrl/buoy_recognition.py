import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from enum import Enum

model = YOLO(f"/root/roboboat_ws/src/boat_ctrl/boat_ctrl/V9_model.pt")

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.create_subscription(Image, "/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw", self.callback, 10)
        
    def callback(self, data: Image):
        self.camera_output = CvBridge().imgmsg_to_cv2(data, "bgr8")

        # Hack to get color picker inside vscode
        class rgb():
            def __init__(self, r, g, b):
                self.r = r
                self.g = g
                self.b = b
            
            def __str__(self):
                return f"rgb({self.r}, {self.g}, {self.b})"

            def __repr__(self):
                return f"rgb({self.r}, {self.g}, {self.b})"
            
            def as_bgr(self) -> tuple:
                return (self.b, self.g, self.r)

        colors: dict[rgb] = {
            "blue_ball": rgb(0, 0, 255),
            "dock": rgb(109, 67, 3),
            "green_ball": rgb(0, 255, 0),
            "green_pole": rgb(0, 255, 0),
            "misc_buoy": rgb(0, 217, 255),
            "red_ball": rgb(255, 0, 0),
            "red_pole": rgb(255, 0, 0),
            "yellow_ball": rgb(255, 255, 0),
        }

        frame = self.camera_output
        original_frame = self.camera_output
        x_scale_factor = original_frame.shape[1] / 640
        y_scale_factor = original_frame.shape[0] / 640
        x_orig, y_orig = original_frame.shape[1], original_frame.shape[0]
        frame = cv2.resize(frame, (640, 640))

        result = model(frame)

        for pred in result:
            names = pred.names

            for i in range(len(pred.boxes)):
                name = names.get(int(pred.boxes.cls[i]))
                confidence = pred.boxes.conf[i]
                bounding_box = pred.boxes[i].xyxy[0]
                bounding_box = [
                    bounding_box[0] * x_scale_factor,
                    bounding_box[1] * y_scale_factor,
                    bounding_box[2] * x_scale_factor,
                    bounding_box[3] * y_scale_factor
                ]

                print(f"{name} {int(confidence*100)}% {bounding_box}")

                color = colors.get(name, rgb(255, 255, 255))

                original_frame = cv2.putText(original_frame, 
                                            f"{name} {int(confidence*100)}%",
                                            (int(bounding_box[0]), int(bounding_box[1])-5),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color.as_bgr(), 1)
                original_frame = cv2.rectangle(original_frame,
                                            (int(bounding_box[0]), int(bounding_box[1])),
                                            (int(bounding_box[2]), int(bounding_box[3])), 
                                            color.as_bgr(), 1)

        # original_frame = cv2.putText(original_frame, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow("result", original_frame)
        cv2.waitKey(1)
        # if c == 27:
        #     break

        # if cv2.getWindowProperty("result", cv2.WND_PROP_VISIBLE) < 1:
        #     break

def main(args=None):
    rclpy.init(args=args)
    cam_sub = CameraSubscriber()
    rclpy.spin(cam_sub)
    cam_sub.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
        
        
