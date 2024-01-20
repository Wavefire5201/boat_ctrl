import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg._float64 import Float64

from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


model = YOLO(f"/root/ros_ws/src/boat_ctrl/boat_ctrl/V9_model.pt")

# SCREEN_SIZE = 1280x720
SCREEN_MIDDLE = 640 / 2


class TaskOne(Node):
    def __init__(self):
        super().__init__("taskone")
        self.create_subscription(
            Image,
            "/color/image_raw",
            self.callback,
            10,
        )
        self.left_pub = self.create_publisher(
            Float64, "/wamv/thrusters/left/thrust", 10
        )
        self.right_pub = self.create_publisher(
            Float64, "/wamv/thrusters/right/thrust", 10
        )
        self.start = False
        self.left = Float64()
        self.right = Float64()
        self.left.data = 0.0
        self.right.data = 0.0

    def callback(self, data: Image):
        if not self.start:
            input("Start? Press any key to start!")
            self.start = True

        # Convert camera topic output to cv2 processable data
        self.camera_output = CvBridge().imgmsg_to_cv2(data, "bgr8")

        # Hack to get color picker inside vscode
        class rgb:
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

        # Since model is trained to recognize buoys on a 640x640 image,
        # scale camera output down to that size, then scale back up so we can view it normally
        frame = self.camera_output
        original_frame = self.camera_output
        x_scale_factor = original_frame.shape[1] / 640
        y_scale_factor = original_frame.shape[0] / 640
        x_orig, y_orig = original_frame.shape[1], original_frame.shape[0]
        frame = cv2.resize(frame, (640, 640))

        result = model(frame, verbose=False)

        poles = []

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
                    bounding_box[3] * y_scale_factor,
                ]

                # Add all poles for task one
                if "pole" in name:
                    x_left = int(bounding_box[0])
                    y_top = int(bounding_box[1])
                    x_right = int(bounding_box[2])
                    y_bottom = int(bounding_box[3])
                    length_x = int(bounding_box[2]) - int(bounding_box[0])
                    length_y = int(bounding_box[3]) - int(bounding_box[1])
                    area = length_x * length_y
                    # Filtering out weird pole data on bottom of screen
                    pole_data = {
                        "name": name,
                        "area": area,
                        "x_left": x_left,
                        "x_right": x_right,
                        "y_top": y_top,
                        "y_bottom": y_bottom,
                    }
                    poles.append(pole_data)

                # print(f"{name} {int(confidence*100)}% {bounding_box}")

                # Draw boxes and text onto screen
                color = colors.get(name, rgb(255, 255, 255))
                original_frame = cv2.putText(
                    original_frame,
                    f"{name} {int(confidence*100)}%",
                    (int(bounding_box[0]), int(bounding_box[1]) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    color.as_bgr(),
                    1,
                )
                original_frame = cv2.rectangle(
                    original_frame,
                    (int(bounding_box[0]), int(bounding_box[1])),
                    (int(bounding_box[2]), int(bounding_box[3])),
                    color.as_bgr(),
                    1,
                )

        # original_frame = cv2.putText(original_frame, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # print("unmodified")
        # print(poles)

        # If it doesn't detect more than one pole, then pass so it doesn't crash
        if len(poles) < 2:
            print("Only one or no pole found")
            self.left.data = 50.0
            self.right.data = 50.0
            self.left_pub.publish(self.left)
            self.right_pub.publish(self.right)
            cv2.imshow("result", original_frame)
            return

        # Sort the list of poles by area in descending order
        sort_by_area = sorted(poles, key=lambda x: x["area"], reverse=True)
        # Get the two largest areas (closest poles to boat)
        poles = [pole for pole in sort_by_area[:2]]

        if poles[0]["name"] == poles[1]["name"]:
            print("Two poles of same color found, keep going forward")
            cv2.imshow("result", original_frame)
            return

        # poles = get_closest_buoy(poles)
        # print("two largest area (closest)")
        # print(poles)

        # Set left pole and right pole based on x values
        if poles[0]["x_left"] > poles[1]["x_left"]:
            left_buoy = poles[1]
            right_buoy = poles[0]
        elif poles[0]["x_left"] < poles[1]["x_left"]:
            left_buoy = poles[0]
            right_buoy = poles[1]

        # print(left_buoy)
        # print(right_buoy)

        # Calculate middle of two poles
        buoy_middle_x = left_buoy["x_right"] + ((right_buoy["x_left"] - left_buoy["x_right"]) / 2)

        # cv2.line(image, start_point, end_point, color, thickness) 
        original_frame = cv2.line(original_frame, (int(buoy_middle_x), int(left_buoy["y_top"])), ((int(buoy_middle_x), int(left_buoy["y_bottom"]))), (255, 0, 0), 2)
        
        # If both poles are on left side of boat, angle towards left
        # Vice versa for poles on right side of screen
        # if abs(SCREEN_MIDDLE - left_buoy["x_right"]) < abs(
        #     SCREEN_MIDDLE - right_buoy["x_left"]
        # ):
        #     buoy_middle = buoy_middle + SCREEN_MIDDLE
        # elif abs(SCREEN_MIDDLE - left_buoy["x_right"]) > abs(
        #     SCREEN_MIDDLE - right_buoy["x_left"]
        # ):
        #     buoy_middle = SCREEN_MIDDLE - buoy_middle

        print(buoy_middle_x)
        print(original_frame.shape[1])

        if (
            left_buoy["x_right"] < SCREEN_MIDDLE
            and right_buoy["x_left"] < SCREEN_MIDDLE
        ):
            print("TWO BUOYS ON LEFT SIDE OF SCREEN")
            self.left.data = -50.0
            self.right.data = 50.0
        elif (
            left_buoy["x_right"] > SCREEN_MIDDLE
            and right_buoy["x_left"] > SCREEN_MIDDLE
        ):
            print("TWO BUOYS ON RIGHT SIDE OF SCREEN")
            self.left.data = 50.0
            self.right.data = -50.0
        elif buoy_middle_x - SCREEN_MIDDLE > 5:
            print("TOO LEFT")
            self.left.data = 10.0
            self.right.data = -10.0
        elif SCREEN_MIDDLE - buoy_middle_x > 5:
            print("TOO RIGHT")
            self.left.data = -10.0
            self.right.data = 10.0
        elif SCREEN_MIDDLE - buoy_middle_x < 5 and buoy_middle_x - SCREEN_MIDDLE < 5:
            print("MOVING FORWARD")
            self.left.data = 200.0
            self.right.data = 200.0

        self.left_pub.publish(self.left)
        self.right_pub.publish(self.right)

        cv2.imshow("result", original_frame)
        cv2.waitKey(1)


"""
buoy_middle = (left_buoy["x_right"] - right_buoy["x_left"]) / 2 + SCREEN_SIZE
buoy_middle_y = left_buoy["y_top"] + ((right_buoy["y_bottom"] - left_buoy["y_top"]) / 2)
"""


def main(args=None):
    rclpy.init(args=args)
    taskone = TaskOne()
    rclpy.spin(taskone)
    taskone.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
