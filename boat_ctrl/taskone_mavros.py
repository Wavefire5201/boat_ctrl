import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from boat_ctrl.arm_boat import ArmingService
from boat_ctrl.boat_mode import ModeService
from boat_ctrl.boat_controller import BoatController

import json

class TaskOne(Node):
    def __init__(self):
        super().__init__("taskone_node")

        # self.arm_boat = ArmingService()
        # self.arm_boat.arm_boat(True)

        # self.boat_mode = ModeService()
        # self.boat_mode.set_mode("GUIDED")

        self.boat_controller = BoatController()

        self.pole_subscriber = self.create_subscription(String, "taskone/poles", self.callback, 10)

        self.start = False

    def callback(self, data: String):
        if not self.start:
            input("Start? Press any key to start!")
            self.start = True

        poles = json.loads(data.data)
        SCREEN_MIDDLE = 640 / 2
        
        # If it doesn't detect more than one pole, then pass so it doesn't crash
        if len(poles) < 2:
            print("Only one or no pole found")
            self.boat_controller.set_linear_velocity(0.0)
            self.boat_controller.set_angular_velocity(0.0)
            return

        # Sort the list of poles by area in descending order
        sort_by_area = sorted(poles, key=lambda x: x["area"], reverse=True)
        # Get the two largest areas (closest poles to boat)
        poles = [pole for pole in sort_by_area[:2]]

        if poles[0]["name"] == poles[1]["name"]:
            print("Two poles of same color found, keep going forward")
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
        buoy_middle = left_buoy["x_right"] + ((right_buoy["x_left"] - left_buoy["x_right"]) / 2)
        
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

        # print(buoy_middle)

        if (
            left_buoy["x_right"] < SCREEN_MIDDLE
            and right_buoy["x_left"] < SCREEN_MIDDLE
        ):
            # Turn left to orientate boat between two buoys
            print("TWO BUOYS ON LEFT SIDE OF SCREEN")
            self.boat_controller.set_angular_velocity(5.0)
        elif (
            left_buoy["x_right"] > SCREEN_MIDDLE
            and right_buoy["x_left"] > SCREEN_MIDDLE
        ):
            # Turn right to orientate boat between two buoys
            print("TWO BUOYS ON RIGHT SIDE OF SCREEN")
            self.boat_controller.set_angular_velocity(-5.0)
        elif buoy_middle - SCREEN_MIDDLE > 5:
            print("TOO LEFT")
            self.boat_controller.set_angular_velocity(-1.0)
        elif SCREEN_MIDDLE - buoy_middle > 5:
            print("TOO RIGHT")
            self.boat_controller.set_angular_velocity(1.0)

        elif SCREEN_MIDDLE - buoy_middle < 5 and buoy_middle - SCREEN_MIDDLE < 5:
            print("MOVING FORWARD")
            self.boat_controller.set_linear_velocity(10.0)

        self.boat_controller.cmd_vel_publisher.publish(self.boat_controller.cmd_vel)

    


def main(args=None):
    rclpy.init(args=args)

    taskone = TaskOne()
    rclpy.spin(taskone)

    taskone.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()
