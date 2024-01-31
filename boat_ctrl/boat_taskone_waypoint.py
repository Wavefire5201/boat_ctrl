import time
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
)

from mavros_msgs.srv import CommandHome
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from boat_interfaces.msg import BuoyCoordinates


class TaskoneWaypoint(Node):
    def __init__(self):
        super().__init__("taskone_waypoint_node")

        self.qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self.current_lat = 0
        self.current_long = 0

        self.set_home_service = self.create_client(CommandHome, "/mavros/cmd/set_home")
        self.waypoint_publisher = self.create_publisher(
            GeoPoseStamped, "/mavros/setpoint_position/global", 10
        )
        # self.create_subscription(
        #     BuoyCoordinates, "/AverageBuoyCoordinates", self.callback, 10
        # )
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_callback,
            qos_profile=self.qos_profile,
        )

    def gps_callback(self, data: NavSatFix):
        self.current_lat = data.latitude
        self.current_long = data.longitude

    def set_home(self):
        request = CommandHome.Request()
        request.latitude = float(self.current_lat)
        request.longitude = float(self.current_long)
        self.future = self.set_home_service.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_waypoint(self, lat: float, long: float):
        msg = GeoPoseStamped()
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = long
        self.waypoint_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    taskone = TaskoneWaypoint()

    # taskone.set_home()
    time.sleep(5)
    # while True:
    taskone.set_waypoint(-35.3632556, 149.1652358)
    # taskone.set_waypoint(-35.3632694, 149.1652390)
    # taskone.set_waypoint(-35.3632680, 149.1652475)

    taskone.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
