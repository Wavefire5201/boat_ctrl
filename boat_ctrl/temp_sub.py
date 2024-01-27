import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy
from boat_interfaces.msg import BuoyCoordinates
from sensor_msgs.msg import NavSatFix

from geopy import distance

class Temp(Node):
    def __init__(self):
        super().__init__("temp")
        
        self.qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )
        
        self.create_subscription(BuoyCoordinates, "/AverageBuoyCoordinates", self.callback, 10)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_callback, qos_profile=self.qos_profile)
        
        
        
    def callback(self, data: BuoyCoordinates):
        coords = []
        for lat, long in zip(data.latitudes, data.longitudes):
            coords.append((lat, long))
        try:
            coord1 = coords[0]
            coord2 = coords[1]
        except IndexError:
            pass
        midpoint = self.find_midpoint(coord1, coord2)
        print(f"Coord1: {coord1}")
        print(f"Coord2: {coord2}")
        print(f"Midpoint: {midpoint}")
        print(f"Distance: {distance.distance(coord1, coord2).meters}")
        
    def gps_callback(self, data: NavSatFix):
        print(f"GPS: {data.latitude}, {data.longitude}")
        
    def find_midpoint(self, coord1, coord2):
        try:
            midpoint_lat = (coord1[0] + coord2[0]) / 2
            midpoint_lon = (coord1[1] + coord2[1]) / 2
            return midpoint_lat, midpoint_lon
        except IndexError:
            pass


def main(args=None):
    rclpy.init(args=args)
    cam_sub = Temp()
    rclpy.spin(cam_sub)
    cam_sub.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
        
        