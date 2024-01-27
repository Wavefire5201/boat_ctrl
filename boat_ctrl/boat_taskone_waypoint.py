import rclpy
from rclpy.node import Node

from geographic_msgs.msg import GeoPoseStamped

class TaskoneWaypoint(Node):
    def __init__(self):
        super().__init__("taskone_waypoint_node")
        self.waypoint_publisher = self.create_publisher(GeoPoseStamped, "/mavros/setpoint_position/global", 10)
    
    def set_waypoint(self, lat, long):
        msg = GeoPoseStamped()
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = long
        self.waypoint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    taskone_waypoint = TaskoneWaypoint()
    rclpy.spin(taskone_waypoint)
    taskone_waypoint.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()