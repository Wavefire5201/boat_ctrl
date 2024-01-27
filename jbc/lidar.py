import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.create_subscription(PointCloud2, "/wamv/sensors/lidars/lidar_wamv_sensor/points", self.callback, 10)
        
    def callback(self, msg: PointCloud2):
        data = msg.data
        point_step = msg.point_step
        fields = msg.fields

        num_points = int(len(data) / point_step)

        for i in range(num_points):
            offset = i * point_step
            point = {}
            for field in fields:
                field_name = field.name
                field_offset = field.offset
                field_dtype = field.datatype
                if field_dtype == PointField.FLOAT32:
                    value = struct.unpack('f', data[offset + field_offset:offset + field_offset + 4])[0]
                elif field_dtype == PointField.FLOAT64:
                    value = struct.unpack('d', data[offset + field_offset:offset + field_offset + 8])[0]
                elif field_dtype == PointField.INT32 or field_dtype == PointField.UINT32:
                    value = struct.unpack('I', data[offset + field_offset:offset + field_offset + 4])[0]
                # Add more cases for other datatypes as needed
                point[field_name] = value

            x = point['x']
            y = point['y']
            z = point['z']
            self.get_logger().info(f"Point: x={x:.2f}, y={y:.2f}, z={z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    lidar_sub = LidarSubscriber()
    rclpy.spin(lidar_sub)
    lidar_sub.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
        
        