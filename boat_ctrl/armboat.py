import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class MavrosServiceCaller(Node):
    def __init__(self):
        super().__init__("mavros_service_caller")
        self.arm_service = self.create_client(CommandBool, "/mavros/cmd/arming")
        while not self.arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def arm_boat(self):
        request = CommandBool.Request()
        request.value = True
        self.future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    service_caller = MavrosServiceCaller()
    service_caller.arm_boat()
    service_caller.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
        
        