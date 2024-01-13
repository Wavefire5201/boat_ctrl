import sys
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode

class MavrosServiceCaller(Node):
    def __init__(self):
        super().__init__("mavros_service_caller")
        
        self.modes = {
            "MAV_MODE_PREFLIGHT"          : 0,
            "MAV_MODE_STABILIZE_DISARMED" : 80,
            "MAV_MODE_STABILIZE_ARMED"    : 208,
            "MAV_MODE_MANUAL_DISARMED"    : 64,
            "MAV_MODE_MANUAL_ARMED"       : 192,
            "MAV_MODE_GUIDED_DISARMED"    : 88,
            "MAV_MODE_GUIDED_ARMED"       : 216,
            "MAV_MODE_AUTO_DISARMED"      : 92,
            "MAV_MODE_AUTO_ARMED"         : 220,
            "MAV_MODE_TEST_DISARMED"      : 66,
            "MAV_MODE_TEST_ARMED"         : 194
        }
        
        self.arm_service = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_service = self.create_client(SetMode, "/mavros/set_mode")
        
    def arm_boat(self):
        request = CommandBool.Request()
        request.value = True
        self.future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def set_mode(self, mode):
        request = SetMode.Request()
        request.base_mode = self.modes[mode]
        self.future = self.mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    service_caller = MavrosServiceCaller()
    service_caller.arm_boat()
    service_caller.set_mode(sys.argv[1])
    service_caller.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
        
        