import rclpy
from rclpy.node import Node
# from geometry_msgs.msg._twist import Twist

from std_msgs.msg._float64 import Float64
import curses
from curses import wrapper

class ControllerPublisher(Node):
    def __init__(self):
        super().__init__("boat_controller_pub")
        self.left_pub = self.create_publisher(Float64, "/wamv/thrusters/left/thrust", 10)
        self.right_pub = self.create_publisher(Float64, "/wamv/thrusters/right/thrust", 10)
        
    def controller(self, stdscr):
        left = Float64()
        right = Float64()
        
        # Create static info menu
        stdscr.clear()
        stdscr.addstr(0, 0, "-----------------------------")
        stdscr.addstr(1, 0, "Moving around:")
        stdscr.addstr(2, 0, "Control boat")
        stdscr.addstr(3, 0, "       w      ")
        stdscr.addstr(4, 0, "   a   s   d  ")
        stdscr.addstr(5, 0, "       x      ")
        stdscr.addstr(6, 0, " ")
        stdscr.addstr(7, 0, "w/x: increase linear velocity")
        stdscr.addstr(8, 0, "a/d: increase angular velocity")
        stdscr.addstr(9, 0, "s: stop")
        stdscr.addstr(10, 0, " ")
        stdscr.addstr(11, 0, "Press Ctrl C to exit")
        stdscr.move(12, 0)
        stdscr.refresh()
        
        # Create window with updating movement info
        
        output_win = curses.newwin(0, 0, 12, 0)
        
        while True:
            try:
                input = stdscr.getkey()
                match input:
                    case "w":
                        left.data += 10
                        right.data += 10
                        output_win.clear()
                        output_win.addstr(f"currently:   left: {left.data}  right: {right.data}")
                        output_win.refresh()
                    case "a":
                        left.data -= 10
                        right.data += 10
                        output_win.clear()
                        output_win.addstr(f"currently:   left: {left.data}  right: {right.data}")
                        output_win.refresh()
                    case "s":
                        left.data = 0.0
                        right.data = 0.0
                        output_win.clear()
                        output_win.addstr(f"currently:   left: {left.data}  right: {right.data}")
                        output_win.refresh()
                    case "d":
                        left.data += 10
                        right.data -= 10
                        output_win.clear()
                        output_win.addstr(f"currently:   left: {left.data}  right: {right.data}")
                        output_win.refresh()
                    case "x":
                        left.data -= 10
                        right.data -= 10
                        output_win.clear()
                        output_win.addstr(f"currently:   left: {left.data}  right: {right.data}")
                        output_win.refresh() 
                        
            except KeyboardInterrupt:
                left.data = 0.0
                right.data = 0.0
                break
            
            self.left_pub.publish(left)
            self.right_pub.publish(right)
        
        
def main(args=None):
    rclpy.init(args=args)
    
    ctrl_pub = ControllerPublisher()
    wrapper(ctrl_pub.controller)
    
    ctrl_pub.destroy_node()
    

if __name__ == "__main__":
    main()
