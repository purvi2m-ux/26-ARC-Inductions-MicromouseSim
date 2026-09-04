"""
Write your own solver in the scan_callback function
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ==========================================
# These four parameters MUST add up to exactly 30!
# ==========================================
TOP_SPEED = 8
ACCELARATION = 7
TURN_SPEED = 5
SENSOR_RANGE = 10

class StudentSolver(Node):
    def __init__(self):
        super().__init__('student_solver')
        
        # subscriber to read sensor values (L,F,R)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/mouse/scan',
            self.scan_callback,
            10
        )
        
        # publisher to send movement commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mouse/cmd_vel',
            10
        )
        
        self.get_logger().info("Student Solver Node initialized successfully.")
        self.get_logger().info(f"Stats -> Speed: {TOP_SPEED}, Accel: {ACCELARATION}, Turn: {TURN_SPEED}, Range: {SENSOR_RANGE}")

    def scan_callback(self, msg):
        """
        This function runs every time a new sensor reading is received (at 20 Hz).
        msg.ranges contains the distances:
        msg.ranges[0] -> Left ray distance
        msg.ranges[1] -> Front ray distance
        msg.ranges[2] -> Right ray distance
        """
        d_left = msg.ranges[0]
        d_front = msg.ranges[1]
        d_right = msg.ranges[2]
        
        cmd = Twist()
        
        #-------- NEW LOGIC, WRITTEN  ---------

        if not hasattr(self, '_mode'):
            self._mode = 'follow_left'
            self._escape_dir = 1.0
            self._escape_timer = 0
            self._turn_dir = 1.0
            self._stuck_counter = 0
            self._last_pattern = None        
       
        # emergency: fully trapped - reverse and rotate out more aggressively
        if d_front < 0.45 and d_left < 0.45 and d_right < 0.45:
            self._mode = 'escape'
            self._escape_dir = 1.0 if d_left >= d_right else -1.0
            self._escape_timer = 18

        if self._mode == 'escape':
            if self._escape_timer <= 0:
                self._mode = 'follow_left'
            else:
                self._escape_timer -= 1
                cmd.linear.x = -0.32
                cmd.angular.z = 1.45 * self._escape_dir
                self.cmd_pub.publish(cmd)
                return
                
        # goal pocket: wide, open area
        open_threshold = min(2.0, SENSOR_RANGE * 0.75)
        if d_left > open_threshold and d_front > open_threshold and d_right > open_threshold:
            cmd.linear.x = 0.8
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return
            
        # if front is blocked, choose the open side but keep moving
        if d_front < 0.72:
            if d_left > d_right + 0.15:
                self._turn_dir = 1.0
            elif d_right > d_left + 0.15:
                self._turn_dir = -1.0

            cmd.linear.x = 0.22
            cmd.angular.z = 1.05 * self._turn_dir
            self.cmd_pub.publish(cmd)
            return

        # normal left-wall-following maze logic
        if d_left < 0.30:
            cmd.linear.x = 0.35
            cmd.angular.z = -0.8
        elif d_left > 0.85:
            cmd.linear.x = 0.35
            cmd.angular.z = 0.8
        else:
            target = 0.55
            error = d_left - target
            cmd.linear.x = 0.45
            cmd.angular.z = max(-1.0, min(1.0, error * 2.8))

        # slight bias to keep a consistent turn direction when sides are similar
        if abs(d_left - d_right) < 0.12:
            cmd.angular.z = cmd.angular.z * 0.7 + 0.25 * self._turn_dir

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StudentSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
