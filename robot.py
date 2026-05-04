# Stephen Snelson Shanthosh Raaj Gabe Syzybalski
# ENPM661 - RO01 Spring 2026
# Project 5 - Competition with RRT
# Robot


import math
import time

from trajectory import Trajectory


class Robot:

    def __init__(self, rpm1 = 30, rpm2= 60, wheel_rad= 0.033, wheel_dist= 0.287, robot_rad= 0.220, action_duration = 1.0, dt = 0.1, map_start = (0.0, 1500.0, 0.0)):
        self.rpm1 = rpm1
        self.rpm2 = rpm2
        self.wheel_rad = wheel_rad
        self.wheel_dist = wheel_dist
        self.robot_rad = robot_rad
        self.action_duration = action_duration
        self.dt = dt
        self.map_start = map_start

        # ROS COmmands
        self._rclpy = None
        self._node = None
        self._cmd_pub = None
        self._odom_sub = None
        self._twist_cls = None
        self._pose = None

    @staticmethod
    def rpm2rps(rpm): #converts to rads per second
        return rpm * 2.0 * math.pi / 60.0

    @staticmethod
    def norm_angle(theta): #keeps angles clean and within -pi to pi
        return math.atan2(math.sin(theta), math.cos(theta))

    @staticmethod
    def heuristic(x, y, goal_x, goal_y): #straight line distance between points
        return math.hypot(goal_x - x, goal_y - y)

    @staticmethod
    def bound_limit(value, low, high): #limits input values to prevent rapid velocity changes
        return max(low, min(high, value))

    def action_set(self): #sets action sets
        r1 = self.rpm1
        r2 = self.rpm2
        return [
            (0, r1), #left 
            (r1, 0),#right
            (r1, r1),#straight
            (0, r2), #big left
            (r2, 0), #bif right
            (r2, r2), #big straight
            (r1, r2),#slight left
            (r2, r1), #slight right
        ]

    def sim_action(self, state, action): #simulates next state after wheel action for a short time
        x, y, theta = state
        l_rpm, r_rpm = action

        rad_l = self.rpm2rps(l_rpm)
        rad_r = self.rpm2rps(r_rpm)

        t = 0.0
        cost = 0.0
        path = [(x, y, theta)]
        start_state = (x, y, theta)

        while t < self.action_duration:
            x_dot = 0.5 * self.wheel_rad * (rad_l + rad_r) * math.cos(theta)
            y_dot = 0.5 * self.wheel_rad * (rad_l + rad_r) * math.sin(theta)
            theta_dot = (self.wheel_rad / self.wheel_dist) * (rad_r - rad_l)

            x_new = x + x_dot * self.dt
            y_new = y + y_dot * self.dt
            theta_new = self.norm_angle(theta + theta_dot * self.dt)

            cost += math.hypot(x_new - x, y_new - y)

            x, y, theta = x_new, y_new, theta_new
            path.append((x, y,theta))
            t += self.dt

        end_state = (x, y, theta)
        return Trajectory(start_point=start_state, end_point=end_state, trajectory=path, cost=cost)

    def mm_to_odom_m(self, point_mm): #switches from map millimeters to onboard robot meters

        x_mm, y_mm = point_mm
        start_x_mm, start_y_mm, _ = self.map_start
        return ((x_mm - start_x_mm) / 1000.0, (y_mm - start_y_mm) / 1000.0)

    def yaw_from_wxyz(self, q): #switches from ros w, z, x, y to theta
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _init_ros(self):
        try:
            import rclpy
            from geometry_msgs.msg import TwistStamped
            from nav_msgs.msg import Odometry
        except ImportError as exc:
            raise ImportError(
                "ROS 2 imports failed. Run this inside a sourced ROS 2 workspace, e.g.:\n"
                "source /opt/ros/humble/setup.bash\n"
                "source ~/your_ws/install/setup.bash"
            ) from exc

        if not rclpy.ok():
            rclpy.init()

        self._rclpy = rclpy
        self._twist_cls = TwistStamped
        self._node = rclpy.create_node("waypoint_follower")

        self._cmd_pub = self._node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self._odom_sub = self._node.create_subscription(Odometry, "/odometry/filtered", self.current_pose, 10)

    def current_pose(self, msg) : #stores current pose
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = self.yaw_from_wxyz(q)
        self._pose = (p.x, p.y, theta)

    def _publish_cmd(self, linear_x, angular_z):
        msg = self._twist_cls()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z

        self._cmd_pub.publish(msg)

    def stop(self): #stops robot
        if self._cmd_pub is not None:
            self._publish_cmd(0.0, 0.0)

    def follow_waypoints(self,waypoints_mm, linear_speed_max = 0.15,angular_speed_max=1.5,linear_gain = 0.8,angular_gain = 2.0,distance_tolerance = 0.06,heading_slowdown_angle = 0.6,rate_hz = 20.0,skip_first_waypoint = True,):
        self._init_ros() # feeds rrt path into odometery meters, then sends the turtlebot to each waypoint one at a time

        waypoints = list(waypoints_mm)
        if skip_first_waypoint and len(waypoints) > 1: #first waypoint is the start position so we're skipping that for now
            waypoints = waypoints[1:]

        odom_targets = [self.mm_to_odom_m(point) for point in waypoints] #sets points to odometer-friendly targets

        self._node.get_logger().info(f"Loaded {len(odom_targets)} waypoints.") #prints that targets are ready

        while self._rclpy.ok() and self._pose is None:
            self._node.get_logger().info("waiting for /odometry/filtered from turtlebot")
            self._rclpy.spin_once(self._node, timeout_sec=0.5)

        dt = 1.0 / rate_hz

        try:
            for target_index, (target_x, target_y) in enumerate(odom_targets, start=1):
                self._node.get_logger().info(
                    f"Going to waypoint {target_index}/{len(odom_targets)}: "
                    f"x={target_x:.3f}, y={target_y:.3f}"
                )

                while self._rclpy.ok():
                    self._rclpy.spin_once(self._node, timeout_sec=0.05)

                    x, y, theta = self._pose
                    dx = target_x - x
                    dy = target_y - y
                    distance = math.hypot(dx, dy)

                    if distance <= distance_tolerance:
                        self.stop()
                        break

                    target_heading = math.atan2(dy, dx)
                    heading_error = self.norm_angle(target_heading - theta)

                    angular_z = self.bound_limit(angular_gain * heading_error,-angular_speed_max,angular_speed_max,)

                    # Rotate in place if pointed too far away from target
                    if abs(heading_error) > heading_slowdown_angle:
                        linear_x = 0.0
                    else:
                        linear_x = self.bound_limit(linear_gain * distance,0.0,linear_speed_max)

                    self._publish_cmd(linear_x, angular_z)
                    time.sleep(dt)
                    

            self.stop()
            self._node.get_logger().info("Finished waypoint path.")

        finally:
            self.stop()
