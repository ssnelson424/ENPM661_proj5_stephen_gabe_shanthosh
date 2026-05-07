# Stephen Snelson Shanthosh Raaj Gabe Syzybalski
# ENPM661 - RO01 Spring 2026
# Project 5 - Competition with RRT
# Robot


import math
import time
from collections import deque
from trajectory import Trajectory
from map import Map


class Robot:

    def __init__(self,  map:Map, rpm1 = 30, rpm2= 60, wheel_rad= 0.033, wheel_dist= 0.287, robot_rad= 0.220, action_duration = 1.0, dt = 0.1, map_start = (0.0, 1500.0, 0.0),):
        self._map = map
        self.rpm1 = rpm1
        self.rpm2 = rpm2
        self.wheel_rad = wheel_rad
        self.wheel_dist = wheel_dist
        self.robot_rad = robot_rad
        self.action_duration = action_duration
        self.dt = dt
        self.map_start = self._map.start
        

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

    def follow_waypoints(self,waypoints_mm,linear_speed_max=0.15,angular_speed_max=1.5,linear_gain=0.8,angular_gain=2.0,distance_tolerance=0.06,heading_slowdown_angle=0.6,rate_hz=20.0,skip_first_waypoint=True):
        self._init_ros()
        

        waypoints = list(self.create_filleted_path(waypoints_mm))

        odom_targets = [(segment, self.mm_to_odom_m(point_mm))for segment, point_mm in waypoints]

        self._node.get_logger().info(f"Loaded {len(odom_targets)} waypoints.")

        while self._rclpy.ok() and self._pose is None:
            self._node.get_logger().info("waiting for /odometry/filtered from turtlebot")
            self._rclpy.spin_once(self._node, timeout_sec=1)

        dt = 1.0 / rate_hz

        try:
            for target_index, (segment, (target_x, target_y)) in enumerate(odom_targets, start=1):
                self._node.get_logger().info(
                    f"Going to waypoint {target_index}/{len(odom_targets)}: "
                    f"segment={segment}, x={target_x:.3f}, y={target_y:.3f}"
                )

                while self._rclpy.ok():
                    self._rclpy.spin_once(self._node, timeout_sec=0.05)

                    x, y, theta = self._pose

                    dx = target_x - x
                    dy = target_y - y
                    distance = math.hypot(dx, dy)

                    if distance <= distance_tolerance:
                        break

                    if segment == "right":
                        linear_x = 0.5 * self.wheel_rad * (
                            self.rpm2rps(self.rpm1) + self.rpm2rps(self.rpm2)
                        )
                        angular_z = (self.wheel_rad / self.wheel_dist) * (
                            self.rpm2rps(self.rpm2) - self.rpm2rps(self.rpm1)
                        )

                    elif segment == "left":
                        linear_x = 0.5 * self.wheel_rad * (
                            self.rpm2rps(self.rpm2) + self.rpm2rps(self.rpm1)
                        )
                        angular_z = (self.wheel_rad / self.wheel_dist) * (
                            self.rpm2rps(self.rpm1) - self.rpm2rps(self.rpm2)
                        )

                    else:
                        target_heading = math.atan2(dy, dx)
                        heading_error = self.norm_angle(target_heading - theta)

                        angular_z = self.bound_limit(
                            angular_gain * heading_error,
                            -angular_speed_max,
                            angular_speed_max,
                        )

                        if abs(heading_error) > heading_slowdown_angle:
                            linear_x = 0.0
                        else:
                            linear_x = self.bound_limit(
                                linear_gain * distance,
                                0.0,
                                linear_speed_max,
                            )

                    self._publish_cmd(linear_x, angular_z)
                    time.sleep(dt)

            self.stop()
            self._node.get_logger().info("Finished waypoint path.")

        finally:
            self.stop()
            
    def prune_short_segments(self,path:deque[tuple[float,float]]) -> deque[tuple[float,float]]:
        """Analysis the provided waypoints to find any line segments which are two short to be trimmed.
        
            args:
                path(deque): Path of (x,y) waypoints created by the planning algorithm
            returns:
                path(deque): With x,y waypoitns with small waypoints trimmed out
                """
        
        turn_radius = self.wheel_dist / 2 * (self.rpm2 + self.rpm1) / (self.rpm2 - self.rpm1) * 1000
        points = list(path.copy())        
        changes_made = True
        
        while changes_made:
            wp_dists = []
            wp_angles = []
            wp_trims = [0]
            
            #calculates the distances of each segment
            for i in range(1,len(points)):
                wp_dists.append(self.get_distance(points[i-1],points[i]))
            
            #calculate the angles at each waypoint
            for i in range(1,len(points)-1):
                wp_angles.append(self.get_angle(points[i-1],points[i],points[i+1]))
            
            #calculate the trim distance
            for i in range(0,len(wp_angles)):
                trim_distance = turn_radius * math.tan(wp_angles[i]/2)
                wp_trims.append(trim_distance)
            
            #last point has no trim
            wp_trims.append(0)
            
            for i in range(1,len(points)-2):
                
                if abs(wp_dists[i]) < abs(wp_trims[i])+abs(wp_trims[i+1]):
                    new_point = self.get_intersection(points[i-1],points[i],points[i+1],points[i+2])
                    points = points[:i] + [new_point] + points[i+2:]
                    break
            else:
                changes_made = False
        
        pruned_path = deque(points)
        return pruned_path
    
    # def generate_arc_points(self, entry, corner, exit, radius, num_points=5):
    #     # Direction entering the corner: entry -> corner
    #     d1 = (corner[0] - entry[0], corner[1] - entry[1])
    #     # Direction leaving the corner: corner -> exit
    #     d2 = (exit[0] - corner[0], exit[1] - corner[1])

    #     l1 = math.hypot(d1[0], d1[1])
    #     l2 = math.hypot(d2[0], d2[1])

    #     if l1 == 0 or l2 == 0:
    #         return [entry, exit]

    #     u1 = (d1[0] / l1, d1[1] / l1)
    #     u2 = (d2[0] / l2, d2[1] / l2)

    #     cross = u1[0] * u2[1] - u1[1] * u2[0]
    #     side = 1 if cross > 0 else -1

    #     if abs(cross) < 1e-9:
    #         return [entry, exit]

    #     # Left turn uses left normals; right turn uses right normals
    #     if cross > 0:
    #         n1 = (-u1[1], u1[0])
    #         n2 = (-u2[1], u2[0])
    #     else:
    #         n1 = (u1[1], -u1[0])
    #         n2 = (u2[1], -u2[0])

    #     c1 = (entry[0] + radius * n1[0], entry[1] + radius * n1[1])
    #     c2 = (exit[0] + radius * n2[0], exit[1] + radius * n2[1])

    #     # Average the two center estimates to reduce numeric noise
    #     center = (
    #         0.5 * (c1[0] + c2[0]),
    #         0.5 * (c1[1] + c2[1]),
    #     )

    #     start_angle = math.atan2(entry[1] - center[1], entry[0] - center[0])
    #     end_angle = math.atan2(exit[1] - center[1], exit[0] - center[0])

    #     if side > 0 and end_angle < start_angle:
    #         end_angle += 2 * math.pi
    #     elif side < 0 and end_angle > start_angle:
    #         end_angle -= 2 * math.pi

    #     path = []
    #     for i in range(num_points):
    #         a = start_angle + (end_angle - start_angle) * i / (num_points)
    #         path.append((center[0] + radius * math.cos(a),center[1] + radius * math.sin(a)))

    #     return path

    
    def create_filleted_path(self, raw_path: deque[tuple[float,float]]) -> deque[tuple[float,float]]:
        """Creates a path with edges filleted for robot to turn

        Args:
            raw_path (deque[tuple[float,float]]): raw unfilled path from path planner

        Returns:
            deque[tuple[float,float]]: filleted path
        """
        
        #remove segments too short to be filleted
        path = self.prune_short_segments(raw_path)
        points = list(path)
        
        #calc turn radius
        turn_radius = self.wheel_dist / 2 * (self.rpm2 + self.rpm1) / (self.rpm2 - self.rpm1) * 1000
        
        #create filleted path queue and add start node (wont be filleted)
        fillet_path = deque()
        fillet_path.append(("straight",points[0]))
        #fillet_path.append(points[0])
        
        #iterate through each waypoint given
        for i in range(1, len(points) - 1):

            node1 = points[i - 1]
            node2 = points[i]
            node3 = points[i + 1]

            #calculate total distance of each adjoining segment
            dist1 = self.get_distance(node1, node2)
            dist2 = self.get_distance(node2, node3)

            #calculate the angle between the adjoining segments, then determine how much trim is needed
            turn_angle = self.get_angle(node1, node2, node3)
            trim_distance = turn_radius * math.tan(turn_angle / 2)

            #get step determination
            t1 = trim_distance / dist1
            t2 = trim_distance / dist2
            
            #calculate new turn entry/exit points
            entry = (node2[0] + t1 * (node2[0] - node1[0]),node2[1] + t1 * (node2[1] - node1[1]))
            exit = (node2[0] - t2 * (node3[0] - node2[0]),node2[1] - t2 * (node3[1] - node2[1]))

            #determine turn direction
            dx1 = node2[0] - node1[0]
            dy1 = node2[1] - node1[1]

            dx2 = node3[0] - node2[0]
            dy2 = node3[1] - node2[1]

            cross = dx1 * dy2 - dy1 * dx2

            if cross > 0:
                direction = "left"
            elif cross < 0:
                direction = "right"
            else:
                direction = "straight"
            
            fillet_path.append(("straight",entry))
            fillet_path.append((direction,exit))
            #fillet_path.append(entry)
            #fillet_path.append(exit)
            
        
        #add last node to filleted_path queue
        fillet_path.append(("straight",points[-1]))
        #fillet_path.append(points[-1])
        return fillet_path
            
            
    def get_angle(self, first:tuple[float,float], middle:tuple[float,float],last:tuple[float,float])->float:
        """determines the angle made by the line segments leading from first-middle and middle-last
        args:
            first(tuple[float,float]): x,y - coords of the first point
            middle(tuple[float,float]): x,y,-coords of the middle/turn around point
            last(tuple[float,float]):x,y-coords of the end point
        
        return:
            float: the angle made by the two line segments in radians
        """
        #unpack tuples
        x1, y1 = first
        x2, y2 = middle
        x3,y3 = last
        
        #compute difference in x and y of two line segments
        dx1 = x2-x1
        dy1 = y2-y1        
        dx2 = x3-x2
        dy2 = y3-y2
        
        #dot product of two line segments
        dot = dx1*dx2 + dy1*dy2
        mag1 = math.sqrt(dx1**2 + dy1**2)
        mag2 = math.sqrt(dx2**2 + dy2**2)
        return math.acos(dot / (mag1*mag2))
                
            
    def get_distance(self,first:tuple[float,float],second:tuple[float,float])->float:
        """gets the distance between two coordinates

        Args:
            first (tuple[float,float]): first coordinates (x,y)
            second (tuple[float,float]): second coordinates (x,y)

        Returns:
            float: distance between coords in mm
        """
        x1,y1 = first
        x2,y2 = second
        dx = x2-x1
        dy = y2-y1        
        return math.sqrt(dx**2 + dy**2)            
    
    def get_intersection(self, point_1:tuple[float,float], point_2:tuple[float,float], point_3:tuple[float,float], point_4:tuple[float,float]) -> tuple[float,float]:
        """Finds the intersetion of the 1-2 line segment and 3-4 line segment assuming 2-3 is too small

        Args:
            point_1 (tuple[float,float]): x,y-cords of first point
            point_2 (tuple[float,float]): x,y-cords of second point
            point_3 (tuple[float,float]): x,y-cords of third point
            point_4 (tuple[float,float]): x,y-cords of ffourth point

        Returns:
            tuple[float,float]: _description_
        """
        x1, y1 = point_1
        x2, y2 = point_2
        x3, y3 = point_3
        x4, y4 = point_4

        denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)

        px = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denom
        py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denom

        return (px, py)#

