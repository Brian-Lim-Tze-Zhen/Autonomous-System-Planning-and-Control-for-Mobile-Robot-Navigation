#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf.transformations
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger, TriggerResponse


class PotentialFieldPlanner:
    def __init__(self):
        rospy.init_node("potential_field_node")
        rospy.loginfo("PotentialFieldPlanner initialized.")

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/potential_path", Path, queue_size=1)
        self.marker_pub = rospy.Publisher("/force_vectors", Marker, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        # Internal state
        self.pose = None  # (x, y, theta)
        self.latest_scan = None
        self.goal = None  # (x, y)

        # Path history for RViz
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.last_path_time = rospy.Time.now()

        # Service
        self.potential_field_srv = rospy.Service("/potential_field_cmd", Trigger, self.handle_potential_field_request)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = (pos.x, pos.y, yaw)

        # Update path for RViz
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position = pos
        pose_stamped.pose.orientation = ori
        self.path_msg.poses.append(pose_stamped)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"New goal received: {self.goal}")

    def handle_potential_field_request(self, req):
        if self.pose is None or self.latest_scan is None or self.goal is None:
            if self.goal is None:
                rospy.loginfo_throttle(5, "Waiting for next goal...")
            else:
                if self.pose is None:
                    rospy.logwarn("Missing robot pose (/odom not received yet).")
                if self.latest_scan is None:
                    rospy.logwarn("Missing LIDAR scan (/scan not received yet).")
            return TriggerResponse(success=False, message="Missing pose, scan, or goal")

        # Check goal distance BEFORE calling run_potential_field_with_goal
        x, y, _ = self.pose
        goal_x, goal_y = self.goal
        distance = math.hypot(goal_x - x, goal_y - y)

        # Allow robot to slow down and stabilize before declaring goal reached
        if distance < 0.2:
            rospy.loginfo("Goal reached. Robot stopped.")
            self.goal = None
            self.cmd_pub.publish(Twist())

            # Compute time-to-goal
            if rospy.has_param("/goal_start_time"):
                start_time = rospy.get_param("/goal_start_time")
                duration = rospy.Time.now().to_sec() - start_time
                rospy.loginfo(f"Time to goal: {duration:.2f} seconds")
                rospy.delete_param("/goal_start_time")

            rospy.loginfo("Waiting for next goal via /move_base_simple/goal...")
            return TriggerResponse(success=True, message="Goal reached.")

        # Otherwise, continue with potential field
        cmd, F_att, F_rep = self.run_potential_field_with_goal(
            robot_pose=self.pose,
            goal_pose=self.goal,
            laser_msg=self.latest_scan
        )

        self.cmd_pub.publish(cmd)
        self.publish_force_vector(F_att, F_rep)

        return TriggerResponse(success=True, message="Potential field command executed")

    def run_potential_field_with_goal(self, robot_pose, goal_pose, laser_msg,
                                      k_att=0.8, k_rep=0.4, d0=0.65,
                                      max_speed=0.1, angular_gain=1.0):

        x, y, theta = robot_pose
        goal_x, goal_y = goal_pose

        dx = goal_x - x
        dy = goal_y - y
        distance_to_goal = math.hypot(dx, dy)

        # 1. Attractive force
        F_att = k_att * np.linalg.norm([dx, dy]) * np.array([dx, dy]) / (np.linalg.norm([dx, dy]) + 1e-5)

        # 2. Repulsive force
        F_rep = np.zeros(2)
        angle = laser_msg.angle_min
        ranges = np.array(laser_msg.ranges)
        ranges = np.convolve(ranges, np.ones(5) / 5, mode='same')

        for r in ranges:
            if angle < -math.radians(100) or angle > math.radians(100):
                angle += laser_msg.angle_increment
                continue
            if np.isinf(r) or r < 0.2 or r > d0:
                angle += laser_msg.angle_increment
                continue

            # Robot local frame: repulsion points backwards along laser ray
            obs_dir_robot = -np.array([np.cos(angle), np.sin(angle)])

            # Rotate to world frame
            rot = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta), np.cos(theta)]])
            obs_dir_world = rot @ obs_dir_robot

            inv_r = 1.0 / max(r, 0.2)
            inv_d0 = 1.0 / d0
            magnitude = k_rep * ((inv_r - inv_d0) / (r ** 2))
            magnitude = min(magnitude, 1.0)

            F_rep += magnitude * obs_dir_world
            angle += laser_msg.angle_increment

        # 3. Combine forces
        F_total = F_att + F_rep
        force_norm = np.linalg.norm(F_total)
        if force_norm == 0:
            return Twist(), F_att, F_rep

        angle_to_force = math.atan2(F_total[1], F_total[0])
        heading_error = math.atan2(math.sin(angle_to_force - theta), math.cos(angle_to_force - theta))
        heading_error = max(-math.pi / 2, min(math.pi / 2, heading_error))

        # 4. Velocity command with limits
        cmd = Twist()
        raw_linear = force_norm * math.cos(heading_error)

        if distance_to_goal > 0.2:
            cmd.linear.x = max(0.03, min(raw_linear, max_speed))
        else:
            cmd.linear.x = min(raw_linear, max_speed)

        cmd.angular.z = max(-0.2, min(angular_gain * heading_error, 0.2))

        # 5. Path recording
        if (rospy.Time.now() - self.last_path_time).to_sec() > 0.3:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            self.path_msg.poses.append(pose)
            self.path_pub.publish(self.path_msg)
            self.last_path_time = rospy.Time.now()

        return cmd, F_att, F_rep

    def publish_force_vector(self, F_att, F_rep):
        if self.pose is None:
            return

        x, y, _ = self.pose

        def publish_vector(name, vec, color_rgb, marker_id):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.pose.orientation.w = 1.0
            marker.ns = name
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.scale.y = 0.04
            marker.scale.z = 0.0
            marker.color.a = 1.0
            marker.color.r = color_rgb[0]
            marker.color.g = color_rgb[1]
            marker.color.b = color_rgb[2]

            start_point = Point(x, y, 0)
            end_point = Point(x + vec[0], y + vec[1], 0)
            marker.points = [start_point, end_point]
            self.marker_pub.publish(marker)

        publish_vector("F_att", F_att, (0.0, 1.0, 0.0), 1)  # green
        publish_vector("F_rep", F_rep, (1.0, 0.0, 0.0), 2)  # red


if __name__ == "__main__":
    try:
        PotentialFieldPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
