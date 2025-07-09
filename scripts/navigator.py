#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


class Navigator:
    def __init__(self):
        rospy.init_node("navigator")
        rospy.loginfo("Navigator node initialized.")

        # Publisher
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.lidar_marker_pub = rospy.Publisher("/lidar_rays", Marker, queue_size=1)

        # Subscriber
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        # Control state
        self.override_until = rospy.Time.now()
        self.last_obstacle_time = rospy.Time(0)
        self.goal = None
        self.pf_goal_reached = False

        # Start control loop
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"New navigation goal received: {self.goal}")

    def scan_callback(self, msg):
        if self.pf_goal_reached:
            rospy.loginfo_throttle(5.0, "Goal already reached. Not calling potential field.")
            return

        obstacle_detected = any(0.12 < r < 0.4 for r in msg.ranges)
        if obstacle_detected:
            now = rospy.Time.now()
            if (now - self.last_obstacle_time).to_sec() > 2.0:
                self.last_obstacle_time = now
                rospy.logwarn("Obstacle detected within 0.4 m.")

            try:
                rospy.wait_for_service('/potential_field_cmd', timeout=1.0)
                pf_service = rospy.ServiceProxy('/potential_field_cmd', Trigger)
                resp = pf_service()
                if resp.success:
                    if "Goal reached" in resp.message:
                        rospy.loginfo("Potential field: Goal reached. Stopping navigation.")
                        self.pf_goal_reached = True
                    else:
                        rospy.loginfo("Potential field command used.")
                    self.override_until = rospy.Time.now() + rospy.Duration(1.5)
                else:
                    rospy.logwarn("Potential field service responded with failure.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to call potential field service: {e}")

            self.publish_lidar_rays(msg)

    def control_loop(self, event):
        if rospy.Time.now() < self.override_until:
            rospy.loginfo_throttle(1.0, "In override mode, suppressing kinematic controller.")
        else:
            if self.goal is None:
                rospy.loginfo_throttle(5.0, "No active goal. Waiting for /move_base_simple/goal...")
            else:
                rospy.loginfo_throttle(5.0, f"Navigating toward goal: {self.goal}")

    def publish_lidar_rays(self, scan_msg):
        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lidar"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.pose.orientation.w = 1.0

        for i, r in enumerate(scan_msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.12 or r > 3.5:
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            start = Point(0, 0, 0)
            end = Point(r * math.cos(angle), r * math.sin(angle), 0)
            marker.points.append(start)
            marker.points.append(end)

        self.lidar_marker_pub.publish(marker)

if __name__ == "__main__":
    try:
        Navigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
