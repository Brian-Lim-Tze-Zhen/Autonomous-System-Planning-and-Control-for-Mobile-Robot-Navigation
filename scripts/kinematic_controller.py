#!/usr/bin/env python3

import rospy
import math
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


class KinematicController:
    def __init__(self):
        rospy.init_node("kinematic_controller")
        rospy.loginfo("Kinematic controller node started.")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/planned_path", Path, self.path_callback)

        self.pose = None
        self.path = []
        self.epsilon = 0.1

        # Control gains (adjust as needed)
        self.k_rho = 0.5
        self.k_alpha = 1.0
        self.k_beta = -0.3

        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        rospy.loginfo("Path received: %d waypoints." % len(self.path))

        self.total_distance = self.compute_total_distance(self.path)
        rospy.loginfo("Total path length: %.2f meters." % self.total_distance)
        rospy.set_param("/goal_start_time", rospy.Time.now().to_sec())

    def compute_total_distance(self, path):
        distance = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            distance += math.hypot(dx, dy)
        return distance

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = (pos.x, pos.y, yaw)

    def control_loop(self, event):
        if self.pose is None or not self.path:
            return

        x, y, theta = self.pose
        xg, yg = self.path[0]

        dx = xg - x
        dy = yg - y
        rho = math.hypot(dx, dy)
        alpha = math.atan2(dy, dx) - theta
        beta = -theta - alpha

        # Normalize angles
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        beta = math.atan2(math.sin(beta), math.cos(beta))

        # Switch to next waypoint if close enough
        if rho < self.epsilon:
            self.path.pop(0)
            if not self.path:
                rospy.loginfo("Final goal reached.")
                self.cmd_pub.publish(Twist())

                # Report elapsed time to goal
                if rospy.has_param("/goal_start_time"):
                    start_time = rospy.get_param("/goal_start_time")
                    elapsed = rospy.Time.now().to_sec() - start_time
                    avg_speed = self.total_distance / elapsed if elapsed > 0 else 0.0
                    rospy.loginfo(f"[Kinematic] Time to goal: {elapsed:.2f} seconds")
                    rospy.loginfo(f"[Kinematic] Path length: {self.total_distance:.2f} m")
                    rospy.loginfo(f"[Kinematic] Average speed: {avg_speed:.2f} m/s")
                else:
                    rospy.logwarn("No goal start time found in /goal_start_time param")
            else:
                rospy.loginfo("Waypoint reached. %d left." % len(self.path))
            return

        # Compute and publish command
        cmd = Twist()
        cmd.linear.x = self.k_rho * rho
        cmd.angular.z = self.k_alpha * alpha + self.k_beta * beta
        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    try:
        KinematicController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
