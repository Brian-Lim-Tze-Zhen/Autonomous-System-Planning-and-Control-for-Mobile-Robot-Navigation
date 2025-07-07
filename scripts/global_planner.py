#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import heapq
import math
from scipy.ndimage import grey_dilation  # ✅ NEW: For obstacle inflation

class GlobalPlanner:
    def __init__(self):
        rospy.init_node("global_planner")

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=1)

        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.goal_start_time = None  

        self.start_pose = None
        rospy.loginfo("✅ GlobalPlanner initialized.")

    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

        # ✅ Load raw data and reshape
        raw_data = np.array(msg.data).reshape((self.map_height, self.map_width))

        # ✅ Inflate obstacles (adjust kernel size as needed)
        inflated = grey_dilation(raw_data, size=(5, 5))  # 5x5 kernel = 0.25m if res=0.05
        inflated[raw_data == -1] = 100  # keep unknown areas blocked

        self.map_data = inflated
        rospy.loginfo("🛡️ Map inflated and loaded.")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.start_pose = (pos.x, pos.y)

    def goal_callback(self, msg):
        if self.map_data is None:
            rospy.logwarn("⚠️ Map not received yet.")
            return
        if self.start_pose is None:
            rospy.logwarn("⚠️ Start pose not set.")
            return

        start = self.world_to_grid(self.start_pose)
        goal = self.world_to_grid((msg.pose.position.x, msg.pose.position.y))

        path = self.a_star(start, goal)
        if path:
            self.publish_path(path)
            rospy.set_param("/goal_start_time", rospy.Time.now().to_sec())
            rospy.loginfo("🕒 Goal time recorded.")
        else:
            rospy.logwarn("❌ A* failed to find a path.")


    def world_to_grid(self, pos):
        x = int((pos[0] - self.map_origin.position.x) / self.map_resolution)
        y = int((pos[1] - self.map_origin.position.y) / self.map_resolution)
        return (x, y)

    def grid_to_world(self, grid):
        x = grid[0] * self.map_resolution + self.map_origin.position.x
        y = grid[1] * self.map_resolution + self.map_origin.position.y
        return (x, y)

    def is_occupied(self, x, y):
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return True
        return self.map_data[y, x] > 50  # threshold for occupied

    def get_neighbors(self, node):
        x, y = node
        neighbors = [
            (x+1,y), (x-1,y), (x,y+1), (x,y-1),
            (x+1,y+1), (x-1,y-1), (x+1,y-1), (x-1,y+1)
        ]
        return [n for n in neighbors if not self.is_occupied(n[0], n[1])]

    def heuristic(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
                    came_from[neighbor] = current
        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return self.downsample_path(path)

    def downsample_path(self, path, step=3):
        if len(path) <= 2:
            return path
        return path[::step] + [path[-1]]

    def publish_path(self, grid_path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for x, y in grid_path:
            wx, wy = self.grid_to_world((x, y))
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position = Point(wx, wy, 0)
            pose.pose.orientation = Quaternion(0, 0, 0, 1)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        rospy.loginfo("✅ Published path with %d waypoints." % len(grid_path))

if __name__ == "__main__":
    planner = GlobalPlanner()
    rospy.spin()
