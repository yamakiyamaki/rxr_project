#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
from nav2_simple_commander.robot_navigator import BasicNavigator


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__("path_planner_node")
        self.basic_navigator = BasicNavigator()  # Can be uncommented to get Global Costmap in create_plan_cb
        
        # Creating a new service "create_plan", which is called by our Nav2 C++ planner plugin
        # to receive a plan from us.
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

    # def create_plan_cb(self, request, response):
    #     # Getting all the information to plan the path
    #     goal_pose = request.goal
    #     start_pose = request.start
    #     time_now = self.get_clock().now().to_msg()
    #     # global_costmap = self.basic_navigator.getGlobalCostmap()  # Can be uncommented to get Global CostMap

    #     response.path = create_straight_plan(start_pose, goal_pose, time_now)
    #     return response

    def create_plan_cb(self, request, response):
        goal_pose = request.goal
        start_pose = request.start
        time_now = self.get_clock().now().to_msg()

        # Fetch the global costmap
        try:
            global_costmap = self.basic_navigator.getGlobalCostmap()
            # self.get_logger().info(f"Global Costmap: {type(global_costmap)}")
        except Exception as e:
            self.get_logger().error(f"Failed to get global costmap: {str(e)}")
            return response

        # Generate the path using a path planning algorithm like A*
        response.path = create_obstacle_avoiding_plan(start_pose, goal_pose, global_costmap, time_now)
        return response

# def create_straight_plan(start, goal, time_now):
#     """ 
#     Creates a straight plan between start and goal points.
#     Does not use the global costmap to plan around obstacles, as normal planners would.
#     """
#     path = Path()

#     # Set the frame_id and stamp for the Path header. Use frame_id from the goal header,
#     #  and time_now for the current time.
#     path.header.frame_id = goal.header.frame_id
#     path.header.stamp = time_now

#     # Let's create a straight plan between our start and goal poses.
#     # It is not enough if we provide only the start and end positions as a path.
#     # For controller to follow path correctly, we will need to provide also
#     # points along this straight path with small intervals. There is a function
#     # "interpolate_coordinates" implemented for you that does this. It only needs
#     # the coordinates in a tuple format, for example:
#     # interpolate_coordinates((0, 0), (0, 0.5))
#     # This will give you coordinates between these two points with 0.1 interval:
#     # [(0.0, 0.0), (0.0, 0.1), (0.0, 0.2), (0.0, 0.3), (0.0, 0.4), (0.0, 0.5)]
#     # Interpolate the coordinates between start and goal positions
#     interpolated_coordinates = interpolate_coordinates(
#         (start.pose.position.x, start.pose.position.y),
#         (goal.pose.position.x, goal.pose.position.y),
#     )
#     # node.get_logger().debug('My log message %d' % (4))
#     # Loop through these interpolated coordinates and create a new PoseStamped()
#     #  message for each of them. You can set the same stamp and frame_id as for the Path().
#     #  Finally, add all of these points into the path.poses -array.
#     print(interpolated_coordinates)
#     for point in interpolated_coordinates:
#         pose = PoseStamped()
#         pose.pose.position.x = point[0]
#         pose.pose.position.y = point[1]
#         pose.header.stamp = time_now
#         pose.header.frame_id = goal.header.frame_id
#         path.poses.append(pose)

#     return path
def create_obstacle_avoiding_plan(start, goal, costmap, time_now):
    path = Path()
    path.header.frame_id = goal.header.frame_id
    path.header.stamp = time_now

    # Extract metadata (resolution, dimensions, origin)
    map_info = costmap.metadata
    resolution = map_info.resolution
    width = map_info.size_x
    height = map_info.size_y
    origin = map_info.origin

    # Convert costmap data to a 2D numpy array
    costmap_data = np.array(costmap.data).reshape(height, width)

    # Convert start and goal positions to grid indices
    start_idx = world_to_map(start.pose.position.x, start.pose.position.y, map_info)
    goal_idx = world_to_map(goal.pose.position.x, goal.pose.position.y, map_info)

    # Perform A* algorithm to find the path
    path_indices = a_star(costmap_data, start_idx, goal_idx)

    # Convert the path indices back to world coordinates
    for idx in path_indices:
        x, y = map_to_world(idx[0], idx[1], map_info)
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.header.stamp = time_now
        pose.header.frame_id = goal.header.frame_id
        path.poses.append(pose)

    return path

def interpolate_coordinates(start, end, increment=0.1):
    """
    Interpolate coordinates between two points with a fixed increment.
    This method calculates the coordinates of the points on the straight-line path that we are computing.
    
    Args:
        start (tuple): Starting coordinate (x1, y1).
        end (tuple): Ending coordinate (x2, y2).
        increment (float): Distance between interpolated points.

    Returns:
        list: List of interpolated points as (x, y) tuples.
    """
    x1, y1 = start
    x2, y2 = end

    # Calculate total distance using the Euclidean formula
    dx = x2 - x1
    dy = y2 - y1
    distance = (dx ** 2 + dy ** 2) ** 0.5

    # Calculate the number of steps
    num_steps = int(distance / increment)

    # Generate interpolated points
    points = []
    for i in range(num_steps + 1):  # +1 to include the end point
        t = i / num_steps  # Normalized step (0.0 to 1.0)
        x = x1 + t * dx  # Linear interpolation for x
        y = y1 + t * dy  # Linear interpolation for y
        points.append((x, y))
    print("Hello")
    # Node.get_logger().debug('My log %d'%(4))
    return points

def world_to_map(x, y, metadata):
    """
    Convert world coordinates to map indices.
    """
    mx = int((x - metadata.origin.position.x) / metadata.resolution)
    my = int((y - metadata.origin.position.y) / metadata.resolution)
    return mx, my

def map_to_world(mx, my, metadata):
    """
    Convert map indices to world coordinates.
    """
    x = mx * metadata.resolution + metadata.origin.position.x
    y = my * metadata.resolution + metadata.origin.position.y
    return x, y



def a_star(costmap, start, goal):
    """
    Perform A* path planning on a 2D costmap.

    Args:
        costmap: 2D numpy array representing the costmap.
        start: Tuple (x, y) representing the start index.
        goal: Tuple (x, y) representing the goal index.

    Returns:
        List of tuples [(x1, y1), (x2, y2), ...] representing the path.
    """
    height, width = costmap.shape
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        neighbors = get_neighbors(current, height, width)
        for neighbor in neighbors:
            tentative_g_score = g_score[current] + costmap[neighbor[1]][neighbor[0]]

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # Return an empty path if no path is found

def heuristic(pos1, pos2):
    """
    Heuristic function for A* (Euclidean distance).
    """
    return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5

def get_neighbors(pos, height, width):
    """
    Get valid neighbors of a grid cell.
    """
    x, y = pos
    neighbors = [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
    return [(nx, ny) for nx, ny in neighbors if 0 <= nx < width and 0 <= ny < height]

def reconstruct_path(came_from, current):
    """
    Reconstruct the path from the `came_from` dictionary.
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]

def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()
    
    try:
        print("start")
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass

    path_planner_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    
    main()
