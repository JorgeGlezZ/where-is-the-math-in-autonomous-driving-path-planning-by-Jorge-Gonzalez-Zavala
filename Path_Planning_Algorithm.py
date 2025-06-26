import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches, animation
from scipy.spatial import KDTree
from scipy.optimize import minimize
from scipy.interpolate import interp1d
from skopt import gp_minimize
from skopt.space import Real

np.random.seed(15)

# === GLOBAL GENERAL PARAMETERS ===
road_width = 10
screen_width = 30
car_x = 2                               # Constant x position of car
car_y = 5                               # Initial y position of car
goal_offset = 25                        # How far ahead the path should be planned
obstacle_size = np.array([2.5, 1.5])    # Size of the obstacles
obstacle_inflation = 1                  # How large the obstacles must be inflated to avoid collisions
lane_centers = [3, 5, 7]                # y position of the centers of the 3 lanes
lane_penalty_weight = 25                # Penalty for deviating from a lane center
scroll_speed = 2                        # Speed of car
replan_interval = 5                     # How often must the path be re-calculated

# === OBSTACLE MANAGEMENT PARAMETERS ===
obstacles = []                          # Memory of active obstacles
frame_counter = 0                       # Counter increasing every frame
max_obstacles = 4                       # Maximum number of active obstacles
# obstacle_cooldown = 0                 # Removed because deemed unnecessary from testing

# === PATH PLANNING & MOVEMENT ===
path_points = []                        # Array of y coordinates for the car to follow
path_index = 0                          # Which is the part of the path the car will follow next
current_rough_path = None
current_smooth_path = None

# === GENERATION OF OBSTACLES ===
def generate_new_obstacle(initial = False):
    max_attempts = 10

    for i in range(max_attempts):
        if initial:                     # Condition to generate the first batch of obstacles
            x = np.random.uniform(car_x + 10, screen_width - obstacle_size[0] - 1)
        else:                           # Condition to generate the following obstacle batches
            x = screen_width + np.random.uniform(0, 5)
        
        y = np.random.choice(lane_centers)
        new_pos = [x, y - obstacle_size[1] / 2]
        new_rect = patches.Rectangle(new_pos, *obstacle_size)
        
        overlap = False                 # Overlapping check (no obstacles can overlap on each other)

        for obs in obstacles:
            existing_rect = patches.Rectangle(obs['pos'], *obs['size'])
            
            if new_rect.get_bbox().overlaps(existing_rect.get_bbox()):
                overlap = True
                break
        
        if not overlap:
            return {'pos': new_pos, 'size': obstacle_size}
    
    return None                         # If too many failed attempts

# === INFLATION OF OBSTACLES ===
def inflate_obstacles(obstacles):       # Obstacles are inflated (in area) to avoid collisions when smoothing path
    return [{
        'pos': [obs['pos'][0] - obstacle_inflation / 2, obs['pos'][1] - obstacle_inflation / 2],
        'size': obstacle_size + obstacle_inflation
    } for obs in obstacles]

# === COLLISION CHECK ===
def collides(point, inflated_obstacles):# Collisions with the inflated obstacles are to be avoided in rough path
    for inflated in inflated_obstacles:
        x, y = inflated['pos']
        w, h = inflated['size']

        if x <= point[0] <= x + w and y <= point[1] <= y + h:
            return True
    
    return False

# === RRT* PATH PLANNING COMPONENTS ===
class Node:
    def __init__(self, pos):            # Each node will store:
        self.pos = np.array(pos)        # - Its position
        self.parent = None              # - Its parent node
        self.cost = 0                   # - The cumulative cost to get to it from the root node

# Calculation of distance between points a & b
def dist(a, b):
    return np.linalg.norm(a - b)

# Search for the closest node to a point
def get_nearest(nodes, point):
    tree = KDTree([node.pos for node in nodes])
    _, idx = tree.query(point)          # Gets the index of the closest node

    return nodes[idx]

# Search for all neighboring nodes to a point within a radius
def get_neighbors(nodes, point, radius = 2.5):
    tree = KDTree([node.pos for node in nodes])
    idxs = tree.query_ball_point(point, radius)

    return [nodes[i] for i in idxs]

# Growth of the tree from a node to a point
def steer(from_node, to_point, step_size = 1.5):
    direction = to_point - from_node.pos
    length = np.linalg.norm(direction)

    if length == 0:
        return from_node.pos
    
    direction = direction / length
    new_pos = from_node.pos + step_size * direction
    new_pos[1] = np.clip(new_pos[1], 2, 8)

    return new_pos

# === RRT* PATH PLANNING ALGORITHM ===
def rrt_star(start, goal, inflated_obstacles, iterations = 300):
    root_node = Node(start)             # Starting node (vehicle's position)
    nodes = [root_node]

    for _ in range(iterations):
        x_new = np.random.uniform([car_x, 2], [car_x + goal_offset, 8])

        if collides(x_new, inflated_obstacles):
            continue                    # If there is a collision, point is invalid, skip to the next one

        nearest_node = get_nearest(nodes, x_new)
        new_pos = steer(nearest_node, x_new)    # Gets the position of the new node from the branching

        if collides(new_pos, inflated_obstacles):
            continue                    # If there is a collision, node is invalid, skip to the next one

        # Information record of new node
        new_node = Node(new_pos)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + dist(nearest_node.pos, new_node.pos)

        # Penalization for the new node if it deviates much from the center of a lane
        nearest_lane = min(lane_centers, key=lambda y: abs(y - new_node.pos[1]))
        lane_error = abs(new_node.pos[1] - nearest_lane)
        new_node.cost += lane_penalty_weight * lane_error**1    # Adjust power accordingly

        # Tree rewire check
        neighbors = get_neighbors(nodes, new_node.pos)

        for neighbor in neighbors:      # Assigns a neighbor node as parent to the new node
            cost = neighbor.cost + dist(neighbor.pos, new_node.pos)
            if cost < new_node.cost and not collides(steer(neighbor, new_node.pos), inflated_obstacles):
                new_node.parent = neighbor
                new_node.cost = cost
        
        for neighbor in neighbors:      # Assigns the new node as parent of a neighbor node
            cost = new_node.cost + dist(neighbor.pos, new_node.pos)
            if cost < neighbor.cost and not collides(steer(new_node, neighbor.pos), inflated_obstacles):
                neighbor.parent = new_node
                neighbor.cost = cost
        
        nodes.append(new_node)
    
    # Destination region check
    near_goal = [node for node in nodes if abs(node.pos[0] - goal[0]) < 2.0]

    if not near_goal:                   # In case no nodes are in the destination region
        near_goal = [min(nodes, key = lambda n: dist(n.pos, goal))]
    
    best = min(near_goal, key = lambda n: n.cost)
    goal_node = Node(goal.copy())
    goal_node.parent = best
    goal_node.cost = best.cost + dist(best.pos, goal_node.pos)
    path = []
    current = goal_node

    # Rebuilding the path from the destination to the start
    while current:
        path.append(current.pos)
        current = current.parent
    path.reverse()                      # Reversing the path to be from start to goal

    return np.array(path)

# === PATH SMOOTHING COST FUNCTION ===
def cost_function(f_flat, g, ds, w1 = 1, w2 = 1, w3 = 1, w4 = 2):
    f = f_flat.reshape(-1, 2)           # Reshapes the 1D array of coordinates to a 2D array
    df = np.diff(f, axis = 0) / ds        # First derivative (smoothness)
    ddf = np.diff(df, axis = 0) / ds      # Second derivative (curvature)
    dddf = np.diff(ddf, axis = 0) / ds    # Third derivative (jerk)

    cost_smooth = np.sum(df**2) * w1        # Smoothness integral
    cost_curvature = np.sum(ddf**2) * w2    # Curvature integral
    cost_jerk = np.sum(dddf**2) * w3        # Jerk integral
    cost_distance = np.sum((f - g)**2) * w4 # Fidelity integral

    return cost_smooth + cost_curvature + cost_jerk + cost_distance

# === INCREASE IN PATH RESOLUTION ===
def densify_path(path, num_points = 50):      # Linearly interpolates between points in original path to increase their number
    if len(path) < 2:
        return path
    
    distances = np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))
    distances = np.insert(distances, 0, 0)
    interpolators = [interp1d(distances, path[:, i], kind='linear') for i in range(2)]
    new_distances = np.linspace(0, distances[-1], num_points)

    return np.vstack([interp(new_distances) for interp in interpolators]).T

# === SMOOTHING OF PATH ===
def smooth_path(rough_path):                    # Minimizes the cost function
    ds = 1.0
    g = rough_path
    f0 = g.flatten()
    result = minimize(cost_function, f0, args=(g, ds), method='L-BFGS-B')
    f = result.x.reshape(-1, 2)

    return f

# === ANIMATION SETUP ===
fig, ax = plt.subplots(figsize=(12, 6))
ax.set_xlim(0, screen_width)
ax.set_ylim(0, road_width)

road_patch = patches.Rectangle((0, 0), screen_width, road_width, color='lightgray')
car_marker, = ax.plot([], [], 'ro', markersize=10)
obstacle_patches = []

# === ANIMATION INITIALIZING ===
def init():
    car_marker.set_data([], [])
    ax.set_xticks([])
    ax.set_yticks([])

    return [car_marker]

# === ANIMATION FRAME ===
def animate(frame):
    global frame_counter, car_y, obstacles
    # global obstacle_cooldown
    global current_rough_path, current_smooth_path, path_points, path_index

    ax.clear()
    ax.set_xlim(0, screen_width)
    ax.set_ylim(0, road_width)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.add_patch(patches.Rectangle((0, 0), screen_width, road_width, color='lightgray'))

    # Draw lane boundaries
    lane_width = lane_centers[1] - lane_centers[0]
    for lane in lane_centers:
        lower = lane - lane_width / 2
        upper = lane + lane_width / 2
        ax.axhline(y=lower, color='white', linestyle='--', linewidth=1)
        ax.axhline(y=upper, color='white', linestyle='--', linewidth=1)

    # Scroll obstacles
    for obs in obstacles:
        obs['pos'][0] -= scroll_speed

    # Remove off-screen
    obstacles[:] = [obs for obs in obstacles if obs['pos'][0] + obs['size'][0] > 0]

    """
    # Occasionally add a new obstacle (with cooldown)
    if len(obstacles) < max_obstacles and np.random.rand() < 0.3 and obstacle_cooldown <= 0:
        obstacles.append(generate_new_obstacle())
        obstacle_cooldown = 10
    else:
        obstacle_cooldown -= 1
    """

    # Occasionally add a new obstacle (no cooldown)
    if len(obstacles) < max_obstacles and np.random.rand() < 0.3:
        new_obs = generate_new_obstacle()
        
        if new_obs:
            obstacles.append(new_obs)

    # Replan Path
    if frame_counter % replan_interval == 0 or path_index >= len(path_points):
        inflated = inflate_obstacles(obstacles)
        goal_pos = np.array([car_x + goal_offset, np.random.choice(lane_centers)])
        rough_path = rrt_star(np.array([car_x, car_y]), goal_pos, inflated)
        dense_path = densify_path(rough_path)
        smoothed_path = smooth_path(dense_path)
        path_points = smoothed_path
        path_index = 1

        current_rough_path = rough_path
        current_smooth_path = smoothed_path

    # Shift and draw paths
    if current_rough_path is not None and len(current_rough_path) > 1:
        current_rough_path -= np.array([scroll_speed, 0])
        ax.plot(current_rough_path[:, 0], current_rough_path[:, 1], color='orange', linewidth=1.5)

    if current_smooth_path is not None and len(current_smooth_path) > 1:
        current_smooth_path -= np.array([scroll_speed, 0])
        ax.plot(current_smooth_path[:, 0], current_smooth_path[:, 1], color='green', linewidth=2)

    # Continue following path
    for _ in range(2):
        if path_index < len(path_points):
            car_y = path_points[path_index][1]
            path_index += 1

    # Draw car
    ax.plot(car_x, car_y, 'ro', markersize=10)

    # Draw obstacles
    for obs in obstacles:
        rect = patches.Rectangle(obs['pos'], *obs['size'], color='blue')
        ax.add_patch(rect)

    inflated = inflate_obstacles(obstacles)
    for obs in inflated:
        rect = patches.Rectangle(obs['pos'], *obs['size'], color='red', alpha=0.3)
        ax.add_patch(rect)

    frame_counter += 1
    return []

for _ in range(3):
    new_obs = generate_new_obstacle(initial = True)

    if new_obs:
        obstacles.append(new_obs)

# Generate and save animation
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=100, interval=200, repeat=False, blit=False)
ani.save("dynamic_path_planning.gif", writer="pillow")