import numpy as np
from Path_Planning_Algorithm import generate_new_obstacle, inflate_obstacles
from Path_Planning_Algorithm import rrt_star, densify_path, cost_function
from scipy.optimize import minimize
from skopt import gp_minimize
from skopt.space import Real

# === GLOBAL GENERAL PARAMETERS ===
car_x = 2
car_y = 5
goal_offset = 25
lane_centers = [3, 5, 7]

# === AUTOMATIC WEIGHT ASSIGNMENT FOR PATH SMOOTHING COST FUNCTION ===
def evaluate_weights(weights):
    w1, w2, w3, w4 = weights

    # Static set of obstacles for consistency
    static_obstacles = [generate_new_obstacle(initial= True) for _ in range(4)]
    inflated = inflate_obstacles(static_obstacles)

    start = np.array([car_x, car_y])
    goal = np.array([car_x + goal_offset, np.random.choice(lane_centers)])

    rough_path = rrt_star(start, goal, inflated)
    dense_path = densify_path(rough_path)

    # Penalization on failed paths
    if len(dense_path) < 2:
        return 1000
    
    def custom_cost(f_flat):
        return cost_function(f_flat, dense_path, ds = 1, w1 = w1, w2 = w2, w3 = w3, w4 = w4)
    
    f0 = dense_path.flatten()
    result = minimize(custom_cost, f0, method = 'L-BFGS-B')
    smoothed_path = result.x.reshape(-1, 2)

    # Penalization on deviation
    deviation = np.mean(np.linalg.norm(smoothed_path - dense_path, axis = 1))

    # Penalization on smoothness & curvature
    ddf = np.diff(np.diff(smoothed_path, axis = 0), axis = 0)
    curvature = np.sum(np.linalg.norm(ddf, axis = 1))

    # Penalization on lane center deviation
    lane_penalty = 0
    for pt in smoothed_path:
        nearest_lane = min(lane_centers, key = lambda y: abs(y - pt[1]))
        lane_penalty += abs(pt[1] - nearest_lane)
    
    score = deviation + .1 * curvature + .5 * lane_penalty

    return score

weights_space = [
    Real(.1, 10, name = 'w1'),          # Smoothness
    Real(.1, 10, name = 'w2'),          # Curvature
    Real(.1, 10, name = 'w3'),          # Jerk
    Real(.1, 10, name = 'w4')           # Fidelity
]

result = gp_minimize(
    func = evaluate_weights,
    dimensions = weights_space,
    acq_func = 'EI',                    # Expected Improvement
    n_calls = 30,                       # Number of evaluations
    random_state = 3                    # Random Seed
)

print("Best score: ", result.fun)
print("Best weights: ")

for name, val in zip(['w1', 'w2', 'w3', 'w4'], result.x):
    print(f"{name} = {val:.3f}")


# Results with Random Seed = 42:
"""
Score = 5.817

w1 = 9.392
w2 = 0.108
w3 = 9.923
w4 = 6.213

INTERPRETATION:

Smoothness: Highly prioritized, comfortable transitions
Curvature: Not critical, sharp turns acceptable
Jerk: Highly prioritized, sudden curvature changes are heavily penalized
Fidelity: Moderately prioritized, value in staying close, but not above others
"""

# Results with Random Seed = 23:
"""
Score = 5.708

w1 = 0.101
w2 = 9.762
w3 = 4.249
w4 = 6.847

INTERPRETATION:

Smoothness: Not critical, abrupt shifts are fine
Curvature: Highly prioritized, sharp turns avoidance
Jerk: Mildly prioritized, sudden curvature changes not preferred
Fidelity: Moderately prioritized, value in staying close, but not above others
"""

# Results with Random Seed = 3:
"""
Score = 5.746

w1 = 2.234
w2 = 3.604
w3 = 4.977
w4 = 9.142

INTERPRETATION:

Smoothness: Not critical, abrupt shifts are fine but no jitter
Curvature: Mildly prioritized, sharp turns not preferred
Jerk: Mildly prioritized, sudden curvature changes not preferred
Fidelity: Highly prioritized, original path adherence is crucial
"""