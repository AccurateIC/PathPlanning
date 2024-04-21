import math
import numpy as np
import matplotlib.pyplot as plt
from dstar import PathPlanner
from environment import Environment

def get_path_from_numpy_array(array, start_value, end_value, obstacle_value, movement='king'):
    if movement == 'king':
        moves_xyc = [[0, 1, 1], [0, -1, 1], [1, 0, 1], [-1, 0, 1], [1, 1, math.sqrt(2)], [-1, -1, math.sqrt(2)], [1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)]]
    elif movement == 'rook':
        moves_xyc = [[0, 1, 1], [0, -1, 1], [1, 0, 1], [-1, 0, 1]]
    elif movement == 'bishop':
        moves_xyc = [[1, 1, math.sqrt(2)], [1, -1, math.sqrt(2)], [1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)]]
    elif movement == 'knight':
        moves_xyc = [[2, 1, 1 + math.sqrt(2)], [2, -1, 1 + math.sqrt(2)], [-2, 1, 1 + math.sqrt(2)], [-2, -1, 1 + math.sqrt(2)], [1, 2, 1 + math.sqrt(2)], [-1, 2, 1 + math.sqrt(2)], [1, -2, 1 + math.sqrt(2)], [-1, -2, 1 + math.sqrt(2)]]

    grid_h, grid_w = array.shape
    start_y, start_x = np.where(array == start_value)
    start_y, start_x = start_y.tolist()[0], start_x.tolist()[0]
    robot_x, robot_y = start_x, start_y
    robot_dx, robot_dy = 1, 0
    end_y, end_x = np.where(array == end_value)
    end_y, end_x = end_y.tolist()[0], end_x.tolist()[0]
    obstacles_y, obstacles_x = np.where(array == obstacle_value)
    obstacles_y, obstacles_x = obstacles_y.tolist(), obstacles_x.tolist()
    obstacles_dx, obstacles_dy = [0 for _ in obstacles_x], [0 for _ in obstacles_y]

    environment = Environment(grid_h, grid_w)
    environment.put_start(start_x, start_y)
    environment.put_robot(robot_x, robot_y, robot_dx, robot_dy)
    environment.put_end(end_x, end_y)
    environment.put_multiple_obstacles(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy)

    planner = PathPlanner(environment, moves_xyc, 100, 0)
    planner.calculate_cost_and_heuristics()
    planner.add_repulsion_penalty_from_numpy_array(array, start_value, end_value, obstacle_value)
    path = planner.raw_path_finder()
    path = planner.remove_knots_from_path(path)
    environment.plot_environment(path)
    return path

if __name__ == '__main__':
    array = np.load('og.npy')
    # unique_values, counts = np.unique(array, return_counts=True)
    # print(unique_values, counts)
    # plt.imshow(array, cmap='flag')
    # plt.show()
    path = get_path_from_numpy_array(array, 1, 5, 100)
    print(path)
