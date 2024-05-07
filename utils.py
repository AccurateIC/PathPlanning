import time
import numpy as np
import matplotlib.pyplot as plt
from dstar import PathPlanner
from environment import Environment

def get_path_from_numpy_array(array, robot_value, end_value, obstacle_value, movement='queen', k_factor=0.5):
    grid_h, grid_w = array.shape
    robot_y, robot_x = np.where(array == robot_value)
    robot_y, robot_x = robot_y.tolist()[0], robot_x.tolist()[0]
    robot_dx, robot_dy = 1, 0
    end_y, end_x = np.where(array == end_value)
    end_y, end_x = end_y.tolist()[0], end_x.tolist()[0]
    end_dx, end_dy = 1, 0
    obstacles_y, obstacles_x = np.where(array == obstacle_value)
    obstacles_y, obstacles_x = obstacles_y.tolist(), obstacles_x.tolist()
    obstacles_dx, obstacles_dy = [0 for _ in obstacles_x], [0 for _ in obstacles_y]

    env = Environment(grid_h, grid_w)
    env.put_robot(robot_x, robot_y, robot_dx, robot_dy)
    env.put_end(end_x, end_y, end_dx, end_dy)
    env.put_obstacles(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy, [3 for _ in obstacles_x])
    env.plot_environment()

    planner = PathPlanner(env, 500, 0)
    s = time.time()
    planner.calculate_cost_and_heuristics_from_end_to_start(movement, k_factor)
    # planner.add_repulsion_penalty_from_numpy_array(array, robot_value, end_value, obstacle_value)
    e = time.time()
    print(f'COST AND HEURISTIC TIME: {e - s}')
    env.plot_environment()
    path = planner.raw_path_finder(movement)
    env.plot_environment(path)
    # # s = time.time()
    # environment.plot_environment([])
    # e = time.time()
    # print(f'ADD REPULSION TIME: {e - s}')
    # s = time.time()
    # paths = planner.raw_paths_finder([0])
    # e = time.time()
    # print(f'RAW PATHS TIME: {e - s}')
    # environment.plot_environment(paths)
    # s = time.time()
    # path_costs = {}
    # for key in paths:
    #     paths[key] = planner.remove_knots_from_path(paths[key])
    #     path_costs[key] = planner.calculate_path_cost(paths[key])
        # if not planner.does_path_hit_obstacles(paths[key]):
        # print('------------------------------------------------------------------------------------------------')
    # print('PATHS:', paths)
    # e = time.time()
    # print(f'path cost TIME: {e - s}')
    # # environment.plot_environment(paths)
    # print('PATH COSTS:', path_costs)
    # s = time.time()
    # best_path = paths[list(path_costs.values()).index(min(list(path_costs.values())))]
    # e = time.time()
    # print(f'BEST PATH TIME: {e - s}')
    # s = time.time()
    # dubins_path = planner.plan_dubins_path(best_path, 5, 5, 1)
    # e = time.time()
    # print(f'dubins PATH TIME: {e - s}')
    # print('===========================================================================')
    # environment.plot_environment({'A* PATH': best_path, 'DUBINS PATH': dubins_path})
    return path

if __name__ == '__main__':
    # for i in [10, 20, 50, 100, 150, 300, 500]:
    #     print(f'GRID SIZE: {i}')
    #     grid_h, grid_w = i, i
    #     array = np.zeros(shape=(grid_h, grid_w), dtype='int32')
    #     array[0, 0] = 1
    #     array[rn.randint(int(grid_h / 2.0), grid_h - 1), rn.randint(int(grid_h / 2.0), grid_w - 1)] = 2
    #     for _ in range(int(i * 0.2) * 2):
    #         array[rn.randint(int(grid_h / 2.0), grid_h - 1), rn.randint(int(grid_w / 2.0), grid_w - 1)] = 500
    array = np.load('bhavya.npy')
    unique_values, counts = np.unique(array, return_counts=True)
    print('UNIQUE:', unique_values, counts)
    # for i in range(array.shape[0]):
    #     for j in range(array.shape[1]):
    #         plt.text(j, i, f'{array[i, j]:.2f}', ha='center', va='center', color='white')
    # plt.show()
    get_path_from_numpy_array(array, 1, 2, 500)
    # path = get_path_from_numpy_array(array, 1, 2, 500, 3)
    # print(path)
