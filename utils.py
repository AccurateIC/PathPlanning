import numpy as np
import matplotlib.pyplot as plt
from dstar import PathPlanner
from environment import Environment

def get_path_from_numpy_array(array, robot_value, end_value, obstacle_value, movement='queen'):
    grid_h, grid_w = array.shape
    robot_y, robot_x = np.where(array == robot_value)
    robot_y, robot_x = robot_y.tolist()[0], robot_x.tolist()[0]
    robot_dx, robot_dy = 1, 0
    end_y, end_x = np.where(array == end_value)
    end_y, end_x = end_y.tolist()[0], end_x.tolist()[0]
    print('END:', end_x, end_y)
    end_dx, end_dy = 1, 0
    obstacles_y, obstacles_x = np.where(array == obstacle_value)
    obstacles_y, obstacles_x = obstacles_y.tolist(), obstacles_x.tolist()
    obstacles_dx, obstacles_dy = [0 for _ in obstacles_x], [0 for _ in obstacles_y]

    environment = Environment(grid_h, grid_w)
    environment.put_robot(robot_x, robot_y, robot_dx, robot_dy)
    environment.put_end(end_x, end_y, end_dx, end_dy)
    environment.put_obstacles(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy)

    planner = PathPlanner(environment, movement, 500, 0, 0)
    planner.calculate_cost_and_heuristics()
    environment.plot_environment()
    planner.add_repulsion_penalty_from_numpy_array(array, robot_value, end_value, obstacle_value)
    paths = planner.raw_paths_finder([0])
    environment.plot_environment(paths)
    path_costs = {}
    for key in paths:
        paths[key] = planner.remove_knots_from_path(paths[key])
        environment.plot_environment({key: paths[key]})
        path_costs[key] = planner.calculate_path_cost(paths[key])
        # if not planner.does_path_hit_obstacles(paths[key]):
        # print('------------------------------------------------------------------------------------------------')
    print('PATHS:', paths)
    environment.plot_environment(paths)
    print('PATH COSTS:', path_costs)
    best_path = paths[list(path_costs.values()).index(min(list(path_costs.values())))]
    dubins_path = planner.plan_dubins_path(best_path, 10, 10, 1)
    environment.plot_environment({'A* PATH': best_path, 'DUBINS PATH': dubins_path})
    return best_path

if __name__ == '__main__':
    array = np.load('bhavya.npy')
    # print('SHAPE:', array.shape)
    # print(array)
    unique_values, counts = np.unique(array, return_counts=True)
    print('UNIQUE:', unique_values, counts)
    # plt.imshow(array)
    # for i in range(array.shape[0]):
    #     for j in range(array.shape[1]):
    #         plt.text(j, i, f'{array[i, j]:.2f}', ha='center', va='center', color='white')
    # plt.show()
    path = get_path_from_numpy_array(array, 1, 2, 500)
    # print(path)
