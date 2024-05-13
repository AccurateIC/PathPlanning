import numpy as np
import matplotlib.pyplot as plt
from bstar import PathPlanner
from environment import Environment

# def get_initial_path_from_numpy_array(array, robot_value, end_value, obstacle_value, repulsion_distance, movement='queen', k_factor=0.5):
#     grid_h, grid_w = array.shape
#     robot_y, robot_x = np.where(array == robot_value)
#     robot_y, robot_x = robot_y.tolist()[0], robot_x.tolist()[0]
#     end_y, end_x = np.where(array == end_value)
#     end_y, end_x = end_y.tolist()[0], end_x.tolist()[0]
#     obstacles_y, obstacles_x = np.where(array == obstacle_value)
#     obstacles_y, obstacles_x = obstacles_y.tolist(), obstacles_x.tolist()
#     obstacles_dy, obstacles_dx = [0 for _ in obstacles_y], [0 for _ in obstacles_x]
#     obstacle_paths = [[] for _ in obstacles_y]
#     repulsion_distances = [repulsion_distance for _ in obstacles_y]
#     env = Environment(grid_h, grid_w)
#     env.put_robot(robot_x, robot_y, 0, -1)
#     env.put_end(end_x, end_y, 0, -1)
#     env.put_obstacles(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy, obstacle_paths, repulsion_distances)
#     # env.plot_environment()
#     planner = PathPlanner(env, 500, 10)
#     planner.calculate_cost_and_heuristics_from_end_to_robot(movement, k_factor)
#     path = planner.raw_path_finder_from_robot_to_end(movement)
#     env.plot_environment(paths={'Robot': path})
#     return path

def get_path_from_numpy_array(array, robot_value, end_value, obstacles_range, collision_value, repulsion_distance=0, movement='queen', k_factor=0.5):
    grid_h, grid_w = array.shape
    robot_y, robot_x = np.where(array == robot_value)
    robot_y, robot_x = robot_y.tolist()[0], robot_x.tolist()[0]
    end_y, end_x = np.where(array == end_value)
    end_y, end_x = end_y.tolist()[0], end_x.tolist()[0]
    collisions_x, collisions_y = np.where(array == collision_value)
    collisions_x, collisions_y = collisions_x.tolist(), collisions_y.tolist()
    print('COLLISIONS:', [[x, y] for x, y in zip(collisions_x, collisions_y)])
    obstacles = {}
    for obstacle_id, obstacle_range in enumerate(obstacles_range):
        obstacles[obstacle_id] = {'movement': {}}#, 'orientation': {}, 'repulsion': {}}
        for timestep, obstacle_value in enumerate(range(obstacle_range[0], obstacle_range[1], 1)):
            obstacle_y, obstacle_x = np.where(array == obstacle_value)
            obstacle_y, obstacle_x = obstacle_y.tolist(), obstacle_x.tolist()
            if len(obstacle_x) > 0 and len(obstacle_y) > 0:
                obstacle_x, obstacle_y = obstacle_x[0], obstacle_y[0]
            else:
                obstacle_x, obstacle_y = None, None
            obstacles[obstacle_id]['movement'][timestep] = [obstacle_x, obstacle_y]
    for obstacle_id in obstacles:
        print(obstacle_id, obstacles[obstacle_id]['movement'], end='\n======================================================================\n')
        for timestep in obstacles[obstacle_id]['movement']:
            if obstacles[obstacle_id]['movement'][timestep] == [None, None]:
                pass
    # robot = {
    #     'movement': {0: [robot_x, robot_y]},
    #     'orientation': {0: [0, -1]}
    #     }
    # end = {
    #     'movement': {0: [end_x, end_y]},
    #     'orientation': {0: [0, -1]}
    # }
    # obstacles = [{'movement': {}, 'orientation': {}, 'repulsion': {}}]
    # env = Environment(grid_h, grid_w)
    # env.put_robot(robot)
    # env.put_end(end)
    # env.put_obstacles(obstacles)
    # env.plot_environment()
    # planner = PathPlanner(env, 500, 10)
    # planner.calculate_cost_and_heuristics_from_end_to_robot(movement, k_factor)
    # path = planner.raw_path_finder_from_robot_to_end(movement)
    # env.plot_environment(paths={'Robot': path})
    # return path

if __name__ == '__main__':
    array = np.load('bhavya2.npy')
    unique_values, counts = np.unique(array, return_counts=True)
    print('UNIQUE:', unique_values)
    print('COUNTS:', counts)
    get_path_from_numpy_array(array, 100, 110, [[10, 30], [40, 60], [70, 90]], 150)
    # path = get_initial_path_from_numpy_array(array, 10, 20, 50, 1)
    # path = get_path_from_numpy_array(array, 100, 110, [[0, 19], [30, 49]], 150)
    # print(path)
