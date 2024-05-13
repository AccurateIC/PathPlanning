import time
import bstar
import environment
import random as rn

if __name__ == '__main__':
    MOVEMENT = 'queen'
    GRID_W, GRID_H = 8, 8
    ROBOT = {
        'movement': {0: [3, 7]},
        'orientation': {0: [0, -1]}
        }
    END = {
        'movement': {0: [3, 0]},
        'orientation': {0: [0, -1]}
        }
    OBSTACLES = {
        0: {
            'movement': {0: [0, 1], 1: [1, 2], 2: [2, 3], 3: [3, 4], 4: [4, 5], 5: [5, 6], 6: [6, 7]},
            'orientation': {0: [1, 1], 1: [1, 1], 2: [1, 1], 3: [1, 1], 4: [1, 1], 5: [1, 1], 6: [1, 1]},
            'repulsion': {0: 1, 1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1}
            },
        }
    OBSTACLE_PENALTY = 500.0
    REPULSION_PENALTY = 10.0
    K_FACTOR = 0.5

    env = environment.Environment(GRID_H, GRID_W)
    env.put_robot(ROBOT)
    env.put_end(END)
    env.put_obstacles(OBSTACLES)
    env.plot_environment()

    planner = bstar.PathPlanner(env, OBSTACLE_PENALTY, REPULSION_PENALTY)
    ######################################################################################
    start = time.time()
    planner.calculate_cost_and_heuristics_from_end_to_robot(MOVEMENT, K_FACTOR)
    end = time.time()
    print(end - start)
    start = time.time()
    path, orientation = planner.raw_path_finder_from_robot_to_end(MOVEMENT)
    end = time.time()
    env.robot = {'movement': path, 'orientation': orientation}
    env.plot_environment(paths={'Robot': path})
    ######################################################################################
    # start = time.time()
    # planner.calculate_cost_and_heuristics_from_robot_to_end(MOVEMENT, K_FACTOR)
    # end = time.time()
    # print(end - start)
    # start = time.time()
    # path = planner.raw_path_finder_from_end_to_robot(MOVEMENT)
    # end = time.time()
    # print(end - start)
    # env.plot_environment(path)
    ######################################################################################
    # start = time.time()
    # planner.calculate_non_obstacle_cost_and_heuristics_from_end_to_robot(MOVEMENT, K_FACTOR)
    # end = time.time()
    # print(end - start)
    # start = time.time()
    # path = planner.raw_path_finder_from_robot_to_end(MOVEMENT)
    # end = time.time()
    # print(end - start)
    # env.plot_environment(path)
    ######################################################################################
    # start = time.time()
    # planner.calculate_non_obstacle_cost_and_heuristics_from_robot_to_end(MOVEMENT, K_FACTOR)
    # end = time.time()
    # print(end - start)
    # start = time.time()
    # path = planner.raw_path_finder_from_end_to_robot(MOVEMENT)
    # end = time.time()
    # print(end - start)
    # env.plot_environment(path)
    ######################################################################################

    # planner.calculate_angular_cost_and_heuristics_from_end_to_robot(K_FACTOR)
    # planner.calculate_angular_cost_and_heuristics_from_robot_to_end(K_FACTOR)
    # path = planner.plan_dubins_path(path, 4, 4, 1.0)
    # env.plot_environment(path)
    # costs = {}
    # for key in paths:
    #     paths[key] = planner.remove_knots_from_path(paths[key])
    #     costs[key] = planner.calculate_path_cost(paths[key])
    # environment.plot_environment(paths)
    # best_path = planner.choose_best_path(costs, paths)
    # print('BEST PATH:', best_path)
    # environment.plot_environment({'A* Path': best_path})
    # dubins_path = planner.plan_dubins_path(best_path, window_size=4, strides=4, curvature=0.5)
    # print('DUBINS PATH:', dubins_path)
    # environment.plot_environment({'A* Path': best_path, 'Dubins Path': dubins_path})

    for timestep in range(1, len(path) - 1, 1):
        env.move_all_obstacles(timestep)
        env.move_robot(timestep)
        env.plot_environment(paths={'Robot': path})
        # print(environment.robot_x, environment.robot_y, environment.robot_dx, environment.robot_dy)
        # print('lllllllllllllllllllllll', x, y)
        # environment.move_robot(x, y)
        # future_grid = environment.see_one_timestep_in_future(path)
        # if environment.is_collision_imminent(future_grid):
        #     print('COLLISION IMMINENT. REROUTING....')
        #     environment.grid = environment.reset_cost_and_heuristics()
        #     path = dstar.calculate_cost_and_heuristics_and_get_optimal_path()
        # else:
        #     environment.move_robot_along_path_by_one_timestep(path)
        #     environment.move_dynamic_obstacles_by_one_timestep()
