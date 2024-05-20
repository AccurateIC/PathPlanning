import bstar
import time
import random as rn
import environment

if __name__ == '__main__':
    MOVEMENT = 'queen'
    GRID_W, GRID_H = 50, 50
    ROBOT_X, ROBOT_Y = 25, 49
    ROBOT_DX, ROBOT_DY = 0, -1
    END_X, END_Y = 25, 0
    DISPLAY = 'all'
    OBSTACLE_PENALTY = 500.0
    REPULSION_PENALTY = 10.0
    K_FACTOR = 0.5
    OBSTACLES_X, OBSTACLES_Y = [1, 20], [25, 8] # , 45] , 49]
    # OBSTACLES_X, OBSTACLES_Y = [45, 20], [49, 8] # , 1] , 25]
    OBSTACLES_DX, OBSTACLES_DY = [1, 0], [0, 0] # , -1] , -1]
    # OBSTACLES_DX, OBSTACLES_DY = [-1, 0], [-1, 0] # , 1] , 0]
    MAJOR_RANGES, MINOR_RANGES = [[-1, 3], [-1, 1]], [[-2, 2], [-2, 2]] # , [0, 2]] , [-1, 1]]

    if [ROBOT_X, ROBOT_Y] in [[x, y] for x, y in zip(OBSTACLES_X, OBSTACLES_Y)]:
        index = [[x, y] for x, y in zip(OBSTACLES_X, OBSTACLES_Y)].index([ROBOT_X, ROBOT_Y])
        OBSTACLES_X.pop(index)
        OBSTACLES_Y.pop(index)
        OBSTACLES_DX.pop(index)
        OBSTACLES_DY.pop(index)
    if [END_X, END_Y] in [[x, y] for x, y in zip(OBSTACLES_X, OBSTACLES_Y)]:
        index = [[x, y] for x, y in zip(OBSTACLES_X, OBSTACLES_Y)].index([END_X, END_Y])
        OBSTACLES_X.pop(index)
        OBSTACLES_Y.pop(index)
        OBSTACLES_DX.pop(index)
        OBSTACLES_DY.pop(index)

    env = environment.Environment(GRID_H, GRID_W)
    env.put_robot_in_memory(ROBOT_X, ROBOT_Y, ROBOT_DX, ROBOT_DY)
    env.put_end_in_memory(END_X, END_Y)
    env.put_obstacles_in_memory(OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY, MAJOR_RANGES, MINOR_RANGES)
    env.put_collision_points_in_memory()
    env.plot_environment_in_memory(pause_time=5)
    env.get_occupancy_grid()
    env.plot_environment_on_grid(pause_time=5)

    planner = bstar.PathPlanner(env, OBSTACLE_PENALTY, REPULSION_PENALTY)
    ######################################################################################
    start = time.time()
    planner.calculate_all_cost_and_heuristics_from_end_to_robot(MOVEMENT, K_FACTOR)
    end = time.time()
    print(end - start)
    print('NODES EXPANDED:', len(planner.closed))
    # env.plot_environment_on_grid(pause_time=100)
    start = time.time()
    path, orientation = planner.raw_path_finder_from_robot_to_end(MOVEMENT)
    end = time.time()
    print(end - start)
    env.robot_path, env.robot_orientation = path, orientation
    print('BSTAR PATH:', path, end='\n===================================================\n')
    print('BSTAR ORIENTATION:', orientation, end='\n===================================================\n')
    # env.plot_environment_in_memory(pause_time=3)
    #####################################################################################
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
    # start = time.time()
    # planner.calculate_cost_and_heuristics_from_end_to_robot(MOVEMENT, K_FACTOR, 0.0001)
    # end = time.time()
    # print(end - start)
    # print('NODES EXPANDED:', len(planner.closed))
    # env.plot_environment_on_grid()
    # start = time.time()
    # path, orientation = planner.raw_path_finder_from_robot_to_end(MOVEMENT)
    # end = time.time()
    # print(end - start)
    # print('BSTAR PATH:', path, end='\n===================================================\n')
    # print('BSTAR ORIENTATION:', orientation, end='\n===================================================\n')
    # env.robot_path, env.robot_orientation = path, orientation
    # env.plot_environment_in_memory()
    ######################################################################################
    # start = time.time()
    # planner.calculate_cost_and_heuristics_from_robot_to_end(MOVEMENT, K_FACTOR, 0.0001)
    # end = time.time()
    # print(end - start)
    # print('NODES EXPANDED:', len(planner.closed))
    # env.plot_environment_on_grid()
    # start = time.time()
    # path, orientation = planner.raw_path_finder_from_end_to_robot(MOVEMENT)
    # end = time.time()
    # print(end - start)
    # print('BSTAR PATH:', path, end='\n===================================================\n')
    # print('BSTAR ORIENTATION:', orientation, end='\n===================================================\n')
    # env.robot_path, env.robot_orientation = path, orientation
    # env.plot_environment_in_memory()
    ######################################################################################
    # segments = planner.find_segments_in_path(path, orientation)
    # print('SEGMENTS:', segments)
    # dubins_path = planner.plan_dubins_path(path, 12, 12, 1.0)
    # env.plot_environment(robot_path={'BSTAR': path, 'DUBINS': dubins_path})
    # env.plot_environment(path)
    # path = planner.baziers_curve(path, 0.1)
    # env.plot_environment_on_grid(path)
    for timestep, _ in enumerate(path[1:], 1):
        env.plot_environment_in_memory()
        env.move_obstacles_on_grid(timestep)
        env.move_robot_on_grid(timestep)
