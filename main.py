import bstar
import time
import random as rn
import environment

if __name__ == '__main__':
    MOVEMENT = 'queen'
    GRID_W, GRID_H = 20, 20
    ROBOT_X, ROBOT_Y = 10, 19
    ROBOT_DX, ROBOT_DY = 0, -1
    END_X, END_Y = 10, 0
    END_DX, END_DY = 0, -1
    DISPLAY = 'all'
    OBSTACLE_PENALTY = 500.0
    REPULSION_PENALTY = 10.0
    K_FACTOR = 0.5
    ######### STATIC OBSTACLES
    # OBSTACLES_X, OBSTACLES_Y = [10, 13], [5, 12]
    # OBSTACLES_DX, OBSTACLES_DY = [0 for _ in OBSTACLES_X], [0 for _ in OBSTACLES_Y]
    # REPULSION_X, REPULSION_Y = [2 for _ in OBSTACLES_X], [2 for _ in OBSTACLES_Y]
    ######### DYNAMIC OBSTACLES
    OBSTACLES_X, OBSTACLES_Y = [0, 0, 10], [100, 4, 10]
    OBSTACLES_DX, OBSTACLES_DY = [1, 1, 0], [0, 1, 0]
    REPULSION_X, REPULSION_Y = [2 for _ in OBSTACLES_X], [2 for _ in OBSTACLES_Y]
    ######### RANDOM STATIC OBSTACLES
    # OBSTACLES = 20
    # OBSTACLES_X, OBSTACLES_Y = [rn.randint(0, GRID_W - 1) for _ in range(OBSTACLES)], [rn.randint(0, GRID_H - 1) for _ in range(OBSTACLES)]
    # OBSTACLES_DX, OBSTACLES_DY = [0 for _ in range(OBSTACLES)], [0 for _ in range(OBSTACLES)]
    # REPULSION_X, REPULSION_Y = [2 for _ in OBSTACLES_X], [2 for _ in OBSTACLES_Y]
    ######### RANDOM DYNAMIC OBSTACLES
    # OBSTACLES = 5
    # OBSTACLES_X, OBSTACLES_Y = [rn.randint(0, GRID_W - 1) for _ in range(OBSTACLES)], [rn.randint(0, GRID_H - 1) for _ in range(OBSTACLES)]
    # OBSTACLES_DX, OBSTACLES_DY = [rn.randint(-1, 1) for _ in range(OBSTACLES)], [rn.randint(-1, 1) for _ in range(OBSTACLES)]
    # REPULSION_X, REPULSION_Y = [2 for _ in OBSTACLES_DX], [2 for _ in OBSTACLES_DY]

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
    env.put_robot(ROBOT_X, ROBOT_Y, ROBOT_DX, ROBOT_DY)
    env.put_end(END_X, END_Y, END_DX, END_DY)
    env.put_obstacles(OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY, REPULSION_X, REPULSION_Y)
    env.plot_environment()

    planner = bstar.PathPlanner(env, OBSTACLE_PENALTY, REPULSION_PENALTY)
    ######################################################################################
    start = time.time()
    planner.calculate_all_cost_and_heuristics_from_end_to_robot(MOVEMENT, K_FACTOR)
    end = time.time()
    print(end - start)
    print('NODES EXPANDED:', len(planner.closed))
    start = time.time()
    path, orientation = planner.raw_path_finder_from_robot_to_end(MOVEMENT)
    end = time.time()
    print(end - start)
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
    # path = planner.plan_dubins_path(path, 4, 4, 1.0)
    # env.plot_environment(path)
    for (x, y), (dx, dy) in zip(path[1:], orientation[1:]):
        env.plot_environment(path)
        env.move_obstacles(env.grid_obstacles)
        env.move_robot(x, y, dx, dy)
