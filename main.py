import dstar
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
    OBSTACLE_PENALTY = 300.0
    REPULSION_PENALTY = 10.0
    K_FACTOR = 0.5
    ######### STATIC OBSTACLES
    OBSTACLES_X, OBSTACLES_Y = [10, 13], [5, 12]
    OBSTACLES_DX, OBSTACLES_DY = [0 for _ in OBSTACLES_X], [0 for _ in OBSTACLES_Y]
    REPULSION_DISTANCES = [2 for _ in OBSTACLES_X]
    ######### DYNAMIC OBSTACLES
    # OBSTACLES_X, OBSTACLES_Y = [0, 0], [3, 4]
    # OBSTACLES_DX, OBSTACLES_DY = [1, 1], [0, 1]
    ######### RANDOM STATIC OBSTACLES
    # OBSTACLES = 10
    # OBSTACLES_X, OBSTACLES_Y = [rn.randint(0, GRID_W - 1) for _ in range(OBSTACLES)], [rn.randint(0, GRID_H - 1) for _ in range(OBSTACLES)]
    # OBSTACLES_DX, OBSTACLES_DY = [0 for _ in range(OBSTACLES)], [0 for _ in range(OBSTACLES)]
    # REPULSION_DISTANCES = [1 for _ in range(OBSTACLES)]
    ######### RANDOM DYNAMIC OBSTACLES
    # OBSTACLES = 2
    # OBSTACLES_X, OBSTACLES_Y = [rn.randint(0, GRID_W - 1) for _ in range(OBSTACLES)], [rn.randint(0, GRID_H - 1) for _ in range(OBSTACLES)]
    # OBSTACLES_DX, OBSTACLES_DY = [rn.randint(-1, 1) for _ in range(OBSTACLES)], [rn.randint(-1, 1) for _ in range(OBSTACLES)]

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
    env.put_obstacles(OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY, REPULSION_DISTANCES)
    env.plot_environment()

    planner = dstar.PathPlanner(env, OBSTACLE_PENALTY, REPULSION_PENALTY)
    start = time.time()
    planner.calculate_cost_and_heuristics_from_end_to_robot(MOVEMENT, K_FACTOR)
    # planner.calculate_cost_and_heuristics_from_robot_to_end(MOVEMENT, K_FACTOR)
    # planner.calculate_angular_cost_and_heuristics_from_end_to_robot(K_FACTOR)
    # planner.calculate_angular_cost_and_heuristics_from_robot_to_end(K_FACTOR)
    end = time.time()
    print(end - start)
    env.plot_environment()
    # path = planner.raw_path_finder(MOVEMENT)
    # env.plot_environment(path)
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

    # for x, y in best_path[1:]:
    #     environment.plot_environment(best_path)
    #     environment.move_obstacles(environment.obstacles, 1)
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
