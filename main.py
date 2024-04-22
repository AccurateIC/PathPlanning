import dstar
import random as rn
import environment

if __name__ == '__main__':
    MOVEMENT = 'queen'
    GRID_W, GRID_H = 20, 20
    START_X, START_Y = 10, 19
    ROBOT_X, ROBOT_Y = START_X, START_Y
    ROBOT_DX, ROBOT_DY = 0, -1
    END_X, END_Y = 10, 0
    DISPLAY = 'all'
    OBSTACLE_PENALTY = 100.0
    REPULSION_DISTANCE = 1
    REPULSION_PENALTY = 5.0
    # STATIC OBSTACLES
    OBSTACLES_X, OBSTACLES_Y = [2, 2, 3, 3, 4, 4, 5, 5, 9, 9, 10, 10, 11, 11, 14, 14, 15, 15, 15, 15, 16, 16, 17, 17, 5, 5, 5, 6, 6, 6, 7, 7, 7], [4, 5, 4, 5, 1, 2, 1, 2, 5, 6, 5, 6, 5, 6, 5, 6, 5, 6, 11, 12, 11, 12, 11, 12, 9, 10, 11, 9, 10, 11, 9, 10, 11]
    OBSTACLES_DX, OBSTACLES_DY = [0 for _ in OBSTACLES_X], [0 for _ in OBSTACLES_Y]
    # DYNAMIC OBSTACLES
    # OBSTACLES_X, OBSTACLES_Y = [0, 0], [3, 4]
    # OBSTACLES_DX, OBSTACLES_DY = [1, 1], [0, 1]
    # RANDOM STATIC OBSTACLES
    # OBSTACLES = 50
    # OBSTACLES_X, OBSTACLES_Y = [rn.randint(0, GRID_W - 1) for _ in range(OBSTACLES)], [rn.randint(0, GRID_H - 1) for _ in range(OBSTACLES)]
    # OBSTACLES_DX, OBSTACLES_DY = [0 for _ in range(OBSTACLES)], [0 for _ in range(OBSTACLES)]
    # RANDOM DYNAMIC OBSTACLES
    # OBSTACLES = 2
    # OBSTACLES_X, OBSTACLES_Y = [rn.randint(0, GRID_W - 1) for _ in range(OBSTACLES)], [rn.randint(0, GRID_H - 1) for _ in range(OBSTACLES)]
    # OBSTACLES_DX, OBSTACLES_DY = [rn.randint(-1, 1) for _ in range(OBSTACLES)], [rn.randint(-1, 1) for _ in range(OBSTACLES)]

    if [START_X, START_Y] in [[x, y] for x, y in zip(OBSTACLES_X, OBSTACLES_Y)]:
        index = [[x, y] for x, y in zip(OBSTACLES_X, OBSTACLES_Y)].index([START_X, START_Y])
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

    environment = environment.Environment(GRID_H, GRID_W, DISPLAY)
    planner = dstar.PathPlanner(environment, MOVEMENT, OBSTACLE_PENALTY, REPULSION_DISTANCE, REPULSION_PENALTY)

    environment.put_start(START_X, START_Y)
    environment.put_robot(ROBOT_X, ROBOT_Y, ROBOT_DX, ROBOT_DY)
    environment.put_end(END_X, END_Y)
    environment.put_obstacles(OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY)

    planner.calculate_cost_and_heuristics()
    environment.plot_environment()
    path = planner.raw_path_finder()
    print('RAW:', path, end='\n=================================================================\n')
    environment.plot_environment(path)
    path = planner.remove_knots_from_path(path)
    print('CLEAN:', path)
    environment.plot_environment(path)

    # for x, y in path[1:]:
    #     environment.plot_environment(path)
    #     environment.move_obstacles(environment.obstacles, 1)
    #     print(environment.robot_x, environment.robot_y, environment.robot_dx, environment.robot_dy)
    #     print('lllllllllllllllllllllll', x, y)
        # environment.move_robot(x, y)
    #     future_grid = environment.see_one_timestep_in_future(path)
    #     if environment.is_collision_imminent(future_grid):
    #         print('COLLISION IMMINENT. REROUTING....')
    #         environment.grid = environment.reset_cost_and_heuristics()
    #         path = dstar.calculate_cost_and_heuristics_and_get_optimal_path()
    #     else:
    #         environment.move_robot_along_path_by_one_timestep(path)
    #         environment.move_dynamic_obstacles_by_one_timestep()
