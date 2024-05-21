import math
import bstar
import environment

if __name__ == '__main__':
    MOVEMENT = 'queen'
    GRID_W, GRID_H = 50, 50
    START_X, START_Y = 25, 49
    END_X, END_Y = 25, 0
    DISPLAY = 'all'
    OBSTACLE_PENALTY = 500.0
    REPULSION_PENALTY = 10.0
    K_FACTOR = 0.5
    PATH_LENGTH = int(math.sqrt(math.pow(GRID_H, 2) + math.pow(GRID_W, 2)))
    # OBSTACLES_X, OBSTACLES_Y = [1, 20], [25, 8] # , 45] , 49]
    OBSTACLES_X, OBSTACLES_Y = [45, 20], [49, 8] # , 1] , 25]
    # OBSTACLES_DX, OBSTACLES_DY = [1, 0], [0, 0] # , -1] , -1]
    OBSTACLES_DX, OBSTACLES_DY = [-1, 0], [-1, 0] # , 1] , 0]
    MAJOR_RANGES, MINOR_RANGES = [[-1, 3], [-1, 1]], [[-2, 2], [-2, 2]] # , [0, 2]] , [-1, 1]]

    env = environment.Environment(GRID_H, GRID_W)
    env.place_start(START_X, START_Y)
    env.place_end(END_X, END_Y)
    env.set_robot_path()
    env.place_obstacles(OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY, MAJOR_RANGES, MINOR_RANGES)
    env.predict_all_obstacles_path(PATH_LENGTH)
    env.get_collision_points()
    print(env.collisions)
    env.plot_environment_in_memory(pause_time=15)
    # env.get_occupancy_grid()
    # env.plot_environment_on_grid(pause_time=5)

    # planner = bstar.PathPlanner(env, OBSTACLE_PENALTY, REPULSION_PENALTY)
    # planner.calculate_all_cost_and_heuristics_from_end_to_robot(MOVEMENT, K_FACTOR)
    # path, orientation = planner.raw_path_finder_from_robot_to_end(MOVEMENT)
    # for timestep, _ in enumerate(path[1:], 1):
    #     env.plot_environment_in_memory(pause_time=1)
    #     env.put_collision_points_in_memory()
    #     env.get_occupancy_grid()
    #     planner = bstar.PathPlanner(env, OBSTACLE_PENALTY, REPULSION_PENALTY)
    #     planner.calculate_all_cost_and_heuristics_from_end_to_robot(MOVEMENT, K_FACTOR)
    #     path, orientation = planner.raw_path_finder_from_robot_to_end(MOVEMENT)
    #     env.move_obstacles_on_grid(timestep)
    #     env.move_robot_on_grid(timestep)
