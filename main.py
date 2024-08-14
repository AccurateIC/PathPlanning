import json
import os
import time
import bstar
import environment
from post_process import PathPlanner
import numpy as np

def read_json(file_path):
    if os.path.exists(file_path):
        try:
            with open(file_path, 'r') as file:
                return json.load(file)
        except json.JSONDecodeError:
            return {"paths": [], "obstacles": []}
    else:
        return {"paths": [], "obstacles": []}


def write_json(data, file_path):
    with open(file_path, 'w') as file:
        json.dump(data, file, indent=4)


def remove_position_from_obstacles(position):
    obstacles_positions = [[x, y] for x, y in zip(OBSTACLES_X, OBSTACLES_Y)]
    if position in obstacles_positions:
        index = obstacles_positions.index(position)
        OBSTACLES_X.pop(index)
        OBSTACLES_Y.pop(index)
        OBSTACLES_DX.pop(index)
        OBSTACLES_DY.pop(index)


if __name__ == '__main__':
    MOVEMENT = 'queen'
    GRID_W, GRID_H = 50, 50
    ROBOT_X, ROBOT_Y = 25, 49
    ROBOT_DX, ROBOT_DY = 0, 0
    END_X, END_Y = 25, 0
    DISPLAY = 'all'
    OBSTACLE_PENALTY = 500.0
    REPULSION_PENALTY = 10.0
    K_FACTOR = 0.5
    OBSTACLES_X, OBSTACLES_Y = [1, 10, 4 ], [25, 8 , 4]
    OBSTACLES_DX, OBSTACLES_DY = [1, 0 , 0], [0, 0, 0]
    MAJOR_RANGES, MINOR_RANGES = [[-1, 3], [-1, 1]], [[-2, 2], [-2, 2]]

    # Check if the robot's position is in the list of obstacles
    robot_position = [ROBOT_X, ROBOT_Y]
    remove_position_from_obstacles(robot_position)

    # Check if the end position is in the list of obstacles
    end_position = [END_X, END_Y]
    remove_position_from_obstacles(end_position)


    env = environment.Environment(GRID_H, GRID_W)
    env.put_robot_in_memory(ROBOT_X, ROBOT_Y, ROBOT_DX, ROBOT_DY)
    env.put_end_in_memory(END_X, END_Y)
    env.put_obstacles_in_memory(OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY, MAJOR_RANGES, MINOR_RANGES)
    env.put_collision_points_in_memory()
    env.plot_environment_in_memory(pause_time=2)
    env.get_occupancy_grid()
    
    
    planner = bstar.PathPlanner(env, OBSTACLE_PENALTY, REPULSION_PENALTY)
    start = time.time()
    planner.calculate_all_cost_and_heuristics_from_end_to_robot(MOVEMENT, K_FACTOR)
    end = time.time()
    print(end - start)
    print('NODES EXPANDED:', len(planner.closed))

    start = time.time()
    path, orientation = planner.raw_path_finder_from_robot_to_end(MOVEMENT)
    end = time.time()
    print(end - start)
    env.robot_path, env.robot_orientation = path, orientation
    print('BSTAR PATH:', path, end='\n ===================================================\n')
    print('BSTAR ORIENTATION:', orientation, end='\n===================================================\n')

    # Post-processing using PathPlanner
    all_repulsions = env.all_repulsions
    repulsions_x,repulsions_y = all_repulsions["x_repulsions"], all_repulsions["y_repulsions"]

    path_points = np.asarray(path)

    post_planner = PathPlanner(path_points, repulsions_x, repulsions_y, epsilon=1.0)
    post_planner.plot()

    # Write to JSON file
    json_file_path = 'path_and_obstacles.json'
    data = read_json(json_file_path)

    # Ensure the keys 'paths' and 'obstacles' exist and are lists
    if not isinstance(data.get('paths', []), list):
        data['paths'] = []
    if not isinstance(data.get('obstacles', []), list):
        data['obstacles'] = []

    data['paths'].append(path)
    data['obstacles'].append({
        'x': OBSTACLES_X,
        'y': OBSTACLES_Y,
        'dx': OBSTACLES_DX,
        'dy': OBSTACLES_DY
    })
    write_json(data, json_file_path)
   
    for timestep, _ in enumerate(path[1:], 1):
        env.plot_environment_in_memory()
        env.move_obstacles_on_grid(timestep)
        env.move_robot_on_grid(timestep)
