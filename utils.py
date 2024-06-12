import time
import numpy as np
import matplotlib.pyplot as plt
from bstar import PathPlanner
from environment import Environment

def get_path_from_numpy_array(array, robot_value, end_value, obstacle_value, repulsion_value, movement='queen', k_factor=0.5):
    grid_h, grid_w = array.shape
    robot_y, robot_x = np.where(array == robot_value)
    print(f"robot_x : {robot_x}, robot_y : {robot_y}")
    robot_y, robot_x = robot_y.tolist()[0], robot_x.tolist()[0]
    robot_dx, robot_dy = 1, 0
    end_y, end_x = np.where(array == end_value)
    end_y, end_x = end_y.tolist()[0], end_x.tolist()[0]
    end_dx, end_dy = 1, 0
    obstacles_y, obstacles_x = np.where(array == obstacle_value)
    obstacles_y, obstacles_x = obstacles_y.tolist(), obstacles_x.tolist()
    print("obstracles",obstacles_x, obstacles_y)
    obstacles_dx, obstacles_dy = [0 for _ in obstacles_x], [0 for _ in obstacles_y]
    repulsions_y, repulsions_x = np.where(array == repulsion_value)
    repulsions_y, repulsions_x = repulsions_y.tolist(), repulsions_x.tolist()

    env = Environment(grid_h, grid_w)
    
    env.put_robot_in_memory(robot_x, robot_y, robot_dx, robot_dy)
    env.put_end_in_memory(end_x, end_y)
    
    # print(f"obstracle x : {obstacles_x}, obstacles_y : {obstacles_y},obs_dx : {obstacles_dx}, obstacles")
    
    env.put_obstacle_in_memory(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy, [0 for _ in obstacles_x], [0 for _ in obstacles_y])
    env.put_collision_points_in_memory()
    env.put_repulsions(repulsions_x, repulsions_y, [1.0 for _ in repulsions_x])
    env.get_occupancy_grid()
    env.plot_environment_on_grid(pause_time=5)

    planner = PathPlanner(env, 500, 0)
    
    planner.calculate_all_cost_and_heuristics_from_end_to_robot(movement, k_factor)
    
    # planner.calculate_all_cost_and_heuristics_from_robot_to_end(movement, k_factor)
    # planner.calculate_non_obstacle_cost_and_heuristics_from_end_to_robot(movement, k_factor)
    # planner.calculate_non_obstacle_cost_and_heuristics_from_robot_to_end(movement, k_factor)
    
    path, orientation = planner.raw_path_finder_from_robot_to_end(movement)

    # path, orientation = planner.raw_path_finder_from_end_to_robot(movement)
    e = time.time()
    print('FIND TIME:', e - s)
    
    print("-->",path)
    for timestep, _ in enumerate(path[1:], 1):
        env.plot_environment_in_memory()
        env.move_obstacles_on_grid(timestep)
        env.move_robot_on_grid(timestep)
    return path, orientation

if __name__ == '__main__':
    array = np.load('bhavya 10.npy')
    unique_values, counts = np.unique(array, return_counts=True)
    
    print(array)
    print('UNIQUE:', unique_values)
    print('COUNTS:', counts)
    plt.imshow(array)
    plt.show()
    path, orientation = get_path_from_numpy_array(array, robot_value = 1, end_value= 2, obstacle_value=500,repulsion_value =3)
    