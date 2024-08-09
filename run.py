import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from singaboat_vrx.custom_plan1.path_planning_utils import environment,bstar
import math
from singaboat_vrx.custom_plan1.path_planning_utils.post_process import PostPlanner
import cProfile
import pstats
import io
import os
import time
import datetime
@dataclass
class PathParameters:
    """
    Data class to store parameters for path planning.

    Attributes:
        GRID_H (int): Height of the grid.
        GRID_W (int): Width of the grid.
        ROBOT_X (int): X-coordinate of the robot.
        ROBOT_Y (int): Y-coordinate of the robot.
        ROBOT_DX (float): X-direction vector of the robot.
        ROBOT_DY (float): Y-direction vector of the robot.
        END_X (int): X-coordinate of the end point.
        END_Y (int): Y-coordinate of the end point.
        OBSTACLES_X (list): List of X-coordinates of obstacles.
        OBSTACLES_Y (list): List of Y-coordinates of obstacles.
        OBSTACLES_DX (list): List of X-direction vectors of obstacles.
        OBSTACLES_DY (list): List of Y-direction vectors of obstacles.
        MAJOR_RANGES (list): List of major ranges for obstacles.
        MINOR_RANGES (list): List of minor ranges for obstacles.
        OBSTACLE_PENALTY (float): Penalty factor for obstacles.
        REPULSION_PENALTY (float): Penalty factor for repulsion.
        REPULSION_X (list): List of X-coordinates of repulsion points.
        REPULSION_Y (list): List of Y-coordinates of repulsion points.
        REPULSION_VALUES (list): List of repulsion values.
        MOVEMENT (str): Movement type (default is "queen").
        K_FACTOR (float): K-factor for path planning (default is 0.5).
    """
    GRID_H: int
    GRID_W: int
    ROBOT_X: int
    ROBOT_Y: int
    ROBOT_DX: float
    ROBOT_DY: float
    END_X: int
    END_Y: int
    OBSTACLES_X: list
    OBSTACLES_Y: list
    OBSTACLES_DX: list
    OBSTACLES_DY: list
    MAJOR_RANGES: list
    MINOR_RANGES: list
    OBSTACLE_PENALTY: float
    REPULSION_PENALTY: float
    REPULSION_X: list
    REPULSION_Y: list
    REPULSION_VALUES: list
    MOVEMENT: str = "queen"
    K_FACTOR: float = 0.5


class RobotPathPlanner:
    def __init__(self, array, robot_value=1, end_value=2, obstacle_value=500, repulsion_value=3):
        """
        Initializes the RobotPathPlanner with the given array and values representing robot, end, obstacles, and repulsion.

        Args:
            array (np.ndarray): The numpy array representing the environment.
            robot_value (int): The value in the array representing the robot's position.
            end_value (int): The value in the array representing the end position.
            obstacle_value (int): The value in the array representing obstacles.
            repulsion_value (int): The value in the array representing repulsion areas.
        """
        self.array = array
        self.robot_value = robot_value
        self.end_value = end_value
        self.obstacle_value = obstacle_value
        self.repulsion_value = repulsion_value
        self.params = self.get_path_parameters_from_numpy_array()
        self.env = self.environment_setup()
        self.planner = bstar.PathPlanner(self.env, self.params.OBSTACLE_PENALTY, self.params.REPULSION_PENALTY)

    def get_dx_dy(self, start, end):
        """
        Calculate the dx and dy as unit vector components towards the end position.

        Args:
            start (list): The starting coordinates [x, y].
            end (list): The end coordinates [x, y].

        Returns:
            tuple: Normalized (dx, dy) representing unit vector components.
        """
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance != 0:
            dx /= distance
            dy /= distance
        
        return dx, dy

    def remove_position_from_obstacles(self, position, OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY):
        """
        Removes a given position from the list of obstacles if it exists.

        Args:
            position (list): The position [x, y] to remove.
            OBSTACLES_X (list): List of x coordinates of obstacles.
            OBSTACLES_Y (list): List of y coordinates of obstacles.
            OBSTACLES_DX (list): List of x directions of obstacles.
            OBSTACLES_DY (list): List of y directions of obstacles.

        Returns:
            tuple: Updated lists of obstacle coordinates and directions.
        """
        obstacles_positions = [[x, y] for x, y in zip(OBSTACLES_X, OBSTACLES_Y)]
        if position in obstacles_positions:
            index = obstacles_positions.index(position)
            OBSTACLES_X.pop(index)
            OBSTACLES_Y.pop(index)
            OBSTACLES_DX.pop(index)
            OBSTACLES_DY.pop(index)
        return OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY

    def environment_setup(self):
        """
        Sets up the environment with the given parameters and plots the initial state.

        Returns:
            Environment: The initialized environment object.
        """
        env = environment.Environment(self.params.GRID_H, self.params.GRID_W)
        env.put_robot_and_end_in_memory(self.params.ROBOT_X, self.params.ROBOT_Y, self.params.ROBOT_DX, self.params.ROBOT_DY, self.params.END_X, self.params.END_Y)
        env.put_obstacles_in_memory(self.params.OBSTACLES_X, self.params.OBSTACLES_Y, self.params.OBSTACLES_DX, self.params.OBSTACLES_DY)
        env.put_repulsion_in_memory(self.params.REPULSION_X, self.params.REPULSION_Y, self.params.REPULSION_VALUES)
        return env

    def get_path_parameters_from_numpy_array(self ):
        """
        Extracts the necessary path planning parameters from the numpy array.

        Returns:
            PathParameters: A dataclass containing the extracted parameters.
        """
       
        try:
            length=int(math.sqrt(len(self.array)))
            self.array = np.array(self.array).reshape((length, length))
        except :
            print("!!!!!!! \t ERROR OCCURD WHILE READING OCCUPANCY GRID \t !!!!!!!")
            print("Type is ",type(self.array), "\t shape : ",self.array.shape)
            print("unique: \t",np.unique(self.array))
            print("_"*50)
        self.array = np.array(self.array).reshape((length, length))
        GRID_H, GRID_W = self.array.shape
        
        ROBOT_Y, ROBOT_X = np.where(self.array == self.robot_value)
        ROBOT_Y, ROBOT_X = ROBOT_Y.tolist()[0], ROBOT_X.tolist()[0]
        
        # try:
        END_X, END_Y = np.where(self.array == self.end_value)
        END_Y, END_X = END_X.tolist()[0], END_Y.tolist()[0]

        OBSTACLES_Y, OBSTACLES_X = np.where(self.array == self.obstacle_value)
        OBSTACLES_Y, OBSTACLES_X = OBSTACLES_Y.tolist(), OBSTACLES_X.tolist()
        
        OBSTACLES_DX, OBSTACLES_DY = [0] * len(OBSTACLES_X), [0] * len(OBSTACLES_Y)
        OBSTACLE_PENALTY = 500.0
        REPULSION_PENALTY = 100.0
        
        MAJOR_RANGES, MINOR_RANGES = [0, 0] * len(OBSTACLES_X), [0, 0] * len(OBSTACLES_Y)  
        
        # Check if the robot's position is in the list of obstacles
        robot_position = [ROBOT_X, ROBOT_Y]
        
        OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY = self.remove_position_from_obstacles(robot_position, OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY)

        # Check if the end position is in the list of obstacles
        end_position = [END_X, END_Y]
        OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY = self.remove_position_from_obstacles(end_position, OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY)
        
        ROBOT_DX, ROBOT_DY = self.get_dx_dy([ROBOT_X, ROBOT_Y], [END_X, END_Y])
        
        REPULSION_X, REPULSION_Y, REPULSION_VALUES = [], [], []
        for y in range(self.array.shape[0]):
            for x in range(self.array.shape[1]):
                if self.array[y, x] == 5:
                    REPULSION_X.append(x)
                    REPULSION_Y.append(y)
                    REPULSION_VALUES.append(self.array[y, x])
        
        return PathParameters(
            GRID_H=GRID_H, GRID_W=GRID_W, ROBOT_X=ROBOT_X, ROBOT_Y=ROBOT_Y, 
            ROBOT_DX=ROBOT_DX, ROBOT_DY=ROBOT_DY, END_X=END_X, END_Y=END_Y, 
            OBSTACLES_X=OBSTACLES_X, OBSTACLES_Y=OBSTACLES_Y, OBSTACLES_DX=OBSTACLES_DX, 
            OBSTACLES_DY=OBSTACLES_DY, MAJOR_RANGES=MAJOR_RANGES, MINOR_RANGES=MINOR_RANGES, 
            OBSTACLE_PENALTY=OBSTACLE_PENALTY, REPULSION_PENALTY=REPULSION_PENALTY,
            REPULSION_X=REPULSION_X, REPULSION_Y=REPULSION_Y, REPULSION_VALUES=REPULSION_VALUES
        )

    def run(self):
        """
        Executes the path planning algorithm and plots the robot's movement through each timestep.
        
        Returns:
            list: Smoothed path for the robot as a list of [x, y] coordinates.
        """
        
        if len(self.params.OBSTACLES_X) == 0 and len(self.params.REPULSION_X) == 0:
            # Generate a straight linear path with 500 coordinates
            x_coords = np.linspace(self.params.ROBOT_X, self.params.END_X, 500)
            y_coords = np.linspace(self.params.ROBOT_Y, self.params.END_Y, 500)
            path = [[x, y] for x, y in zip(x_coords, y_coords)]
            repulsions_x , repulsions_y = [],[]
        
        else:    
            self.planner.calculate_all_cost_and_heuristics_from_end_to_robot(self.params.MOVEMENT, self.params.K_FACTOR)

            path, orientation = self.planner.raw_path_finder_from_robot_to_end(self.params.MOVEMENT)
            if len(path) ==0 or len(orientation)==0:
                return []

            all_repulsions = self.env.all_repulsions
            repulsions_x, repulsions_y = all_repulsions["x_repulsions"], all_repulsions["y_repulsions"]

        path_points = np.asarray(path)
        post_planner = PostPlanner(path_points, repulsions_x, repulsions_y, epsilon=1.0)
        x_path, y_path, _ = post_planner.infer_spline()
            
        self.path=[[x_cords, y_cords] for x_cords, y_cords in zip (x_path, y_path)]
        points_list_with_1 = np.array([point + [1] for point in self.path])
    
        points_list_with_1[:, 0] -= 5
        points_list_with_1[:, 1] -= 50
        
        # post_planner.plot()
        # return [[x_cords, y_cords] for x_cords, y_cords in zip(x_path, y_path)]
        return points_list_with_1
    
def main():
    """
    Main function to load the environment array, initialize the planner, run the planning algorithm,
    and visualize the results.
    """
  
    
    folder_path = 'npy_files_og/npy_files'

    
    def process_npy_file(file_path):

        # Load the .npy file

        data = np.load(file_path)

        print(f"Processing {file_path}: shape = {data.shape}")

        array = np.load(file_path)
        # array[array == 500] =10
        array = np.rot90(array)
        array = np.fliplr(array)
       
        planner = RobotPathPlanner(array)
        path = planner.run()

    

    for filename in os.listdir(folder_path):

        if filename.endswith('.npy'):

            file_path = os.path.join(folder_path, filename)
            print("file_path:",file_path)
            process_npy_file(file_path)

   
if __name__ == '__main__':
    main()