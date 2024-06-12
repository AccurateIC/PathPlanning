import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from path_planning_utils import environment,bstar
from path_planning_utils.post_process import PathPlanner


@dataclass
class PathParameters:
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
    REPULSION_X:list
    REPULSION_Y :list
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
        env = environment.Environment(self.params.GRID_H, self.params.GRID_W, repulsion_x=self.params.REPULSION_X, repulsion_y=self.params.REPULSION_Y)
        env.put_robot_in_memory(self.params.ROBOT_X, self.params.ROBOT_Y, self.params.ROBOT_DX, self.params.ROBOT_DY)
        env.put_end_in_memory(self.params.END_X, self.params.END_Y)
        env.put_obstacles_in_memory(self.params.OBSTACLES_X, self.params.OBSTACLES_Y, self.params.OBSTACLES_DX, self.params.OBSTACLES_DY, self.params.MAJOR_RANGES, self.params.MINOR_RANGES)
        env.put_collision_points_in_memory()
        # env.plot_environment_in_memory(pause_time=2)
        env.get_occupancy_grid()
        # env.plot_environment_on_grid(pause_time=5)
    

        # print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        # print("obstacle path :: " ,   env.obstacles_path)
        # print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

        return env

    def get_path_parameters_from_numpy_array(self):
        """
        Extracts the necessary path planning parameters from the numpy array.

        Returns:
            PathParameters: A dataclass containing the extracted parameters.
        """
        GRID_H, GRID_W = self.array.shape
        
        ROBOT_Y, ROBOT_X = np.where(self.array == self.robot_value)
        ROBOT_Y, ROBOT_X = ROBOT_Y.tolist()[0], ROBOT_X.tolist()[0]
        
        END_Y, END_X = np.where(self.array == self.end_value)
        END_Y, END_X = END_Y.tolist()[0], END_X.tolist()[0]

        OBSTACLES_Y, OBSTACLES_X = np.where(self.array == self.obstacle_value)
        OBSTACLES_Y, OBSTACLES_X = OBSTACLES_Y.tolist(), OBSTACLES_X.tolist()
        
        OBSTACLES_DX, OBSTACLES_DY = [0] * len(OBSTACLES_X), [0] * len(OBSTACLES_Y)
        OBSTACLE_PENALTY = 500.0
        REPULSION_PENALTY = 10.0
        
        MAJOR_RANGES, MINOR_RANGES = [[-1, 3], [-1, 1], [-1, 1],[-1, 1]], [[-2, 2], [-2, 2],[-2, 2],[-2, 2]]
        
        # Check if the robot's position is in the list of obstacles
        robot_position = [ROBOT_X, ROBOT_Y]
        
        OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY = self.remove_position_from_obstacles(robot_position, OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY)

        # Check if the end position is in the list of obstacles
        end_position = [END_X, END_Y]
        OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY = self.remove_position_from_obstacles(end_position, OBSTACLES_X, OBSTACLES_Y, OBSTACLES_DX, OBSTACLES_DY)
        
        ROBOT_DX, ROBOT_DY = self.get_dx_dy([ROBOT_X, ROBOT_Y], [END_X, END_Y])
        
        REPULSION_X, REPULSION_Y, repulsion_values = [], [], []
        for y in range(self.array.shape[0]):
            for x in range(self.array.shape[1]):
                if self.array[y, x]  in range(140,149):
                    
                    REPULSION_X.append(x)
                    REPULSION_Y.append(y)
                    repulsion_values.append(self.array[y, x])
                    
        
        return PathParameters(
            GRID_H=GRID_H, GRID_W=GRID_W, ROBOT_X=ROBOT_X, ROBOT_Y=ROBOT_Y, 
            ROBOT_DX=ROBOT_DX, ROBOT_DY=ROBOT_DY, END_X=END_X, END_Y=END_Y, 
            OBSTACLES_X=OBSTACLES_X, OBSTACLES_Y=OBSTACLES_Y, OBSTACLES_DX=OBSTACLES_DX, 
            OBSTACLES_DY=OBSTACLES_DY, MAJOR_RANGES=MAJOR_RANGES, MINOR_RANGES=MINOR_RANGES, 
            OBSTACLE_PENALTY=OBSTACLE_PENALTY, REPULSION_PENALTY=REPULSION_PENALTY,
            REPULSION_X=REPULSION_X , REPULSION_Y=REPULSION_Y)

    def run(self):
        """
        Executes the path planning algorithm and plots the robot's movement through each timestep.
        """
        self.planner.calculate_all_cost_and_heuristics_from_end_to_robot(self.params.MOVEMENT, self.params.K_FACTOR)
        
        path, orientation = self.planner.raw_path_finder_from_robot_to_end(self.params.MOVEMENT)
        
        self.env.robot_path, self.env.robot_orientation = path, orientation
        all_repulsions = self.env.all_repulsions
        repulsions_x, repulsions_y = all_repulsions["x_repulsions"], all_repulsions["y_repulsions"]

        path_points = np.asarray(path)

        post_planner = PathPlanner(path_points, repulsions_x, repulsions_y, epsilon=1.0)
        x_path , y_path , min_distance = post_planner.infer_spline()
        # for timestep, _ in enumerate(path[1:], 1):
        #     self.env.plot_environment_in_memory()
        #     self.env.move_obstacles_on_grid(timestep)
        #     self.env.move_robot_on_grid(timestep)
        return [[x_cords, y_cords] for x_cords, y_cords in zip (x_path, y_path)]
    
if __name__ == '__main__':
    file_path = 'custom_grid_3.npy'
    array = np.load(file_path)
    
    unique_values, counts = np.unique(array, return_counts=True)
    # plt.imshow(array)
    # plt.show()
    
    planner = RobotPathPlanner(array)
    path =  planner.run()
    
