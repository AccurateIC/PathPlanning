from singaboat_vrx.custom_plan1.path_planning_utils import node
import math
import matplotlib.pyplot as plt

class Environment:
    __slots__ = ['grid_h', 'grid_w', 'display', 'repulsion_offset', 'all_repulsions', 'current_obstacles_position', 'obstacles_path', 'collisions', 'grid', 'robot_x', 'robot_y', 'robot_dx', 'robot_dy', 'global_path', 'global_orientation', 'end_x', 'end_y', 'robot_path', 'robot_orientation']

    def __init__(self, grid_h, grid_w, display=[], repulsion_offset=5):
        """
        Initializes the environment.

        Args:
            grid_h (int): Height of the grid.
            grid_w (int): Width of the grid.
            display (list, optional): Display settings. Defaults to [].
            repulsion_offset (int, optional): Offset for repulsion. Defaults to 5.
        """
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.display = display
        self.repulsion_offset = repulsion_offset
        self.all_repulsions = {"x_repulsions": [], "y_repulsions": [], "repulsion_factor": []}

        self.current_obstacles_position = {}
        self.obstacles_path = {}       
        self.collisions = {}
        self.grid = self.create_grid()
        self.global_path = []
        self.global_orientation= []
    def is_inside_grid(self, x, y):
        """
        Checks if a point is inside the grid.

        Args:
            x (int): X coordinate.
            y (int): Y coordinate.

        Returns:
            bool: True if the point is inside the grid, False otherwise.
        """
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    def euclidian_distance(self, x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.

        Args:
            x1 (int): X coordinate of the first point.
            y1 (int): Y coordinate of the first point.
            x2 (int): X coordinate of the second point.
            y2 (int): Y coordinate of the second point.

        Returns:
            float: Euclidean distance.
        """
        return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

    def plot_repulsion(self, repulsion_x: list, repulsion_y: list, plot_text: str):
        """
        Plots the repulsion points and their offsets.

        Args:
            repulsion_x (list): X coordinates of the repulsion points.
            repulsion_y (list): Y coordinates of the repulsion points.
            plot_text (str): Title for the plot.
        """
        plt.figure(figsize=(10, 10))
        plt.scatter(repulsion_x, repulsion_y, color='blue', label='Original Points')
        plt.title('Repulsion Points and Their Offsets')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.xlim(0, self.grid_w)
        plt.ylim(0, self.grid_h)
        plt.gca().invert_yaxis()  # Invert the y-axis to place origin at top-left
        plt.show()

    def put_robot_and_end_in_memory(self, robot_x, robot_y, robot_dx, robot_dy, end_x, end_y):
        """
        Stores the robot and end positions in memory and calculates the global path.

        Args:
            robot_x (int): X coordinate of the robot.
            robot_y (int): Y coordinate of the robot.
            robot_dx (float): X direction of the robot.
            robot_dy (float): Y direction of the robot.
            end_x (int): X coordinate of the end point.
            end_y (int): Y coordinate of the end point.
        """
        self.robot_x, self.robot_y = robot_x, robot_y
        self.robot_dx, self.robot_dy = robot_dx, robot_dy
        self.end_x, self.end_y = end_x, end_y
        self.__put_global_path_in_memory()

    def bresenham(self, x1, y1, x2, y2):
        """
        Bresenham's line algorithm to determine the points on a line between two points.

        Args:
            x1 (int): X coordinate of the first point.
            y1 (int): Y coordinate of the first point.
            x2 (int): X coordinate of the second point.
            y2 (int): Y coordinate of the second point.

        Returns:
            list: List of points on the line.
        """
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            points.append([x1, y1])
            if x1 == x2 and y1 == y2:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy

        return points

    def __put_global_path_in_memory(self):
        """
        Calculates and stores the global path and its orientation from the robot to the end point.
        """
        self.global_path = self.bresenham(self.robot_x, self.robot_y, self.end_x, self.end_y)
        self.global_orientation = [
            [self.global_path[index + 1][0] - x, self.global_path[index + 1][1] - y]
            for index, (x, y) in enumerate(self.global_path[:-1])
        ]

    def put_obstacle_in_memory(self, obstacle_x, obstacle_y, obstacle_dx, obstacle_dy):
        """
        Stores obstacle positions and their movements in memory.

        Args:
            obstacle_x (int): X coordinate of the obstacle.
            obstacle_y (int): Y coordinate of the obstacle.
            obstacle_dx (int): X direction of the obstacle.
            obstacle_dy (int): Y direction of the obstacle.
        """
        # Calculate diagonal length for obstacle movement path
        diagonal = int(math.sqrt(math.pow(self.grid_w, 2) + math.pow(self.grid_h, 2)))
        
        # Compute the obstacle's movement path
        obstacle_movement = [[obstacle_x + i * obstacle_dx, obstacle_y + i * obstacle_dy] for i in range(diagonal)]
        
        # Compute the obstacle's orientation at each step of its path
        obstacle_orientation = [[obstacle_dx, obstacle_dy] for _ in range(diagonal)]
        
        # Assign a unique ID for the obstacle
        obstacle_id = max(list(self.current_obstacles_position.keys()), default=-1) + 1
        
        # Store the obstacle's initial position and movement parameters
        self.current_obstacles_position[obstacle_id] = [obstacle_x, obstacle_y, obstacle_dx, obstacle_dy]
        self.obstacles_path[obstacle_id] = {'movement': obstacle_movement, 'orientation': obstacle_orientation}
        
        # Call the method to actually put the obstacle in the environment
        self.put_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy)

    def put_obstacles_in_memory(self, obstacles_x, obstacles_y, obstacles_dx, obstacles_dy):
        """
        Stores multiple obstacles in memory.

        Args:
            obstacles_x (list): List of X coordinates for the obstacles.
            obstacles_y (list): List of Y coordinates for the obstacles.
            obstacles_dx (list): List of X directions for the obstacles.
            obstacles_dy (list): List of Y directions for the obstacles.
        """
        
        for obstacle_x, obstacle_y, obstacle_dx, obstacle_dy in zip(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy):
            self.put_obstacle_in_memory(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy)

    def plot_environment_in_memory(self, pause_time=1):
        """
        Plots the environment from memory, including the robot, end point, obstacles, and paths.

        Args:
            pause_time (int, optional): Pause time for the plot. Defaults to 1.
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 12))
        ax.add_patch(plt.Rectangle((self.robot_x, self.robot_y), 1, 1, color='green'))
        ax.annotate('', (self.robot_x + 0.5, self.robot_y + 0.5), (self.robot_x + 0.5 + self.robot_dx, self.robot_y + 0.5 + self.robot_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
        ax.add_patch(plt.Rectangle((self.end_x, self.end_y), 1, 1, color='orange'))
        
        for timestep in self.collisions:
            for obstacle_id in self.collisions[timestep]:
                ax.add_patch(plt.Rectangle((self.obstacles_path[obstacle_id]['movement'][timestep][0], self.obstacles_path[obstacle_id]['movement'][timestep][1]), 1, 1, color='red'))
                ax.annotate('', (self.obstacles_path[obstacle_id]['movement'][timestep][0] + 0.5, self.obstacles_path[obstacle_id]['movement'][timestep][1] + 0.5), (self.obstacles_path[obstacle_id]['movement'][timestep][0] + 0.5 + self.obstacles_path[obstacle_id]['orientation'][timestep][0], self.obstacles_path[obstacle_id]['movement'][timestep][1] + 0.5 + self.obstacles_path[obstacle_id]['orientation'][timestep][1]), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
        
        for obstacle_id in self.current_obstacles_position:
            obstacle_x, obstacle_y, obstacle_dx, obstacle_dy = self.current_obstacles_position[obstacle_id]
            if obstacle_x and obstacle_y:
                ax.add_patch(plt.Rectangle((obstacle_x, obstacle_y), 1, 1, color='black'))
                ax.annotate('', (obstacle_x + 0.5, obstacle_y + 0.5), (obstacle_x + 0.5 + obstacle_dx, obstacle_y + 0.5 + obstacle_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
        
        for obstacle_id in self.obstacles_path:
            if self.obstacles_path[obstacle_id]['movement']:
                ax.plot([x + 0.5 for x, y in self.obstacles_path[obstacle_id]['movement']], [y + 0.5 for x, y in self.obstacles_path[obstacle_id]['movement']], label=f'OBSTACLE {obstacle_id}')
                ax.scatter([x + 0.5 for x, y in self.obstacles_path[obstacle_id]['movement']], [y + 0.5 for x, y in self.obstacles_path[obstacle_id]['movement']], label=f'OBSTACLE {obstacle_id}')
        
        if len(self.robot_path) != 0:
            ax.plot([x + 0.5 for x, y in self.robot_path], [y + 0.5 for x, y in self.robot_path], label=f'ROBOT PATH')
            ax.scatter([x + 0.5 for x, y in self.robot_path], [y + 0.5 for x, y in self.robot_path], label=f'ROBOT PATH')
      
        ax.legend(loc='best')
        ax.set_xlim(0, self.grid_w)
        ax.set_ylim(0, self.grid_h)
        ax.set_xticks([i for i in range(self.grid_w + 1)])
        ax.set_yticks([i for i in range(self.grid_h + 1)])
        ax.tick_params(axis='both')
        ax.invert_yaxis()
        ax.grid(True, alpha=1)
        plt.show(block=False)
        plt.pause(pause_time)
        plt.close()

    def create_grid(self):
        """
        Creates a grid of nodes.

        Returns:
            list: 2D list representing the grid.
        """
        return [[node.Node(j, i, display=self.display) for j in range(self.grid_w)] for i in range(self.grid_h)]

    def put_distance_of_each_nodes_to_robot_on_grid(self):
        """
        Calculates and stores the distance of each node to the robot on the grid.
        """
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j].robot_distance = self.euclidian_distance(self.robot_x, self.robot_y, self.grid[i][j].x, self.grid[i][j].y)

    def put_robot_on_grid(self):
        """
        Places the robot on the grid and calculates distances from the robot to each node.
        """
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[self.robot_y][self.robot_x].robot = True
            self.grid[self.robot_y][self.robot_x].robot_movement = [self.robot_dx, self.robot_dy]
        self.put_distance_of_each_nodes_to_robot_on_grid()

    def put_distance_of_each_nodes_to_end_on_grid(self):
        """
        Calculates and stores the distance of each node to the end point on the grid.
        """
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j].end_distance = self.euclidian_distance(self.end_x, self.end_y, self.grid[i][j].x, self.grid[i][j].y)

    def put_end_on_grid(self):
        """
        Places the end point on the grid and calculates distances from the end point to each node.
        """
        if self.is_inside_grid(self.end_x, self.end_y):
            self.grid[self.end_y][self.end_x].end = True
        self.put_distance_of_each_nodes_to_end_on_grid()

    def put_repulsion(self, repulsion_x, repulsion_y, repulsion_factor=0, is_offset=False):
        """
        Places a repulsion point on the grid.

        Args:
            repulsion_x (int): X coordinate of the repulsion point.
            repulsion_y (int): Y coordinate of the repulsion point.
            repulsion_factor (int, optional): Repulsion factor. Defaults to 0.
            is_offset (bool, optional): Whether the repulsion is an offset. Defaults to False.
        """
        if self.is_inside_grid(repulsion_x, repulsion_y):
            self.grid[repulsion_y][repulsion_x].repulsion_factor = repulsion_factor

            if is_offset:
                self.grid[repulsion_y][repulsion_x].repulsion_factor = repulsion_factor + 140

            self.all_repulsions["x_repulsions"].append(repulsion_x)
            self.all_repulsions["y_repulsions"].append(repulsion_y)
            self.all_repulsions["repulsion_factor"].append(repulsion_factor)
            

    def put_repulsions(self, repulsions_x, repulsions_y, repulsions_factor, is_offset=False):
        """
        Places multiple repulsion points on the grid.

        Args:
            repulsions_x (list): List of X coordinates for the repulsion points.
            repulsions_y (list): List of Y coordinates for the repulsion points.
            repulsions_factor (list): List of repulsion factors.
            is_offset (bool, optional): Whether the repulsion is an offset. Defaults to False.
        """
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.put_repulsion(repulsion_x, repulsion_y, repulsion_factor, is_offset)

    def put_obstacle(self, obstacle_x, obstacle_y, obstacle_dx, obstacle_dy):
        """
        Places an obstacle on the grid.

        Args:
            obstacle_x (int): X coordinate of the obstacle.
            obstacle_y (int): Y coordinate of the obstacle.
            obstacle_dx (int): X direction of the obstacle.
            obstacle_dy (int): Y direction of the obstacle.
        """
        if self.is_inside_grid(obstacle_x, obstacle_y):
            self.grid[obstacle_y][obstacle_x].obstacle = True
            self.grid[obstacle_y][obstacle_x].obstacle_movement = [obstacle_dx, obstacle_dy]

    def remove_repulsion_on_grid(self, repulsion_x, repulsion_y, repulsion_factor):
        """
        Removes a repulsion point from the grid.

        Args:
            repulsion_x (int): X coordinate of the repulsion point.
            repulsion_y (int): Y coordinate of the repulsion point.
            repulsion_factor (int): Repulsion factor.
        """
        if self.is_inside_grid(repulsion_x, repulsion_y):
            self.grid[repulsion_y][repulsion_x].repulsion_factor -= repulsion_factor

    def remove_repulsions_on_grid(self, repulsions_x, repulsions_y, repulsions_factor):
        """
        Removes multiple repulsion points from the grid.

        Args:
            repulsions_x (list): List of X coordinates for the repulsion points.
            repulsions_y (list): List of Y coordinates for the repulsion points.
            repulsions_factor (list): List of repulsion factors.
        """
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.remove_repulsion_on_grid(repulsion_x, repulsion_y, repulsion_factor)

    def put_repulsion_in_memory(self, REPULSION_X, REPULSION_Y, REPULSION_VALUES):
        """
        Stores repulsion points in memory and plots the original and offset repulsion points.

        Args:
            REPULSION_X (list): List of X coordinates for the repulsion points.
            REPULSION_Y (list): List of Y coordinates for the repulsion points.
            REPULSION_VALUES (list): List of repulsion factors.
        """
        self.put_repulsions(REPULSION_X, REPULSION_Y, REPULSION_VALUES)
        # self.plot_repulsion(REPULSION_X, REPULSION_Y, "Original input Repulsion")

        offset_x, offset_y = self.get_offset_repulsion(REPULSION_X, REPULSION_Y)

        for offset_repulsion_x, offset_repulsion_y in zip(offset_x, offset_y):
            self.put_repulsion(offset_repulsion_x, offset_repulsion_y, is_offset=True)
        
        self.put_repulsions(offset_x, offset_y, REPULSION_VALUES, is_offset=True)

    def get_offset_repulsion(self, repulsion_x: list, repulsion_y: list) -> (list, list):
        """
        Calculates the offset repulsion points around the given repulsion points.

        Args:
            repulsion_x (list): List of X coordinates for the repulsion points.
            repulsion_y (list): List of Y coordinates for the repulsion points.

        Returns:
            tuple: Lists of X and Y coordinates for the offset repulsion points.
        """
        offset_x = []
        offset_y = []
        for x, y in zip(repulsion_x, repulsion_y):
            for dx in range(-self.repulsion_offset, self.repulsion_offset + 1):
                for dy in range(-self.repulsion_offset, self.repulsion_offset + 1):
                    if dx == 0 and dy == 0:
                        continue
                    new_x = x + dx
                    new_y = y + dy
                    if self.is_inside_grid(new_x, new_y):
                        offset_x.append(new_x)
                        offset_y.append(new_y)
        
        return offset_x, offset_y

    def put_distance_of_each_nodes_to_other_obstacles_on_grid(self, obstacle_x, obstacle_y):
        """
        Calculates and stores the distance of each node to other obstacles on the grid.

        Args:
            obstacle_x (int): X coordinate of the obstacle.
            obstacle_y (int): Y coordinate of the obstacle.
        """
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j].total_obstacle_distance += self.euclidian_distance(obstacle_x, obstacle_y, self.grid[i][j].x, self.grid[i][j].y)

    def __str__(self):
        """
        String representation of the grid.

        Returns:
            str: String representation of the grid.
        """
        text = ''
        for row in self.grid:
            for cell in row:
                text = text + cell.__str__()
            text = text + '\n'
        return text

    def plot_environment_on_grid(self, robot_path=None, pause_time=1):
        """
        Plots the environment on the grid, including the robot, end point, obstacles, repulsion points, and paths.

        Args:
            robot_path (list, optional): List of points representing the robot's path. Defaults to None.
            pause_time (int, optional): Pause time for the plot. Defaults to 1.
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 12))
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                if self.grid[i][j].obstacle:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))
                    ax.annotate('', (j + 0.5, i + 0.5), (j + 0.5 + self.grid[i][j].obstacle_movement[0], i + 0.5 + self.grid[i][j].obstacle_movement[1]), arrowprops={'color': 'pink', 'arrowstyle': '<-'})
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j].k, 1) if self.grid[i][j].k is not None else ""}', horizontalalignment='center', verticalalignment='center', color='white')
                elif self.grid[i][j].robot:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='green'))
                    ax.annotate('', (self.robot_x + 0.5, self.robot_y + 0.5), (self.robot_x + 0.5 + self.robot_dx, self.robot_y + 0.5 + self.robot_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j].k, 1) if self.grid[i][j].k is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j].end:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='red'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j].k, 1) if self.grid[i][j].k is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j].repulsion_factor > 0:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='grey'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j].k, 1) if self.grid[i][j].k is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                else:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='white'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j].k, 1) if self.grid[i][j].k is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
        if robot_path is not None and isinstance(robot_path, list):
            ax.plot([x + 0.5 for x, y in robot_path], [y + 0.5 for x, y in robot_path], label=f'ROBOT')
            ax.scatter([x + 0.5 for x, y in robot_path], [y + 0.5 for x, y in robot_path], label=f'ROBOT')
        if robot_path is not None and isinstance(robot_path, dict):
            for key in robot_path:
                ax.plot([x + 0.5 for x, y in robot_path[key]], [y + 0.5 for x, y in robot_path[key]], label=f'PATH {key}')
                ax.scatter([x + 0.5 for x, y in robot_path[key]], [y + 0.5 for x, y in robot_path[key]], label=f'PATH {key}')
        for obstacle_id in self.obstacles_path:
            ax.plot([x + 0.5 for x, y in self.obstacles_path[obstacle_id]['movement']], [y + 0.5 for x, y in self.obstacles_path[obstacle_id]['movement']], label=f'OBSTACLE {obstacle_id}')
            ax.scatter([x + 0.5 for x, y in self.obstacles_path[obstacle_id]['movement']], [y + 0.5 for x, y in self.obstacles_path[obstacle_id]['movement']], label=f'OBSTACLE {obstacle_id}')
        if len(self.global_path) != 0:
            ax.plot([x + 0.5 for x, y in self.global_path], [y + 0.5 for x, y in self.global_path], label=f'GLOBAL PATH')
            ax.scatter([x + 0.5 for x, y in self.global_path], [y + 0.5 for x, y in self.global_path], label=f'GLOBAL PATH')
        ax.legend(loc='best')
        ax.set_xlim(0, self.grid_w)
        ax.set_ylim(0, self.grid_h)
        ax.set_xticks([i for i in range(self.grid_w + 1)])
        ax.set_yticks([i for i in range(self.grid_h + 1)])
        ax.tick_params(axis='both')
        ax.invert_yaxis()
        ax.grid(True, alpha=1)
        plt.show(block=False)
        plt.pause(pause_time)
        plt.close()

    def plot_environment_movement(self):
        """
        Placeholder for plotting environment movement over time.
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 12))
