from singaboat_vrx.custom_plan1.path_planning_utils import node
import math
import matplotlib.pyplot as plt

class Environment:
    __slots__ = ['grid_h', 'grid_w', 'display', 'repulsion_offset', 'all_repulsions', 'repulsion_x', 'repulsion_y', 'current_obstacles_position', 'obstacles_path', 'collisions', 'grid', 'robot_x', 'robot_y', 'robot_dx', 'robot_dy', 'global_path', 'global_orientation', 'end_x', 'end_y', 'robot_path', 'robot_orientation']

    def __init__(self, grid_h, grid_w, display=[], repulsion_offset=2, repulsion_x=[], repulsion_y=[]):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.display = display
        self.repulsion_offset = repulsion_offset
        self.all_repulsions = {"x_repulsions": [], "y_repulsions": [], "repulsion_factor": []}
        self.repulsion_x = repulsion_x
        self.repulsion_y = repulsion_y
        self.reset_environment()
        self.collisions = {}
        self.grid = self.create_grid()

    def reset_environment(self):
        self.current_obstacles_position = {}
        self.obstacles_path = {}
        self.collisions = {}
        self.all_repulsions = {"x_repulsions": [], "y_repulsions": [], "repulsion_factor": []}


    def is_inside_grid(self, x, y):
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    def euclidian_distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
    def plot_repulsion(self, repulsion_x: list, repulsion_y: list,plot_text: str):

        
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

    def get_oval_repulsion_around_obstacle(self, obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range):
    
        repulsions_x, repulsions_y, repulsions_factor = [], [], []
        for i in range(major_range[0], major_range[1] + 1, 1):
            repulsion_x, repulsion_y = obstacle_x + i * obstacle_dx if obstacle_dx != 0 else obstacle_x, obstacle_y + i * obstacle_dy if obstacle_dy != 0 else obstacle_y
            for dx in range(minor_range[0], minor_range[1] + 1, 1):
                for dy in range(minor_range[0], minor_range[1] + 1, 1):
                    index_x, index_y = repulsion_x + dx, repulsion_y + dy
                    if self.is_inside_grid(index_x, index_y):
                        repulsions_x.append(index_x)
                        repulsions_y.append(index_y)
                        repulsions_factor.append(1.0)
        
      
        return repulsions_x, repulsions_y, repulsions_factor

    def print_obstacle_data(self):
        for obstacle_id, position in self.current_obstacles_position.items():
            print(f"Obstacle ID: {obstacle_id}, Position: {position}")

        print("\nObstacles Path:")
        for obstacle_id, path_info in self.obstacles_path.items():
            print(f"Obstacle ID: {obstacle_id}, Path: {path_info['movement']}, Orientation: {path_info['orientation']}")

    def put_robot_in_memory(self, robot_x, robot_y, robot_dx, robot_dy):
        self.robot_x, self.robot_y = robot_x, robot_y
        self.robot_dx, self.robot_dy = robot_dx, robot_dy
    def bresenham(self,x1, y1, x2, y2):
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
    def put_global_path_in_memory(self):
        self.global_path = self.bresenham(self.robot_x, self.robot_y, self.end_x, self.end_y)
        self.global_orientation = [
        [self.global_path[index + 1][0] - x, self.global_path[index + 1][1] - y]
        for index, (x, y) in enumerate(self.global_path[:-1])
    ]
        # print("self.global_path is" , self.global_path)
    def put_end_in_memory(self, end_x, end_y):
        self.end_x, self.end_y = end_x, end_y
        self.put_global_path_in_memory()

    def put_obstacle_in_memory(self, obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range):
        diagonal = int(math.sqrt(math.pow(self.grid_w, 2) + math.pow(self.grid_h, 2)))
        obstacle_movement = [[obstacle_x + i * obstacle_dx, obstacle_y + i * obstacle_dy] for i in range(diagonal)]
        obstacle_orientation = [[obstacle_dx, obstacle_dy] for _ in range(diagonal)]
        major_ranges = [major_range for _ in range(diagonal)]
        minor_ranges = [minor_range for _ in range(diagonal)]
        obstacle_id = max(list(self.current_obstacles_position.keys())) + 1 if len(self.current_obstacles_position) > 0 else 0
        self.current_obstacles_position[obstacle_id] = [obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range]
        self.obstacles_path[obstacle_id] = {'movement': obstacle_movement, 'orientation': obstacle_orientation, 'major_range': major_ranges, 'minor_range': minor_ranges}
        self.put_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)

    def put_obstacles_in_memory(self, obstacles_x, obstacles_y, obstacles_dx, obstacles_dy, major_ranges, minor_ranges):
        for obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range in zip(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy, major_ranges, minor_ranges):
            self.put_obstacle_in_memory(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)

    def put_collision_points_in_memory(self, robot_path=None, robot_orientation=None):

        if robot_path is None and robot_orientation is None:

            self.robot_path = self.global_path
            self.robot_orientation = self.global_orientation
        else:

            self.robot_path = robot_path
            self.robot_orientation = robot_orientation
        for timestep, (robot_x, robot_y) in enumerate(self.robot_path):
      
            for obstacle_id in self.obstacles_path:
                if self.obstacles_path[obstacle_id]['movement'][timestep] == [robot_x, robot_y]:
                    
                    if timestep in self.collisions:
                        self.collisions[timestep] = self.collisions[timestep] + [obstacle_id]
                       
                    else:
                        self.collisions[timestep] = [obstacle_id]
                      

            if timestep in self.collisions:
                self.collisions[timestep] = sorted(self.collisions[timestep])
      
    def plot_environment_in_memory(self, pause_time=1):
        # print("self.current_obstacles_position : ", self.current_obstacles_position)
        
        fig, ax = plt.subplots(1, 1, figsize=(12, 12))
        ax.add_patch(plt.Rectangle((self.robot_x, self.robot_y), 1, 1, color='green'))
        ax.annotate('', (self.robot_x + 0.5, self.robot_y + 0.5), (self.robot_x + 0.5 + self.robot_dx, self.robot_y + 0.5 + self.robot_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
        ax.add_patch(plt.Rectangle((self.end_x, self.end_y), 1, 1, color='orange'))
        
        for timestep in self.collisions:
            for obstacle_id in self.collisions[timestep]:
                ax.add_patch(plt.Rectangle((self.obstacles_path[obstacle_id]['movement'][timestep][0], self.obstacles_path[obstacle_id]['movement'][timestep][1]), 1, 1, color='red'))
                ax.annotate('', (self.obstacles_path[obstacle_id]['movement'][timestep][0] + 0.5, self.obstacles_path[obstacle_id]['movement'][timestep][1] + 0.5), (self.obstacles_path[obstacle_id]['movement'][timestep][0] + 0.5 + self.obstacles_path[obstacle_id]['orientation'][timestep][0], self.obstacles_path[obstacle_id]['movement'][timestep][1] + 0.5 + self.obstacles_path[obstacle_id]['orientation'][timestep][1]), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
        
        
        for obstacle_id in self.current_obstacles_position:
            obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range = self.current_obstacles_position[obstacle_id]
            if obstacle_x and obstacle_y:
                ax.add_patch(plt.Rectangle((obstacle_x, obstacle_y), 1, 1, color='black'))
                ax.annotate('', (obstacle_x + 0.5, obstacle_y + 0.5), (obstacle_x + 0.5 + obstacle_dx, obstacle_y + 0.5 + obstacle_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
        
        for obstacle_id in self.obstacles_path:
            if self.obstacles_path[obstacle_id]['movement'] :
               
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
        # ax.set_aspect('equal')
        ax.invert_yaxis()
        ax.grid(True, alpha=1)
        plt.show(block=False)
        plt.pause(pause_time)
        plt.close()

    def create_grid(self):
        return [[node.Node(j, i, display=self.display) for j in range(self.grid_w)] for i in range(self.grid_h)]

    def put_distance_of_each_nodes_to_robot_on_grid(self):
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j].robot_distance = self.euclidian_distance(self.robot_x, self.robot_y, self.grid[i][j].x, self.grid[i][j].y)

    def put_robot_on_grid(self):
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[self.robot_y][self.robot_x].robot = True
            self.grid[self.robot_y][self.robot_x].robot_movement = [self.robot_dx, self.robot_dy]
        self.put_distance_of_each_nodes_to_robot_on_grid()

    def put_distance_of_each_nodes_to_end_on_grid(self):
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j].end_distance = self.euclidian_distance(self.end_x, self.end_y, self.grid[i][j].x, self.grid[i][j].y)

    def put_end_on_grid(self):
        if self.is_inside_grid(self.end_x, self.end_y):
            self.grid[self.end_y][self.end_x].end = True
        self.put_distance_of_each_nodes_to_end_on_grid()

    def put_repulsion(self, repulsion_x, repulsion_y, repulsion_factor,is_offset=False):
        if self.is_inside_grid(repulsion_x, repulsion_y):
          
            self.grid[repulsion_y][repulsion_x].repulsion_factor = self.grid[repulsion_y][repulsion_x].repulsion_factor + repulsion_factor
            # if is_offset:
            #     print(f"x = {repulsion_x}, y = {repulsion_y}, repulsion_factor{repulsion_factor}")

                
    def put_repulsions(self, repulsions_x, repulsions_y, repulsions_factor,is_offset =False):
        
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.put_repulsion(repulsion_x, repulsion_y, repulsion_factor,is_offset)
    
    def put_obstacle(self, obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range):
        # print(f"put_obstacle called with: ({obstacle_x}, {obstacle_y}), dx: {obstacle_dx}, dy: {obstacle_dy}")
        if self.is_inside_grid(obstacle_x, obstacle_y):
            # print(f"Placing obstacle at: ({obstacle_x}, {obstacle_y})")
            self.grid[obstacle_y][obstacle_x].obstacle = True
            self.grid[obstacle_y][obstacle_x].obstacle_movement = [obstacle_dx, obstacle_dy]

    def remove_repulsion_on_grid(self, repulsion_x, repulsion_y, repulsion_factor):
        if self.is_inside_grid(repulsion_x, repulsion_y):
            self.grid[repulsion_y][repulsion_x].repulsion_factor = self.grid[repulsion_y][repulsion_x].repulsion_factor - repulsion_factor

    def remove_repulsions_on_grid(self, repulsions_x, repulsions_y, repulsions_factor):
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.remove_repulsion_on_grid(repulsion_x, repulsion_y, repulsion_factor)

    def put_distance_of_each_nodes_to_collision_on_grid(self, collision_x, collision_y):
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j].total_obstacle_distance += self.euclidian_distance(collision_x, collision_y, self.grid[i][j].x, self.grid[i][j].y)

    def put_repulsion_points_on_grid(self):
        for repulsion_x, repulsion_y in zip(self.repulsion_x, self.repulsion_y):
            self.put_distance_of_each_nodes_to_collision_on_grid(repulsion_x, repulsion_y)

    def put_collision_on_grid(self):
        # print("put_collision_on_grid called")
        # print("self.collisions is ###=====>>>>>",self.collisions)
        for timestep in self.collisions:
            for obstacle_id in self.collisions[timestep]:
                collision_x, collision_y = self.obstacles_path[obstacle_id]['movement'][timestep]
                collision_dx, collision_dy = self.obstacles_path[obstacle_id]['orientation'][timestep]
                major_range = self.obstacles_path[obstacle_id]['major_range'][timestep]
                minor_range = self.obstacles_path[obstacle_id]['minor_range'][timestep]
                self.put_obstacle(collision_x, collision_y, collision_dx, collision_dy, major_range, minor_range)
    
                repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(collision_x, collision_y, collision_dx, collision_dy, major_range, minor_range)
                # self.plot_repulsion(repulsions_x, repulsions_y, "Raw data")
    
                self.all_repulsions["x_repulsions"].extend(repulsions_x)
                self.all_repulsions["y_repulsions"].extend(repulsions_y)
                self.all_repulsions["repulsion_factor"].extend(repulsions_factor)
    
                offset_repulsions_x, offset_repulsions_y = self.get_offset_repulsion(repulsions_x, repulsions_y)
                
            
                
                self.all_repulsions["x_repulsions"].extend(offset_repulsions_x)
                self.all_repulsions["y_repulsions"].extend(offset_repulsions_y)
                self.all_repulsions["repulsion_factor"].extend([1.0] * len(offset_repulsions_x))
    
                # self.plot_repulsion(offset_repulsions_x, offset_repulsions_y, "Offset data")
                self.put_repulsions(repulsions_x, repulsions_y, repulsions_factor)
                self.put_repulsions(offset_repulsions_x, offset_repulsions_y, [1.0] * len(offset_repulsions_x), is_offset=True)
                self.put_distance_of_each_nodes_to_collision_on_grid(collision_x, collision_y)

    def get_offset_repulsion(self, repulsion_x: list, repulsion_y: list) -> (list, list):
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
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j].total_obstacle_distance = self.grid[i][j].total_obstacle_distance + self.euclidian_distance(obstacle_x, obstacle_y, self.grid[i][j].x, self.grid[i][j].y)

    def put_other_obstacles_on_grid(self):
        
            
        for obstacle_id in self.obstacles_path:
                obstacle_x, obstacle_y = self.obstacles_path[obstacle_id]['movement'][0]
                obstacle_dx, obstacle_dy = self.obstacles_path[obstacle_id]['orientation'][0]
                major_range = self.obstacles_path[obstacle_id]['major_range'][0]
                minor_range = self.obstacles_path[obstacle_id]['minor_range'][0]
                self.put_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)
                repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)
                offset_repulsions_x, offset_repulsions_y = self.get_offset_repulsion(repulsions_x, repulsions_y)
                self.all_repulsions["x_repulsions"].extend(repulsions_x)
                self.all_repulsions["y_repulsions"].extend(repulsions_y)
                self.all_repulsions["repulsion_factor"].extend(repulsions_factor)
                self.all_repulsions["x_repulsions"].extend(offset_repulsions_x)
                self.all_repulsions["y_repulsions"].extend(offset_repulsions_y)
                self.all_repulsions["repulsion_factor"].extend([1.0] * len(offset_repulsions_x))
                self.put_repulsions(repulsions_x, repulsions_y, repulsions_factor)
                self.put_repulsions(offset_repulsions_x, offset_repulsions_y, [1.0] * len(offset_repulsions_x), is_offset=True)
                # self.plot_repulsion(repulsions_x, repulsions_y, "put other obstacle on the grid RAWWW")
                # self.plot_repulsion(offset_repulsions_x, offset_repulsions_y, "put other obstacle on the grid with Offset")
                self.put_distance_of_each_nodes_to_other_obstacles_on_grid(obstacle_x, obstacle_y)
    
    def move_robot_on_grid(self, timestep):
        current_x, current_y = self.robot_path[timestep - 1]
        if self.is_inside_grid(current_x, current_y):
            self.grid[current_y][current_x].robot = False
            self.grid[current_y][current_x].robot_movement = [0, 0]
        
        future_x, future_y = self.robot_path[timestep]
       
        if timestep < len(self.robot_orientation):
            future_dx, future_dy = self.robot_orientation[timestep]
            if self.is_inside_grid(future_x, future_y):
                self.grid[future_y][future_x].robot = True
                self.grid[future_y][future_x].robot_movement = [future_dx, future_dy]
            self.robot_x, self.robot_y = future_x, future_y
            self.robot_dx, self.robot_dy = future_dx, future_dy

    def move_obstacle_on_grid(self, obstacle_id, timestep):
        if obstacle_id in self.obstacles_path:
            current_x, current_y = self.obstacles_path[obstacle_id]['movement'][timestep - 1]
            current_dx, current_dy = self.obstacles_path[obstacle_id]['orientation'][timestep - 1]
            major_range = self.obstacles_path[obstacle_id]['major_range'][timestep - 1]
            minor_range = self.obstacles_path[obstacle_id]['minor_range'][timestep - 1]
            if self.is_inside_grid(current_x, current_y):
                self.grid[current_y][current_x].obstacle = False
                self.grid[current_y][current_x].obstacle_movement = [0, 0]
            repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(current_x, current_y, current_dx, current_dy, major_range, minor_range)
            self.remove_repulsions_on_grid(repulsions_x, repulsions_y, repulsions_factor)
            future_x, future_y = self.obstacles_path[obstacle_id]['movement'][timestep]
            future_dx, future_dy = self.obstacles_path[obstacle_id]['orientation'][timestep]
            if self.is_inside_grid(future_x, future_y):
                self.grid[future_y][future_x].obstacle = True
                self.grid[future_y][future_x].obstacle_movement = [future_dx, future_dy]
            repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(future_x, future_y, future_dx, future_dy, major_range, minor_range)
            self.put_repulsions(repulsions_x, repulsions_y, repulsions_factor)
            offset_repulsions_x , offset_repulsions_y = self.get_offset_repulsion(repulsions_x,repulsions_y)
            self.all_repulsions["x_repulsions"].extend(repulsions_x)
            self.all_repulsions["y_repulsions"].extend(repulsions_y)
            self.all_repulsions["repulsion_factor"].extend(repulsions_factor)
            self.all_repulsions["x_repulsions"].extend(offset_repulsions_x)
            self.all_repulsions["y_repulsions"].extend(offset_repulsions_y)
            self.all_repulsions["repulsion_factor"].extend(repulsions_factor)
            
            self.put_repulsions(offset_repulsions_x, offset_repulsions_y, repulsions_factor,is_offset =True)
            self.current_obstacles_position[obstacle_id] = [future_x, future_y, future_dx, future_dy, major_range, minor_range]

    def move_obstacles_on_grid(self, timestep):
        for obstacle_id in self.obstacles_path:
            self.move_obstacle_on_grid(obstacle_id, timestep)

    def get_occupancy_grid(self):

        self.grid = self.create_grid()
        self.put_robot_on_grid()
        self.put_end_on_grid()

        self.put_collision_on_grid()
        self.put_other_obstacles_on_grid()


    def __str__(self):
        text = ''
        for row in self.grid:
            for cell in row:
                text = text + cell.__str__()
            text = text + '\n'
        return text

    def plot_environment_on_grid(self, robot_path=None, pause_time=1):
        fig, ax = plt.subplots(1, 1, figsize=(12, 12))
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                if self.grid[i][j].obstacle:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))
                    ax.annotate('', (j + 0.5, i + 0.5), (j + 0.5 + self.grid[i][j].obstacle_movement[0], i + 0.5 + self.grid[i][j].obstacle_movement[1]), arrowprops={'color': 'pink', 'arrowstyle': '<-'})
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='white')
                elif self.grid[i][j].robot:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='green'))
                    ax.annotate('', (self.robot_x + 0.5, self.robot_y + 0.5), (self.robot_x + 0.5 + self.robot_dx, self.robot_y + 0.5 + self.robot_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j].end:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='red'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j].repulsion_factor > 0:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='grey'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                else:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='white'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
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
        fig, ax = plt.subplots(1, 1, figsize=(12, 12))

