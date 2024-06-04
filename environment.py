import node
import math
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w, display=[],repulsion_offset = 5):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.display = display
        self.repulsion_offset = repulsion_offset
        self.reset_environment()

    def reset_environment(self):
        self.current_obstacles_position = {}
        self.obstacles_path = {}
        self.collisions = {}

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
        print("Current Obstacles Position:")
        for obstacle_id, position in self.current_obstacles_position.items():
            print(f"Obstacle ID: {obstacle_id}, Position: {position}")

        print("\nObstacles Path:")
        for obstacle_id, path_info in self.obstacles_path.items():
            print(f"Obstacle ID: {obstacle_id}, Path: {path_info['movement']}, Orientation: {path_info['orientation']}")

    def put_robot_in_memory(self, robot_x, robot_y, robot_dx, robot_dy):
        self.robot_x, self.robot_y = robot_x, robot_y
        self.robot_dx, self.robot_dy = robot_dx, robot_dy

    def put_global_path_in_memory(self):
        if self.robot_x == self.end_x:
            self.global_path = [[self.robot_x, y] for y in range(self.robot_y, self.end_y + 1 if self.robot_y < self.end_y else self.end_y - 1, 1 if self.robot_y < self.end_y else -1)]
            self.global_orientation = [[0, -1 if self.robot_y > self.end_y else 1] for y in range(self.robot_y, self.end_y + 1 if self.robot_y < self.end_y else self.end_y - 1, 1 if self.robot_y < self.end_y else -1)]
        elif self.robot_y == self.end_y:
            self.global_path = [[x, self.robot_y] for x in range(self.robot_x, self.end_x + 1 if self.robot_x < self.end_x else self.end_x - 1, 1 if self.robot_x < self.end_x else -1)]
            self.global_orientation = [[-1 if self.robot_x > self.end_x else 1, 0] for x in range(self.robot_x, self.end_x + 1 if self.robot_x < self.end_x else self.end_x - 1, 1 if self.robot_x < self.end_x else -1)]
        else:
            self.global_path = [[x, int(self.robot_y + (((self.end_y - self.robot_y) / (self.end_x - self.robot_x)) * (x - self.robot_x)))] for x in range(self.robot_x, self.end_x + 1, 1 if self.robot_x < self.end_x else -1)]
            self.global_orientation = [[self.global_path[index + 1][0] - x, self.global_path[index + 1][1] - y] for index, (x, y) in enumerate(self.global_path[:-1])]

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
        print("self.collisions : ",self.collisions)
        
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
            ax.add_patch(plt.Rectangle((obstacle_x, obstacle_y), 1, 1, color='black'))
            ax.annotate('', (obstacle_x + 0.5, obstacle_y + 0.5), (obstacle_x + 0.5 + obstacle_dx, obstacle_y + 0.5 + obstacle_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
        for obstacle_id in self.obstacles_path:
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
                self.grid[i][j]['robot_distance'] = self.euclidian_distance(self.robot_x, self.robot_y, self.grid[i][j]['x'], self.grid[i][j]['y'])

    def put_robot_on_grid(self):
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[self.robot_y][self.robot_x]['robot'] = True
            self.grid[self.robot_y][self.robot_x]['robot_movement'] = [self.robot_dx, self.robot_dy]
        self.put_distance_of_each_nodes_to_robot_on_grid()

    def put_distance_of_each_nodes_to_end_on_grid(self):
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j]['end_distance'] = self.euclidian_distance(self.end_x, self.end_y, self.grid[i][j]['x'], self.grid[i][j]['y'])

    def put_end_on_grid(self):
        if self.is_inside_grid(self.end_x, self.end_y):
            self.grid[self.end_y][self.end_x]['end'] = True
        self.put_distance_of_each_nodes_to_end_on_grid()

    def put_repulsion(self, repulsion_x, repulsion_y, repulsion_factor):
        if self.is_inside_grid(repulsion_x, repulsion_y):
          
            self.grid[repulsion_y][repulsion_x]['repulsion_factor'] = self.grid[repulsion_y][repulsion_x]['repulsion_factor'] + repulsion_factor

    def put_repulsions(self, repulsions_x, repulsions_y, repulsions_factor):
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.put_repulsion(repulsion_x, repulsion_y, repulsion_factor)

    def put_obstacle(self, obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range):
        if self.is_inside_grid(obstacle_x, obstacle_y):
            self.grid[obstacle_y][obstacle_x]['obstacle'] = True
            self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [obstacle_dx, obstacle_dy]

    def remove_repulsion_on_grid(self, repulsion_x, repulsion_y, repulsion_factor):
        if self.is_inside_grid(repulsion_x, repulsion_y):
            self.grid[repulsion_y][repulsion_x]['repulsion_factor'] = self.grid[repulsion_y][repulsion_x]['repulsion_factor'] - repulsion_factor

    def remove_repulsions_on_grid(self, repulsions_x, repulsions_y, repulsions_factor):
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.remove_repulsion_on_grid(repulsion_x, repulsion_y, repulsion_factor)

    def put_distance_of_each_nodes_to_collision_on_grid(self, collision_x, collision_y):
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j]['total_obstacle_distance'] = self.grid[i][j]['total_obstacle_distance'] + self.euclidian_distance(collision_x, collision_y, self.grid[i][j]['x'], self.grid[i][j]['y'])

    def put_collision_on_grid(self):
        for timestep in self.collisions:
            for obstacle_id in self.collisions[timestep]:
                collision_x, collision_y = self.obstacles_path[obstacle_id]['movement'][timestep]
                collision_dx, collision_dy = self.obstacles_path[obstacle_id]['orientation'][timestep]
                major_range = self.obstacles_path[obstacle_id]['major_range'][timestep]
                minor_range = self.obstacles_path[obstacle_id]['minor_range'][timestep]
                self.put_obstacle(collision_x, collision_y, collision_dx, collision_dy, major_range, minor_range)
            
                repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(collision_x, collision_y, collision_dx, collision_dy, major_range, minor_range)
                self.plot_repulsion(repulsions_x, repulsions_y,"Raw data")
                
                offset_repulsions_x , offset_repulsions_y = self.get_offset_repulsion(repulsions_x,repulsions_y)    
                self.plot_repulsion(offset_repulsions_x, offset_repulsions_y,"Offset data")
                self.put_repulsions(repulsions_x, repulsions_y, repulsions_factor)
                self.put_repulsions(offset_repulsions_x, offset_repulsions_y, repulsions_factor)
                self.put_distance_of_each_nodes_to_collision_on_grid(collision_x, collision_y)

    def get_offset_repulsion(self, repulsion_x: list, repulsion_y: list) -> (list, list):
        xmin , ymin = min(repulsion_x), min(repulsion_y) 
        xmax , ymax = max(repulsion_x), max(repulsion_y)
        offset_x = []
        offset_y = []
        for x_cord in repulsion_x:
            for y_cord in repulsion_y:
                if x_cord == xmin: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord - offset)
                        offset_y.append(y_cord)
                        
                if x_cord == xmax: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord + offset)
                        offset_y.append(y_cord)
                
                if y_cord == ymin: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord)
                        offset_y.append(y_cord - offset)
                        
                if y_cord == ymax: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord )
                        offset_y.append(y_cord + offset)
                
                if x_cord == xmin and  y_cord == ymin: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord - offset)
                        offset_y.append(y_cord - offset)
                
                if x_cord == xmin and  y_cord == ymax: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord - offset)
                        offset_y.append(y_cord + offset)
                
                if x_cord == xmax and  y_cord == ymin: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord + offset)
                        offset_y.append(y_cord - offset)
                        
                if x_cord == xmax and  y_cord == ymax: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord + offset)
                        offset_y.append(y_cord + offset)
                
        return offset_x, offset_y
                        
                
            
    
    def put_distance_of_each_nodes_to_other_obstacles_on_grid(self, obstacle_x, obstacle_y):
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j]['total_obstacle_distance'] = self.grid[i][j]['total_obstacle_distance'] + self.euclidian_distance(obstacle_x, obstacle_y, self.grid[i][j]['x'], self.grid[i][j]['y'])

    def put_other_obstacles_on_grid(self):
        for timestep in self.collisions:
            for obstacle_id in self.obstacles_path:
                if obstacle_id not in self.collisions[timestep]:
                    obstacle_x, obstacle_y = self.obstacles_path[obstacle_id]['movement'][timestep]
                    obstacle_dx, obstacle_dy = self.obstacles_path[obstacle_id]['orientation'][timestep]
                    major_range = self.obstacles_path[obstacle_id]['major_range'][timestep]
                    minor_range = self.obstacles_path[obstacle_id]['minor_range'][timestep]
                    self.put_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)
                    repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)
                    offset_repulsions_x , offset_repulsions_y = self.get_offset_repulsion(repulsions_x,repulsions_y)
                    self.put_repulsions(repulsions_x, repulsions_y, repulsions_factor)
                    self.put_repulsions(offset_repulsions_x, offset_repulsions_y, repulsions_factor)
                    self.plot_repulsion(repulsions_x,repulsions_y,"put other obcstracle on the grid RAWWW")
                    self.plot_repulsion(offset_repulsions_x,offset_repulsions_y,"put other obcstracle on the grid with Offset")
                    
                    self.put_distance_of_each_nodes_to_other_obstacles_on_grid(obstacle_x, obstacle_y)
    
    def get_all_repulsions(self):
        all_repulsions_x, all_repulsions_y, all_repulsions_factor = [], [], []
        
        for obstacle_id, position in self.current_obstacles_position.items():
            obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range = position
            repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)
            all_repulsions_x.extend(repulsions_x)
            all_repulsions_y.extend(repulsions_y)
            all_repulsions_factor.extend(repulsions_factor)
        
        for timestep in self.collisions:
            for obstacle_id in self.collisions[timestep]:
                collision_x, collision_y = self.obstacles_path[obstacle_id]['movement'][timestep]
                collision_dx, collision_dy = self.obstacles_path[obstacle_id]['orientation'][timestep]
                major_range = self.obstacles_path[obstacle_id]['major_range'][timestep]
                minor_range = self.obstacles_path[obstacle_id]['minor_range'][timestep]
                repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(collision_x, collision_y, collision_dx, collision_dy, major_range, minor_range)
                all_repulsions_x.extend(repulsions_x)
                all_repulsions_y.extend(repulsions_y)
                all_repulsions_factor.extend(repulsions_factor)
        
        return all_repulsions_x, all_repulsions_y, all_repulsions_factor
    
    def move_robot_on_grid(self, timestep):
        current_x, current_y = self.robot_path[timestep - 1]
        if self.is_inside_grid(current_x, current_y):
            self.grid[current_y][current_x]['robot'] = False
            self.grid[current_y][current_x]['robot_movement'] = [0, 0]
        
        future_x, future_y = self.robot_path[timestep]
       
        if timestep < len(self.robot_orientation):
            future_dx, future_dy = self.robot_orientation[timestep]
            if self.is_inside_grid(future_x, future_y):
                self.grid[future_y][future_x]['robot'] = True
                self.grid[future_y][future_x]['robot_movement'] = [future_dx, future_dy]
            self.robot_x, self.robot_y = future_x, future_y
            self.robot_dx, self.robot_dy = future_dx, future_dy

    def move_obstacle_on_grid(self, obstacle_id, timestep):
        if obstacle_id in self.obstacles_path:
            current_x, current_y = self.obstacles_path[obstacle_id]['movement'][timestep - 1]
            current_dx, current_dy = self.obstacles_path[obstacle_id]['orientation'][timestep - 1]
            major_range = self.obstacles_path[obstacle_id]['major_range'][timestep - 1]
            minor_range = self.obstacles_path[obstacle_id]['minor_range'][timestep - 1]
            if self.is_inside_grid(current_x, current_y):
                self.grid[current_y][current_x]['obstacle'] = False
                self.grid[current_y][current_x]['obstacle_movement'] = [0, 0]
            repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(current_x, current_y, current_dx, current_dy, major_range, minor_range)
            self.remove_repulsions_on_grid(repulsions_x, repulsions_y, repulsions_factor)
            future_x, future_y = self.obstacles_path[obstacle_id]['movement'][timestep]
            future_dx, future_dy = self.obstacles_path[obstacle_id]['orientation'][timestep]
            if self.is_inside_grid(future_x, future_y):
                self.grid[future_y][future_x]['obstacle'] = True
                self.grid[future_y][future_x]['obstacle_movement'] = [future_dx, future_dy]
            repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(future_x, future_y, future_dx, future_dy, major_range, minor_range)
            self.put_repulsions(repulsions_x, repulsions_y, repulsions_factor)
            offset_repulsions_x , offset_repulsions_y = self.get_offset_repulsion(repulsions_x,repulsions_y)
            self.put_repulsions(offset_repulsions_x, offset_repulsions_y, repulsions_factor)
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
                if self.grid[i][j]['obstacle']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))
                    ax.annotate('', (j + 0.5, i + 0.5), (j + 0.5 + self.grid[i][j]['obstacle_movement'][0], i + 0.5 + self.grid[i][j]['obstacle_movement'][1]), arrowprops={'color': 'pink', 'arrowstyle': '<-'})
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='white')
                elif self.grid[i][j]['robot']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='green'))
                    ax.annotate('', (self.robot_x + 0.5, self.robot_y + 0.5), (self.robot_x + 0.5 + self.robot_dx, self.robot_y + 0.5 + self.robot_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j]['end']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='red'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j]['repulsion_factor'] > 0:
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

