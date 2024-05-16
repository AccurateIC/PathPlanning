import node
import math
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w, display=[]):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.display = display
        self.reset_environment()

    def reset_environment(self):
        self.current_obstacles_position = {}
        self.obstacles_path = {}
        self.collisions = []
        self.global_path = []

    def is_inside_grid(self, x, y):
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    # def find_neighbour_offsets(self, distance_x, distance_y):
    #     moves_x = [i for i in range(-1 * distance_x, distance_x + 1, 1)]
    #     moves_y = [i for i in range(-1 * distance_y, distance_y + 1, 1)]
    #     neighbours = [[dx, dy] for dx in moves_x for dy in moves_y]
    #     neighbours.remove([0, 0])
    #     return neighbours

    def euclidian_distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

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

####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################

    def put_robot_in_memory(self, robot_x, robot_y, robot_dx, robot_dy):
        self.robot_x, self.robot_y = robot_x, robot_y
        self.robot_dx, self.robot_dy = robot_dx, robot_dy

    def put_end_in_memory(self, end_x, end_y, end_dx, end_dy):
        self.end_x, self.end_y = end_x, end_y
        self.end_dx, self.end_dy = end_dx, end_dy

    def get_global_path_in_memory(self):
        if self.robot_x == self.end_x:
            self.global_path = [[self.robot_x, y] for y in range(self.robot_y, self.end_y + 1 if self.robot_y < self.end_y else self.end_y - 1, 1 if self.robot_y < self.end_y else -1)]
        elif self.robot_y == self.end_y:
            self.global_path = [[x, self.robot_y] for x in range(self.robot_x, self.end_x + 1 if self.robot_x < self.end_x else self.end_x - 1, 1 if self.robot_x < self.end_x else -1)]
        else:
            self.global_path = [[x, int(self.robot_y + (((self.end_y - self.robot_y) / (self.end_x - self.robot_x)) * (x - self.robot_x)))] for x in range(self.robot_x, self.end_x + 1, 1 if self.robot_x < self.end_x else -1)]

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

    # def remove_robot(self):
    #     if self.is_inside_grid(self.robot_x, self.robot_y):
    #         self.grid[self.robot_y][self.robot_x]['robot'] = False
    #         self.grid[self.robot_y][self.robot_x]['robot_movement'] = [0, 0]
    #     self.robot_x, self.robot_y = None, None
    #     self.robot_dx, self.robot_dy = 0, 0
    #     for i in range(self.grid_h):
    #         for j in range(self.grid_w):
    #             self.grid[i][j]['robot_distance'] = 0.0

    # def remove_end(self):
    #     if self.is_inside_grid(self.end_x, self.end_y):
    #         self.grid[self.end_y][self.end_x]['end'] = False
    #         self.grid[self.end_y][self.end_x]['end_movement'] = [0, 0]
    #     self.end_x, self.end_y = None, None
    #     self.end_dx, self.end_dy = 0, 0
    #     for i in range(self.grid_h):
    #         for j in range(self.grid_w):
    #             self.grid[i][j]['end_distance'] = 0.0

    # def remove_repulsion_on_grid(self, repulsion_x, repulsion_y, repulsion_factor):
    #     if self.is_inside_grid(repulsion_x, repulsion_y):
    #         self.grid[repulsion_y][repulsion_x]['repulsion_factor'] = self.grid[repulsion_y][repulsion_x]['repulsion_factor'] - repulsion_factor

    # def remove_repulsions_on_grid(self, repulsions_x, repulsions_y, repulsions_factor):
    #     for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
    #         self.remove_repulsion_on_grid(repulsion_x, repulsion_y, repulsion_factor)

    # def remove_obstacle(self, obstacle_id):
    #     if obstacle_id in self.obstacles:
    #         obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsions_x, repulsions_y, repulsions_factor = self.obstacles[obstacle_id]
    #         if self.is_inside_grid(obstacle_x, obstacle_y):
    #             self.grid[obstacle_y][obstacle_x]['obstacle'] = False
    #             self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [0, 0]
    #         for repulsion_x, repulsion_y, repulsion_factor in (repulsions_x, repulsions_y, repulsions_factor):
    #             if self.is_inside_grid(repulsion_x, repulsion_y):
    #                 self.remove_repulsion(repulsion_x, repulsion_y, repulsion_factor)
    #         self.obstacles.pop(obstacle_id)
    #         self.obstacles_path.pop(obstacle_id)

    # def remove_obstacles(self, obstacle_ids):
    #     for obstacle_id in obstacle_ids:
    #         self.remove_obstacle(obstacle_id)

    def find_collision_points(self, robot_path=None):
        if robot_path is None:
            for timestep, (robot_x, robot_y) in enumerate(self.global_path):
                for obstacle_id in self.obstacles_path:
                    if self.obstacles_path[obstacle_id]['movement'][timestep] == [robot_x, robot_y]:
                        self.collisions.append(timestep)
        else:
            for timestep, (robot_x, robot_y) in enumerate(robot_path):
                for obstacle_id in self.obstacles_path:
                    if self.obstacles_path[obstacle_id]['movement'][timestep] == [robot_x, robot_y]:
                        self.collisions.append(timestep)
        self.collisions.sort()

####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################

    def put_distance_of_each_nodes_to_robot_on_grid(self):
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j]['robot_distance'] = self.euclidian_distance(self.robot_x, self.robot_y, self.grid[i][j]['x'], self.grid[i][j]['y'])

    def put_robot_on_grid(self):
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[self.robot_y][self.robot_x]['robot'] = True
            self.grid[self.robot_y][self.robot_x]['robot_movement'] = [self.robot_dx, self.robot_dy]
        self.put_distance_of_each_nodes_to_robot_on_grid()

    def move_robot_on_grid(self, future_x, future_y, future_dx, future_dy):
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[self.robot_y][self.robot_x]['robot'] = False
            self.grid[self.robot_y][self.robot_x]['robot_movement'] = [0, 0]
        if self.is_inside_grid(future_x, future_y):
            self.grid[future_y][future_x]['robot'] = True
            self.grid[future_y][future_x]['robot_movement'] = [future_dx, future_dy]
        self.robot_x, self.robot_y = future_x, future_y
        self.robot_dx, self.robot_dy = future_dx, future_dy

    def put_distance_of_each_nodes_to_end_on_grid(self):
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j]['end_distance'] = self.euclidian_distance(self.end_x, self.end_y, self.grid[i][j]['x'], self.grid[i][j]['y'])

    def put_end_on_grid(self):
        if self.is_inside_grid(self.end_x, self.end_y):
            self.grid[self.end_y][self.end_x]['end'] = True
            self.grid[self.end_y][self.end_x]['end_movement'] = [self.end_dx, self.end_dy]
        self.put_distance_of_each_nodes_to_end_on_grid()

    def put_repulsion_on_grid(self, repulsion_x, repulsion_y, repulsion_factor):
        if self.is_inside_grid(repulsion_x, repulsion_y):
            self.grid[repulsion_y][repulsion_x]['repulsion_factor'] = self.grid[repulsion_y][repulsion_x]['repulsion_factor'] + repulsion_factor

    def put_repulsions_on_grid(self, repulsions_x, repulsions_y, repulsions_factor):
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.put_repulsion_on_grid(repulsion_x, repulsion_y, repulsion_factor)

    def put_obstacle_on_grid(self, obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range):
        if self.is_inside_grid(obstacle_x, obstacle_y):
            self.grid[obstacle_y][obstacle_x]['obstacle'] = True
            self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [obstacle_dx, obstacle_dy]
        repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)
        self.put_repulsions_on_grid(repulsions_x, repulsions_y, repulsions_factor)

    def move_obstacle_on_grid(self, obstacle_id):
        if obstacle_id in self.current_obstacles_position:
            current_x, current_y, current_dx, current_dy, major_range, minor_range = self.current_obstacles_position[obstacle_id]
            if self.is_inside_grid(current_x, current_y):
                self.grid[current_y][current_x]['obstacle'] = False
                self.grid[current_y][current_x]['obstacle_movement'] = [0, 0]
            repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(current_x, current_y, current_dx, current_dy, major_range, minor_range)
            self.remove_repulsions(repulsions_x, repulsions_y, repulsions_factor)
            future_x, future_y = self.obstacles_path[obstacle_id][self.obstacles_path[obstacle_id].index([current_x, current_y]) + 1]
            if self.is_inside_grid(future_x, future_y):
                self.grid[future_y][future_x]['obstacle'] = True
                self.grid[future_y][future_x]['obstacle_movement'] = [current_dx, current_dy]
            repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(future_x, future_y, current_dx, current_dy, major_range, minor_range)
            self.put_repulsions(repulsions_x, repulsions_y, repulsions_factor)
            self.current_obstacles_position[obstacle_id] = [future_x, future_y, current_dx, current_dy, major_range, minor_range]

    def move_obstacles_on_grid(self):
        for obstacle_id in self.current_obstacles_position:
            self.move_obstacle(obstacle_id)

    def put_collision_as_obstacles_on_grid(self):
        for timestep in self.collisions:
            for obstacle_id in self.obstacles_path:
                obstacle_x, obstacle_y = self.obstacles_path[obstacle_id]['movement'][timestep]
                obstacle_dx, obstacle_dy = self.obstacles_path[obstacle_id]['orientation'][timestep]
                major_range = self.obstacles_path[obstacle_id]['major_range'][timestep]
                minor_range = self.obstacles_path[obstacle_id]['minor_range'][timestep]
                self.put_obstacle_on_grid(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)
                repulsions_x, repulsions_y, repulsions_factor = self.get_oval_repulsion_around_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, major_range, minor_range)
                self.put_repulsions_on_grid(repulsions_x, repulsions_y, repulsions_factor)

####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################

    def get_occupancy_grid(self):
        self.grid = [[node.Node(j, i, display=self.display) for j in range(self.grid_w)] for i in range(self.grid_h)]
        self.put_robot_on_grid()
        self.put_end_on_grid()
        self.put_collision_as_obstacles_on_grid()

    def __str__(self):
        text = ''
        for row in self.grid:
            for cell in row:
                text = text + cell.__str__()
            text = text + '\n'
        return text

    def plot_environment(self, robot_path=None):
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                if self.grid[i][j]['obstacle']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))
                    ax.annotate('', (j + 0.5, i + 0.5), (j + 0.5 + self.grid[i][j]['obstacle_movement'][0], i + 0.5 + self.grid[i][j]['obstacle_movement'][1]), arrowprops={'color': 'pink', 'arrowstyle': '<-'})
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='white')
                elif self.grid[i][j]['robot']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='green'))
                    ax.annotate('', (self.robot_x + 0.5, self.robot_y + 0.5), (self.robot_x + 0.5 + self.robot_dx, self.robot_y + 0.5 + self.robot_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j]['end']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='red'))
                    ax.annotate('', (self.end_x + 0.5, self.end_y + 0.5), (self.end_x + 0.5 + self.end_dx, self.end_y + 0.5 + self.end_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j]['repulsion_factor'] > 0:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='grey'))
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["repulsion_factor"], 1) if self.grid[i][j]["repulsion_factor"] > 0.0 else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                else:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='white'))
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
        # if robot_path is not None:
        #     ax.plot([x + 0.5 for x, y in robot_path], [y + 0.5 for x, y in robot_path], label=f'ROBOT')
        #     ax.scatter([x + 0.5 for x, y in robot_path], [y + 0.5 for x, y in robot_path], label=f'ROBOT')
        if isinstance(robot_path, dict):
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
        # ax.set_aspect('equal')
        ax.invert_yaxis()
        ax.grid(True, alpha=1)
        plt.show()
