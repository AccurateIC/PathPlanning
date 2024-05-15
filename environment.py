import node
import math
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w, display=[]):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.display = display
        self.grid, self.grid_obstacles = self.reset_grid()

    def is_inside_grid(self, x, y):
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    def find_neighbour_offsets(self, distance_x, distance_y):
        moves_x = [i for i in range(-1 * distance_x, distance_x + 1, 1)]
        moves_y = [i for i in range(-1 * distance_y, distance_y + 1, 1)]
        neighbours = [[dx, dy] for dx in moves_x for dy in moves_y]
        neighbours.remove([0, 0])
        return neighbours

    def is_valid(self, x, y):
        return not self.grid[y][x]['obstacle']

    def reset_grid(self):
        grid = [[node.Node(j, i, display=self.display) for j in range(self.grid_w)] for i in range(self.grid_h)]
        grid_obstacles = {}
        return grid, grid_obstacles

    def euclidian_distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

    def put_robot(self, robot_x, robot_y, robot_dx, robot_dy):
        if self.is_inside_grid(robot_x, robot_y):
            self.grid[robot_y][robot_x]['robot'] = True
            self.grid[robot_y][robot_x]['robot_movement'] = [robot_dx, robot_dy]
        self.robot_x, self.robot_y = robot_x, robot_y
        self.robot_dx, self.robot_dy = robot_dx, robot_dy
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j]['robot_distance'] = self.euclidian_distance(self.robot_x, self.robot_y, self.grid[i][j]['x'], self.grid[i][j]['y'])

    def remove_robot(self):
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[self.robot_y][self.robot_x]['robot'] = False
            self.grid[self.robot_y][self.robot_x]['robot_movement'] = [0, 0]
        self.robot_x, self.robot_y = None, None
        self.robot_dx, self.robot_dy = 0, 0
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j]['robot_distance'] = 0.0

    def move_robot(self, future_x, future_y, future_dx, future_dy):
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[self.robot_y][self.robot_x]['robot'] = False
            self.grid[self.robot_y][self.robot_x]['robot_movement'] = [0, 0]
        if self.is_inside_grid(future_x, future_y):
            self.grid[future_y][future_x]['robot'] = True
            self.grid[future_y][future_x]['robot_movement'] = [future_dx, future_dy]
        self.robot_x, self.robot_y = future_x, future_y
        self.robot_dx, self.robot_dy = future_dx, future_dy

    def put_repulsion(self, repulsion_x, repulsion_y, repulsion_factor):
        if self.is_inside_grid(repulsion_x, repulsion_y):
            self.grid[repulsion_y][repulsion_x]['repulsion_factor'] = self.grid[repulsion_y][repulsion_x]['repulsion_factor'] + repulsion_factor

    def put_repulsions(self, repulsions_x, repulsions_y, repulsions_factor):
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.put_repulsion(repulsion_x, repulsion_y, repulsion_factor)

    def put_repulsion_around(self, obstacle_x, obstacle_y, repulsion_x, repulsion_y, repulsion_factor):
        for dx, dy in self.find_neighbour_offsets(repulsion_x, repulsion_y):
            index_x, index_y = obstacle_x + dx, obstacle_y + dy
            if self.is_inside_grid(index_x, index_y):
                self.put_repulsion(index_x, index_y, repulsion_factor)

    def remove_repulsion(self, repulsion_x, repulsion_y, repulsion_factor=0.0):
        if self.is_inside_grid(repulsion_x, repulsion_y):
            if repulsion_factor > 0.0:
                self.grid[repulsion_y][repulsion_x]['repulsion_factor'] = self.grid[repulsion_y][repulsion_x]['repulsion_factor'] - repulsion_factor
            else:
                self.grid[repulsion_y][repulsion_x]['repulsion_factor'] = 0.0

    def remove_repulsions(self, repulsions_x, repulsions_y, repulsions_factor):
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.remove_repulsion(repulsion_x, repulsion_y, repulsion_factor)

    def remove_repulsion_around(self, obstacle_x, obstacle_y, repulsion_x, repulsion_y, repulsion_factor=0.0):
        for dx, dy in self.find_neighbour_offsets(repulsion_x, repulsion_y):
            index_x, index_y = obstacle_x + dx, obstacle_y + dy
            if self.is_inside_grid(index_x, index_y):
                self.remove_repulsion(index_x, index_y, repulsion_factor)

    def put_obstacle(self, obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_x, repulsion_y):
        if self.is_inside_grid(obstacle_x, obstacle_y):
            self.grid[obstacle_y][obstacle_x]['obstacle'] = True
            self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [obstacle_dx, obstacle_dy]
        obstacle_id = max(list(self.grid_obstacles.keys())) + 1 if len(self.grid_obstacles) > 0 else 0
        self.grid_obstacles[obstacle_id] = [obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_x, repulsion_y]
        for dx, dy in self.find_neighbour_offsets(repulsion_x, repulsion_y):
            index_x, index_y = obstacle_x + dx, obstacle_y + dy
            if self.is_inside_grid(index_x, index_y):
                repulsion_factor = 1.0 / max(abs(dx), abs(dy))
                self.put_repulsion(index_x, index_y, repulsion_factor)

    def put_obstacles(self, obstacles_x, obstacles_y, obstacles_dx, obstacles_dy, repulsions_x, repulsions_y):
        for obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_x, repulsion_y in zip(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy, repulsions_x, repulsions_y):
            self.put_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_x, repulsion_y)

    def remove_obstacle(self, obstacle_id):
        if obstacle_id in self.grid_obstacles:
            obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_x, repulsion_y = self.grid_obstacles[obstacle_id]
            if self.is_inside_grid(obstacle_x, obstacle_y):
                self.grid[obstacle_y][obstacle_x]['obstacle'] = False
                self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [0, 0]
            for dx, dy in self.find_neighbour_offsets(repulsion_x, repulsion_y):
                index_x, index_y = obstacle_x + dx, obstacle_y + dy
                if self.is_inside_grid(index_x, index_y):
                    self.remove_repulsion(index_x, index_y)
            self.grid_obstacles.pop(obstacle_id)

    def remove_obstacles(self, obstacle_ids):
        for obstacle_id in obstacle_ids:
            self.remove_obstacle(obstacle_id)

    def move_obstacle(self, obstacle_id):
        if obstacle_id in self.grid_obstacles:
            current_x, current_y, current_dx, current_dy, repulsion_x, repulsion_y = self.grid_obstacles[obstacle_id]
            if self.is_inside_grid(current_x, current_y):
                self.grid[current_y][current_x]['obstacle'] = False
                self.grid[current_y][current_x]['obstacle_movement'] = [0, 0]
            for dx, dy in self.find_neighbour_offsets(repulsion_x, repulsion_y):
                index_x, index_y = current_x + dx, current_y + dy
                if self.is_inside_grid(index_x, index_y):
                    self.remove_repulsion(index_x, index_y)
            future_x, future_y = current_x + current_dx, current_y + current_dy
            if self.is_inside_grid(future_x, future_y):
                self.grid[future_y][future_x]['obstacle'] = True
                self.grid[future_y][future_x]['obstacle_movement'] = [dx, dy]
            for dx, dy in self.find_neighbour_offsets(repulsion_x, repulsion_y):
                index_x, index_y = future_x + dx, future_y + dy
                if self.is_inside_grid(index_x, index_y):
                    repulsion_factor = 1.0 / max(abs(dx), abs(dy))
                    self.put_repulsion(index_x, index_y, repulsion_factor)
            self.grid_obstacles[obstacle_id] = [future_x, future_y, current_dx, current_dy, repulsion_x, repulsion_y]

    def move_obstacles(self, obstacle_ids):
        for obstacle_id in obstacle_ids:
            self.move_obstacle(obstacle_id)

    def put_end(self, end_x, end_y, end_dx, end_dy):
        if self.is_inside_grid(end_x, end_y):
            self.grid[end_y][end_x]['end'] = True
            self.grid[end_y][end_x]['end_movement'] = [end_dx, end_dy]
        self.end_x, self.end_y = end_x, end_y
        self.end_dx, self.end_dy = end_dx, end_dy
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j]['end_distance'] = self.euclidian_distance(self.end_x, self.end_y, self.grid[i][j]['x'], self.grid[i][j]['y'])

    def remove_end(self):
        if self.is_inside_grid(self.end_x, self.end_y):
            self.grid[self.end_y][self.end_x]['end'] = False
            self.grid[self.end_y][self.end_x]['end_movement'] = [0, 0]
        self.end_x, self.end_y = None, None
        self.end_dx, self.end_dy = 0, 0
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                self.grid[i][j]['end_distance'] = 0.0

    def __str__(self):
        text = ''
        for row in self.grid:
            for cell in row:
                text = text + cell.__str__()
            text = text + '\n'
        return text

    def plot_environment(self, paths=None):
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
                else:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='white'))
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
        if isinstance(paths, dict):
            for key in paths:
                ax.plot([x + 0.5 for x, y in paths[key]], [y + 0.5 for x, y in paths[key]], label=f'{key}')
                ax.scatter([x + 0.5 for x, y in paths[key]], [y + 0.5 for x, y in paths[key]], label=f'{key}')
            ax.legend(loc='best')
        elif isinstance(paths, list):
            ax.plot([x + 0.5 for x, y in paths], [y + 0.5 for x, y in paths], c='violet')
            ax.scatter([x + 0.5 for x, y in paths], [y + 0.5 for x, y in paths], c='blue')
        ax.set_xlim(0, self.grid_w)
        ax.set_ylim(0, self.grid_h)
        ax.set_xticks([i for i in range(self.grid_w + 1)])
        ax.set_yticks([i for i in range(self.grid_h + 1)])
        ax.tick_params(axis='both')
        # ax.set_aspect('equal')
        ax.invert_yaxis()
        ax.grid(True, alpha=1)
        plt.show()
