import node
import random as rn
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w, display=[]):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.display = display
        self.grid, self.grid_obstacles = self.reset_grid()

    def is_inside_grid(self, x, y):
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    def find_neighbours(self, x, y, distance):
        moves = [i for i in range(-1 * distance, distance + 1, 1)]
        neighbours = [[x + dx, y + dy] for dx in moves for dy in moves]
        neighbours.remove([x, y])
        return neighbours

    def is_valid(self, x, y):
        return not self.grid[y][x]['obstacle']

    def reset_grid(self):
        grid = [[node.Node(j, i, display=self.display) for j in range(self.grid_w)] for i in range(self.grid_h)]
        grid_obstacles = {}
        return grid, grid_obstacles

    def put_robot(self, x, y, dx, dy):
        if self.is_inside_grid(x, y):
            self.grid[y][x]['robot'] = True
            self.grid[y][x]['robot_movement'] = [dx, dy]
        self.robot_x, self.robot_y = x, y
        self.robot_dx, self.robot_dy = dx, dy

    def remove_robot(self):
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[self.robot_y][self.robot_x]['robot'] = False
            self.grid[self.robot_y][self.robot_x]['robot_movement'] = [0, 0]
        self.robot_x, self.robot_y = None, None
        self.robot_dx, self.robot_dy = 0, 0

    def move_robot(self, future_x, future_y, future_dx, future_dy):
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[self.robot_y][self.robot_x]['robot'] = False
            self.grid[self.robot_y][self.robot_x]['robot_movement'] = [0, 0]
        if self.is_inside_grid(future_x, future_y):
            self.grid[future_y][future_x]['robot'] = True
            self.grid[future_y][future_x]['robot_movement'] = [future_dx, future_dy]
        self.robot_x, self.robot_y = future_x, future_y
        self.robot_dx, self.robot_dy = future_dx, future_dy

    def put_repulsion(self, obstacle_x, obstacle_y, repulsion_distance):
        for x, y in self.find_neighbours(obstacle_x, obstacle_y, repulsion_distance):
            if self.is_inside_grid(x, y):
                self.grid[y][x]['repulsion_factor'] = self.grid[y][x]['repulsion_factor'] + (1.0 / max(abs(obstacle_x - x), abs(obstacle_y - y)))

    def put_obstacle(self, obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_distance):
        if self.is_inside_grid(obstacle_x, obstacle_y):
            self.grid[obstacle_y][obstacle_x]['obstacle'] = True
            self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [obstacle_dx, obstacle_dy]
        obstacle_id = max(list(self.grid_obstacles.keys())) + 1 if len(self.grid_obstacles) > 0 else 0
        self.grid_obstacles[obstacle_id] = [obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_distance]
        self.put_repulsion(obstacle_x, obstacle_y, repulsion_distance)

    def put_obstacles(self, obstacles_x, obstacles_y, obstacles_dx, obstacles_dy, repulsion_distances):
        for obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_distance in zip(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy, repulsion_distances):
            self.put_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_distance)

    def remove_repulsion(self, obstacle_x, obstacle_y, repulsion_distance):
        for x, y in self.find_neighbours(obstacle_x, obstacle_y, repulsion_distance):
            if self.is_inside_grid(x, y):
                self.grid[y][x]['repulsion_factor'] = self.grid[y][x]['repulsion_factor'] - (1.0 / max(abs(obstacle_x - x), abs(obstacle_y - y)))

    def remove_obstacle(self, obstacle_id):
        if obstacle_id in self.grid_obstacles:
            obstacle_x, obstacle_y, obstacle_dx, obstacle_dy, repulsion_distance = self.grid_obstacles[obstacle_id]
            if self.is_inside_grid(obstacle_x, obstacle_y):
                self.grid[obstacle_y][obstacle_x]['obstacle'] = False
                self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [0, 0]
            self.remove_repulsion(obstacle_x, obstacle_y, repulsion_distance)
            self.grid_obstacles.pop(obstacle_id)

    def remove_obstacles(self, obstacle_ids):
        for obstacle_id in obstacle_ids:
            self.remove_obstacle(obstacle_id)

    def move_obstacle(self, obstacle_id):
        if obstacle_id in self.grid_obstacles:
            current_x, current_y, dx, dy, repulsion_distance = self.grid_obstacles[obstacle_id]
            if self.is_inside_grid(current_x, current_y):
                self.grid[current_y][current_x]['obstacle'] = False
                self.grid[current_y][current_x]['obstacle_movement'] = [0, 0]
            self.remove_repulsion(current_x, current_y, repulsion_distance)
            future_x, future_y = current_x + dx, current_y + dy
            if self.is_inside_grid(future_x, future_y):
                self.grid[future_y][future_x]['obstacle'] = True
                self.grid[future_y][future_x]['obstacle_movement'] = [dx, dy]
            self.put_repulsion(future_x, future_y, repulsion_distance)
            self.grid_obstacles[obstacle_id] = [future_x, future_y, dx, dy, repulsion_distance]

    def move_obstacles(self, obstacle_ids):
        for obstacle_id in obstacle_ids:
            self.move_obstacle(obstacle_id)

    def put_end(self, x, y, dx, dy):
        if self.is_inside_grid(x, y):
            self.grid[y][x]['end'] = True
            self.grid[y][x]['end_movement'] = [dx, dy]
        self.end_x, self.end_y = x, y
        self.end_dx, self.end_dy = dx, dy

    def remove_end(self):
        if self.is_inside_grid(self.end_x, self.end_y):
            self.grid[self.end_y][self.end_x]['end'] = False
            self.grid[self.end_y][self.end_x]['end_movement'] = [0, 0]
        self.end_x, self.end_y = None, None
        self.end_dx, self.end_dy = 0, 0

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
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='white')
                elif self.grid[i][j]['robot']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='green'))
                    ax.annotate('', (self.robot_x + 0.5, self.robot_y + 0.5), (self.robot_x + 0.5 + self.robot_dx, self.robot_y + 0.5 + self.robot_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j]['end']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='red'))
                    ax.annotate('', (self.end_x + 0.5, self.end_y + 0.5), (self.end_x + 0.5 + self.end_dx, self.end_y + 0.5 + self.end_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j]['repulsion_factor'] > 0:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='grey'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                else:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='white'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
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

if __name__ == '__main__':
    env = Environment(8, 8, display=['repulsion_factor'])
    env.put_robot(0, 0, 1, 0)
    env.put_end(7, 7, 1, 0)
    env.put_obstacle(3, 3, 1, 1, 2)
    env.put_obstacle(6, 4, 1, 1, 1)
    env.plot_environment()
    print(env)
