import node
import random as rn
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w, repulsion_distance=1):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.repulsion_distance = repulsion_distance
        self.repulsions = [[dx, dy] for dy in range(-1 * self.repulsion_distance, self.repulsion_distance + 1, 1) for dx in range(-1 * self.repulsion_distance, self.repulsion_distance + 1, 1)]
        self.repulsions.remove([0, 0])
        self.grid, self.obstacles = self.reset_environment()

    def is_inside_grid(self, x, y):
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    def is_valid(self, x, y):
        return not self.grid[y][x]['obstacle']

    def reset_environment(self):
        grid = [[node.Node(j, i) for j in range(self.grid_w)] for i in range(self.grid_h)]
        obstacles = {}
        return grid, obstacles

    def put_robot(self, robot_x: int, robot_y: int, robot_dx: int, robot_dy: int):
        self.robot_x = robot_x
        self.robot_y = robot_y
        self.robot_dx = robot_dx
        self.robot_dy = robot_dy
        self.grid[self.robot_y][self.robot_x]['robot'] = True
        self.grid[self.robot_y][self.robot_x]['robot_movement'] = [robot_dx, robot_dy]

    def remove_robot(self):
        self.grid[self.robot_y][self.robot_x]['robot'] = False
        self.grid[self.robot_y][self.robot_x]['robot_movement'] = [0, 0]
        self.robot_x, self.robot_y = None, None
        self.robot_dx, self.robot_dy = 0, 0

    def move_robot(self, x, y):
        if self.is_inside_grid(x, y):
            self.grid[self.robot_y][self.robot_x]['robot'] = False
            self.grid[self.robot_y][self.robot_x]['robot_movement'] = [0, 0]
            self.grid[y][x]['robot'] = True
            self.robot_dx, self.robot_dy = x - self.robot_x, y - self.robot_y
            self.robot_x, self.robot_y = x, y
            self.grid[y][x]['robot_movement'] = [self.robot_dx, self.robot_dy]

    def put_end(self, end_x: int, end_y: int, end_dx: int, end_dy: int):
        self.end_x = end_x
        self.end_y = end_y
        self.end_dx = end_dx
        self.end_dy = end_dy
        self.grid[self.end_y][self.end_x]['end'] = True
        self.grid[self.end_y][self.end_x]['end_movement'] = [end_dx, end_dy]

    def remove_end(self):
        self.grid[self.end_y][self.end_x]['end'] = False
        self.grid[self.end_y][self.end_x]['end_movement'] = [0, 0]
        self.end_x = None
        self.end_y = None

    def put_repulsion(self, x, y):
        for dx, dy in self.repulsions:
            index_x, index_y = x + dx, y + dy
            if self.is_inside_grid(index_x, index_y) and self.is_valid(index_x, index_y):
                self.grid[index_y][index_x]['repulsions'].append([x, y])

    # def remove_repulsion(self, x, y):

    def put_obstacle(self, obstacle_x: list, obstacle_y: list, obstacle_dx: list, obstacle_dy: list):
        obstacle_id = max(list(self.obstacles.keys())) + 1 if len(self.obstacles) > 0 else 0
        self.obstacles[obstacle_id] = [obstacle_x, obstacle_y, obstacle_dx, obstacle_dy]
        self.grid[obstacle_y][obstacle_x]['obstacle'] = True
        self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [obstacle_dx, obstacle_dy]
        self.put_repulsion(obstacle_x, obstacle_y)

    def remove_obstacle(self, obstacle_id):
        obstacle_x, obstacle_y, obstacle_dx, obstacle_dy = self.obstacles[obstacle_id]
        self.grid[obstacle_y][obstacle_x]['obstacle'] = False
        self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [0, 0]
        for dx, dy in self.repulsions:
            index_x, index_y = obstacle_x + dx, obstacle_y + dy
            if [obstacle_x, obstacle_y] in self.grid[index_y][index_x]['repulsion']:
                self.grid[index_y][index_x]['repulsions'].remove([obstacle_x, obstacle_y])
        self.obstacles.pop(obstacle_id)

    def move_obstacle(self, obstacle_id):
        current_obstacle_x, current_obstacle_y, obstacle_dx, obstacle_dy = self.obstacles[obstacle_id]
        for dx, dy in self.repulsions:
            index_x, index_y = current_obstacle_x + dx, current_obstacle_y + dy
            self.grid[index_y][index_x]['repulsions'].remove([current_obstacle_x, current_obstacle_y])
        if self.is_inside_grid(current_obstacle_x, current_obstacle_y):
            future_obstacle_x, future_obstacle_y = current_obstacle_x + obstacle_dx, current_obstacle_y + obstacle_dy
            self.grid[current_obstacle_y][current_obstacle_x]['obstacle'] = False
            self.grid[current_obstacle_y][current_obstacle_x]['obstacle_movement'] = [0, 0]
            if self.is_inside_grid(future_obstacle_x, future_obstacle_y):
                self.grid[future_obstacle_y][future_obstacle_x]['obstacle'] = True
                self.grid[future_obstacle_y][future_obstacle_x]['obstacle_movement'] = [obstacle_dx, obstacle_dy]
            for dx, dy in self.repulsions:
                index_x, index_y = future_obstacle_x + dx, future_obstacle_y + dy
                if self.is_inside_grid(index_x, index_y) and self.is_valid(index_x, index_y):
                    self.grid[index_y][index_x]['repulsions'].append([future_obstacle_x, future_obstacle_y])
            self.obstacles[obstacle_id] = [future_obstacle_x, future_obstacle_y, obstacle_dx, obstacle_dy]

    def put_obstacles(self, obstacles_x, obstacles_y, obstacles_dx, obstacles_dy):
        for obstacle_x, obstacle_y, obstacle_dx, obstacle_dy in zip(obstacles_x, obstacles_y, obstacles_dx, obstacles_dy):
            self.put_obstacle(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy)

    def remove_obstacles(self, obstacles_id):
        for obstacle_id in obstacles_id:
            self.remove_obstacle(obstacle_id)

    def move_obstacles(self, obstacles_id, timesteps):
        for _ in range(timesteps):
            for obstacle_id in obstacles_id:
                self.move_obstacle(obstacle_id)

    def __str__(self):
        text = ''
        for row in self.grid:
            for element in row:
                if element['robot']:
                    text = text + '|  START  '
                elif element['end']:
                    text = text + '|   END   '
                elif element['obstacle']:
                    text = text + '|OBSTACLE '
                elif len(element['repulsions']):
                    text = text + '|REPULSION'
                else:
                    text = text + '|  EMPTY  '
            text = text + '|\n'
        return text

    def plot_environment(self, paths=None):
        fig, ax = plt.subplots(1, 1, figsize=(15, 15))
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                if self.grid[i][j]['obstacle']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))
                    ax.annotate('', (j + 0.5, i + 0.5), (j + 0.5 + self.grid[i][j]['obstacle_movement'][0], i + 0.5 + self.grid[i][j]['obstacle_movement'][1]), arrowprops={'color': 'pink', 'arrowstyle': '<-'})
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='white')
                elif self.grid[i][j]['robot']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='green'))
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                    ax.annotate('', (self.robot_x + 0.5, self.robot_y + 0.5), (self.robot_x + 0.5 + self.robot_dx, self.robot_y + 0.5 + self.robot_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                elif self.grid[i][j]['end']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='red'))
                    # ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                    ax.annotate('', (self.end_x + 0.5, self.end_y + 0.5), (self.end_x + 0.5 + self.end_dx, self.end_y + 0.5 + self.end_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                # elif len(self.grid[i][j]['repulsions']) > 0:
                #     ax.add_patch(plt.Rectangle((j, i), 1, 1, color='grey'))
                #     ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
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
        # Set axis limits and aspect
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
    grid_w, grid_h = 10, 10
    robot_x, robot_y = 0, 0
    robot_dx, robot_dy = 0, -1
    end_x, end_y = 4, 4
    end_dx, end_dy = 1, 0
    repulsion_distance = 1
    obstacles = 5
    obstacle_x, obstacle_y = [rn.randint(0, grid_w - 1) for _ in range(obstacles)], [rn.randint(0, grid_w - 1) for _ in range(obstacles)]
    obstacle_dx, obstacle_dy = [rn.randint(-1, 1) for _ in range(obstacles)], [rn.randint(-1, 1) for _ in range(obstacles)]

    environment = Environment(grid_h, grid_w, repulsion_distance)
    environment.put_robot(robot_x, robot_y, robot_dx, robot_dy)
    environment.put_end(end_x, end_y, end_dx, end_dy)
    environment.put_obstacles(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy)
    print(environment)
    environment.plot_environment()
