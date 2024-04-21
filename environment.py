import node
import random as rn
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w, display='both'):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.display = display
        self.grid, self.obstacles = self.reset_environment()

    def is_inside_grid(self, x, y):
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    def reset_environment(self):
        grid = [[node.Node(j, i) for j in range(self.grid_w)] for i in range(self.grid_h)]
        obstacles = {}
        return grid, obstacles

    def put_start(self, start_x: int, start_y: int):
        self.start_x = start_x
        self.start_y = start_y
        self.grid[self.start_y][self.start_x]['start'] = True

    def remove_start(self):
        self.grid[self.start_y][self.start_x]['start'] = False
        self.start_x = None
        self.start_y = None

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
        temp = self.is_inside_grid(x, y)
        print(temp)
        if temp:
            self.grid[self.robot_y][self.robot_x]['robot'] = False
            self.grid[self.robot_y][self.robot_x]['robot_movement'] = [0, 0]
            self.grid[y][x]['robot'] = True
            self.robot_dx, self.robot_dy = x - self.robot_x, y - self.robot_y
            self.robot_x, self.robot_y = x, y
            print('wwwwwwwwwwwwwwwwwwwwwwwwwww', self.robot_x, self.robot_y, 'fefffefefefefefefef', self.robot_dx, self.robot_dy)
            self.grid[y][x]['robot_movement'] = [self.robot_dx, self.robot_dy]

    def put_end(self, end_x: int, end_y: int):
        self.end_x = end_x
        self.end_y = end_y
        self.grid[self.end_y][self.end_x]['end'] = True

    def remove_end(self):
        self.grid[self.end_y][self.end_x]['end'] = False
        self.end_x = None
        self.end_y = None

    def put_obstacle(self, obstacle_x: list, obstacle_y: list, obstacle_dx: list, obstacle_dy: list):
        obstacle_id = max(list(self.obstacles.keys())) + 1 if len(self.obstacles) > 0 else 0
        self.obstacles[obstacle_id] = [obstacle_x, obstacle_y, obstacle_dx, obstacle_dy]
        self.grid[obstacle_y][obstacle_x]['obstacle'] = True
        self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [obstacle_dx, obstacle_dy]

    def remove_obstacle(self, obstacle_id):
        obstacle_x, obstacle_y, obstacle_dx, obstacle_dy = self.obstacles[obstacle_id]
        self.grid[obstacle_y][obstacle_x]['obstacle'] = False
        self.grid[obstacle_y][obstacle_x]['obstacle_movement'] = [0, 0]
        # self.obstacles.pop(obstacle_id)

    def move_obstacle(self, obstacle_id):
        current_obstacle_x, current_obstacle_y, obstacle_dx, obstacle_dy = self.obstacles[obstacle_id]
        if self.is_inside_grid(current_obstacle_x, current_obstacle_y):
            obstacle_dx, obstacle_dy = self.grid[current_obstacle_y][current_obstacle_x]['obstacle_movement']
            future_obstacle_x, future_obstacle_y = current_obstacle_x + obstacle_dx, current_obstacle_y + obstacle_dy
            self.grid[current_obstacle_y][current_obstacle_x]['obstacle'] = False
            self.grid[current_obstacle_y][current_obstacle_x]['obstacle_movement'] = [0, 0]
            if self.is_inside_grid(future_obstacle_x, future_obstacle_y):
                self.grid[future_obstacle_y][future_obstacle_x]['obstacle'] = True
                self.grid[future_obstacle_y][future_obstacle_x]['obstacle_movement'] = [obstacle_dx, obstacle_dy]
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
                if element['start']:
                    if self.display == 'type':
                        text = text + f'|ST'
                    elif self.display == 'coordinates':
                        text = text + f'|({element["x"]}, {element["y"]})'
                    else:
                        text = text + f'|({element["x"]}, {element["y"]}), ST, {round(element["k"], 1) if element["k"] is not None else element["k"]}, {element["obstacle_movement"]}'
                elif element['robot']:
                    if self.display == 'type':
                        text = text + f'|RB'
                    elif self.display == 'coordinates':
                        text = text + f'|({element["x"]}, {element["y"]})'
                    else:
                        text = text + f'|({element["x"]}, {element["y"]}), RB, {round(element["k"], 1) if element["k"] is not None else element["k"]}, {element["obstacle_movement"]}'
                elif element['end']:
                    if self.display == 'type':
                        text = text + f'|EN'
                    elif self.display == 'coordinates':
                        text = text + f'|({element["x"]}, {element["y"]})'
                    else:
                        text = text + f'|({element["x"]}, {element["y"]}), EN, {round(element["k"], 1) if element["k"] is not None else element["k"]}, {element["obstacle_movement"]}'
                elif element['obstacle']:
                    if self.display == 'type':
                        text = text + f'|XX'
                    elif self.display == 'coordinates':
                        text = text + f'|({element["x"]}, {element["y"]})'
                    else:
                        text = text + f'|({element["x"]}, {element["y"]}), XX, {round(element["k"], 1) if element["k"] is not None else element["k"]}, {element["obstacle_movement"]}'
                else:
                    if self.display == 'type':
                        text = text + f'|OO'
                    elif self.display == 'coordinates':
                        text = text + f'|({element["x"]}, {element["y"]})'
                    else:
                        text = text + f'|({element["x"]}, {element["y"]}), OO, {round(element["k"], 1) if element["k"] is not None else element["k"]}, {element["obstacle_movement"]}'
            text = text + '|\n'
        return text

    def plot_environment(self, path=None):
        fig, ax = plt.subplots(1, 1, figsize=(15, 15))
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                if self.grid[i][j]['obstacle']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))
                    ax.annotate('', (j + 0.5, i + 0.5), (j + 0.5 + self.grid[i][j]['obstacle_movement'][0], i + 0.5 + self.grid[i][j]['obstacle_movement'][1]), arrowprops={'color': 'pink', 'arrowstyle': '<-'})
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else "NONE"}', horizontalalignment='center', verticalalignment='center', color='white')
                elif not self.grid[i][j]['obstacle'] and not self.grid[i][j]['start'] and not self.grid[i][j]['end'] and not self.grid[i][j]['robot']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='white'))
                    ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else "NONE"}', horizontalalignment='center', verticalalignment='center', color='black')
        # Plot start, robot, end, obstacles
        ax.add_patch(plt.Rectangle((self.start_x, self.start_y), 1, 1, color='green'))
        ax.add_patch(plt.Rectangle((self.end_x, self.end_y), 1, 1, color='red'))
        ax.add_patch(plt.Rectangle((self.robot_x + 0.25, self.robot_y + 0.25), 0.5, 0.5, color='yellow'))
        ax.annotate('', (self.robot_x + 0.5, self.robot_y + 0.5), (self.robot_x + 0.5 + self.robot_dx, self.robot_y + 0.5 + self.robot_dy), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
        if path is not None:
            ax.plot([x + 0.5 for x, y in path], [y + 0.5 for x, y in path], c='purple')
            ax.scatter([x + 0.5 for x, y in path], [y + 0.5 for x, y in path], c='blue')
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
    grid_w, grid_h = 5, 5
    start_x, start_y = 0, 0
    end_x, end_y = 4, 4
    obstacles = 5
    obstacle_x, obstacle_y = [rn.randint(0, grid_w - 1) for _ in range(obstacles)], [rn.randint(0, grid_w - 1) for _ in range(obstacles)]
    obstacle_dx, obstacle_dy = [rn.randint(-1, 1) for _ in range(obstacles)], [rn.randint(-1, 1) for _ in range(obstacles)]

    environment = Environment(grid_h, grid_w)
    environment.put_start(start_x, start_y)
    environment.put_robot(start_x, start_y, 0, 0)
    environment.put_end(end_x, end_y)
    environment.put_multiple_obstacles(obstacle_x, obstacle_y, obstacle_dx, obstacle_dy)
    print(environment)
    environment.plot_environment()
