import node
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w, display=[]):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.display = display
        self.grid, self.grid_obstacles = self.reset_grid()

    def is_inside_grid(self, x, y):
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    def find_neighbour_offsets(self, distance):
        moves = [i for i in range(-1 * distance, distance + 1, 1)]
        neighbours = [[dx, dy] for dx in moves for dy in moves]
        neighbours.remove([0, 0])
        return neighbours

    def is_valid(self, x, y):
        return not self.grid[y][x]['obstacle']

    def reset_grid(self):
        grid = [[node.Node(j, i, display=self.display) for j in range(self.grid_w)] for i in range(self.grid_h)]
        grid_obstacles = {}
        return grid, grid_obstacles

    def put_robot(self, robot):
        robot_x, robot_y = robot['movement'][0]
        robot_dx, robot_dy = robot['orientation'][0]
        if self.is_inside_grid(robot_x, robot_y):
            self.grid[robot_y][robot_x]['robot'] = True
            self.grid[robot_y][robot_x]['robot_orientation'] = [robot_dx, robot_dy]
        self.robot = robot

    def remove_robot(self):
        robot_x, robot_y = self.robot['movement'][0]
        if self.is_inside_grid(self.robot_x, self.robot_y):
            self.grid[robot_y][robot_x]['robot'] = False
            self.grid[robot_y][robot_x]['robot_orientation'] = [0, 0]
        self.robot = None

    def move_robot(self, timestep):
        current_x, current_y = self.robot['movement'][timestep - 1]
        if self.is_inside_grid(current_x, current_y):
            self.grid[current_y][current_x]['robot'] = False
            self.grid[current_y][current_x]['robot_orientation'] = [0, 0]
        future_x, future_y = self.robot['movement'][timestep]
        future_dx, future_dy = self.robot['orientation'][timestep]
        if self.is_inside_grid(future_x, future_y):
            self.grid[future_y][future_x]['robot'] = True
            self.grid[future_y][future_x]['robot_orientation'] = [future_dx, future_dy]

    def put_repulsion(self, repulsion_x, repulsion_y, repulsion_factor):
        if self.is_inside_grid(repulsion_x, repulsion_y):
            self.grid[repulsion_y][repulsion_x]['repulsion_factor'] = self.grid[repulsion_y][repulsion_x]['repulsion_factor'] + repulsion_factor

    def put_repulsions(self, repulsions_x, repulsions_y, repulsions_factor):
        for repulsion_x, repulsion_y, repulsion_factor in zip(repulsions_x, repulsions_y, repulsions_factor):
            self.put_repulsion(repulsion_x, repulsion_y, repulsion_factor)

    def put_repulsion_around(self, obstacle_x, obstacle_y, repulsion_distance):
        for dx, dy in self.find_neighbour_offsets(repulsion_distance):
            index_x, index_y = obstacle_x + dx, obstacle_y + dy
            if self.is_inside_grid(index_x, index_y):
                repulsion_factor = 1.0 / max(abs(dx), abs(dy))
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

    def remove_repulsion_around(self, obstacle_x, obstacle_y, repulsion_distance, repulsion_factor=0.0):
        for dx, dy in self.find_neighbour_offsets(repulsion_distance):
            repulsion_x, repulsion_y = obstacle_x + dx, obstacle_y + dy
            if self.is_inside_grid(repulsion_x, repulsion_y):
                self.remove_repulsion(repulsion_x, repulsion_y, repulsion_factor)

    def put_obstacles(self, obstacles: dict):
        for obstacle_id in obstacles:
            obstacle = obstacles[obstacle_id]
            self.grid_obstacles[obstacle_id] = obstacle
            obstacle_x, obstacle_y = obstacle['movement'][0]
            obstacle_dx, obstacle_dy = obstacle['orientation'][0]
            if self.is_inside_grid(obstacle_x, obstacle_y):
                self.grid[obstacle_y][obstacle_x]['obstacle'] = True
                self.grid[obstacle_y][obstacle_x]['obstacle_orientation'] = [obstacle_dx, obstacle_dy]
            self.put_repulsion_around(obstacle_x, obstacle_y, obstacle['repulsion'][0])

    def remove_obstacles(self, obstacles_id: list):
        for obstacle_id in obstacles_id:
            if obstacle_id in self.grid_obstacles:
                obstacle = self.grid_obstacles[obstacle_id]
                obstacle_x, obstacle_y = obstacle['movement'][0]
                if self.is_inside_grid(obstacle_x, obstacle_y):
                    self.grid[obstacle_y][obstacle_x]['obstacle'] = False
                    self.grid[obstacle_y][obstacle_x]['obstacle_orientation'] = [0, 0]
                self.remove_repulsion_around(obstacle_x, obstacle_y, obstacle['repulsion'][0])
                self.grid_obstacles.pop(obstacle_id)

    def move_all_obstacles(self, next_timestep):
        for obstacle_id in self.grid_obstacles:
            if obstacle_id in self.grid_obstacles:
                obstacle = self.grid_obstacles[obstacle_id]
                current_x, current_y = obstacle['movement'][next_timestep - 1]
                if self.is_inside_grid(current_x, current_y):
                    self.grid[current_y][current_x]['obstacle'] = False
                    self.grid[current_y][current_x]['obstacle_orientation'] = [0, 0]
                self.remove_repulsion_around(current_x, current_y, obstacle['repulsion'][next_timestep - 1])
                future_x, future_y = obstacle['movement'][next_timestep]
                future_dx, future_dy = obstacle['orientation'][next_timestep]
                if self.is_inside_grid(future_x, future_y):
                    self.grid[future_y][future_x]['obstacle'] = True
                    self.grid[future_y][future_x]['obstacle_orientation'] = [future_dx, future_dy]
                self.put_repulsion_around(future_x, future_y, obstacle['repulsion'][next_timestep])

    def put_end(self, end):
        end_x, end_y = end['movement'][0]
        end_dx, end_dy = end['orientation'][0]
        if self.is_inside_grid(end_x, end_y):
            self.grid[end_y][end_x]['end'] = True
            self.grid[end_y][end_x]['end_orientation'] = [end_dx, end_dy]
        self.end = end

    def remove_end(self):
        end_x, end_y = self.end['movement'][0]
        if self.is_inside_grid(end_x, end_y):
            self.grid[end_y][end_x]['end'] = False
            self.grid[end_y][end_x]['end_orientation'] = [0, 0]
        self.end = None

    def put_collision(self, collision_x, collision_y):
        if self.is_inside_grid(collision_x, collision_y):
            self.grid[collision_y][collision_x]['collision'] = True

    def put_collisions(self, collisions_x, collisions_y):
        for collision_x, collision_y in zip(collisions_x, collisions_y):
            self.put_collision(collision_x, collision_y)

    def __str__(self):
        text = ''
        for row in self.grid:
            for cell in row:
                text = text + cell.__str__()
            text = text + '\n'
        return text

    def plot_environment(self, show_text=True, show_obstacle_path=True, paths=None):
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                if self.grid[i][j]['obstacle']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))
                    ax.annotate('', (j + 0.5, i + 0.5), (j + 0.5 + self.grid[i][j]['obstacle_orientation'][0], i + 0.5 + self.grid[i][j]['obstacle_orientation'][1]), arrowprops={'color': 'pink', 'arrowstyle': '<-'})
                    if show_text:
                        ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='white')
                elif self.grid[i][j]['robot']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='green'))
                    ax.annotate('', (j + 0.5, i + 0.5), (j + 0.5 + self.grid[i][j]['robot_orientation'][0], i + 0.5 + self.grid[i][j]['robot_orientation'][1]), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                    if show_text:
                        ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j]['end']:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='red'))
                    ax.annotate('', (j + 0.5, i + 0.5), (j + 0.5 + self.grid[i][j]['end_orientation'][0], i + 0.5 + self.grid[i][j]['end_orientation'][1]), arrowprops={'color': 'purple', 'arrowstyle': '<-'})
                    if show_text:
                        ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                elif self.grid[i][j]['repulsion_factor'] > 0:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='grey'))
                    if show_text:
                        ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
                else:
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='white'))
                    if show_text:
                        ax.text(j + 0.5, i + 0.8, f'{round(self.grid[i][j]["k"], 1) if self.grid[i][j]["k"] is not None else ""}', horizontalalignment='center', verticalalignment='center', color='black')
        if show_obstacle_path:
            for obstacle_id in self.grid_obstacles:
                obstacle = self.grid_obstacles[obstacle_id]
                ax.plot([obstacle['movement'][timestep][0] + 0.5 for timestep in obstacle['movement']], [obstacle['movement'][timestep][1] + 0.5 for timestep in obstacle['movement']], label=f'obstacle_{obstacle_id}')
                ax.scatter([obstacle['movement'][timestep][0] + 0.5 for timestep in obstacle['movement']], [obstacle['movement'][timestep][1] + 0.5 for timestep in obstacle['movement']], label=f'obstacle_{obstacle_id}')
            ax.legend(loc='best')
        if isinstance(paths, dict):
            for key in paths:
                ax.plot([x + 0.5 for x, _ in paths[key] if -1 < x < self.grid_w], [y + 0.5 for _, y in paths[key] if -1 < y < self.grid_h], label=f'{key}')
                ax.scatter([x + 0.5 for x, _ in paths[key] if -1 < x < self.grid_w], [y + 0.5 for _, y in paths[key] if -1 < y < self.grid_h], label=f'{key}')
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
