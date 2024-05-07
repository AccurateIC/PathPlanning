import math
import dubin
import numpy as np

class PathPlanner:
    def __init__(self, environment, obstacle_penalty: int, repulsion_penalty: int):
        self.environment = environment
        self.obstacle_penalty = obstacle_penalty
        self.repulsion_penalty = repulsion_penalty
        self.open = []
        self.closed = []

    def add_repulsion_penalty_from_numpy_array(self, array, start_value, end_value, obstacle_value):
        array = np.where(array == start_value, 0, array)
        array = np.where(array == end_value, 0, array)
        array = np.where(array == obstacle_value, 0, array)
        repulsions_y, repulsions_x = np.where(array != 0)
        repulsions_y, repulsions_x = repulsions_y.tolist(), repulsions_x.tolist()
        for x, y in zip(repulsions_x, repulsions_y):
            self.environment.grid[y][x]['k'] = self.environment.grid[y][x]['k'] + array[y][x] if self.environment.grid[y][x]['k'] is not None else self.environment.grid[y][x]['k']

    def euclidian_distance(self, x1, y1, x2, y2):
        return math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

    def manhattan_distance(self, x1, y1, x2, y2):
        return abs(x2 - x1) + abs(y2 - y1)

    def is_inside_grid(self, x, y):
        return -1 < x < self.environment.grid_w and -1 < y < self.environment.grid_h

    def is_valid(self, x, y):
        return not self.environment.grid[y][x]['obstacle'] and self.environment.grid[y][x]['k'] is not None

    def is_free_to_move(self, x, y):
        return not self.environment.grid[y][x]['obstacle'] and self.environment.grid[y][x]['repulsion_factor'] == 0

    def is_never_visited(self, x, y):
        return self.environment.grid[y][x] not in self.open and self.environment.grid[y][x] not in self.closed

    def get_offset(self, max_distance):
        return [i for i in range(-1 * max_distance, max_distance + 1, 1)]

    def get_moves(self, movement: str, distance):
        if movement == 'queen':
            moves = [[dx, dy] for dx in self.get_offset(distance) for dy in self.get_offset(distance)]
        elif movement == 'rook':
            moves = [[dx, dy] for dx in self.get_offset(distance) for dy in self.get_offset(distance)]
            for dx, dy in moves:
                if dx != 0 and dy != 0:
                    moves.remove([dx, dy])
        elif movement == 'bishop':
            moves = [[dx, dy] for dx in self.get_offset(distance) for dy in self.get_offset(distance)]
            for dx, dy in moves:
                if dx != dy:
                    moves.remove([dx, dy])
        moves.remove([0, 0])
        return moves

    def find_neighbours_and_offsets(self, x, y, movement, distance):
        return [[x + dx, y + dy, dx, dy] for dx, dy in self.get_moves(movement, distance)]

    def find_valid_neighbours_and_offsets(self, x, y, movement, distance):
        return [[x + dx, y + dy, dx, dy] for dx, dy in self.get_moves(movement, distance) if self.is_inside_grid(x + dx, y + dx) and self.is_valid(x + dx, y + dy)]

    def update_node_in_open_list(self, x, y, k):
        for index, _ in enumerate(self.open):
            if self.open[index]['x'] == x and self.open[index]['y'] == y:
                self.open[index]['k'] = k
                break

    def expand_neighbours_from_end_to_start(self, x, y, movement, distance, k_factor):
        for index_x, index_y, dx, dy in self.find_neighbours_and_offsets(x, y, movement, distance):
            if self.is_inside_grid(index_x, index_y):
                if self.is_free_to_move(index_x, index_y):
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x]['k'] = self.environment.grid[y][x]['k'] + self.euclidian_distance(x, y, index_x, index_y)
                        self.open.append(self.environment.grid[index_y][index_x])
                    else:
                        new_k = self.environment.grid[y][x]['k'] + self.euclidian_distance(0, 0, dx, dy)
                        if new_k < self.environment.grid[index_y][index_x]['k']:
                            self.update_node_in_open_list(index_x, index_y, new_k)
                else:
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x]['k'] = self.obstacle_penalty
                        self.open.append(self.environment.grid[index_y][index_x])
            else:
                continue
        self.open = sorted(self.open, key=lambda a: (k_factor * a['k']) + ((1 - k_factor) * self.euclidian_distance(self.environment.robot_x, self.environment.robot_y, a['x'], a['y'])))
        # self.open = sorted(self.open, key=lambda a: (a['k'], self.manhattan_distance(self.environment.robot_x, self.environment.robot_y, a['x'], a['y'])))
        # self.open = sorted(self.open, key=lambda a: a['k'])
        top_node = self.open.pop(0)
        return top_node

    def expand_neighbours_from_start_to_end(self, x, y, movement, distance, k_factor):
        for index_x, index_y, dx, dy in self.find_neighbours_and_offsets(x, y, movement, distance):
            if self.is_inside_grid(index_x, index_y):
                if self.is_free_to_move(index_x, index_y):
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x]['k'] = self.environment.grid[y][x]['k'] + self.euclidian_distance(x, y, index_x, index_y)
                        self.open.append(self.environment.grid[index_y][index_x])
                    else:
                        new_k = self.environment.grid[y][x]['k'] + self.euclidian_distance(0, 0, dx, dy)
                        if new_k < self.environment.grid[index_y][index_x]['k']:
                            self.update_node_in_open_list(index_x, index_y, new_k)
                else:
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x]['k'] = self.obstacle_penalty
                        self.open.append(self.environment.grid[index_y][index_x])
            else:
                continue
        self.open = sorted(self.open, key=lambda a: (k_factor * a['k']) + ((1 - k_factor) * self.euclidian_distance(self.environment.end_x, self.environment.end_y, a['x'], a['y'])))
        # self.open = sorted(self.open, key=lambda a: (a['k'], self.manhattan_distance(self.environment.robot_x, self.environment.robot_y, a['x'], a['y'])))
        # self.open = sorted(self.open, key=lambda a: a['k'])
        top_node = self.open.pop(0)
        return top_node

    def add_repulsion_penalty(self):
        for y in range(self.environment.grid_h):
            for x in range(self.environment.grid_w):
                self.environment.grid[y][x]['k'] = self.environment.grid[y][x]['k'] + (self.environment.grid[y][x]['repulsion_factor'] * self.repulsion_penalty) if self.environment.grid[y][x]['k'] is not None else self.environment.grid[y][x]['k']

    def calculate_cost_and_heuristics_from_end_to_start(self, movement, distance, k_factor):
        x, y = self.environment.end_x, self.environment.end_y
        self.environment.grid[y][x]['k'] = 0
        self.environment.grid[y][x]['b'] = None
        self.open.append(self.environment.grid[y][x])
        while True:
            if any([c['x'] == self.environment.robot_x and c['y'] == self.environment.robot_y for c in self.closed]):
                break
            else:
                top_node = self.expand_neighbours_from_end_to_start(x, y, movement, distance, k_factor)
                self.closed.append(top_node)
                if len(self.open) > 0:
                    x = self.open[0]['x']
                    y = self.open[0]['y']
        self.add_repulsion_penalty()

    def calculate_cost_and_heuristics_from_start_to_end(self, movement, distance, k_factor):
        x, y = self.environment.robot_x, self.environment.robot_y
        self.environment.grid[y][x]['k'] = 0
        self.environment.grid[y][x]['b'] = None
        self.open.append(self.environment.grid[y][x])
        while True:
            if any([c['x'] == self.environment.end_x and c['y'] == self.environment.end_y for c in self.closed]):
                break
            else:
                top_node = self.expand_neighbours_from_start_to_end(x, y, movement, distance, k_factor)
                self.closed.append(top_node)
                if len(self.open) > 0:
                    x = self.open[0]['x']
                    y = self.open[0]['y']
        self.add_repulsion_penalty()

    def raw_paths_finder(self, all_expanding_indices: list):
        paths = {}

    # def calculate_path_cost(self, path):
    #     return sum([self.environment.grid[y][x]['k'] for x, y in path])

    # def choose_best_path(self, costs: dict, paths: dict):
    #     assert len(costs) == len(paths)
    #     min_cost = float('inf')
    #     best_path = None
    #     for key in costs:
    #         if costs[key] < min_cost:
    #             min_cost = costs[key]
    #             best_path = paths[key]
    #     return best_path

    # def remove_knots_from_path(self, path):
    #     new_path = []
    #     forward_index = 0
    #     forward_x, forward_y = path[forward_index]
    #     while True:
    #         new_path.append([forward_x, forward_y])
    #         if forward_x != self.environment.end_x or forward_y != self.environment.end_y:
    #             reverse_path = path[:forward_index + 1:-1]
    #             for reverse_index, (reverse_x, reverse_y) in enumerate(reverse_path):
    #                 if forward_x - reverse_x in [x for x, _, _ in self.moves_xyc] and forward_y - reverse_y in [y for _, y, _ in self.moves_xyc]:
    #                     new_path.append([reverse_x, reverse_y])
    #                     forward_index = len(path) - 1 - reverse_index
    #                     break
    #             forward_index = forward_index + 1
    #             forward_x, forward_y = path[forward_index]
    #         else:
    #             return new_path

    # def plan_dubins_path(self, path, window_size, strides, curvature):
    #     dubins_path = []
    #     for start_index in range(0, len(path), strides):
    #         start_x, start_y = path[start_index]
    #         start_yaw = math.atan2(path[start_index + 1][1] - start_y, path[start_index + 1][0] - start_x)
    #         end_index = start_index + window_size - 1
    #         if end_index >= len(path) - 1:
    #             end_index = len(path) - 1
    #             end_x, end_y = path[end_index]
    #             end_yaw = math.atan2(self.environment.end_dy, self.environment.end_dx)
    #         else:
    #             end_x, end_y = path[end_index]
    #             end_yaw = math.atan2(path[end_index + 1][1] - end_y, path[end_index + 1][0] - end_x)
    #         path_x, path_y, path_yaw, mode, lengths = dubin.plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
    #         dubins_path = dubins_path + [[x, y] for x, y in zip(path_x.tolist(), path_y.tolist())]
    #     return dubins_path

    # def does_path_hit_obstacles(self, path):
    #     for x_path, y_path in path:
    #         if self.environment.grid[y_path][x_path]['obstacle']:
    #             return False
    #     return True

    # def find_neighbours(self, x, y, distance):
    #     neighbours = []
    #     for dx, dy, _ in [[i, j, self.euclidian_distance(0, 0, i, j)] for i in range(-1 * distance, distance + 1, 1) for j in range(-1 * distance, distance + 1, 1)]:
    #         index_x, index_y = x + dx, y + dy
    #         neighbours.append([index_x, index_y])
    #     return neighbours

    # def straight_line_path(self, start_x, start_y, end_x, end_y):
    #     path = []
    #     if start_x == end_x:
    #         for y in range(start_y, end_y + 1 if end_y - start_y > 0 else end_y - 1, 1 if end_y - start_y > 0 else -1):
    #             path.append([start_x, y])
    #     elif start_y == end_y:
    #         for x in range(start_x, end_x + 1 if end_x - start_x > 0 else end_x - 1, 1 if end_x - start_x > 0 else -1):
    #             path.append([x, start_y])
    #     else:
    #         pass
            # x, y = start_x, start_y
            # path.append([math.floor(x), math.floor(y)])
            # while True:
            #     if x <= end_x and y <= end_y:
            #         pass
                # for step_size in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
                #     x = x + step_size if end_x - start_x > 0 else x - step_size
                #     y = ((x * (end_y - start_y)) - (end_x * (end_y - start_y)) + (end_y * (end_x - start_x))) / (end_x - start_x)
                #     if [math.floor(x), math.floor(y)] in self.find_neighbours(path[-1][0], path[-1][1]):
                #         break
                # path.append([math.floor(x), math.floor(y)])
        # return path

    # def path_finder(self, path, min_distance):
    #     new_path = []
    #     for index1, (x, y) in enumerate(path):
    #         new_path.append([x, y])
    #         if any([self.environment.grid[index_y][index_x]['obstacle'] or self.environment.grid[index_y][index_x]['repulsion_factor'] > 0 for index_x, index_y in self.find_neighbours(x, y, min_distance)]):
    #             for index2, (future_x, future_y) in enumerate(path[index1:]):
    #                 if not self.environment.grid[future_y][future_x]:
    #                 astar_end_x, astar_end_y = 
    #             self.calculate_cost_and_heuristics(self.environment.robot_x, self.environment.robot_y, self.environment)
