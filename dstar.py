import math
import dubin
import numpy as np

class PathPlanner:
    def __init__(self, environment, movement: str, obstacle_penalty: int, repulsion_distance:int, repulsion_penalty: int):
        if movement == 'queen':
            self.moves_xyc = [[0, 1, 1], [0, -1, 1], [1, 0, 1], [-1, 0, 1], [1, 1, math.sqrt(2)], [-1, -1, math.sqrt(2)], [1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)]]
        elif movement == 'pawn_up':
            self.moves_xyc = [[0, 1, 1], [-1, 1, math.sqrt(2)], [1, 1, math.sqrt(2)]]
        elif movement == 'pawn_down':
            self.moves_xyc = [[0, -1, 1], [-1, -1, math.sqrt(2)], [1, -1, math.sqrt(2)]]
        elif movement == 'pawn_right':
            self.moves_xyc = [[1, 0, 1], [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]
        elif movement == 'pawn_left':
            self.moves_xyc = [[-1, 0, 1], [-1, 1, math.sqrt(2)], [-1, -1, math.sqrt(2)]]
        elif movement == 'rook':
            self.moves_xyc = [[0, 1, 1], [0, -1, 1], [1, 0, 1], [-1, 0, 1]]
        elif movement == 'bishop':
            self.moves_xyc = [[1, 1, math.sqrt(2)], [1, -1, math.sqrt(2)], [1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)]]
        elif movement == 'knight':
            self.moves_xyc = [[2, 1, 1 + math.sqrt(2)], [2, -1, 1 + math.sqrt(2)], [-2, 1, 1 + math.sqrt(2)], [-2, -1, 1 + math.sqrt(2)], [1, 2, 1 + math.sqrt(2)], [-1, 2, 1 + math.sqrt(2)], [1, -2, 1 + math.sqrt(2)], [-1, -2, 1 + math.sqrt(2)]]
        elif movement == 'custom':
            self.moves_xyc = []
        self.environment = environment
        self.obstacle_penalty = obstacle_penalty
        self.repulsion_distance = repulsion_distance
        self.repulsion_penalty = repulsion_penalty
        self.open = []
        self.closed = []

    def update_node_in_open_list(self, x, y, k):
        for index, _ in enumerate(self.open):
            if self.open[index]['x'] == x and self.open[index]['y'] == y:
                self.open[index]['k'] = k
                break

    def expand(self, x, y):
        for dx, dy, increment in self.moves_xyc:
            index_x, index_y = x + dx, y + dy
            if self.is_inside_grid(index_x, index_y):
                if not self.environment.grid[index_y][index_x]['obstacle']:
                    if self.environment.grid[index_y][index_x] not in self.open and self.environment.grid[index_y][index_x] not in self.closed:
                        self.environment.grid[index_y][index_x]['k'] = self.environment.grid[y][x]['k'] + increment
                        self.open.append(self.environment.grid[index_y][index_x])
                    else:
                        new_k = self.environment.grid[y][x]['k'] + increment
                        if new_k < self.environment.grid[index_y][index_x]['k']:
                            self.update_node_in_open_list(index_x, index_y, new_k)
                else:
                    if self.environment.grid[index_y][index_x] not in self.open and self.environment.grid[index_y][index_x] not in self.closed:
                        self.environment.grid[index_y][index_x]['h'] = self.obstacle_penalty
                        self.environment.grid[index_y][index_x]['k'] = self.obstacle_penalty
                        self.open.append(self.environment.grid[index_y][index_x])
            else:
                continue
        self.open = sorted(self.open, key=lambda a: a['k'])
        top_node = self.open.pop(0)
        return top_node

    def is_inside_grid(self, x, y):
        return -1 < x < self.environment.grid_w and -1 < y < self.environment.grid_h

    def add_repulsion_penalty_from_numpy_array(self, array, start_value, end_value, obstacle_value):
        array = np.where(array == start_value, 0, array)
        array = np.where(array == end_value, 0, array)
        array = np.where(array == obstacle_value, 0, array)
        repulsions_y, repulsions_x = np.where(array != 0)
        repulsions_y, repulsions_x = repulsions_y.tolist(), repulsions_x.tolist()
        for x, y in zip(repulsions_x, repulsions_y):
            self.environment.grid[y][x]['k'] = self.environment.grid[y][x]['k'] + array[y][x] if self.environment.grid[y][x]['k'] is not None else self.environment.grid[y][x]['k']

    def square_repulsion(self):
        offsets = []
        moves = [i for i in range(-1 * self.repulsion_distance, self.repulsion_distance + 1, 1)]
        for i in moves:
            for j in moves:
                if i == 0 and j == 0:
                    pass
                else:
                    offsets.append([i, j])
        return offsets

    def add_repulsion_penalty(self):
        for obstacle_id in self.environment.obstacles:
            obstacle_x, obstacle_y, obstacle_dx, obstacle_dy = self.environment.obstacles[obstacle_id]
            for dx, dy in self.square_repulsion():
                index_x, index_y = obstacle_x - dx, obstacle_y - dy
                if self.is_inside_grid(index_x, index_y):
                    self.environment.grid[index_y][index_x]['k'] = self.environment.grid[index_y][index_x]['k'] + self.repulsion_penalty if self.environment.grid[index_y][index_x]['k'] is not None else self.environment.grid[index_y][index_x]['k']

    def calculate_cost_and_heuristics(self):
        x, y = self.environment.end_x, self.environment.end_y
        self.environment.grid[y][x]['k'] = 0
        self.environment.grid[y][x]['h'] = 0
        self.environment.grid[y][x]['b'] = None
        self.open.append(self.environment.grid[y][x])
        while True:
            if any([i['x'] == self.environment.robot_x and i['y'] == self.environment.robot_y for i in self.closed]):
                break
            else:
                top_node = self.expand(x, y)
                self.closed.append(top_node)
                if len(self.open) > 0:
                    x = self.open[0]['x']
                    y = self.open[0]['y']
        self.add_repulsion_penalty()

    def is_valid(self, x, y):
        return not self.environment.grid[y][x]['obstacle'] and self.environment.grid[y][x]['k'] is not None

    # def distance(self, x1, y1, x2, y2):
    #     return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

    # def get_future_moves(self, x, y, path):
    #     future_xykd = [[x + dx, y + dy, self.environment.grid[y + dy][x + dx]['k'], self.distance(self.environment.end_x, self.environment.end_y, x + dx, y + dy)] for dx, dy, _ in self.moves_xyc if self.is_inside_grid(x + dx, y + dy) and self.is_valid(x + dx, y + dy) and not self.is_in_path(x + dx, y + dy, path)]
    #     future_xykd.sort(key=lambda a: (a[2], a[3]))
    #     return future_xykd

    # def is_in_path(self, x, y, path):
    #     for current_x, current_y in path:
    #         if x == current_x and y == current_y:
    #             return True
    #     return False

    # def recursive_path_finder(self, child_x, child_y, path):
    #     print('PATH:', path.keys())
    #     print('===============================================================')
    #     print('CHILD:', child_x, child_y)
    #     self.environment.plot_environment(list(path.keys()))
    #     if child_x == self.environment.end_x and child_y == self.environment.end_y:
    #         path[(child_x, child_y)] = []
    #         return path
    #     else:
    #         future_xykd = self.get_future_moves(child_x, child_y, path)
    #         print('FUTURE:', future_xykd)
    #         if len(future_xykd) != 0:
    #             path[(child_x, child_y)] = future_xykd
    #             future_x, future_y, _, _ = future_xykd[0]
    #             return self.recursive_path_finder(future_x, future_y, path)
    #         else:
    #             (past_x, past_y), past_xykd = path.popitem()
    #             print('PAST:', past_x, past_y, past_xykd)
    #             past_xykd.pop(0)
    #             print('PAST:', past_x, past_y, past_xykd)
    #             # if len(past_xykd) != 0:
    #             #     path[(past_x, past_y)] = past_xykd
    #             #     current_x, current_y, _, _ = past_xykd[0]
    #             # else:
    #             self.recursive_path_finder(child_x, child_y, path)

    # def raw_path_finder(self):
    #     path = {}
    #     path = self.recursive_path_finder(self.environment.robot_x, self.environment.robot_y, path)
    #     return path

    def find_valid_neighbours(self, x, y, path):
        neighbours = []
        for dx, dy, _ in self.moves_xyc:
            index_x, index_y = x + dx, y + dy
            if self.is_inside_grid(index_x, index_y) and self.is_valid(index_x, index_y) and [index_x, index_y] not in path:
                neighbours.append([index_x, index_y])
        neighbours.sort(key=lambda a: self.environment.grid[a[1]][a[0]]['k'])
        return neighbours

    def raw_paths_finder(self, all_expanding_indices: list):
        paths = {}
        for neighbour_index in range(len(self.moves_xyc)):
            path = []
            current_x, current_y = self.environment.robot_x, self.environment.robot_y
            counter = 0
            while True:
                path.append([current_x, current_y])
                if current_x == self.environment.end_x and current_y == self.environment.end_y:
                    paths[neighbour_index] = path
                    break
                else:
                    neighbours = self.find_valid_neighbours(current_x, current_y, path)
                    if len(neighbours) != 0:
                        if len(neighbours) > neighbour_index and counter in all_expanding_indices:
                            current_x, current_y = neighbours[neighbour_index]
                        else:
                            current_x, current_y = neighbours[0]
                        counter = counter + 1
                    else:
                        break
        return paths

    def calculate_path_cost(self, path):
        return sum([self.environment.grid[y][x]['k'] for x, y in path])

    def choose_best_path(self, costs: dict, paths: dict):
        assert len(costs) == len(paths)
        min_cost = float('inf')
        best_path = None
        for key in costs:
            if costs[key] < min_cost:
                min_cost = costs[key]
                best_path = paths[key]
        return best_path

    # def remove_knots_from_path(self, path):
    #     new_path = []
    #     forward_index = 0
    #     forward_x, forward_y = path[forward_index]
    #     while True:
    #         new_path.append([forward_x, forward_y])
    #         if forward_x == self.environment.end_x and forward_y == self.environment.end_y:
    #             return new_path
    #         else:
    #             reverse_path = path[:forward_index + 1:-1]
    #             hit = False
    #             for reverse_index, (reverse_x, reverse_y) in enumerate(reverse_path):
    #                 if forward_x - reverse_x in [x for x, _, _ in self.moves_xyc] and forward_y - reverse_y in [y for _, y, _ in self.moves_xyc]:
    #                     hit = True
    #                     break
    #             if hit:
    #                 forward_index = len(path) - 1 - reverse_index
    #                 forward_x, forward_y = reverse_x, reverse_y
    #             else:
    #                 forward_index = forward_index + 1
    #                 forward_x, forward_y = path[forward_index]

    def remove_knots_from_path(self, path):
        new_path = []
        forward_index = 0
        forward_x, forward_y = path[forward_index]
        while True:
            new_path.append([forward_x, forward_y])
            if forward_x != self.environment.end_x or forward_y != self.environment.end_y:
                reverse_path = path[:forward_index + 1:-1]
                for reverse_index, (reverse_x, reverse_y) in enumerate(reverse_path):
                    if forward_x - reverse_x in [x for x, _, _ in self.moves_xyc] and forward_y - reverse_y in [y for _, y, _ in self.moves_xyc]:
                        new_path.append([reverse_x, reverse_y])
                        forward_index = len(path) - 1 - reverse_index
                        break
                forward_index = forward_index + 1
                forward_x, forward_y = path[forward_index]
            else:
                return new_path

    def plan_dubins_path(self, path, window_size, strides, curvature):
        dubins_path = []
        print(path)
        for start_index in range(0, len(path), strides):
            start_x, start_y = path[start_index]
            start_yaw = math.atan2(path[start_index + 1][1] - start_y, path[start_index + 1][0] - start_x)
            end_index = start_index + window_size - 1
            if end_index >= len(path) - 1:
                end_index = len(path) - 1
                end_x, end_y = path[end_index]
                end_yaw = math.atan2(self.environment.end_dy, self.environment.end_dx)
            else:
                end_x, end_y = path[end_index]
                end_yaw = math.atan2(path[end_index + 1][1] - end_y, path[end_index + 1][0] - end_x)
            path_x, path_y, path_yaw, mode, lengths = dubin.plan_dubins_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
            dubins_path = dubins_path + [[x, y] for x, y in zip(path_x.tolist(), path_y.tolist())]
        return dubins_path

    def does_path_hit_obstacles(self, path):
        print('DUBINS PATH:', path)
        for x_path, y_path in path:
            if self.environment.grid[y_path][x_path]['obstacle']:
                return False
        return True
