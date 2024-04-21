import math
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

    def square(self):
        offsets = []
        moves = [i for i in range(-1 * self.repulsion_distance, self.repulsion_distance, 1)]
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
            for dx, dy in self.square():
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
            if any([i['x'] == self.environment.start_x and i['y'] == self.environment.start_y for i in self.closed]):
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

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

    def get_future_moves(self, x, y):
        future_xykd = [[x + dx, y + dy, self.environment.grid[y + dy][x + dx]['k'], self.distance(self.environment.end_x, self.environment.end_y, x + dx, y + dy)] for dx, dy, _ in self.moves_xyc if self.is_inside_grid(x + dx, y + dy) and self.is_valid(x + dx, y + dy)]
        future_xykd.sort(key=lambda a: (a[2], a[3]))
        return future_xykd

    def is_in_path(self, x, y, path):
        for current_x, current_y in path:
            if x == path[(current_x, current_y)][0] and y == path[(current_x, current_y)][1]:
                return True
        return False

    def recursive_path_finder(self, current_x, current_y, path):
        print('PATH:', path)
        print('===============================================================')
        print('CURRENT:', current_x, current_y)
        self.environment.plot_environment(list(path.keys()))
        if current_x == self.environment.end_x and current_y == self.environment.end_y:
            path[(current_x, current_y)] = []
            return path
        else:
            future_xykd = self.get_future_moves(current_x, current_y)
            print('FUTURE:', future_xykd)
            if len(future_xykd) == 0:
                [past_x, past_y], past_xykd = path.popitems()
                past_xykd.remove([current_x, current_y])
                path[[past_x, past_y]] = past_xykd
                current_x, current_y = past_xykd[0]
                self.recursive_path_finder(current_x, current_y, path)
            else:
                path[(current_x, current_y)] = future_xykd
                current_x, current_y, _, _ = future_xykd[0]
                return self.recursive_path_finder(current_x, current_y, path)

    def raw_path_finder(self):
        path = {}
        path = self.recursive_path_finder(self.environment.robot_x, self.environment.robot_y, path)
        return path

    def remove_knots_from_path(self, path):
        new_path = []
        forward_index = 0
        forward_x, forward_y = path[forward_index]
        while forward_x != self.environment.end_x or forward_y != self.environment.end_y:
            new_path.append([forward_x, forward_y])
            reverse_path = path[:forward_index + 1:-1]
            for reverse_index, (reverse_x, reverse_y) in enumerate(reverse_path):
                if forward_x - reverse_x in [x for x, _, _ in self.moves_xyc] and forward_y - reverse_y in [y for _, y, _ in self.moves_xyc]:
                    new_path.append([reverse_x, reverse_y])
                    forward_index = len(path) - 1 - reverse_index
                    break
            forward_index = forward_index + 1
            forward_x, forward_y = path[forward_index]
        return new_path
