import math
import threading as t

class BidirectionalBstar:
    def __init__(self, environment, obstacle_penalty, repulsion_penalty):
        self.environment = environment
        self.obstacle_penalty = obstacle_penalty
        self.repulsion_penalty = repulsion_penalty
        self.end_to_robot_open = []
        self.end_to_robot_closed = []
        self.robot_to_end_open = []
        self.robot_to_end_closed = []

    def euclidian_distance(self, x1, y1, x2, y2):
        return math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

    def manhattan_distance(self, x1, y1, x2, y2):
        return abs(x2 - x1) + abs(y2 - y1)

    def is_inside_grid(self, x, y):
        return -1 < x < self.environment.grid_w and -1 < y < self.environment.grid_h

    def is_valid(self, x, y):
        return not self.environment.grid[y][x]['obstacle'] and self.environment.grid[y][x]['k'] is not None and self.environment.grid[y][x]['repulsion_factor'] == 0

    def is_free_to_move(self, x, y):
        return not self.environment.grid[y][x]['obstacle'] and self.environment.grid[y][x]['repulsion_factor'] == 0

    def is_never_visited(self, x, y):
        return self.environment.grid[y][x] not in self.open and self.environment.grid[y][x] not in self.closed

    def get_offset_range(self, max_distance):
        return [i for i in range(-1 * max_distance, max_distance + 1, 1)]

    def find_all_neighbour_offsets(self, movement: str, max_distance: int):
        if movement == 'queen':
            moves = [[dx, dy] for dx in self.get_offset_range(max_distance) for dy in self.get_offset_range(max_distance)]
        elif movement == 'rook':
            moves = [[dx, dy] for dx in self.get_offset_range(max_distance) for dy in self.get_offset_range(max_distance)]
            for dx, dy in moves:
                if dx != 0 and dy != 0:
                    moves.remove([dx, dy])
        elif movement == 'bishop':
            moves = [[dx, dy] for dx in self.get_offset_range(max_distance) for dy in self.get_offset_range(max_distance)]
            for dx, dy in moves:
                if dx != dy:
                    moves.remove([dx, dy])
        moves.remove([0, 0])
        return moves

    def update_node_in_open_list(self, x, y, k):
        for index, _ in enumerate(self.open):
            if self.open[index]['x'] == x and self.open[index]['y'] == y:
                self.open[index]['k'] = k
                break

    def add_repulsion_penalty(self):
        for y in range(self.environment.grid_h):
            for x in range(self.environment.grid_w):
                self.environment.grid[y][x]['k'] = self.environment.grid[y][x]['k'] + (self.environment.grid[y][x]['repulsion_factor'] * self.repulsion_penalty) if self.environment.grid[y][x]['k'] is not None else self.environment.grid[y][x]['k']

    def calculate_cost_and_heuristics_from_both_sides(self):
        pass
