import math
import heapq
from path_planning_utils import dubin
from matplotlib import pyplot as plt
import numpy as np

class PathPlanner:
    def __init__(self, environment, obstacle_penalty: int, repulsion_penalty: int):
        self.environment = environment
       
        self.obstacle_penalty = obstacle_penalty
        self.repulsion_penalty = repulsion_penalty
        self.open = []
        self.closed = set()
        self.open_set = set()


    def euclidian_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def manhattan_distance(self, x1, y1, x2, y2):
        return abs(x2 - x1) + abs(y2 - y1)

    def is_inside_grid(self, x, y):
        return 0 <= x < self.environment.grid_w and 0 <= y < self.environment.grid_h

    def is_valid(self, x, y):
        cell = self.environment.grid[y][x]
        return not cell.obstacle and cell.k is not None and cell.repulsion_factor == 0

    def is_free_to_move(self, x, y):
        cell = self.environment.grid[y][x]
        return not cell.obstacle and cell.repulsion_factor == 0

    def is_never_visited(self, x, y):
        return (x, y) not in self.open_set and (x, y) not in self.closed

    def get_offset_range(self, max_distance):
        return range(-max_distance, max_distance + 1)

    def find_all_neighbour_offsets(self, movement, max_distance):
        if movement == 'queen':
            return [[dx, dy] for dx in self.get_offset_range(max_distance) for dy in self.get_offset_range(max_distance) if dx != 0 or dy != 0]
        elif movement == 'rook':
            return [[dx, dy] for dx in self.get_offset_range(max_distance) for dy in self.get_offset_range(max_distance) if dx == 0 or dy == 0]
        elif movement == 'bishop':
            return [[dx, dy] for dx in self.get_offset_range(max_distance) for dy in self.get_offset_range(max_distance) if abs(dx) == abs(dy)]
        return []

    def update_node_in_open_list(self, x, y, k):
        for node in self.open:
            if node[1].x == x and node[1].y == y:
                node[1].k = k
                heapq.heapify(self.open)
                break

    def add_repulsion_penalty(self):
        for row in self.environment.grid:
            for cell in row:
                if cell.k is not None:
                    cell.k += cell.repulsion_factor * self.repulsion_penalty

    def sort_based_on_weighted_distance_to_robot_and_heuristic(self, node, k_factor):
        return (k_factor * node.k) + ((1 - k_factor) * node.robot_distance)

    def sort_based_on_weighted_distance_to_end_and_heuristic(self, node, k_factor):
        return (k_factor * node.k) + ((1 - k_factor) * node.end_distance)

    def sort_based_on_weighted_distance_to_robot_and_heuristic_and_obstacle(self, node, k_factor, o_factor):
        return (k_factor * node.k) + ((1 - k_factor) * node.robot_distance) + (o_factor * node.total_obstacle_distance)

    def sort_based_on_weighted_distance_to_end_and_heuristic_and_obstacle(self, node, k_factor, o_factor):
        return (k_factor * node.k) + ((1 - k_factor) * node.end_distance) + (o_factor * node.total_obstacle_distance)

    def expand_all_neighbours_from_end_to_robot(self, x, y, movement, k_factor):
        for dx, dy in self.find_all_neighbour_offsets(movement, 1):
            index_x, index_y = x + dx, y + dy
            if self.is_inside_grid(index_x, index_y):
                if self.is_free_to_move(index_x, index_y):
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x].k = self.environment.grid[y][x].k + self.euclidian_distance(x, y, index_x, index_y)
                        heapq.heappush(self.open, (self.sort_based_on_weighted_distance_to_robot_and_heuristic(self.environment.grid[index_y][index_x], k_factor), self.environment.grid[index_y][index_x]))
                        self.open_set.add((index_x, index_y))
                    else:
                        new_k = self.environment.grid[y][x].k + self.euclidian_distance(0, 0, dx, dy)
                        if new_k < self.environment.grid[index_y][index_x].k:
                            self.update_node_in_open_list(index_x, index_y, new_k)
                else:
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x].k = self.obstacle_penalty
                        heapq.heappush(self.open, (self.sort_based_on_weighted_distance_to_robot_and_heuristic(self.environment.grid[index_y][index_x], k_factor), self.environment.grid[index_y][index_x]))
                        self.open_set.add((index_x, index_y))
        top_node = heapq.heappop(self.open)[1]
        self.open_set.remove((top_node.x, top_node.y))
        return top_node

    def calculate_all_cost_and_heuristics_from_end_to_robot(self, movement, k_factor):
        x, y = self.environment.end_x, self.environment.end_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while True:
            if any((node.x == self.environment.robot_x and node.y == self.environment.robot_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_all_neighbours_from_end_to_robot(x, y, movement, k_factor)
                self.closed.add((top_node.x, top_node.y))
                if self.open:
                    x, y = self.open[0][1].x, self.open[0][1].y
        self.add_repulsion_penalty()

    def expand_all_neighbours_from_robot_to_end(self, x, y, movement, k_factor):
        for dx, dy in self.find_all_neighbour_offsets(movement, 1):
            index_x, index_y = x + dx, y + dy
            if self.is_inside_grid(index_x, index_y):
                if self.is_free_to_move(index_x, index_y):
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x].k = self.environment.grid[y][x].k + self.euclidian_distance(x, y, index_x, index_y)
                        heapq.heappush(self.open, (self.sort_based_on_weighted_distance_to_end_and_heuristic(self.environment.grid[index_y][index_x], k_factor), self.environment.grid[index_y][index_x]))
                        self.open_set.add((index_x, index_y))
                    else:
                        new_k = self.environment.grid[y][x].k + self.euclidian_distance(0, 0, dx, dy)
                        if new_k < self.environment.grid[index_y][index_x].k:
                            self.update_node_in_open_list(index_x, index_y, new_k)
                else:
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x].k = self.obstacle_penalty
                        heapq.heappush(self.open, (self.sort_based_on_weighted_distance_to_end_and_heuristic(self.environment.grid[index_y][index_x], k_factor), self.environment.grid[index_y][index_x]))
                        self.open_set.add((index_x, index_y))
        top_node = heapq.heappop(self.open)[1]
        self.open_set.remove((top_node.x, top_node.y))
        return top_node

    def calculate_all_cost_and_heuristics_from_robot_to_end(self, movement, k_factor):
        x, y = self.environment.robot_x, self.environment.robot_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while True:
            if any((node.x == self.environment.end_x and node.y == self.environment.end_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_all_neighbours_from_robot_to_end(x, y, movement, k_factor)
                self.closed.add((top_node.x, top_node.y))
                if self.open:
                    x, y = self.open[0][1].x, self.open[0][1].y
        self.add_repulsion_penalty()

    def expand_non_obstacle_neighbours_from_end_to_robot(self, x, y, movement, k_factor):
        for dx, dy in self.find_all_neighbour_offsets(movement, 1):
            index_x, index_y = x + dx, y + dy
            if self.is_inside_grid(index_x, index_y):
                if self.is_free_to_move(index_x, index_y):
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x].k = self.environment.grid[y][x].k + self.euclidian_distance(x, y, index_x, index_y)
                        heapq.heappush(self.open, (self.sort_based_on_weighted_distance_to_robot_and_heuristic(self.environment.grid[index_y][index_x], k_factor), self.environment.grid[index_y][index_x]))
                        self.open_set.add((index_x, index_y))
                    else:
                        new_k = self.environment.grid[y][x].k + self.euclidian_distance(0, 0, dx, dy)
                        if new_k < self.environment.grid[index_y][index_x].k:
                            self.update_node_in_open_list(index_x, index_y, new_k)
        top_node = heapq.heappop(self.open)[1]
        self.open_set.remove((top_node.x, top_node.y))
        return top_node

    def calculate_non_obstacle_cost_and_heuristics_from_end_to_robot(self, movement, k_factor):
        x, y = self.environment.end_x, self.environment.end_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while True:
            if any((node.x == self.environment.robot_x and node.y == self.environment.robot_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_non_obstacle_neighbours_from_end_to_robot(x, y, movement, k_factor)
                self.closed.add((top_node.x, top_node.y))
                if self.open:
                    x, y = self.open[0][1].x, self.open[0][1].y
        self.add_repulsion_penalty()

    def expand_non_obstacle_neighbours_from_robot_to_end(self, x, y, movement, k_factor):
        for dx, dy in self.find_all_neighbour_offsets(movement, 1):
            index_x, index_y = x + dx, y + dy
            if self.is_inside_grid(index_x, index_y):
                if self.is_free_to_move(index_x, index_y):
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x].k = self.environment.grid[y][x].k + self.euclidian_distance(x, y, index_x, index_y)
                        heapq.heappush(self.open, (self.sort_based_on_weighted_distance_to_end_and_heuristic(self.environment.grid[index_y][index_x], k_factor), self.environment.grid[index_y][index_x]))
                        self.open_set.add((index_x, index_y))
                    else:
                        new_k = self.environment.grid[y][x].k + self.euclidian_distance(0, 0, dx, dy)
                        if new_k < self.environment.grid[index_y][index_x].k:
                            self.update_node_in_open_list(index_x, index_y, new_k)
        top_node = heapq.heappop(self.open)[1]
        self.open_set.remove((top_node.x, top_node.y))
        return top_node

    def calculate_non_obstacle_cost_and_heuristics_from_robot_to_end(self, movement, k_factor):
        x, y = self.environment.robot_x, self.environment.robot_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while True:
            if any((node.x == self.environment.end_x and node.y == self.environment.end_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_non_obstacle_neighbours_from_robot_to_end(x, y, movement, k_factor)
                self.closed.add((top_node.x, top_node.y))
                if self.open:
                    x, y = self.open[0][1].x, self.open[0][1].y
        self.add_repulsion_penalty()

    def expand_neighbours_from_end_to_robot(self, x, y, movement, k_factor, o_factor):
        for dx, dy in self.find_all_neighbour_offsets(movement, 1):
            index_x, index_y = x + dx, y + dy
            if self.is_inside_grid(index_x, index_y):
                if self.is_free_to_move(index_x, index_y):
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x].k = self.environment.grid[y][x].k + self.euclidian_distance(x, y, index_x, index_y)
                        heapq.heappush(self.open, (self.sort_based_on_weighted_distance_to_robot_and_heuristic_and_obstacle(self.environment.grid[index_y][index_x], k_factor, o_factor), self.environment.grid[index_y][index_x]))
                        self.open_set.add((index_x, index_y))
                    else:
                        new_k = self.environment.grid[y][x].k + self.euclidian_distance(0, 0, dx, dy)
                        if new_k < self.environment.grid[index_y][index_x].k:
                            self.update_node_in_open_list(index_x, index_y, new_k)
        top_node = heapq.heappop(self.open)[1]
        self.open_set.remove((top_node.x, top_node.y))
        return top_node

    def calculate_cost_and_heuristics_from_end_to_robot(self, movement, k_factor, o_factor):
        x, y = self.environment.end_x, self.environment.end_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while True:
            if any((node.x == self.environment.robot_x and node.y == self.environment.robot_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_neighbours_from_end_to_robot(x, y, movement, k_factor, o_factor)
                self.closed.add((top_node.x, top_node.y))
                if self.open:
                    x, y = self.open[0][1].x, self.open[0][1].y
        self.add_repulsion_penalty()

    def expand_neighbours_from_robot_to_end(self, x, y, movement, k_factor, o_factor):
        for dx, dy in self.find_all_neighbour_offsets(movement, 1):
            index_x, index_y = x + dx, y + dy
            if self.is_inside_grid(index_x, index_y):
                if self.is_free_to_move(index_x, index_y):
                    if self.is_never_visited(index_x, index_y):
                        self.environment.grid[index_y][index_x].k = self.environment.grid[y][x].k + self.euclidian_distance(x, y, index_x, index_y)
                        heapq.heappush(self.open, (self.sort_based_on_weighted_distance_to_robot_and_heuristic_and_obstacle(self.environment.grid[index_y][index_x], k_factor, o_factor), self.environment.grid[index_y][index_x]))
                        self.open_set.add((index_x, index_y))
                    else:
                        new_k = self.environment.grid[y][x].k + self.euclidian_distance(0, 0, dx, dy)
                        if new_k < self.environment.grid[index_y][index_x].k:
                            self.update_node_in_open_list(index_x, index_y, new_k)
        top_node = heapq.heappop(self.open)[1]
        self.open_set.remove((top_node.x, top_node.y))
        return top_node

    def calculate_cost_and_heuristics_from_robot_to_end(self, movement, k_factor, o_factor):
        x, y = self.environment.robot_x, self.environment.robot_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while True:
            if any((node.x == self.environment.end_x and node.y == self.environment.end_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_neighbours_from_robot_to_end(x, y, movement, k_factor, o_factor)
                self.closed.add((top_node.x, top_node.y))
                if self.open:
                    x, y = self.open[0][1].x, self.open[0][1].y
        self.add_repulsion_penalty()

    def raw_path_finder_from_robot_to_end(self, movement):
        path = []
        orientations = []
        x, y = self.environment.robot_x, self.environment.robot_y
        while True:
            path.append([x, y])
            if x == self.environment.end_x and y == self.environment.end_y:
                break
            neighbours = [[x + dx, y + dy] for dx, dy in self.find_all_neighbour_offsets(movement, 1) if self.is_inside_grid(x + dx, y + dy) and self.is_valid(x + dx, y + dy) and [x + dx, y + dy] not in path]
            neighbours.sort(key=lambda a: self.environment.grid[a[1]][a[0]].k)
            dx, dy = neighbours[0][0] - x, neighbours[0][1] - y
            orientations.append([dx, dy])
            x, y = neighbours[0]
        return path, orientations

    def raw_path_finder_from_end_to_robot(self, movement):
        path = []
        orientations = []
        x, y = self.environment.end_x, self.environment.end_y
        while True:
            path.append([x, y])
            if x == self.environment.robot_x and y == self.environment.robot_y:
                break
            neighbours = [[x + dx, y + dy] for dx, dy in self.find_all_neighbour_offsets(movement, 1) if self.is_inside_grid(x + dx, y + dy) and self.is_valid(x + dx, y + dy) and [x + dx, y + dy] not in path]
            neighbours.sort(key=lambda a: self.environment.grid[a[1]][a[0]].k)
            dx, dy = neighbours[0][0] - x, neighbours[0][1] - y
            orientations.append([dx, dy])
            x, y = neighbours[0]
        return path, orientations

    def find_segments_in_path(self, path, orientation):
        segments = [[path[0]]]
        previous_yaw = orientation[0]
        for index, yaw in enumerate(orientation[1:], 1):
            if yaw == previous_yaw:
                segments[-1].append(path[index])
            else:
                segments.append([path[index]])
            previous_yaw = yaw
        segments[-1].append(path[-1])
        return segments

    # def bazier_coefficient(self, n, k, a, b):
    #     return (math.factorial(n) * math.pow(a, n - k) * math.pow(b, k)) / (math.factorial(k) * math.factorial(n - k))

    # def baziers_curve(self, path: list, step_size: float):
    #     new_path = []
    #     degree = len(path) - 1
    #     for t in np.arange(0.0, 1.0 + step_size, step_size, dtype='float32'):
    #         t_matrix = np.array([self.bazier_coefficient(degree, power, t, 1-t) for power in range(degree, -1, -1)], dtype='float32')
    #         x_matrix = np.array([x for x, y in path], dtype='float32')
    #         y_matrix = np.array([y for x, y in path], dtype='float32')
    #         x = t_matrix @ x_matrix
    #         y = t_matrix @ y_matrix
    #         new_path.append([x, y])
    #     return new_path

    # def calculate_path_cost(self, path):
    #     return sum([self.environment.grid[y][x].k for x, y in path])

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
    #         if self.environment.grid[y_path][x_path].obstacle:
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
    #         if any([self.environment.grid[index_y][index_x].obstacle or self.environment.grid[index_y][index_x].repulsion_factor > 0 for index_x, index_y in self.find_neighbours(x, y, min_distance)]):
    #             for index2, (future_x, future_y) in enumerate(path[index1:]):
    #                 if not self.environment.grid[future_y][future_x]:
    #                 astar_end_x, astar_end_y = 
    #             self.calculate_cost_and_heuristics(self.environment.robot_x, self.environment.robot_y, self.environment)
