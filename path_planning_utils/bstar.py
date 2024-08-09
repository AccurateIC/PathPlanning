import math
import heapq
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
                heapq.heapify(self.open)  # Maintain heap property
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
        if self.open:  # Ensure there is a node to pop
            top_node = heapq.heappop(self.open)[1]
            self.open_set.remove((top_node.x, top_node.y))
            return top_node
        return None  # Handle case where no nodes are left

    def calculate_all_cost_and_heuristics_from_end_to_robot(self, movement, k_factor):
        x, y = self.environment.end_x, self.environment.end_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while self.open:
            if any((node.x == self.environment.robot_x and node.y == self.environment.robot_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_all_neighbours_from_end_to_robot(x, y, movement, k_factor)
                if top_node:
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
        if self.open:  # Ensure there is a node to pop
            top_node = heapq.heappop(self.open)[1]
            self.open_set.remove((top_node.x, top_node.y))
            return top_node
        return None  # Handle case where no nodes are left

    def calculate_all_cost_and_heuristics_from_robot_to_end(self, movement, k_factor):
        x, y = self.environment.robot_x, self.environment.robot_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while self.open:
            if any((node.x == self.environment.end_x and node.y == self.environment.end_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_all_neighbours_from_robot_to_end(x, y, movement, k_factor)
                if top_node:
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
        if self.open:  # Ensure there is a node to pop
            top_node = heapq.heappop(self.open)[1]
            self.open_set.remove((top_node.x, top_node.y))
            return top_node
        return None  # Handle case where no nodes are left

    def calculate_cost_and_heuristics_from_end_to_robot(self, movement, k_factor, o_factor):
        x, y = self.environment.end_x, self.environment.end_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while self.open:
            if any((node.x == self.environment.robot_x and node.y == self.environment.robot_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_neighbours_from_end_to_robot(x, y, movement, k_factor, o_factor)
                if top_node:
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
                        heapq.heappush(self.open, (self.sort_based_on_weighted_distance_to_end_and_heuristic_and_obstacle(self.environment.grid[index_y][index_x], k_factor, o_factor), self.environment.grid[index_y][index_x]))
                        self.open_set.add((index_x, index_y))
                    else:
                        new_k = self.environment.grid[y][x].k + self.euclidian_distance(0, 0, dx, dy)
                        if new_k < self.environment.grid[index_y][index_x].k:
                            self.update_node_in_open_list(index_x, index_y, new_k)
        if self.open:  # Ensure there is a node to pop
            top_node = heapq.heappop(self.open)[1]
            self.open_set.remove((top_node.x, top_node.y))
            return top_node
        return None  # Handle case where no nodes are left

    def calculate_cost_and_heuristics_from_robot_to_end(self, movement, k_factor, o_factor):
        x, y = self.environment.robot_x, self.environment.robot_y
        self.environment.grid[y][x].k = 0
        self.environment.grid[y][x].b = None
        heapq.heappush(self.open, (0, self.environment.grid[y][x]))
        self.open_set.add((x, y))
        while self.open:
            if any((node.x == self.environment.end_x and node.y == self.environment.end_y) for _, node in self.open):
                break
            else:
                top_node = self.expand_neighbours_from_robot_to_end(x, y, movement, k_factor, o_factor)
                if top_node:
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

    def find_path(self, movement):
        path = []
        x, y = self.environment.robot_x, self.environment.robot_y
        while (x, y) != (self.environment.end_x, self.environment.end_y):
            path.append((x, y))
            neighbours = [(x + dx, y + dy) for dx, dy in self.find_all_neighbour_offsets(movement, 1)
                          if self.is_inside_grid(x + dx, y + dy) and self.is_valid(x + dx, y + dy)]
            if not neighbours:
                break
            neighbours.sort(key=lambda n: self.environment.grid[n[1]][n[0]].k)
            x, y = neighbours[0]
        path.append((self.environment.end_x, self.environment.end_y))
        return path

    def plot_multiple_shortest_paths(self, num_paths=5, movement='queen'):
        all_paths = []
        heuristic_weights = np.linspace(0.4, 0.6, num_paths)  # Slightly vary the heuristic weight

        for k_factor in heuristic_weights:
            self.open = []
            self.closed = set()
            self.open_set = set()
          # Reset the environment if a method exists
            self.calculate_all_cost_and_heuristics_from_end_to_robot(movement, k_factor)
            path = self.find_path(movement)
            all_paths.append(path)

        plt.figure(figsize=(10, 10))
        max_k = max(cell.k for row in self.environment.grid for cell in row if cell.k is not None)

        for y in range(self.environment.grid_h):
            for x in range(self.environment.grid_w):
                cell = self.environment.grid[y][x]
                if cell.obstacle:
                    plt.plot(x, y, 'ks')  # obstacle in black
                elif cell.k is not None:
                    alpha = min(cell.k / max_k, 1.0)  # ensure alpha is within [0, 1]
                    plt.plot(x, y, 'co', alpha=alpha)  # explored node

        colors = ['b', 'g', 'r', 'c', 'm']
        for i, path in enumerate(all_paths):
            if path:  # Check if the path is not empty
                path_x, path_y = zip(*path)
                plt.plot(path_x, path_y, colors[i % len(colors)], label=f'Path {i+1}')

        plt.plot(self.environment.robot_x, self.environment.robot_y, 'ro')  # robot start in red
        plt.plot(self.environment.end_x, self.environment.end_y, 'go')  # goal in green
        plt.gca().invert_yaxis()
        plt.legend()
        plt.show()

