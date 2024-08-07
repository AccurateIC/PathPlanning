import numpy as np

# Constants for different elements in the grid
EMPTY = 0
ROBOT_START = 1
END_POINT = 2
OBSTACLE = 500
REPULSION = 140
TRACE = 450

class Grid:
    def __init__(self, size):
        self.size = size
        self.grid = np.zeros((size, size), dtype=int)
        self.grid[size][size/2] = 1
        self.grid[size][0] = 2
        self.current_element = ROBOT_START

    def update_grid(self, x, y):
        y = self.size - 1 - y  # Invert the y coordinate
        if self.current_element == ROBOT_START:
            self.grid[y, x] = ROBOT_START
            self.current_element = END_POINT
        elif self.current_element == END_POINT:
            self.grid[y, x] = END_POINT
            self.current_element = OBSTACLE
        elif self.current_element == OBSTACLE:
            self.grid[y, x] = OBSTACLE
        elif self.current_element == REPULSION:
            self.grid[y, x] = REPULSION
        elif self.current_element == TRACE:
            self.grid[y, x] = TRACE
        elif self.current_element == TRACE:
            self.grid[y, x] = TRACE

    def change_current_element(self, element):
        self.current_element = element

    def save_grid(self, filename):
        np.save(filename, self.grid)
        print(f'Grid saved to {filename}')
