import numpy as np
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w, display=[], repulsion_offset=5):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.display = display
        self.repulsion_offset = repulsion_offset
        self.reset_environment()

    def reset_environment(self):
        self.current_obstacles_position = {}
        self.obstacles_path = {}
        self.collisions = {}

    def is_inside_grid(self, x, y):
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    def get_offset_repulsion(self, repulsion_x: list, repulsion_y: list) -> (list, list):
        # Convert input lists to numpy arrays
        repulsion_points = np.array(list(zip(repulsion_x, repulsion_y)))

        # Get the bounding box of the repulsion points
        xmin, ymin = repulsion_points.min(axis=0)
        xmax, ymax = repulsion_points.max(axis=0)

        # Identify the border points
        border_points = repulsion_points[
            (repulsion_points[:, 0] == xmin) |
            (repulsion_points[:, 0] == xmax) |
            (repulsion_points[:, 1] == ymin) |
            (repulsion_points[:, 1] == ymax)
        ]

        # Scaling matrix
        S = np.array([
            [1, 0],
            [0, 1]
        ])

        # Apply the transformation
        offset_points = []
        for point in border_points:
            for dx in [-self.repulsion_offset, 0, self.repulsion_offset]:
                for dy in [-self.repulsion_offset, 0, self.repulsion_offset]:
                    if dx == 0 and dy == 0:
                        continue
                    new_point = point + np.dot(S, np.array([dx, dy]))
                    if self.is_inside_grid(new_point[0], new_point[1]):
                        offset_points.append(new_point)

        # Separate the offset points into x and y coordinates
        offset_points = np.array(offset_points)
        offset_x, offset_y = offset_points[:, 0], offset_points[:, 1]

        return offset_x.tolist(), offset_y.tolist()

    def plot_grid(self, repulsion_x, repulsion_y, offset_repulsion_x, offset_repulsion_y):
        fig, ax = plt.subplots(figsize=(10, 10))

        # Plot normal repulsion points
        ax.scatter(repulsion_x, repulsion_y, color='blue', label='Normal Repulsion Points')

        # Plot offset repulsion points
        ax.scatter(offset_repulsion_x, offset_repulsion_y, color='red', label='Offset Repulsion Points')

        # Setting grid limits
        ax.set_xlim(0, self.grid_w)
        ax.set_ylim(0, self.grid_h)

        # Adding grid
        ax.grid(True)

        # Invert y-axis to have the origin at the top left
        ax.invert_yaxis()

        # Adding legend
        ax.legend()

        # Show plot
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

# Example usage
env = Environment(grid_h=50, grid_w=50, repulsion_offset=5)

# Example square input
repulsion_x = [20, 20, 20, 20, 30, 30, 30, 30, 40, 40, 40, 40, 50, 50, 50, 50]
repulsion_y = [20, 30, 40, 50, 20, 30, 40, 50, 20, 30, 40, 50, 20, 30, 40, 50]

# Calculate offsets
offset_x, offset_y = env.get_offset_repulsion(repulsion_x, repulsion_y)

# Print results
print("Normal Repulsion X:", repulsion_x)
print("Normal Repulsion Y:", repulsion_y)
print("Offset Repulsion X:", offset_x)
print("Offset Repulsion Y:", offset_y)

# Plot grid with repulsion points
env.plot_grid(repulsion_x, repulsion_y, offset_x, offset_y)
