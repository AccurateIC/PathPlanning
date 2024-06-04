import numpy as np
import matplotlib.pyplot as plt
from rdp import rdp

class PathPlanner:
    def __init__(self, path_points, repulsions_x, repulsions_y, epsilon=1.0, alpha=0.01, beta=0.1, iterations=100):
        self.path_points = np.array(path_points, dtype=np.float64)  # Ensure path_points is of type float64
        self.repulsions_x = np.array(repulsions_x, dtype=np.float64)  # Ensure repulsions_x is of type float64
        self.repulsions_y = np.array(repulsions_y, dtype=np.float64)  # Ensure repulsions_y is of type float64
        self.epsilon = epsilon
        self.alpha = alpha  # Learning rate
        self.beta = beta  # Regularization term
        self.iterations = iterations
        self.reduced_path_points = self.simplify_path()
        self.smoothed_path_points = self.smooth_path(self.reduced_path_points)

    def simplify_path(self):
        return rdp(self.path_points, epsilon=self.epsilon)

    def smooth_path(self, path_points):
        smoothed_path = np.copy(path_points)
        for _ in range(self.iterations):
            smoothed_path = self.gradient_descent_step(smoothed_path, path_points)
        return smoothed_path

    def gradient_descent_step(self, smoothed_path, original_path):
        gradient = np.zeros_like(smoothed_path, dtype=np.float64)
        for i in range(1, len(smoothed_path) - 1):
            gradient[i] = 2 * (smoothed_path[i] - original_path[i]) + \
                          self.beta * (2 * smoothed_path[i] - smoothed_path[i-1] - smoothed_path[i+1])
        smoothed_path -= self.alpha * gradient
        return smoothed_path

    def closest_distance_to_repulsion(self, smoothed_path):
        min_distance = float('inf')
        for (x, y) in smoothed_path:
            distances = np.sqrt((self.repulsions_x - x)**2 + (self.repulsions_y - y)**2)
            min_index = np.argmin(distances)
            if distances[min_index] < min_distance:
                min_distance = distances[min_index]
                self.closest_spline_point = (x, y)
                self.closest_repulsion_point = (self.repulsions_x[min_index], self.repulsions_y[min_index])
        return min_distance

    def plot(self):
        smoothed_path = self.smoothed_path_points
        min_distance = self.closest_distance_to_repulsion(smoothed_path)
        print("Minimum Euclidean distance to any repulsion point: ", min_distance)

        fig, ax = plt.subplots()

        # Plot original path points
        ax.plot(self.path_points[:, 0], self.path_points[:, 1], 'bo-', label='Original A* Path')
        # Plot reduced path points
        ax.plot(self.reduced_path_points[:, 0], self.reduced_path_points[:, 1], 'go-', label='Reduced Path Points')
        # Plot smoothed path points
        ax.plot(smoothed_path[:, 0], smoothed_path[:, 1], 'r-', label='Smoothed Path')

        # Plot repulsion points
        ax.scatter(self.repulsions_x, self.repulsions_y, c='red', marker='x', label='Repulsion Points')

        # Plot minimum distance line
        if self.closest_spline_point is not None and self.closest_repulsion_point is not None:
            ax.plot([self.closest_spline_point[0], self.closest_repulsion_point[0]], 
                    [self.closest_spline_point[1], self.closest_repulsion_point[1]], 
                    color='blue', linestyle='--', label='Min Euclidean Distance')
            ax.scatter(self.closest_spline_point[0], self.closest_spline_point[1], c='blue', marker='o')
            ax.scatter(self.closest_repulsion_point[0], self.closest_repulsion_point[1], c='blue', marker='o')

        ax.set_title('A* Path Smoothing with Gradient Descent')
        ax.legend()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_xticks([i for i in range(0, 50, 1)])
        ax.set_yticks([i for i in range(0, 50, 1)])
        ax.invert_yaxis()  # Invert y-axis for top-left origin system
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    # Example path points
    path_points = np.array([[25, 49], [24, 48], [23, 47], [22, 46], [22, 45], [22, 44],
                            [22, 43], [22, 42], [22, 41], [22, 40], [22, 39], [22, 38],
                            [22, 37], [22, 36], [22, 35], [22, 34], [22, 33], [22, 32],
                            [22, 31], [22, 30], [22, 29], [22, 28], [22, 27], [22, 26],
                            [22, 25], [22, 24], [23, 23], [24, 22], [25, 21], [25, 20],
                            [25, 19], [25, 18], [25, 17], [25, 16], [25, 15], [25, 14],
                            [25, 13], [25, 12], [25, 11], [25, 10], [25, 9], [25, 8],
                            [25, 7], [25, 6], [25, 5], [25, 4], [25, 3], [25, 2], [25, 1], [25, 0]])

    # Example repulsions
    repulsions_x = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30]
    repulsions_y = [23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27]

    planner = PathPlanner(path_points, repulsions_x, repulsions_y, epsilon=1.0, alpha=0.01, beta=0.1, iterations=100)
    planner.plot()
