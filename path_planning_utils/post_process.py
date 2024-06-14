import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from rdp import rdp

class PathPlanner:
    def __init__(self, path_points, repulsions_x, repulsions_y, epsilon=1.0, spline_smoothness=5, spline_degree=2, max_distance=5):
        self.path_points = np.array(path_points)
        self.repulsions_x = np.array(repulsions_x)
        self.repulsions_y = np.array(repulsions_y)
        self.epsilon = epsilon
        self.spline_smoothness = spline_smoothness
        self.spline_degree = spline_degree
        self.max_distance = max_distance
        self.reduced_path_points = self.simplify_path()
        self.closest_spline_point = None
        self.closest_repulsion_point = None

    def find_closest_point(self, midpoint):
        distances = np.sqrt((self.path_points[:, 0] - midpoint[0])**2 + (self.path_points[:, 1] - midpoint[1])**2)
        closest_index = np.argmin(distances)
        return self.path_points[closest_index]

    def simplify_path(self):
        if len(self.path_points) < 6:
            return self.path_points
        
        start_points = self.path_points[:3]
        end_points = self.path_points[-3:]
        middle_points = self.path_points[3:-3]

        reduced_middle = rdp(middle_points, epsilon=self.epsilon) if len(middle_points) > 2 else middle_points
        combined_path = np.vstack((start_points, reduced_middle, end_points))

        refined_path = [combined_path[0]]
        for i in range(1, len(combined_path)):
            start = combined_path[i-1]
            end = combined_path[i]

            while np.linalg.norm(start - end) > self.max_distance:
                midpoint = (start + end) / 2
                closest_point = self.find_closest_point(midpoint)
                refined_path.append(closest_point)
                start = closest_point
            
            refined_path.append(end)

        return np.array(refined_path)

    def get_b_spline(self, degree, num_points=500):
        x = self.reduced_path_points[:, 0]
        y = self.reduced_path_points[:, 1]

        if len(x) >= 4:
            tck, _ = splprep([x, y], s=self.spline_smoothness, k=degree)
            u_fine = np.linspace(0, 1, num_points)
            return splev(u_fine, tck)
        return (None, None)

    def closest_distance_to_repulsion(self, x_fine, y_fine):
        min_distance = float('inf')
        closest_spline_point = None
        closest_repulsion_point = None

        for (x, y) in zip(x_fine, y_fine):
            distances = np.sqrt((self.repulsions_x - x)**2 + (self.repulsions_y - y)**2)
            min_index = np.argmin(distances)
            if distances[min_index] < min_distance:
                min_distance = distances[min_index]
                closest_spline_point = (x, y)
                closest_repulsion_point = (self.repulsions_x[min_index], self.repulsions_y[min_index])

        self.closest_spline_point = closest_spline_point
        self.closest_repulsion_point = closest_repulsion_point
        return min_distance, closest_spline_point

    def infer_spline(self):
        x_fine, y_fine = self.get_b_spline(self.spline_degree)

        if x_fine is not None and y_fine is not None:
            # min_distance, _ = self.closest_distance_to_repulsion(x_fine, y_fine)
            min_distance = 0
            return x_fine, y_fine, min_distance

        return None, None, None

    def plot(self):
        x_fine, y_fine, min_distance = self.infer_spline()
       
        
        if x_fine is not None and y_fine is not None:
            fig, ax = plt.subplots(figsize=(10, 6))

            # Plot original path points
            ax.plot(self.path_points[:, 0], self.path_points[:, 1], 'bo-', label='Original A* Path', linewidth=1.5, markersize=5)

            # Plot reduced path points used for B-Spline fitting
            ax.plot(self.reduced_path_points[:, 0], self.reduced_path_points[:, 1], 'go-', label='Reduced Path Points', linewidth=1.5, markersize=6)

            # Plot B-Spline fitted curve
            ax.plot(x_fine, y_fine, 'r-', label='B-Spline Curve', linewidth=2)

            # Plot repulsion points
            ax.scatter(self.repulsions_x, self.repulsions_y, c='black', marker='x', label='Repulsion Points')

            # Plot minimum distance line
            if self.closest_spline_point and self.closest_repulsion_point:
                ax.plot([self.closest_spline_point[0], self.closest_repulsion_point[0]],
                        [self.closest_spline_point[1], self.closest_repulsion_point[1]],
                        'b--', label='Min Euclidean Distance', linewidth=1.5)
                ax.scatter(*self.closest_spline_point, c='blue', edgecolors='k', s=100, zorder=5)
                ax.scatter(*self.closest_repulsion_point, c='blue', edgecolors='k', s=100, zorder=5)

            # Annotations
            ax.annotate('Start', xy=(self.path_points[0]), xytext=(self.path_points[0] + np.array([1, 1])),
                        arrowprops=dict(facecolor='black', shrink=0.05), fontsize=12)
            ax.annotate('End', xy=(self.path_points[-1]), xytext=(self.path_points[-1] + np.array([1, 1])),
                        arrowprops=dict(facecolor='black', shrink=0.05), fontsize=12)

            # Enhancing grid and axis
            ax.set_title('A* Path and Fitted Splines', fontsize=16)
            ax.legend(fontsize=12)
            ax.set_xlabel('X', fontsize=14)
            ax.set_ylabel('Y', fontsize=14)
            ax.set_xticks(range(0, 250, 5))
            ax.set_yticks(range(0, 250, 5))
            ax.invert_yaxis()
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.tick_params(axis='both', which='major', labelsize=12)

            plt.tight_layout()
            plt.show()
