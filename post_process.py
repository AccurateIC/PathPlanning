import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from rdp import rdp

class PathPlanner:
    def __init__(self, path_points, repulsions_x, repulsions_y, epsilon=1.0, spline_smoothness=5, spline_degree=3):
        self.path_points = path_points
        self.repulsions_x = np.array(repulsions_x)
        self.repulsions_y = np.array(repulsions_y)
        self.epsilon = epsilon
        self.spline_smoothness = spline_smoothness
        self.spline_degree = spline_degree
        self.reduced_path_points = self.simplify_path()
        self.closest_spline_point = None
        self.closest_repulsion_point = None

    def simplify_path(self):
        return rdp(self.path_points, epsilon=self.epsilon)

    def get_b_spline(self, degree,num_points = 1000):
        x = self.reduced_path_points[:, 0]
        y = self.reduced_path_points[:, 1]
        tck =  splprep([x, y], s=self.spline_smoothness, k=degree)
        u_fine = np.linspace(0, 1, num_points)
        return splev(u_fine, tck[0])
    

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
        min_distance, corresponding_spline = self.closest_distance_to_repulsion(x_fine, y_fine)
        
        print("Corresponding spline point: ", corresponding_spline)

        if min_distance < 1:
            # Convert reduced_path_points to a numpy array for easier distance calculations
            reduced_path_points_array = np.array(self.reduced_path_points)

            # Calculate distances from corresponding_spline to all reduced_path_points
            reduced_path_distances = np.sqrt((reduced_path_points_array[:, 0] - corresponding_spline[0])**2 + 
                                             (reduced_path_points_array[:, 1] - corresponding_spline[1])**2)

            updated_points = []
            for i in range(len(reduced_path_points_array)):
                if reduced_path_distances[i] < 10:
                   self.adjust_reduced_path_points(reduced_path_points_array[i],corresponding_spline)
                else:
                    updated_points.append(reduced_path_distances[i])
            # Find indices of the nearest and second nearest points
            print("reduced_path_distances : ",reduced_path_distances)
            nearest_index = np.argmin(reduced_path_distances)
            nearest_reduced_path_point = self.reduced_path_points[nearest_index]

            # Temporarily set the nearest distance to infinity to find the second nearest
            reduced_path_distances[nearest_index] = np.inf
            second_nearest_index = np.argmin(reduced_path_distances)
            second_nearest_reduced_path_point = self.reduced_path_points[second_nearest_index]

            print("Nearest point in reduced_path_points: ", nearest_reduced_path_point)
            print("Second nearest point in reduced_path_points: ", second_nearest_reduced_path_point)

        return x_fine, y_fine, min_distance

    def adjust_reduced_path_points(self,point,corresponding_spline):
        print("_________________________________________________________________________________________")
        deta_x,delta_y = corresponding_spline[0]- point[0] , corresponding_spline[0]-point[0]
        print("Adjusting",deta_x,delta_y)
        deta_x,delta_y = corresponding_spline - point
        print("Adjusting",deta_x,delta_y)
        print("_________________________________________________________________________________________")
        
    def plot(self):
        x_fine, y_fine, min_distance = self.infer_spline()
       
        print("Minimum Euclidean distance to any repulsion point: ", min_distance)

        fig, ax = plt.subplots()

        # Plot original path points
        ax.plot(self.path_points[:, 0], self.path_points[:, 1], 'bo-', label='Original A* Path')
        # Plot reduced path points used for B-Spline fitting
        ax.plot(self.reduced_path_points[:, 0], self.reduced_path_points[:, 1], 'go', label='Reduced Path Points')
        # Plot B-Spline fitted curve
        ax.plot(x_fine, y_fine, 'r-', label='B-Spline Curve')
        # Plot repulsion points
        ax.scatter(self.repulsions_x, self.repulsions_y, c='red', marker='x', label='Repulsion Points')

        # Plot minimum distance line
        if self.closest_spline_point is not None and self.closest_repulsion_point is not None:
            ax.plot([self.closest_spline_point[0], self.closest_repulsion_point[0]], 
                    [self.closest_spline_point[1], self.closest_repulsion_point[1]], 
                    color='blue', linestyle='--', label='Min Euclidean Distance')
            ax.scatter(self.closest_spline_point[0], self.closest_spline_point[1], c='blue', marker='o')
            ax.scatter(self.closest_repulsion_point[0], self.closest_repulsion_point[1], c='blue', marker='o')

        ax.set_title('A* Path and Fitted Splines')
        ax.legend()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_xticks([i for i in range(0, 50, 1)])
        ax.set_yticks([i for i in range(0, 50, 1)])
        ax.invert_yaxis()  # Invert y-axis for top-left origin system
        plt.grid(True)
        plt.show()

        return min_distance



if __name__ == '__main__':
    
    path = [[25, 49], [24, 48], [23, 47], [22, 46], [21, 45], [21, 44], [21, 43], [21, 42], [21, 41], [21, 40], [21, 39], [21, 38], [21, 37], [21, 36], [21, 35], [21, 34], [21, 33], [21, 32], [21, 31], [21, 30], [21, 29], [21, 28], [21, 27], [21, 26], [21, 25], [21, 24], [21, 23], [22, 22], [23, 21], [24, 20], [25, 19], [25, 18], [25, 17], [25, 16], [25, 15], [25, 14], [25, 13], [25, 12], [25, 11], [25, 10], [25, 9], [25, 8], [25, 7], [25, 6], [25, 5], [25, 4], [25, 3], [25, 2], [25, 1], [25, 0]]
    
    repulsion_x = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30]
    repulsion_y = [23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27, 23, 24, 25, 26, 27]
    path_points = np.asarray(path)

    post_planner = PathPlanner(path_points, repulsion_x, repulsion_y, epsilon=1.0)
    post_planner.plot()