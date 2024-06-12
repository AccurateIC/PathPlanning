import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.collisions = {}  # Initialize as an empty dictionary
        # Other initialization code...

    def detect_collisions(self, timestep, robot_position, obstacles):
        """
        Detect and record collisions for a given timestep.
        """
        for obstacle_id, obstacle_position in obstacles.items():
            if self.check_collision(robot_position, obstacle_position):
                if timestep not in self.collisions:
                    self.collisions[timestep] = []
                self.collisions[timestep].append(obstacle_id)

    def check_collision(self, pos1, pos2):
        """
        Check if two positions collide.
        """
        return pos1 == pos2  # Simplified collision check

    def plot_collisions(self, ax):
        """
        Plot the recorded collisions on the given axes.
        """
        for timestep in self.collisions:
            for obstacle_id in self.collisions[timestep]:
                collision_position = self.obstacles_path[obstacle_id]['movement'][timestep]
                ax.add_patch(plt.Rectangle(collision_position, 1, 1, color='red'))
                ax.annotate(
                    '', 
                    (collision_position[0] + 0.5, collision_position[1] + 0.5), 
                    (collision_position[0] + 0.5 + self.obstacles_path[obstacle_id]['orientation'][timestep][0], collision_position[1] + 0.5 + self.obstacles_path[obstacle_id]['orientation'][timestep][1]), 
                    arrowprops={'color': 'purple', 'arrowstyle': '<-'}
                )

# Example usage
env = Environment(20, 20)
robot_position = (5, 5)
obstacles = {1: (5, 5), 2: (10, 10)}  # Example positions
env.detect_collisions(0, robot_position, obstacles)

# Define ax using matplotlib
fig, ax = plt.subplots(figsize=(10, 10))
env.plot_collisions(ax)  # Now ax is defined

plt.xlim(0, 20)
plt.ylim(0, 20)
plt.gca().invert_yaxis()
plt.grid(True)
plt.show()
