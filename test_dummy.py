import matplotlib.pyplot as plt

class Environment:
    def __init__(self, grid_h, grid_w, repulsion_offset=2):
        self.grid_h = grid_h
        self.grid_w = grid_w
        self.repulsion_offset = repulsion_offset

    def is_inside_grid(self, x, y):
        return -1 < x < self.grid_w and -1 < y < self.grid_h

    def get_offset_repulsion(self, repulsion_x: list, repulsion_y: list) -> (list, list):
        xmin , ymin = min(repulsion_x), min(repulsion_y) 
        xmax , ymax = max(repulsion_x), max(repulsion_y)
        offset_x = []
        offset_y = []
        for x_cord in repulsion_x:
            for y_cord in repulsion_y:
                if x_cord == xmin: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord - offset)
                        offset_y.append(y_cord)
                        
                if x_cord == xmax: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord + offset)
                        offset_y.append(y_cord)
                
                if y_cord == ymin: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord)
                        offset_y.append(y_cord - offset)
                        
                if y_cord == ymax: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord )
                        offset_y.append(y_cord + offset)
                
                if x_cord == xmin and  y_cord == ymin: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord - offset)
                        offset_y.append(y_cord - offset)
                
                if x_cord == xmin and  y_cord == ymax: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord - offset)
                        offset_y.append(y_cord + offset)
                
                if x_cord == xmax and  y_cord == ymin: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord + offset)
                        offset_y.append(y_cord - offset)
                        
                if x_cord == xmax and  y_cord == ymax: 
                    for offset in range(1,self.repulsion_offset):
                        offset_x.append(x_cord + offset)
                        offset_y.append(y_cord + offset)
                
        return offset_x, offset_y

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

        # Adding legend
        ax.legend()

        # Show plot
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

# Example usage
env = Environment(grid_h=50, grid_w=50, repulsion_offset=5)

# Example square input
repulsion_x = [2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5]
repulsion_y = [2, 3, 4, 5, 2, 3, 4, 5, 2, 3, 4, 5, 2, 3, 4, 5]

# Calculate offsets
offset_x, offset_y = env.get_offset_repulsion(repulsion_x, repulsion_y)

# Print results
print("Normal Repulsion X:", repulsion_x)
print("Normal Repulsion Y:", repulsion_y)
print("Offset Repulsion X:", offset_x)
print("Offset Repulsion Y:", offset_y)

# Plot grid with repulsion points
env.plot_grid(repulsion_x, repulsion_y, offset_x, offset_y)
