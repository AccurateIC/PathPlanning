import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import matplotlib.colors as mcolors

# Constants for different elements in the grid
EMPTY = 0
ROBOT_START = 1
END_POINT = 2
OBSTACLE = 500
REPULSION = 140

# Colors for different elements
color_map = {
    EMPTY: 'white',
    ROBOT_START: 'green',
    END_POINT: 'red',
    OBSTACLE: 'black',
    REPULSION: 'blue'
}

# Initialize global variables
grid_size = 50
grid = np.zeros((grid_size, grid_size), dtype=int)
current_element = ROBOT_START

# Function to handle mouse clicks
def on_click(event):
    global current_element
    if event.inaxes is not None:
        x, y = int(event.xdata), int(event.ydata)
        y = grid_size - 1 - y  # Invert the y coordinate
        if current_element == ROBOT_START:
            grid[y, x] = ROBOT_START
            current_element = END_POINT
        elif current_element == END_POINT:
            grid[y, x] = END_POINT
            current_element = OBSTACLE
        elif current_element == OBSTACLE:
            grid[y, x] = OBSTACLE
        elif current_element == REPULSION:
            grid[y, x] = REPULSION
        update_plot()

# Function to handle keyboard events
def on_key(event):
    global current_element
    if event.key == 'o':  # Switch to obstacle mode
        current_element = OBSTACLE
    elif event.key == 'r':  # Switch to repulsion mode
        current_element = REPULSION

# Function to update the plot
def update_plot():
    color_grid = np.zeros((grid_size, grid_size, 3))
    for y in range(grid_size):
        for x in range(grid_size):
            color_name = color_map[grid[y, x]]
            color_rgb = mcolors.to_rgb(color_name)
            color_grid[y, x] = color_rgb
    
    plt.imshow(color_grid, origin='upper', extent=[0, grid_size, 0, grid_size])
    plt.title('Left Click to place points\nKeys: "o" - Obstacle, "r" - Repulsion')
    plt.draw()

# Function to create the legend
def create_legend(ax):
    legend_elements = [
        Patch(facecolor='white', edgecolor='black', label='Empty'),
        Patch(facecolor='green', edgecolor='black', label='Robot Start'),
        Patch(facecolor='red', edgecolor='black', label='End Point'),
        Patch(facecolor='black', edgecolor='black', label='Obstacle'),
        Patch(facecolor='blue', edgecolor='black', label='Repulsion')
    ]
    ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.15, 1))

# Main function to create the plot and handle events
def main():
    global grid
    fig, ax = plt.subplots()
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_xticks(np.arange(0, grid_size+1, 1))
    ax.set_yticks(np.arange(0, grid_size+1, 1))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.grid(True)

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    
    update_plot()
    create_legend(ax)
    plt.show()

    np.save('custom_grid_3.npy', grid)
    print('Grid saved to custom_grid.npy')

if __name__ == '__main__':
    main()
