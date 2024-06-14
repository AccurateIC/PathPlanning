import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import matplotlib.colors as mcolors
import numpy as np
import os
from grid import Grid, EMPTY, ROBOT_START, END_POINT, OBSTACLE, REPULSION

# Colors for different elements
color_map = {
    EMPTY: 'white',
    ROBOT_START: 'green',
    END_POINT: 'red',
    OBSTACLE: 'black',
    REPULSION: 'blue'
}

class GridVisualizer:
    def __init__(self, grid):
        self.grid = grid
        self.fig, self.ax = plt.subplots()
        self.zoom_factor = 1.0
        self.setup_plot()

    def setup_plot(self):
        self.ax.set_xlim(0, self.grid.size)
        self.ax.set_ylim(0, self.grid.size)
        self.ax.set_xticks(np.arange(0, self.grid.size + 1, 1))
        self.ax.set_yticks(np.arange(0, self.grid.size + 1, 1))
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        self.ax.grid(True)
        self.create_legend()

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.update_plot()

    def on_click(self, event):
        if event.inaxes is not None:
            x, y = int(event.xdata), int(event.ydata)
            self.grid.update_grid(x, y)
            self.update_plot()

    def on_key(self, event):
        if event.key == 'o':  # Switch to obstacle mode
            self.grid.change_current_element(OBSTACLE)
        elif event.key == 'r':  # Switch to repulsion mode
            self.grid.change_current_element(REPULSION)
        elif event.key == 's':  # Switch to robot start mode
            self.grid.change_current_element(ROBOT_START)
        elif event.key == 'e':  # Switch to end point mode
            self.grid.change_current_element(END_POINT)

    def on_scroll(self, event):
        base_scale = 1.1
        if event.button == 'up':
            self.zoom_factor *= base_scale
        elif event.button == 'down':
            self.zoom_factor /= base_scale
        self.zoom_factor = min(max(self.zoom_factor, 1), 10)  # limit zoom factor
        self.apply_zoom(event)

    def apply_zoom(self, event):
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        xdata = event.xdata
        ydata = event.ydata
        new_width = (cur_xlim[1] - cur_xlim[0]) / self.zoom_factor
        new_height = (cur_ylim[1] - cur_ylim[0]) / self.zoom_factor
        relx = (cur_xlim[1] - xdata) / (cur_xlim[1] - cur_xlim[0])
        rely = (cur_ylim[1] - ydata) / (cur_ylim[1] - cur_ylim[0])
        self.ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * relx])
        self.ax.set_ylim([ydata - new_height * (1 - rely), ydata + new_height * rely])
        self.update_plot()

    def update_plot(self):
        color_grid = np.zeros((self.grid.size, self.grid.size, 3))
        for y in range(self.grid.size):
            for x in range(self.grid.size):
                color_name = color_map[self.grid.grid[y, x]]
                color_rgb = mcolors.to_rgb(color_name)
                color_grid[y, x] = color_rgb
        
        self.ax.imshow(color_grid, origin='upper', extent=[0, self.grid.size, 0, self.grid.size])
        self.ax.set_title('Left Click to place points\nKeys: "o" - Obstacle, "r" - Repulsion, "s" - Start, "e" - End')
        plt.draw()

    def create_legend(self):
        legend_elements = [
            Patch(facecolor='white', edgecolor='black', label='Empty'),
            Patch(facecolor='green', edgecolor='black', label='Robot Start'),
            Patch(facecolor='red', edgecolor='black', label='End Point'),
            Patch(facecolor='black', edgecolor='black', label='Obstacle'),
            Patch(facecolor='blue', edgecolor='black', label='Repulsion')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.15, 1))

    def show(self):
        plt.show()

def main():
    mode = input("Enter mode (new/edit): ").strip().lower()
    filename = 'custom_grid_250.npy'
    grid_size = 250

    if mode == 'edit' and os.path.exists(filename):
        grid_data = np.load(filename)
        grid = Grid(grid_size)
        grid.grid = grid_data
        print("Loaded existing grid.")
    else:
        grid = Grid(grid_size)
        print("Created new grid.")

    visualizer = GridVisualizer(grid)
    visualizer.show()
    grid.save_grid(filename)

if __name__ == '__main__':
    main()
