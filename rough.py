import numpy as np
import random as rn
import matplotlib.pyplot as plt

grid_h, grid_w = 200, 200
robot_x, robot_y = 100, 198
robot_dx, robot_dy = 0, -1
robot_value = 1
end_x, end_y = 100, 0
end_dx, end_dy = 0, -1
end_value = 2
repulsion_value = 3
obstacles = 5
obstacles_x, obstacles_y = [rn.randint(0, grid_w - 1) for _ in range(obstacles)], [rn.randint(0, grid_h - 1) for _ in range(obstacles)]
obstacles_dx, obstacles_dy = [rn.randint(-1, 1) for _ in range(obstacles)], [rn.randint(-1, 1) for _ in range(obstacles)]
obstacle_value = 5
collision_x, collision_y = [100], [100]
collision_dx, collision_dy = [1], [1]

array = np.zeros(shape=(grid_h, grid_w))
array[robot_x, robot_y] = robot_value
array[end_x, end_y] = end_value
for obstacle_x, obstacle_y, obstacle_dx, obstacle_dy in zip(obstacles_x + collision_x, obstacles_y + collision_y, obstacles_dx + collision_dx, obstacles_dy + collision_dy):
    if obstacle_dx != 0 and obstacle_dy != 0:
        for i in range(-1, 5, 1):
            repulsion_x, repulsion_y = obstacle_x + i * obstacle_dx, obstacle_y + i * obstacle_dy
            for dx in range(-1, 2, 1):
                for dy in range(-1, 2, 1):
                    array[repulsion_y + dy, repulsion_x + dx] = repulsion_value
    elif (obstacle_dx != 0 and obstacle_dy == 0) or (obstacle_dx == 0 and obstacle_dy != 0):
        for i in range(-1, 5, 1):
            repulsion_x, repulsion_y = obstacle_x + i * obstacle_dx if obstacle_dx != 0 else obstacle_x, obstacle_y + i * obstacle_dy if obstacle_dy != 0 else obstacle_y
            for dx in range(-1, 2, 1):
                for dy in range(-1, 2, 1):
                    array[repulsion_y + dy, repulsion_x + dx] = repulsion_value
    else:
        for dx in range(-5, 6, 1):
            for dy in range(-5, 6, 1):
                array[obstacle_y + dy, obstacle_x + dx] = repulsion_value
    array[obstacle_y, obstacle_x] = obstacle_value
# for x, y in zip(collision_x, collision_y):
#     array[y][x] = collision_value

# np.save('bhavya2.npy', array)
array = np.load('bhavya2.npy')

plt.figure(figsize=(20, 20))
plt.imshow(array.T, cmap='rainbow')
plt.show()
