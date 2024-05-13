import numpy as np
import matplotlib.pyplot as plt

grid_h, grid_w = 20, 20
obstacle_movement = [
    {(9, 4): 10,
     (9, 4): 11,
     (9, 4): 12,
     (9, 4): 13,
     (9, 4): 14,
     (9, 4): 15,
     (9, 4): 16,
     (9, 4): 17,
     (9, 4): 18,
     (9, 4): 19,
     (9, 4): 20,
     (9, 4): 21,
     (9, 4): 22,
     (9, 4): 23,
     (9, 4): 24,
     (9, 4): 25,
     (9, 4): 26,
     (9, 4): 27,
     (9, 4): 28,
     (9, 4): 29},
    {(0, 10): 40,
     (1, 10): 41,
     (2, 10): 42,
     (3, 10): 43,
     (4, 10): 44,
     (5, 10): 45,
     (6, 10): 46,
     (7, 10): 47,
     (8, 10): 48,
     (9, 10): 49,
     (10, 10): 50,
     (11, 10): 51,
     (12, 10): 52,
     (13, 10): 53,
     (14, 10): 54,
     (15, 10): 55,
     (16, 10): 56,
     (17, 10): 57,
     (18, 10): 58,
     (19, 10): 59},
    {(14, 16): 70,
     (14, 15): 71,
     (14, 14): 72,
     (14, 13): 73,
     (14, 12): 74,
     (14, 11): 75,
     (14, 10): 76,
     (14, 9): 77,
     (14, 8): 78,
     (14, 7): 79,
     (14, 6): 80,
     (14, 5): 81,
     (14, 4): 82,
     (14, 3): 83,
     (14, 2): 84,
     (14, 1): 85,
     (14, 0): 86,
     (14, 0): 87,
     (14, 0): 88,
     (14, 0): 89},
    ]
robot_x, robot_y = 10, 19
robot_value = 100
end_x, end_y = 10, 0
end_value = 110
collisions = [[10, 10]]
collision_value = 150

array = np.zeros(shape=(grid_h, grid_w), dtype='int32')
array[robot_y, robot_x] = robot_value
array[end_y, end_x] = end_value
for obstacle in obstacle_movement:
    for x, y in obstacle:
        array[y, x] = obstacle[(x, y)]
for x, y in collisions:
    if x is not None and y is not None:
        array[y, x] = collision_value

np.save('bhavya2.npy', array)

print(np.unique(array, return_counts=True))

plt.figure(figsize=(12, 12))
plt.imshow(array, cmap='rainbow')
plt.xticks([i for i in range(grid_w)])
plt.yticks([i for i in range(grid_h)])
plt.grid(True)
plt.show()

# array = np.load('bhavya2.npy')
# plt.figure(figsize=(10, 10))
# plt.imshow(array)
# plt.show()
