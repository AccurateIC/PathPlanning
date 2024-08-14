class Node:
    __slots__ = ['x', 'y', 'k', 'b', 'robot', 'robot_movement', 'robot_distance', 'obstacle', 'obstacle_movement', 'total_obstacle_distance', 'repulsion_factor', 'end', 'end_distance', 'display']

    def __init__(self, x: int, y: int, k=None, b=None, robot=False, robot_movement=[0, 0], robot_distance=0.0, obstacle=False, obstacle_movement=[0, 0], total_obstacle_distance=0.0, repulsion_factor=0.0, end=False, end_distance=0.0, display=[]):
        self.x = x
        self.y = y
        self.k = k
        self.b = b
        self.robot = robot
        self.robot_movement = robot_movement
        self.robot_distance = robot_distance
        self.obstacle = obstacle
        self.obstacle_movement = obstacle_movement
        self.total_obstacle_distance = total_obstacle_distance
        self.repulsion_factor = repulsion_factor
        self.end = end
        self.end_distance = end_distance
        self.display = display

    def __str__(self):
        text = ''
        for key in self.display:
            value = getattr(self, key)
            text = text + f'{key}: {value}, '
        text = '[' + text[:-2] + ']'
        return text
    def __lt__(self, other):
        return self.k < other.k