class Node:
    def __init__(self, x: int, y: int, k=None, b=None, robot=False, robot_orientation=[0, 0], obstacle=False, obstacle_orientation=[0, 0], repulsion_factor=0.0, end=False, end_orientation=[0, 0], collision=False, display=[]):
        self.x = x
        self.y = y
        self.k = k
        self.b = b
        self.robot = robot
        self.robot_orientation = robot_orientation
        self.obstacle = obstacle
        self.obstacle_orientation = obstacle_orientation
        self.repulsion_factor = repulsion_factor
        self.end = end
        self.end_orientation = end_orientation
        self.collision = collision
        self.display = display

    def __str__(self):
        text = ''
        for key in self.display:
            text = text + f'{key}: {self[key]}, '
        text = '[' + text[:-2] + ']'
        return text

    def __getitem__(self, key: str):
        if key == 'x':
            return self.x
        elif key == 'y':
            return self.y
        elif key == 'k':
            return self.k
        elif key == 'b':
            return self.b
        elif key == 'robot':
            return self.robot
        elif key == 'robot_orientation':
            return self.robot_orientation
        elif key == 'obstacle':
            return self.obstacle
        elif key == 'obstacle_orientation':
            return self.obstacle_orientation
        elif key == 'repulsion_factor':
            return self.repulsion_factor
        elif key == 'end':
            return self.end
        elif key == 'end_orientation':
            return self.end_orientation
        elif key == 'collision':
            return self.collision

    def __setitem__(self, key: str, value):
        if key == 'x':
            self.x = value
        elif key == 'y':
            self.y = value
        elif key == 'k':
            self.k = value
        elif key == 'b':
            self.b = value
        elif key == 'robot':
            self.robot = value
        elif key == 'robot_orientation':
            self.robot_orientation = value
        elif key == 'obstacle':
            self.obstacle = value
        elif key == 'obstacle_orientation':
            self.obstacle_orientation = value
        elif key == 'repulsion_factor':
            self.repulsion_factor = value
        elif key == 'end':
            self.end = value
        elif key == 'end_orientation':
            self.end_orientation = value
        elif key == 'collision':
            self.collision = value
