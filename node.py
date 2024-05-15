class Node:
    def __init__(self, x: int, y: int, k=None, b=None, robot=False, robot_movement=[0, 0], robot_distance=0.0, obstacle=False, obstacle_movement=[0, 0], repulsion_factor=0.0, end=False, end_movement=[0, 0], end_distance=0.0, display=[]):
        self.x = x
        self.y = y
        self.k = k
        self.b = b
        self.robot = robot
        self.robot_movement = robot_movement
        self.robot_distance = robot_distance
        self.obstacle = obstacle
        self.obstacle_movement = obstacle_movement
        self.repulsion_factor = repulsion_factor
        self.end = end
        self.end_movement = end_movement
        self.end_distance = end_distance
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
        elif key == 'robot_movement':
            return self.robot_movement
        elif key == 'robot_distance':
            return self.robot_distance
        elif key == 'obstacle':
            return self.obstacle
        elif key == 'obstacle_movement':
            return self.obstacle_movement
        elif key == 'repulsion_factor':
            return self.repulsion_factor
        elif key == 'end':
            return self.end
        elif key == 'end_movement':
            return self.end_movement
        elif key == 'end_distance':
            return self.end_distance

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
        elif key == 'robot_movement':
            self.robot_movement = value
        elif key == 'robot_distance':
            self.robot_distance = value
        elif key == 'obstacle':
            self.obstacle = value
        elif key == 'obstacle_movement':
            self.obstacle_movement = value
        elif key == 'repulsion_factor':
            self.repulsion_factor = value
        elif key == 'end':
            self.end = value
        elif key == 'end_movement':
            self.end_movement = value
        elif key == 'end_distance':
            self.end_distance = value
