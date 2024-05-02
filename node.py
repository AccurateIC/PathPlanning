class Node:
    def __init__(self, x: int, y: int, k=None, b=None, is_end=False, end_movement=[0, 0], is_robot=False, robot_movement=[0, 0], is_obstacle=False, obstacle_movement=[0, 0], repulsions=[]):
        self.x = x
        self.y = y
        self.k = k
        self.b = b
        self.is_end = is_end
        self.end_movement = end_movement
        self.is_robot = is_robot
        self.robot_movement = robot_movement
        self.is_obstacle = is_obstacle
        self.obstacle_movement = obstacle_movement
        self.repulsions = repulsions

    def __str__(self):
        return f"x: {self.x}, y: {self.y}, k: {self.k}, b: {self.b}, end: {self.is_end}, end_movement: {self.end_movement}, robot: {self.is_robot}, robot_movement: {self.robot_movement}, obstacle: {self.is_obstacle}, obstacle_movement: {self.obstacle_movement}, repulsion: {self.repulsions}"

    def __getitem__(self, key: str):
        if key == 'x':
            return self.x
        elif key == 'y':
            return self.y
        elif key == 'k':
            return self.k
        elif key == 'b':
            return self.b
        elif key == 'end':
            return self.is_end
        elif key == 'end_movement':
            return self.end_movement
        elif key == 'robot':
            return self.is_robot
        elif key == 'robot_movement':
            return self.robot_movement
        elif key == 'obstacle':
            return self.is_obstacle
        elif key == 'obstacle_movement':
            return self.obstacle_movement
        elif key == 'repulsions':
            return self.repulsions

    def __setitem__(self, key: str, value):
        if key == 'x':
            self.x = value
        elif key == 'y':
            self.y = value
        elif key == 'k':
            self.k = value
        elif key == 'b':
            self.b = value
        elif key == 'end':
            self.is_end = value
        elif key == 'end_movement':
            self.end_movement = value
        elif key == 'robot':
            self.is_robot = value
        elif key == 'robot_movement':
            self.robot_movement = value
        elif key == 'obstacle':
            self.is_obstacle = value
        elif key == 'obstacle_movement':
            self.obstacle_movement = value
        elif key == 'repulsions':
            self.repulsions = value
