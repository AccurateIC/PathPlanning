class Node:
    def __init__(self, x: int, y: int, k=None, b=None, is_start=False, is_end=False, is_robot=False, robot_movement=[0, 0], is_obstacle=False, obstacle_movement=[0, 0]):
        self.x = x
        self.y = y
        self.k = k
        self.b = b
        self.is_start = is_start
        self.is_end = is_end
        self.is_robot = is_robot
        self.robot_movement = robot_movement
        self.is_obstacle = is_obstacle
        self.obstacle_movement = obstacle_movement

    def __str__(self):
        return {'x': self.x, 'y': self.y, 'k': self.k, 'b': self.b, 'start': self.is_start, 'end': self.is_end, 'robot': self.is_robot, 'robot_movement': self.robot_movement, 'obstacle': self.is_obstacle, 'obstacle_movement': self.obstacle_movement}

    def __getitem__(self, key: str):
        if key == 'x':
            return self.x
        elif key == 'y':
            return self.y
        elif key == 'k':
            return self.k
        elif key == 'b':
            return self.b
        elif key == 'start':
            return self.is_start
        elif key == 'end':
            return self.is_end
        elif key == 'robot':
            return self.is_robot
        elif key == 'robot_movement':
            return self.robot_movement
        elif key == 'obstacle':
            return self.is_obstacle
        elif key == 'obstacle_movement':
            return self.obstacle_movement

    def __setitem__(self, key: str, value):
        if key == 'x':
            self.x = value
        elif key == 'y':
            self.y = value
        elif key == 'k':
            self.k = value
        elif key == 'b':
            self.b = value
        elif key == 'start':
            self.is_start = value
        elif key == 'end':
            self.is_end = value
        elif key == 'robot':
            self.is_robot = value
        elif key == 'robot_movement':
            self.robot_movement = value
        elif key == 'obstacle':
            self.is_obstacle = value
        elif key == 'obstacle_movement':
            self.obstacle_movement = value
