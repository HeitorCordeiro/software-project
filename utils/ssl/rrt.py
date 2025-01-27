import random
from utils.Point import Point
from scipy.spatial import KDTree

class Node:
    def __init__(self, point: Point, parent=None):
        self.point = point
        self.parent = parent
        self.cost = 0 if parent is None else parent.cost + point.dist_to(parent.point)

class RRT:
    def __init__(self, start: Point, goal: Point, obstacles: list[Point], max_iter=1000, step_size=0.1, goal_sample_rate=0.1):
        self.start = Node(start)
        self.goal = Node(goal)
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.tree = [self.start]
        self.kd_tree = KDTree([start])


    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.point
        else:
            return Point(random.uniform(-3, 3), random.uniform(-2, 2))

    def get_nearest_node(self, point: Point):
        _, idx = self.kd_tree.query([point.x, point.y])
        return self.tree[idx]

    def is_collision(self, point: Point):
        for obstacle in self.obstacles:
            if point.dist_to(obstacle) < 0.2:  
                return True
        return False

    def steer(self, from_node: Node, to_point: Point):
        direction = (to_point - from_node.point).normalize()
        new_point = from_node.point + direction * self.step_size
        if not self.is_collision(new_point):
            return Node(new_point, from_node)
        return None

    def generate_path(self):
        for _ in range(self.max_iter):
            random_point = self.get_random_point()
            nearest_node = self.get_nearest_node(random_point)
            new_node = self.steer(nearest_node, random_point)
            if new_node:
                self.tree.append(new_node)
                self.kd_tree = KDTree([(node.point.x, node.point.y) for node in self.tree])
                if new_node.point.dist_to(self.goal.point) < self.step_size:
                    #return self.smooth_path(self.extract_path(new_node)) #tem um problema quando fica suavizado
                    return (self.extract_path(new_node)) #sem suavização

        return None

    def extract_path(self, node: Node):
        path = []
        while node:
            path.append(node.point)
            node = node.parent
        return path[::-1]

    def smooth_path(self, path: list[Point]):
        if not path:
            return path
        smoothed_path = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i:
                if not self.is_collision_free_path(path[i], path[j]):
                    j -= 1
                else:
                    break
            smoothed_path.append(path[j])
            i = j
        return smoothed_path

    def is_collision_free_path(self, point1: Point, point2: Point):
        direction = (point2 - point1).normalize()
        distance = point1.dist_to(point2)
        steps = int(distance / self.step_size)
        for step in range(steps):
            intermediate_point = point1 + direction * (step * self.step_size)
            if self.is_collision(intermediate_point):
                return False
        return True