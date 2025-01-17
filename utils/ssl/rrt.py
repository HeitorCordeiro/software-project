import random
import math
from utils.Point import Point
from rsoccer_gym.Entities import Robot

class RRT:
    def __init__(self, start:Point, goal:Point, obstacles, width, height, max_iter=1000):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.width = width
        self.height = height
        self.max_iter = max_iter
        self.tree = [start]

    def distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def is_collision(self, point):
        for obstacle in self.obstacles:
            if self.distance(point, obstacle) < 100:  # Ajuste a distância conforme necessário
                return True
        return False

    def get_random_point(self):
        return Point(random.uniform(0, self.width), random.uniform(0, self.height))

    def get_nearest_point(self, point):
        nearest_point = self.tree[0]
        min_dist = self.distance(point, nearest_point)
        print("aqui")
        for p in self.tree:
            dist = self.distance(point, p)
            if dist < min_dist:
                nearest_point = p
                min_dist = dist
        return nearest_point

    def steer(self, from_point, to_point, extend_length=50):
        direction = math.atan2(to_point.y - from_point.y, to_point.x - from_point.x)
        new_point = Point(
            from_point.x + extend_length * math.cos(direction),
            from_point.y + extend_length * math.sin(direction)
        )
        return new_point

    def plan(self):
        for _ in range(self.max_iter):
            rand_point = self.get_random_point()
            nearest_point = self.get_nearest_point(rand_point)
            new_point = self.steer(nearest_point, rand_point)

            if not self.is_collision(new_point):
                self.tree.append(new_point)
                if self.distance(new_point, self.goal) < 50:  # Ajuste a distância conforme necessário
                    return self.generate_path(new_point)

        return None

    def generate_path(self, end_point):
        path = [end_point]
        current_point = end_point
        while current_point != self.start:
            nearest_point = self.get_nearest_point(current_point)
            path.append(nearest_point)
            current_point = nearest_point
        path.reverse()
        return path