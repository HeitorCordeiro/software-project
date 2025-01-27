from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.ssl.rrt import RRT
from utils.Point import Point

class ExampleAgent(BaseAgent):
    assigned_targets = []

    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.path = []
        self.assigned_target = None

    def decision(self):
        if len(self.targets) == 0:
            return

        min_dist = float('inf')
        for target in self.targets:
            if target in ExampleAgent.assigned_targets:
                continue
            dist = Point(self.robot.x, self.robot.y).dist_to(target)
            if dist < min_dist:
                min_dist = dist
                self.assigned_target = target

        if self.assigned_target:
            ExampleAgent.assigned_targets.append(self.assigned_target)

    def post_decision(self):
        if self.assigned_target is None:
            return

        obstacles = [Point(robot.x, robot.y) for robot in self.opponents.values()]

        if not self.path or self.is_path_blocked(obstacles):
            rrt = RRT(Point(self.robot.x, self.robot.y), self.assigned_target, obstacles)
            self.path = rrt.generate_path()

        if self.path:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.path[0])
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
            if Point(self.robot.x, self.robot.y).dist_to(self.path[0]) < 0.1:
                self.path.pop(0)

    def is_path_blocked(self, obstacles):
        if not self.path:
            return False
        for obstacle in obstacles:
            if self.path[0].dist_to(obstacle) < 0.2:
                return True
        return False
