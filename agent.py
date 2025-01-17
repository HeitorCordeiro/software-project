from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.ssl.rrt import RRT

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.path = []

    def decision(self):
        if len(self.targets) == 0:
            return

        if not self.path:
            self.plan_path()

        if not self.path:
            return

        next_target = self.path[0]
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, next_target)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

        if self.robot.dist_to(next_target) < 50:  # Ajuste a distância conforme necessário
            self.path.pop(0)

    def plan_path(self):
        obstacles = {**self.teammates, **self.opponents}
        rrt = RRT(self.pos, self.targets[0], obstacles, width=6000, height=2000)
        self.path = rrt.plan()

    def post_decision(self):
        pass