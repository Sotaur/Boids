import math
from Node import *


class Boid:
    def __init__(self):
        # [x, y] for velocity and position
        self.velocity = []
        self.new_velocity = []
        self.angle = 0
        self.position = []
        self.node = None
        self.nearest = []

    def __str__(self):
        to_return = ""
        to_return += str(self.velocity)
        to_return += " " + str(self.velocity_mag())
        to_return += " " + str(self.position)
        return to_return

    def velocity_mag(self):
        return math.sqrt(pow(self.velocity[0], 2) + pow(self.velocity[1], 2))

    def position_mag(self):
        return math.sqrt(pow(self.position[0], 2) + pow(self.position[1], 2))

    def calc_velocity(self, time):
        x_avg = self.velocity[0]
        y_avg = self.velocity[1]
        effective_num = 1
        if type(self.node) is ActiveNode:
            for edge in self.node.active:
                weight = edge.update(time)
                if weight != 0:
                    effective_num += abs(weight)
                    x_avg += edge.go_to().boid.velocity[0] * weight
                    y_avg += edge.go_to().boid.velocity[1] * weight
        else:
            for edge in self.node.edges:
                weight = edge.update(time)
                if weight != 0:
                    effective_num += abs(weight)
                    x_avg += edge.go_to().boid.velocity[0] * weight
                    y_avg += edge.go_to().boid.velocity[1] * weight
        x_avg /= effective_num
        y_avg /= effective_num
        norm = math.sqrt(pow(x_avg, 2) + pow(y_avg, 2)) if (x_avg != 0 and y_avg != 0) else 1
        self.new_velocity = [x_avg / norm, y_avg / norm]

    def update_velocity(self):
        self.velocity = self.new_velocity

    def clear_nearest(self):
        self.nearest = []