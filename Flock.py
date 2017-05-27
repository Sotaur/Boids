from Edge import Edge
from Function import Function
import random
import math
import numpy as np
import time
import sys

""""
A Flock takes in the number of neighbors for each boid, the size of the basin, and the velocity for each boid.

initialize_flock gives every boid a random position between -5 and 5 on both the x and y axes, a random angle
between -pi and pi which is used to determine the direction of its velocity

update_flock causes each boid to update its velocity based on the weighted average of its flock mates. The weight
is determined by the weight of the edge between the boids. It also determines if the boid needs to turn around, if the
magnitude of the position is greater than the basin value
"""


class Flock:
    def __init__(self, num, basin, velocity, recalc, update="s"):
        self.boids = []
        self.num_neighbors = num
        self.basin = basin
        self.velocity = velocity
        self.graph = None
        self.current_time = 0
        self.calculate_flock_mates = recalc
        self.neighbor_type = update
        self.frustration_power = 1
        self.nearest_num = 0
        self.reflection_type = ""
        self.alpha = 0
        self.frustration = True
        self.neighbor_select = 'n'
        self.active_type = ''
        self.random_int = 0
        self.num_topological = 0
        self.block_size = 0
        self.setup_done = False
        self.alpha_center = .5
        self.alpha_range = .5
        self.triangle_high = 1
        self.triangle_low = 0

    def __str__(self):
        to_return = ""
        for boid in self.boids:
            to_return += str(boid) + '\n'
        return to_return

    def add_boid(self, boid):
        self.boids.append(boid)

    def initialize_flock(self):
        for boid in self.boids:
            boid.position = [random.uniform(-5, 5), random.uniform(-5, 5)]
            while boid.position_mag() >= 6:
                boid.position = [random.uniform(-5, 5), random.uniform(-5, 5)]
            boid.angle = random.uniform(-math.pi * 2, math.pi * 2)
            boid.velocity = [self.velocity * math.cos(boid.angle), self.velocity * math.sin(boid.angle)]

    def update_flock(self, time):
        self.current_time = time
        for boid in self.boids:
            boid.calc_velocity(time)
        for boid in self.boids:
            boid.update_velocity()
        if self.frustration:
            for boid in self.boids:
                self.reflect(boid)
        for boid in self.boids:
            boid.position = [boid.position[0] + boid.velocity[0], boid.position[1] + boid.velocity[1]]
        if not self.frustration:
            for boid in self.boids:
                self.periodic(boid)
        if (self.neighbor_type == "s") and self.neighbor_select == 'n' and (time % self.calculate_flock_mates == 0):
            self.new_nearest_neighbors()
        elif (self.neighbor_type == "s") and self.neighbor_select == 'r' and (time % self.calculate_flock_mates == 0):
            for boid in self.boids:
                self.new_random_neighbors(boid)
        elif self.neighbor_type == 'a' and (time % self.calculate_flock_mates == 0):
            self.active_update()

    def reflect(self, boid):
        if self.reflection_type == "pu":
            self.position_u_turn(boid)
        elif self.reflection_type == 'vu':
            self.velocity_u_turn(boid)
        elif self.reflection_type == 'p':
            self.pseudo_specular(boid)
        elif self.reflection_type == 's':
            self.specular(boid, 0)
        elif self.reflection_type == 'a':
            self.specular(boid)
        elif self.reflection_type == 'f':
            self.specular(boid, self.alpha)
        elif self.reflection_type == 'an':
            self.alpha_angular(boid)
        elif self.reflection_type == 'bn':
            self.beta_angular(boid)
        elif self.reflection_type == 'dn':
            self.dual_angular(boid)
        elif self.reflection_type == 'ca':
            self.alpha_angular(boid, self.alpha_center, self.alpha_range)
        elif self.reflection_type == 'ta':
            self.triangle_alpha(boid)

    def active_update(self):
        if self.active_type == 'r':
            self.active_random_update()
        elif self.active_type == 'n':
            self.active_nearest_update()
        elif self.active_type == 'rn':
            self.active_random_num_update()
        elif self.active_type == 'bc':
            self.active_block_combo_update()

    def active_random_num_update(self):
        if not self.setup_done:
            self.active_nearest_update()
            self.setup_done = True
        else:
            self.nearest_neighbors()
            for boid in self.boids:
                node = boid.node
                node.toggle_active()
                nearest = boid.nearest
                nearest_edges = []
                for tup in nearest:
                    index = tup[1].node.id - 1
                    nearest_edges.append(node.edges[index])
                for i in range(0, self.random_int):
                    old = random.choice(node.active)
                    node.active.remove(old)
                    new = random.choice(nearest_edges)
                    if not set(nearest_edges).issubset(node.active):
                        while new in node.active:
                            new = random.choice(nearest_edges)
                    node.add_active(new)
                node.toggle_active()

    def active_add_nearest(self, boid, node):
        nearest = boid.nearest
        for tup in nearest:
            neighbor = tup[1]
            index = neighbor.node.id - 1
            node.add_active(node.edges[index])
        node.toggle_active()

    def active_nearest_update(self):
        self.nearest_neighbors()
        for boid in self.boids:
            node = boid.node
            node.toggle_active()
            node.clear_active()
            self.active_add_nearest(boid, node)
            boid.clear_nearest()

    def active_random_update(self):
        max_index = len(self.graph.network) - 1
        for node in self.graph.network:
            node.toggle_active()
            node.clear_active()
            for i in range(0, self.num_neighbors):
                index = random.randint(0, max_index)
                if index != node.id - 1:
                    node.add_active(node.edges[index])
                else:
                    index = (index + 1) % max_index
                    node.add_active(node.edges[index])
            node.toggle_active()

    def active_block_combo_update(self):
        network_size = len(self.graph.network)
        for boid in self.boids:
            boid.node.toggle_active()
        if not self.setup_done:
            self.setup_done = True
            self.calc_block_mates(network_size)
        self.nearest_neighbors()
        for boid in self.boids:
            node = boid.node
            node.active = node.active[:self.num_topological]  # trims nearest neighbors without removing block mates
            self.active_add_nearest(boid, node)
            boid.clear_nearest()

    def add_block_mates(self, block, i, j, node):
        block_mates = []
        if j + self.num_topological < i + self.block_size:
            block_mates = block[j: j + self.num_topological]
        else:
            block_mates = block[j: i + self.block_size]
            block_mates += block[: (j + self.num_topological) % (i + self.block_size)]
        for other_node in block_mates:
            index = other_node.id - 1
            node.add_active(node.edges[index])

    def calc_block_mates(self, network_size):
        for i in range(0, network_size, self.block_size):
            if i + self.block_size < network_size:
                block = self.graph.network[i: i + self.block_size]
                for j in range(i, i + self.block_size):
                    node = self.graph.network[j]
                    self.add_block_mates(block, i, j, node)
            else:
                block = self.graph.network[i: network_size]
                for j in range(i, network_size):
                    node = self.graph.network[j]
                    if network_size - i < self.num_topological:
                        for other_node in block:
                            index = other_node.id - 1
                            node.add_active(node.edges[index])
                    else:
                        node = self.graph.network[j]
                        self.add_block_mates(block, i, j, node)

    def new_nearest_neighbors(self):
        self.nearest_neighbors()
        for boid in self.boids:
            neighbors = boid.nearest
            boid.node.reset_edges()
            for neighbor_tuple in neighbors:
                neighbor = neighbor_tuple[1].node
                self.add_edge(boid, neighbor)
            boid.clear_nearest()

    def new_random_neighbors(self, boid):
        node = boid.node
        node.reset_edges()
        for i in range(0, self.num_neighbors):
            neighbor = self.graph.get_node_by_id(random.randint(0, len(self.graph.network) - 1))
            if neighbor is not None:
                self.add_edge(boid, neighbor)

    def add_edge(self, boid, neighbor):
        edge_function = Function(1)
        edge = Edge(neighbor, edge_function)
        boid.node.add_edge(edge)

    def will_turn(self, boid):
        direction = (boid.position[0]) * boid.velocity[0] + (boid.position[1]) * boid.velocity[1]
        position = boid.position_mag()
        if position <= 5.5 or direction >= 0:
            turn_prob = pow(position / self.basin, self.frustration_power)
            return turn_prob > random.uniform(0, 1)
        return False

    def velocity_u_turn(self, boid):
        if self.will_turn(boid):
            boid.velocity = [-boid.velocity[0], -boid.velocity[1]]

    def position_u_turn(self, boid):
        if self.will_turn(boid):
            angle = math.atan2(boid.position[1], boid.position[0]) + math.pi
            boid.velocity = [self.velocity * math.cos(angle), self.velocity * math.sin(angle)]

    def pseudo_specular(self, boid):
        if self.will_turn(boid):
            initial_angle = math.atan2(boid.position[1], boid.position[0])
            direction = [1, -1]
            new_angle = initial_angle + random.choice(direction) * (1 + random.uniform(0, 1)) * math.pi / 2
            boid.velocity = [self.velocity * math.cos(new_angle), self.velocity * math.sin(new_angle)]

    def specular(self, boid, alpha=None):
        if self.will_turn(boid):
            if alpha is None:
                alpha = random.uniform(0, 2)
            velocity = np.array(boid.velocity)
            position = np.array(boid.position)
            mag_position = np.linalg.norm(position)
            v_parallel = (velocity.dot(position) / mag_position)
            v_parallel *= position / mag_position
            v_perp = velocity - v_parallel
            v_prime = velocity - 2 * (v_parallel + alpha * v_perp)
            prime_mag = np.linalg.norm(v_prime)
            boid.velocity = np.ndarray.tolist(v_prime / prime_mag)

    def alpha_angular(self, boid, center=.5, range=.5):
        if self.will_turn(boid):
            alpha = random.uniform(center - range, center + range)
            p_angle = math.atan2(boid.position[1], boid.position[0])
            v_angle = math.atan2(boid.velocity[1], boid.velocity[0])
            angle = p_angle - math.pi + alpha * v_angle
            boid.velocity = [self.velocity * math.cos(angle), self.velocity * math.sin(angle)]

    def triangle_alpha(self, boid):
        if self.will_turn(boid):
            alpha = random.triangular(self.triangle_low, self.triangle_high, self.alpha_center)
            p_angle = math.atan2(boid.position[1], boid.position[0])
            v_angle = math.atan2(boid.velocity[1], boid.velocity[0])
            angle = p_angle - math.pi + alpha * v_angle
            boid.velocity = [self.velocity * math.cos(angle), self.velocity * math.sin(angle)]

    def beta_angular(self, boid):
        if self.will_turn(boid):
            beta = random.uniform(0, 1)
            p_angle = math.atan2(boid.position[1], boid.position[0])
            v_angle = math.atan2(boid.velocity[1], boid.velocity[0])
            angle = beta*(p_angle - math.pi) + v_angle
            boid.velocity = [self.velocity * math.cos(angle), self.velocity * math.sin(angle)]

    def dual_angular(self, boid):
        if random.uniform(0, 1) >= .5:
            self.beta_angular(boid)
        else:
            self.alpha_angular(boid)

    def periodic(self, boid):
        x = boid.position[0]
        y = boid.position[1]
        if abs(x) >= 8 or abs(y) >= 8:
            boid.position[0] = x % 8 - 8
            boid.position[1] = y % 8 - 8

    def reset(self):
        self.boids = []
        self.setup_done = False

    def nearest_neighbors(self):
        # start = time.perf_counter()
        max_index = len(self.boids)
        for i in range(0, max_index):
            boid = self.boids[i]
            for j in range(i, max_index):
                other_boid = self.boids[j]
                if boid != other_boid:
                    distance = pow(boid.position[0] - other_boid.position[0], 2) + pow(boid.position[1] - other_boid.position[1], 2)
                    boid.nearest.append((distance, other_boid))
                    other_boid.nearest.append((distance, boid))
        # calc = time.perf_counter()
        for boid in self.boids:
            boid.nearest.sort(key=lambda x: x[0])
            boid.nearest = boid.nearest[0:self.nearest_num]
            boid.nearest = random.sample(boid.nearest, k=self.num_neighbors)
        # print(str(calc - start))
        # sys.stdout.flush()