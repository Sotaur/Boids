from Edge import Edge
from Function import Function
import random
import math
import numpy as np
from decimal import Decimal
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
        self.frustration_type = ""
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
        self.group_one_start = 0
        self.group_two_start = 0
        self.group_size = 0
        self.group_one_align = []
        self.group_two_align = []
        self.flock_order_param = []
        self.phase = []
        self.phase_1 = 0
        self.phase_2 = 0
        self.phase_3 = 0
        self.flock_velocity = []
        self.group_one_rotate = []
        self.group_two_rotate = []
        self.reflect = None
        self.one_and_flock = []
        self.one_and_group = []
        self.group_and_group = []
        self.boid_one = None
        self.group_one_corr_start = 0
        self.group_two_corr_start = 0
        self.segment_size = 5
        self.scc_velocity = []
        self.will_turn = None
        self.tail_length = 5
        self.tails = False

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
        self.group_one_start = random.randint(0, len(self.boids) - 1 - self.group_size)
        self.group_two_start = random.randint(0, len(self.boids) - 1 - self.group_size)
        while self.group_one_start < self.group_two_start < self.group_one_start + self.group_size\
                and self.group_one_start < self.group_two_start + self.group_two_start < self.group_one_start + self.group_size:
            self.group_two_start = random.randint(0, len(self.boids) - 1 - self.group_size)
        self.phase_1 = random.randint(0, len(self.boids) - 1)
        self.phase_2 = random.randint(0, len(self.boids) - 1)
        while self.phase_1 == self.phase_2:
            self.phase_2 = random.randint(0, len(self.boids) - 1)
        self.phase_3 = random.randint(0, len(self.boids) - 1)
        while self.phase_3 == self.phase_2 or self.phase_3 == self.phase_1:
            self.phase_3 = random.randint(0, len(self.boids) - 1)
        self.set_frustration()
        self.boid_one = random.randint(0, len(self.boids))
        self.group_one_corr_start = random.randint(0, len(self.boids) - self.num_neighbors * 2)
        while self.group_one_corr_start <= self.boid_one <= self.group_one_corr_start + 2 * self.num_neighbors:
            self.group_one_corr_start = random.randint(0, len(self.boids) - self.num_neighbors * 2)
        self.group_two_corr_start = random.randint(0, len(self.boids) - self.num_neighbors * 2)
        while self.group_one_corr_start <= self.group_two_corr_start <= self.group_one_corr_start + 2 * self.num_neighbors\
                or self.group_two_corr_start <= self.group_one_corr_start <= self.group_two_corr_start + 2 * self.num_neighbors:
            self.group_two_corr_start = random.randint(0, len(self.boids) - self.num_neighbors * 2)

    def update_flock(self, time):
        self.current_time = time
        for boid in self.boids:
            boid.calc_velocity(time)
        for boid in self.boids:
            boid.update_velocity()
        for boid in self.boids:
            self.reflect(boid)
        for boid in self.boids:
            boid.update_position()
        if (self.neighbor_type == "s") and self.neighbor_select == 'n' and (time % self.calculate_flock_mates == 0):
            self.new_nearest_neighbors()
        elif (self.neighbor_type == "s") and self.neighbor_select == 'r' and (time % self.calculate_flock_mates == 0):
            for boid in self.boids:
                self.new_random_neighbors(boid)
        elif self.neighbor_type == 'a' and (time % self.calculate_flock_mates == 0):
            self.active_update()
        self.calculate_parameters()

    def set_reflect(self):
        if self.frustration_type == "pu":
            self.reflect = self.position_u_turn
        elif self.frustration_type == 'vu':
            self.reflect = self.velocity_u_turn
        elif self.frustration_type == 'p':
            self.reflect = self.pseudo_specular
        elif self.frustration_type == 's':
            self.reflect = self.specular
        elif self.frustration_type == 'a':
            self.reflect = self.specular
        elif self.frustration_type == 'f':
            self.reflect = self.specular
        elif self.frustration_type == 'an':
            self.reflect = self.alpha_angular
        elif self.frustration_type == 'bn':
            self.reflect = self.beta_angular
        elif self.frustration_type == 'dn':
            self.reflect = self.dual_angular
        elif self.frustration_type == 'ca':
            self.reflect = self.alpha_angular
        elif self.frustration_type == 'ta':
            self.reflect = self.triangle_alpha
        elif self.frustration_type == 'fa':
            self.reflect = self.fixed_alpha

    def set_frustration(self):
        if self.frustration == 'f':
            self.will_turn = self.basin_reflection
            self.set_reflect()
        elif self.frustration == 'a':
            self.will_turn = lambda x: None
            self.reflect = self.attraction

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

    def basin_reflection(self, boid):
        direction = (boid.position[0]) * boid.velocity[0] + (boid.position[1]) * boid.velocity[1]
        position = boid.position_mag()
        if position <= self.basin or direction >= 0:
            turn_prob = pow(position / self.basin, self.frustration_power)
            return turn_prob > random.uniform(0, 1)
        return False

    # TODO try just left and right, more topological, and larger disruption for inward boids
    def attraction(self, boid):
        pos_angle = math.atan2(boid.position[1], boid.position[0])
        vel_angle = math.atan2(boid.velocity[1], boid.velocity[0])
        adjusted_angle = float(Decimal(vel_angle - (pos_angle - math.pi / 2)) % Decimal(2 * math.pi))
        pos_mag = boid.position_mag()
        if 0.0 < abs(adjusted_angle) < math.pi / 2:
            vel_angle -= random.triangular(0, 1, .5) * pow(pos_mag / self.basin, self.frustration_power)
        elif math.pi / 2 < abs(adjusted_angle) < math.pi:
            vel_angle += random.triangular(0, 1, .5) * pow(pos_mag / self.basin, self.frustration_power)
        else:
            vel_angle += random.uniform(-.25, .25) * pow(pos_mag / self.basin, self.frustration_power)
        boid.velocity = [self.velocity * math.cos(vel_angle), self.velocity * math.sin(vel_angle)]

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

    def specular(self, boid):
        if self.will_turn(boid):
            if self.alpha is None:
                self.alpha = random.uniform(0, 2)
            velocity = np.array(boid.velocity)
            position = np.array(boid.position)
            mag_position = np.linalg.norm(position)
            v_parallel = (velocity.dot(position) / mag_position)
            v_parallel *= position / mag_position
            v_perp = velocity - v_parallel
            v_prime = velocity - 2 * (v_parallel + self.alpha * v_perp)
            prime_mag = np.linalg.norm(v_prime)
            boid.velocity = np.ndarray.tolist(v_prime / prime_mag)

    def alpha_angular(self, boid):
        if self.will_turn(boid):
            alpha = random.uniform(self.alpha_center - self.alpha_range, self.alpha_center + self.alpha_range)
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

    def fixed_alpha(self, boid):
        if self.will_turn(boid):
            p_angle = math.atan2(boid.position[1], boid.position[0])
            v_angle = math.atan2(boid.velocity[1], boid.velocity[0])
            angle = p_angle - math.pi + self.alpha * v_angle
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
        self.group_one_align = []
        self.group_two_align = []
        self.flock_order_param = []
        self.setup_done = False
        self.phase = []
        self.group_two_rotate = []
        self.group_one_rotate = []
        self.flock_velocity = []
        self.one_and_flock = []
        self.one_and_group = []
        self.group_and_group = []
        self.scc_velocity = []
        for boid in self.boids:
            boid.reset()

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

    def calculate_align_param(self, group):
        vx = 0
        vy = 0
        for boid in group:
            vx += boid.velocity[0]
            vy += boid.velocity[1]
        mag = math.sqrt(vx*vx + vy*vy)
        return mag/(len(group) * self.velocity)

    def calculate_scc_align_param(self, velocities):
        vx = 0
        vy = 0
        for velocity in velocities:
            vx += velocity[0]
            vy += velocity[1]
        mag = math.sqrt(vx * vx + vy * vy)
        return mag / (len(velocities) * self.velocity)

    def get_velocity(self, group):
        to_return = []
        for boid in group:
            to_return.append(boid.velocity)
        return to_return

    def calculate_rotation_param(self, group):
        order_param = 0
        if len(group) == 0:
            return 0
        for k in range(0, len(group[0])):
            for j in range(0, len(group) - 1):
                v = group[j][k]
                v1 = group[j + 1][k]
                order_param += v[0] * v1[1] - v1[0] * v[1]
        return order_param

    def calculate_rotation_params(self):
        to_return = []
        for i in range(0, len(self.flock_velocity)):
            end_index = i + self.segment_size if len(self.flock_velocity) - i > self.segment_size else len(self.flock_velocity) - 1
            rotation_flock = self.calculate_rotation_param(self.flock_velocity[i:end_index + 1]) /\
                             (len(self.boids) * self.segment_size * pow(self.velocity, 2))
            rotation_g1 = self.calculate_rotation_param(self.group_one_rotate[i:end_index + 1]) /\
                          (self.group_size * self.segment_size * pow(self.velocity, 2))
            rotation_g2 = self.calculate_rotation_param(self.group_two_rotate[i:end_index + 1]) /\
                          (self.group_size * self.segment_size * pow(self.velocity, 2))
            to_return.append((rotation_flock, rotation_g1, rotation_g2))
        return to_return

    def calculate_scc_rotation(self):
        to_return = []
        for i in range(0, len(self.scc_velocity), self.calculate_flock_mates):
            current_block = self.scc_velocity[i:i + self.calculate_flock_mates]
            for j in range(0, len(current_block)):
                params = []
                for k in range(0, len(current_block[0])):
                    scc = []
                    end_index = j + self.segment_size if j + self.segment_size < len(current_block) else len(current_block) - 1
                    for l in range(j, end_index):
                        level_1 = current_block[l]
                        k_slice = level_1[k:k + 1]
                        scc.append(k_slice[0])

                    param = self.calculate_rotation_param(scc) / (len(scc[0]) * self.segment_size * pow(self.velocity, 2)) \
                            if len(scc) > 0 else 0
                    params.append(param)
                to_return.append(params)
        return to_return

    def calculate_scc_alignment(self):
        to_return = []
        for i in range(0, len(self.scc_velocity), self.calculate_flock_mates):
            current_block = self.scc_velocity[i:i + self.calculate_flock_mates]
            for j in range(0, len(current_block)):
                params = []
                for k in range(0, len(current_block[0])):
                    param = self.calculate_scc_align_param(current_block[j][k])
                    params.append(param)
                to_return.append(params)
        return to_return

    def calculate_scc_correlation(self):
        to_return = []
        for i in range(0, len(self.scc_velocity), self.calculate_flock_mates):
            current_block = self.scc_velocity[i:i + self.calculate_flock_mates]
            for j in range(0, len(current_block)):
                params = []
                for k in range(0, len(current_block[0])):
                    for l in range(k + 1, len(current_block[0])):
                        if k != l:
                            param = self.calculate_scc_correlation_param(current_block[j][k], current_block[j][l])
                            params.append(param)
                to_return.append(params)
        return to_return

    def calculate_scc_correlation_param(self, group1, group2):
        v1 = [0, 0]
        for velocity in group1:
            v1[0] += velocity[0]
            v1[1] += velocity[1]
        v2 = [0, 0]
        for velocity in group2:
            v2[0] += velocity[0]
            v2[1] += velocity[1]
        v1_mag = math.sqrt(pow(v1[0], 2) + pow(v1[1], 2))
        v2_mag = math.sqrt(pow(v2[0], 2) + pow(v2[1], 2))
        return (v1[0] / v1_mag) * (v2[0] / v2_mag) + (v1[1] / v1_mag) * (v2[1] / v2_mag)

    def calculate_correlation(self, group1, group2):
        v1 = [0, 0]
        for boid in group1:
            v1[0] += boid.velocity[0]
            v1[1] += boid.velocity[1]
        v2 = [0, 0]
        for boid in group2:
            v2[0] += boid.velocity[0]
            v2[1] += boid.velocity[1]
        v1_mag = math.sqrt(pow(v1[0], 2) + pow(v1[1], 2))
        v2_mag = math.sqrt(pow(v2[0], 2) + pow(v2[1], 2))
        return (v1[0] / v1_mag) * (v2[0] / v2_mag) + (v1[1] / v1_mag) * (v2[1] / v2_mag)

    def get_phase(self):
        boid1 = self.boids[self.phase_1]
        boid2 = self.boids[self.phase_2]
        boid3 = self.boids[self.phase_3]
        phase = (math.atan2(boid1.position[1], boid1.position[0]), math.atan2(boid1.velocity[1], boid1.velocity[0]),
                 math.atan2(boid2.position[1], boid2.position[0]), math.atan2(boid2.velocity[1], boid2.velocity[0]),
                 math.atan2(boid3.position[1], boid3.position[0]), math.atan2(boid3.velocity[1], boid3.velocity[0]))
        self.phase.append(phase)

    def calculate_parameters(self):
        self.flock_order_param.append(self.calculate_align_param(self.boids))
        self.group_one_align.append(
            self.calculate_align_param(self.boids[self.group_one_start: self.group_one_start + self.group_size]))
        self.group_two_align.append(
            self.calculate_align_param(self.boids[self.group_two_start: self.group_two_start + self.group_size]))
        self.get_phase()
        self.flock_velocity.append(self.get_velocity(self.boids))
        self.group_one_rotate.append(
            self.get_velocity(self.boids[self.group_one_start: self.group_one_start + self.group_size]))
        self.group_two_rotate.append(
            self.get_velocity(self.boids[self.group_two_start: self.group_two_start + self.group_size]))
        self.one_and_flock.append(self.calculate_correlation([self.boids[self.boid_one]], self.boids))
        self.one_and_group.append(self.calculate_correlation([self.boids[self.boid_one]],
                                                             self.boids[self.group_one_corr_start:self.group_one_corr_start + 2 * self.num_neighbors]))
        self.group_and_group.append(self.calculate_correlation(self.boids[self.group_one_corr_start:self.group_one_corr_start + 2 * self.num_neighbors],
                                                               self.boids[self.group_two_corr_start:self.group_two_corr_start + 2 * self.num_neighbors]))