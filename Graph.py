import math
import random
import statistics


class Graph:
    def __init__(self, time_interval):
        self.network = []
        self.time_interval = time_interval
        self.scc_min = math.inf
        self.scc_max = 0
        self.scc_avg = 0
        self.scc_median = 0
        self.scc_high_median = 0
        self.scc_med_mean = 0
        self.scc_std_dev = 0
        self.scc_std_dev_mean = 0
        self.scc_std_dev_median = 0
        self.flock = None
        self.num_sccs_calc = 0
        self.scc_data = []
        self.colors = []
        xkcd_colors = open('rgb.txt', 'r')
        for line in xkcd_colors:
            if line[0] == '#':
                continue
            contents = line.split("#")
            self.colors.append("xkcd:" + contents[0].strip())
        self.color_offset = random.randint(0, len(self.colors) - 1)

    def __str__(self):
        to_return = []
        to_return.append("Num nodes: " + str(len(self.network)) + "\n")
        for node in self.network:
            to_return.append(str(node) + "\n")
        return ''.join(to_return)

    def add_node(self, node):
        self.network.append(node)

    def reset_nodes(self):
        for node in self.network:
            node.visited = False

    # Depth First Search
    def dfs_recurse(self, node, reached):
        if not node.visited:
            node.visited = True
            reached.append(node)
            for edge in node.edges:
                if edge.value != 0:
                    self.dfs_recurse(edge.go_to(), reached)

    def depth_first_search(self, node):
        reached = []
        if not node.visited:
            self.dfs_recurse(node, reached)
        return reached

    def post_order_recurse(self, node, order):
        if not node.visited:
            node.visited = True
            for edge in node.reverse:
                if not edge.go_to().visited and edge.value != 0:
                    self.post_order_recurse(edge.go_to(), order)
            order.append(node)

    def post_order(self):
        order = []
        for node in self.network:
            if not node.visited:
                self.post_order_recurse(node, order)
        self.reset_nodes()
        return order

    def scc_metrics(self, sccs):
        max_len = 0
        min_len = math.inf
        data = []
        for scc in sccs:
            scc_len = len(scc)
            if scc_len > max_len:
                max_len = scc_len
            elif scc_len < min_len:
                min_len = scc_len
            data.append(len(scc))
        avg_len = statistics.mean(data)
        median = statistics.median_high(data)
        std_dev = None
        if len(data) > 1:
            std_dev = statistics.stdev(data)
        self.scc_data.append((len(sccs), max_len, min_len, avg_len, median, std_dev))

    # Strongly Connected Component
    def calculate_scc(self):
        order = self.post_order()
        sccs = []
        self.num_sccs_calc += 1
        for i in range(0, len(order)):
            if not order[len(order) - i - 1].visited:
                component = self.depth_first_search(order[len(order) - i - 1])
                sccs.append(component)
        self.reset_nodes()
        self.scc_metrics(sccs)
        return sccs

    def sort_nodes(self):
        self.network.sort(key=lambda node: node.id)

    def get_colors(self, num_colors):
        if len(self.colors) - self.color_offset < num_colors:
            self.color_offset = random.randint(0, len(self.colors) - num_colors - 1)
        offset = self.color_offset - num_colors if self.color_offset >= num_colors else self.color_offset
        return self.colors[offset: num_colors + offset]

    def get_node_by_id(self, id):
        if not self.sorted:
            self.sort_nodes()
            self.sorted = True
        return self.network[id - 1]

    def reset(self):
        self.network = []
        self.scc_data = []
        self.scc_max = 0
        self.scc_avg = 0
        self.scc_median = 0
        self.scc_high_median = 0
        self.scc_med_mean = 0
        self.scc_std_dev = 0
        self.scc_min = math.inf
        self.sorted = False

    def calculate_scc_stats(self):
        num_median = []
        high_median = []
        median_mean = []
        std_dev_data = []
        for item in self.scc_data:
            scc_num = item[0]
            num_median.append(scc_num)
            high_median.append(item[1])
            median_mean.append(item[4])
            if item[5] is not None:
                std_dev_data.append(item[5])
            if scc_num > self.scc_max:
                self.scc_max = scc_num
            elif scc_num < self.scc_min:
                self.scc_min = scc_num
        self.scc_avg = statistics.mean(num_median)
        self.scc_median = statistics.median(num_median)
        self.scc_high_median = statistics.median(high_median)
        self.scc_med_mean = statistics.mean(median_mean)
        self.scc_std_dev_mean = statistics.mean(std_dev_data) if len(std_dev_data) > 0 else "No data"
        self.scc_std_dev_median = statistics.median(std_dev_data) if len(std_dev_data) > 0 else "No data"
        if len(num_median) > 1:
            self.scc_std_dev = statistics.stdev(num_median)
