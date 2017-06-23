from Flock import Flock
from Boid import Boid
from Graph import Graph
from Edge import *
from Node import *
from Function import *
import os
import time
import datetime
import sys
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import statistics
import progressbar
import cProfile


start_time = time.perf_counter()
random.seed()
graph = Graph(1)
flock = Flock(5, 6, 0.15, 15)
graph.flock = flock
flock.graph = graph
fig = plt.figure()
ax1 = fig.add_subplot(1, 2, 1)
ax2 = fig.add_subplot(1, 2, 2)
base_directory = "D:/Boid-data/"


"""
    This method puts a simple edge from the node to the neighbor with a weight of 1
"""


def simple_neighbor(node, neighbor):
    edge_function = Function(1)
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)

"""
    This method puts an edge with a square wave from the node to the neighbor with an initial phase
    determined by its position in the graph and the length of the period
"""


def uniform_square(node, neighbor, func_args, i):
    number = (i % func_args[0]) + 1
    edge_function = Square(func_args[1], func_args[2], func_args[3], func_args[3] / number)
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)

"""
    Same as the uniform square, but the entire wave is pre-defined and assigned based on the node's position
    and the number of possible waves
"""


def non_uniform_square(node, neighbor, func_args, i):
    number = i % len(func_args)
    wave = func_args[number]
    edge_function = Square(wave[0], wave[1], wave[2], wave[3])
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)

"""
    This method puts an edge between the node and neighbor with a sine wave with a random initial phase
"""


def single_sine(node, neighbor, func_args):
    edge_function = Sine([random.uniform(-math.pi, math.pi)], func_args[0], func_args[1], func_args[2])
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)

"""
    This method puts an edge between the node and neighbor with a pre-defined sine wave based
    on its position in the graph and the number of available sine waves
"""


def multi_sine(node, neighbor, func_args):
    phase = []
    for i in range(0, len(func_args[1])):
        phase.append(random.uniform(-math.pi, math.pi))
    edge_function = Sine(phase, func_args[0], func_args[1], func_args[2])
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)

"""
    This method puts an edge between the node and neighbor that varies like
    e^-position_mag()
"""


def neg_exp(node, neighbor):
    edge_function = NegExp(neighbor.boid, node.boid)
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)

"""
    Same as neg_exp, but when e^-position_mag <= cutoff the edge is of weight 0
"""


def cut_neg_exp(node, neighbor, cutoff):
    edge_function = CutNegExp(neighbor.boid, node.boid, cutoff)
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)


"""
    Creates an edge with a Log function, where value = Ln(6.5 - boid.position_mag())
"""


def log(node, neighbor):
    edge_function = Log(neighbor.boid, node.boid)
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)


def sqrt(node, neighbor):
    edge_function = Sqrt(neighbor.boid)
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)


def diff_sqrt(node, neighbor):
    edge_function = DiffSqrt(neighbor.boid, node.boid)
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)


def complex_exp(node, neighbor, period):
    edge_function = CompExp(neighbor.boid, node.boid, period)
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)


def rand_int(node, neighbor, func_args):
    edge_function = RandInt(func_args[0], func_args[1])
    edge = Edge(neighbor, edge_function)
    node.add_edge(edge)


def active_edge(node, neighbor):
    edge_function = ActiveFunction()
    edge = ActiveEdge(neighbor, edge_function)
    node.add_edge(edge)


def set_neighbor(node, neighbor, i, func_args):
    if flock.neighbor_type == "s":
        simple_neighbor(node, neighbor)
    elif flock.neighbor_type == "u":
        uniform_square(node, neighbor, func_args, i)
    elif flock.neighbor_type == "n":
        non_uniform_square(node, neighbor, func_args, i)
    elif flock.neighbor_type == 'ss':
        single_sine(node, neighbor, func_args)
    elif flock.neighbor_type == 'ms':
        multi_sine(node, neighbor, func_args)
    elif flock.neighbor_type == 'ne':
        neg_exp(node, neighbor)
    elif flock.neighbor_type == 'ce':
        cut_neg_exp(node, neighbor, func_args)
    elif flock.neighbor_type == 'l':
        log(node, neighbor)
    elif flock.neighbor_type == 'sq':
        sqrt(node, neighbor)
    elif flock.neighbor_type == 'dsq':
        diff_sqrt(node, neighbor)
    elif flock.neighbor_type == 'ie':
        complex_exp(node, neighbor, func_args)
    elif flock.neighbor_type == 'ri':
        rand_int(node, neighbor, func_args)

"""
make_flock_mates assigns the boids their flock mates based on neighbor_select.
 Is capable of assigning neighbors in multiple different ways including nearest n
 neighbors, random, and topologically
"""


def make_flock_mates(func_args):
    if flock.neighbor_select == 'n':
        flock.nearest_neighbors()
        for node in graph.network:
            neighbors = node.boid.nearest
            neighbor_num = 0
            for neighbor in neighbors:
                set_neighbor(node, neighbor[1].node, graph.network.index(node) + neighbor_num, func_args)
                neighbor_num += 1
    elif flock.neighbor_select == 'r':
        for node in graph.network:
            for i in range(0, flock.num_neighbors):
                neighbor = graph.get_node_by_id(random.randint(0, len(graph.network) - 1))
                while neighbor is None:
                    neighbor = graph.get_node_by_id(random.randint(0, len(graph.network) - 1))
                set_neighbor(node, neighbor, graph.network.index(node) + i, func_args)
    elif flock.neighbor_select == 't':
        for i in range(0, len(graph.network)):
            neighbors = []
            if i + 1 + flock.num_neighbors > len(graph.network):
                neighbors = graph.network[i + 1: len(graph.network)] + graph.network[:(i + flock.num_neighbors + 1) - len(graph.network)]
            else:
                neighbors = graph.network[i + 1: (i + flock.num_neighbors + 2)]
            neighbor_num = 0
            for neighbor in neighbors:
                set_neighbor(graph.network[i], neighbor, neighbor_num + i, func_args)
                neighbor_num += 1
    elif flock.neighbor_select == 'a':
        for node in graph.network:
            for neighbor in graph.network:
                active_edge(node, neighbor)
        flock.active_update()

"""initialize_graph takes in a number of boids to create, puts them in the graph,
generates the flock with those boids, and then calculates the initial flock mates for
each boid. The algorithm for calculating the flock mates is as follows: find the nearest
20 boids, and pick from them some random number of them as flock mates.
"""


def initialize_graph(num_boids, func_args):
    for i in range(1, num_boids + 1):
        boid = Boid()
        node = Node(boid, i) if flock.neighbor_type != 'a' else ActiveNode(boid, i)
        graph.add_node(node)
    create_flock()
    make_flock_mates(func_args)


def create_flock():
    for node in graph.network:
        flock.add_boid(node.boid)
    flock.initialize_flock()


def update_flock(duration):
    for i in range(duration):
        flock.update_flock(i)
        graph.calculate_scc()

"""
The grid obtained in get_flock_grid is a matrix containing the positions of a particular subgroup of boids
"""


def get_flock_grid(group):
    grid = ([], [])
    for node in group:
        grid[0].append(node.boid.position[0])  # x position
        grid[1].append(node.boid.position[1])  # y position
    return grid


neighbor_string = {
    "s": "standard",
    "u": "uniform square waves",
    "n": "non-uniform square waves",
    "ss": "single sine wave, rand phase",
    "ms": "multiple sine waves, rand phase",
    'ne': 'negative exp',
    'ce': 'cutoff neg exp',
    'l': 'ln',
    'sq': 'sqrt',
    'dsq': 'diff sqrt',
    'ie': 'complex exp',
    'ri': 'rand int',
    'a': 'active'
}
active_type = {
    'r': 'random',
    'n': 'nearest neighbor',
    'bc': 'block combo',
    'rn': 'random nearest'
}
reflection_string = {
    's': "specular reflection",
    'pu': 'position u-turn',
    'vu': 'velocity u-turn',
    'a': 'alpha reflection',
    'p': 'pseudo_specular',
    'f': 'fixed alpha',
    'an': 'alpha angular reflection',
    'bn': 'beta angular reflection',
    'dn': 'dual angular reflection',
    'ca': 'constrained alpha angular',
    'ta': 'triangular alpha angular'

}
selection_string = {
    'n': "nearest neighbors",
    'r': 'random neighbors',
    't': 'topological neighbors',
    'a': 'active'
}


def text_string():
    global neighbor_string
    global selection_string
    global reflection_string
    global active_type
    picked_from = str(flock.nearest_num) if flock.neighbor_type == 's' or flock.neighbor_select == 'n' or flock.neighbor_type == 'a' else 'N/A'
    frustration_string = ", frustration power: " + str(flock.frustration_power) if flock.frustration else ", periodic boundaries"
    reflection = ", reflection type: " + reflection_string[flock.reflection_type] if flock.frustration else ", no reflection"
    neighbor_type = ", neighbor type: " + neighbor_string[flock.neighbor_type]
    pick_type = selection_string[flock.neighbor_select] if flock.neighbor_type != 'a' else active_type[flock.active_type]
    before_split = "Number of boids: " + str(len(flock.boids)) + ", number of flock mates: " + str(flock.num_neighbors) + ", picking from: "\
        + picked_from + " using " + pick_type + reflection +\
        neighbor_type + ", neighbor recalculation interval: " + str(flock.calculate_flock_mates) + frustration_string
    split = before_split.split(" ")
    return " ".join(split[:round(len(split) / 2) + 1]) + '\n' + " ".join(split[round(len(split) / 2) + 1:])

"""
plot_flock does the calculations necessary to draw the flock.
It returns a list of quivers, which are representative of the sccs
from the graph. Assigns a color to each scc, and attempts to keep the
colors consistent between recalculations, but it does not always work.
"""


def plot_flock(interval, bar, individual_bar, text):
    # start = time.perf_counter()
    flock.update_flock(interval)
    # update = time.perf_counter()
    dots = []
    sccs = graph.calculate_scc()
    grids = []
    sccs.sort(key=lambda scc: len(scc))
    for scc in sccs:
        grids.append(get_flock_grid(scc))
    # grid_time = time.perf_counter()
    plt.clf()
    plt.axis([-8, 8, -8, 8])
    fig.text(1, .0085, text, fontsize=7, ha='right', va="bottom", multialignment="right")
    # setup_time = time.perf_counter()
    colors = graph.get_colors(len(sccs))
    for i in range(0, len(grids)):
        grid = grids[i]
        color = colors[i]
    #    before = time.perf_counter()
        dots.append(plt.scatter(grid[0], grid[1], c=color, s=16))
    #   print('one iter', time.perf_counter() - before)
    # quiver_time = time.perf_counter()
    # print(update - start)
    # print(grid_time - update)
    # print(setup_time - grid_time)
    # print(quiver_time - setup_time)
    # sys.stdout.flush()
    if individual_bar:
        bar.update(interval + 1)
    return dots


def plot_single(interval, bar, individual_bar, text):
    # start = time.perf_counter()
    flock.update_flock(interval)
    # update = time.perf_counter()
    dots = []
    boid = [graph.network[0]]
    flock_mates = []
    for edge in boid[0].edges:
        flock_mates.append(edge.go_to())
    grids = []
    grids.append(get_flock_grid(boid))
    grids.append(get_flock_grid(flock_mates))
    # grid_time = time.perf_counter()
    plt.clf()
    plt.axis([-8, 8, -8, 8])
    fig.text(1, .0085, text, fontsize=7, ha='right', va="bottom", multialignment="right")
    colors = graph.get_colors(2)
    for i in range(0, len(grids)):
        grid = grids[i]
        color = colors[i]
        dots.append(plt.scatter(grid[0], grid[1], c=color, s=16))
    # quiver_time = time.perf_counter()
    # print(len(quiver))
    # print(str(update - start))
    # print(str(grid_time - update))
    # print(str(-grid_time + quiver_time))
    # sys.stdout.flush()
    if individual_bar:
        bar.update(interval + 1)
    return dots


def neighbor_data_out(data, func_args):
    global neighbor_string
    global selection_string
    global reflection_string
    global active_type
    data.write("This was generated with calculating the flock mates using the " + neighbor_string[flock.neighbor_type]
               + " edges updating every " + str(flock.calculate_flock_mates) + " iterations\n")
    data.write("Neighbors were picked using " + selection_string[flock.neighbor_select] + " reflecting with " +
               reflection_string[flock.reflection_type] + "\n")
    if flock.neighbor_type == 'a' or flock.neighbor_select == 'n':
        data.write("Neighbors were picked out of " + str(flock.nearest_num) + '\n')
    if flock.neighbor_type == 'u':
        data.write("Number of waves, " + str(func_args[0]) + '\n')
        data.write("Minimum value, " + str(func_args[1]) + '\n')
        data.write("Maximum value, " + str(func_args[2]) + '\n')
        data.write("Period, " + str(func_args[3]) + '\n')
    elif flock.neighbor_type == 'n':
        data.write("Number of waves, " + str(len(func_args)) + '\n')
        wave_num = 0
        for wave in func_args:
            wave_num += 1
            data.write("Wave, " + str(wave_num) + '\n')
            data.write("Minimum value, " + str(wave[0]) + '\n')
            data.write("Maximum value, " + str(wave[1]) + '\n')
            data.write("Period, " + str(wave[2]) + '\n')
            data.write("Initial phase, " + str(wave[3]) + '\n')
    elif flock.neighbor_type == 'ss':
        data.write("Amplitude, " + str(func_args[0]) + '\n')
        data.write("Angular velocity, " + str(func_args[1]) + '\n')
        data.write("Offset, " + str(func_args[2]) + '\n')
    elif flock.neighbor_type == 'ms':
        data.write("Number of waves, " + str(len(func_args)) + '\n')
        wave_num = 0
        for wave in func_args:
            wave_num += 1
            data.write("Wave, " + str(wave_num) + '\n')
            data.write("Amplitude, " + str(wave[0]) + '\n')
            data.write("Angular velocity, " + str(wave[1]) + '\n')
            data.write("Offset, " + str(wave[2]) + '\n')
    elif flock.neighbor_type == 'ce':
        data.write("Cutoff, " + str(func_args) + '\n')
    elif flock.neighbor_type == 'ri':
        data.write("Upper, " + str(func_args[1]) + '\n')
        data.write("Lower, " + str(func_args[0]) + '\n')
    elif flock.neighbor_type == 'a':
        data.write("The active edges were picked using the " + active_type[flock.active_type] + " method\n")
        if flock.active_type == 'bc':
            data.write("Block size, " + str(flock.block_size) + '\n')
            data.write("Number of topological neighbors, " + str(flock.num_topological) + '\n')
    if flock.reflection_type == 'ca':
        data.write('Alpha center, ' + str(flock.alpha_center) + '\n')
        data.write('Alpha range, ' + str(flock.alpha_range) + '\n')
    elif flock.reflection_type == 'ta':
        data.write('Alpha center, ' + str(flock.alpha_center) + '\n')
        data.write('High, ' + str(flock.triangle_high) + '\n')
        data.write('Low, ' + str(flock.triangle_low) + '\n')


def order_parameter_out(data):
    data.write("Group size, " + str(flock.group_size) + '\n')
    data.write("Flock, Group 1, Group 2\n")
    for i in range(0, len(flock.group_two)):
        data.write(str(flock.flock_order_param[i]) + ", "
                   + str(flock.group_one[i]) + ", "
                   + str(flock.group_two[i]) + '\n')


def data_out(directory, func_args, time_started, local_time=None):
    local_time = time.localtime() if local_time is None else local_time  # Lets animation files have the same timestamp as their data files
    data = open(
        directory + "/data-" + str(local_time[3]) + "-" + str(local_time[4]) + "-" + str(local_time[5]) + '.csv', 'w')
    graph.calculate_scc_stats()
    finish = time.perf_counter()
    data.write("Runtime, " + str(finish - time_started) + '\n')
    data.write("Lowest scc count, " + str(graph.scc_min) + '\n')
    data.write("Highest scc count, " + str(graph.scc_max) + '\n')
    data.write("Average scc count, " + str(graph.scc_avg) + '\n')
    data.write("SCC median, " + str(graph.scc_median) + '\n')
    data.write("SCC standard deviation, " + str(graph.scc_std_dev) + '\n')
    data.write("SCC max median, " + str(graph.scc_high_median) + '\n')
    data.write("SCC avg median, " + str(graph.scc_med_mean) + '\n')
    neighbor_data_out(data, func_args)
    data.write("The number of boids was " + str(len(flock.boids)) + '\n')
    data.write("The number of flock mates was " + str(flock.num_neighbors) + '\n')
    if flock.frustration:
        data.write("The frustration power was " + str(flock.frustration_power) + '\n')
    else:
        data.write('Periodic boundary conditions were used\n')
    order_parameter_out(data)
    data.write("Number, Max, Min, Average, Median, Std Dev, Num Large SCC\n")
    for item in graph.scc_data:
        data.write(str(item)[1:-1] + '\n')
    data.write('\n')
    data.close()


"""
display_plot initializes the flock and graph, and plots the flock movements.
Can save the data to an mp4 file if desired, or can directly view the animation.
Some stuttering may be present when viewing directly due to neighbor recalculation
"""


def display_plot(num_boids, save, single, fps, frames, individual_bar, func_args):
    start = time.perf_counter()
    flock.reset()
    graph.reset()
    initialize_graph(num_boids, func_args)
    plt.axis([-8, 8, -8, 8])
    text = text_string()
    if save == 'y':
        date = datetime.date(2017, 1, 1)
        directory = base_directory + 'animation-' + str(date.today())
        if not os.path.exists(directory):
            os.makedirs(directory)
        if frames is None:
            frames = int(input("Length of animation (seconds): "))
        if fps is None:
            fps = int(input("FPS: "))
        bar = progressbar.ProgressBar(max_value=frames * fps) if individual_bar else None
        if individual_bar:
            bar.start()
        ani = animation.FuncAnimation(fig, plot_flock, frames=frames * fps, fargs=[bar, individual_bar, text]) if single == 'f'\
            else animation.FuncAnimation(fig, plot_single, frames=frames * fps, fargs=[bar, individual_bar, text])
        local_time = time.localtime()
        filename = directory + "/animation-" + str(local_time[3]) + "-" + str(local_time[4]) + "-" + str(local_time[5]) + '.mp4'
        ani.save(filename, fps=fps, bitrate=-1)
        data_out(directory, func_args, start, local_time)
    else:
        ani = animation.FuncAnimation(fig, plot_flock, frames=frames * fps, fargs=["", False, text])
        plt.show()


""" run_avg takes in a number of runs, the number of boids, and the number of iterations
to put each flock through. It then runs each flock for the desired amount of time, keeping track
of the average and total times as well as the scc metrics from the graph for each run. Also reports the
maximum number of sccs found between the runs.
"""


def run_avg(runs, num_boids, num_iterations, func_args=None):
    date = datetime.date(2017, 1, 1)
    directory = base_directory + str(num_boids) + 'data/' + str(date.today())
    if not os.path.exists(directory):
        os.makedirs(directory)
    local_time = time.localtime()
    data = open(directory + '/data' + str(local_time[3]) + "-" + str(local_time[4]) + "-" + str(local_time[5]) + '.csv', 'w')
    initialization_avg = 0
    update_avg = 0
    colors_avg = 0
    scc_avg = []
    scc_max = 0
    scc_median = []
    print("Starting runs")
    for i in range(0, runs):
        print("Starting run " + str(i))
        data.write(str(i) + "th run\n")
        start = time.perf_counter()
        initialize_graph(num_boids, func_args)
        data.write("Initialization " + str(i) + " done\n")
        initialization = time.perf_counter()
        update_flock(num_iterations)
        data.write("Update " + str(i) + " done\n")
        update_end = time.perf_counter()
        graph.calculate_scc()
        colors_end = time.perf_counter()
        initialization_avg += initialization - start
        update_avg += update_end - initialization
        colors_avg += colors_end - update_end
        graph.calculate_scc_stats()
        scc_avg.append(graph.scc_avg)
        scc_median.append(graph.scc_median)
        data.write("Lowest scc count, " + str(graph.scc_min) + '\n')
        data.write("Highest scc count, " + str(graph.scc_max) + '\n')
        data.write("Average scc count, " + str(graph.scc_avg) + '\n')
        scc_max = graph.scc_max if graph.scc_max > scc_max else scc_max
        data.write("SCC median, " + str(graph.scc_median) + '\n')
        data.write("SCC standard deviation, " + str(graph.scc_std_dev) + '\n')
        data.write("SCC max median, " + str(graph.scc_high_median) + '\n')
        data.write("SCC avg median, " + str(graph.scc_med_mean) + '\n')
        data.write("Number, Max, Min, Average, Median\n")
        for item in graph.scc_data:
            data.write(str(item)[1:-1] + '\n')
        data.write('\n')
        print("Resetting graph and flock...")
        graph.reset()
        flock.reset()
    print("Runs completed, writing final data...")
    initialization_avg /= runs
    colors_avg /= runs
    update_avg /= runs
    scc_avg.append(runs)
    data.write("\nOverall run data\n")
    data.write("Initialization average, " + str(initialization_avg) + '\n')
    data.write("Initialization total, " + str(initialization_avg * runs) + '\n')
    data.write("Update average, " + str(update_avg) + '\n')
    data.write("Update total, " + str(update_avg * runs) + '\n')
    data.write("SCC average, " + str(statistics.mean(scc_avg)) + ", " + str(colors_avg) + '\n')
    data.write("SCC totals, " + str(sum(scc_avg)) + ", " + str(colors_avg * runs) + '\n')
    data.write("Maximum number of final SCCS, " + str(scc_max) + '\n')
    data.write("Median average, " + str(statistics.mean(scc_median)) + '\n')
    data.write("Median median, " + str(statistics.median(scc_median)) + '\n')
    if runs > 1:
        data.write("Standard deviation of the median, " + str(statistics.stdev(scc_median)) + '\n')
    data.write("Mean of the standard deviation for each SCC, " + str(graph.scc_std_dev_mean) + '\n')
    data.write("Median of the standard deviation for each SCC, " + str(graph.scc_std_dev_median) + '\n')
    data.write("Total average time elapsed, " + str(initialization_avg + colors_avg + update_avg) + '\n')
    data.write("Total time elapsed, " + str(runs * (initialization_avg + colors_avg + update_avg)) + '\n')
    neighbor_data_out(data, func_args=func_args)
    data.write("The number of runs was " + str(runs) + '\n')
    data.write("The number of boids was " + str(num_boids) + '\n')
    data.write("The number of flock mates was " + str(flock.num_neighbors))
    data.write("The number of iterations was " + str(num_iterations) + '\n')
    data.write("The frustration power was " + str(flock.frustration_power))
    data.close()


def order_params_only(num_boids, num_iterations, func_args):
    time_started = time.perf_counter()
    flock.reset()
    graph.reset()
    initialize_graph(num_boids, func_args)
    update_flock(num_iterations)
    date = datetime.date(2017, 1, 1)
    directory = base_directory + 'order-' + str(date.today())
    if not os.path.exists(directory):
        os.makedirs(directory)
    data_out(directory, func_args, time_started)


def gen_data(num_boids, save, single, fps, frames, func_args, mode, num_iterations):
    range_min = flock.calculate_flock_mates if flock.neighbor_type == 's' or flock.neighbor_select == 't' or flock.neighbor_select == 'a' else 0
    vary_range = input("Vary the number of iterations to recalculate neighbors? (y/n) ") if flock.neighbor_type == 's' or flock.neighbor_select == 't' or flock.neighbor_select == 'a' else 'n'
    if vary_range == 'y':
        print("Minimum number of iterations for recalculating neighbors: " + str(flock.calculate_flock_mates))
    range_max = int(input("Maximum number of iterations for recalculating neighbors: ")) + 1 if vary_range == 'y' else range_min + 1
    range_step = int(input("Range step size: ")) if vary_range == 'y' else 1
    vary_neighbor = input("Vary the number of neighbors? (y/n) ")
    min_neighbor = flock.num_neighbors
    if vary_neighbor == 'y':
        print("Minimum number of neighbors: " + str(min_neighbor))
    max_neighbor = int(input("Maximum number of neighbors: ")) + 1 if vary_neighbor == 'y' else min_neighbor + 1
    neighbor_step = int(input("Neighbor step size: ")) if vary_neighbor == 'y' else 1
    min_pick = flock.nearest_num
    vary_pick = input("Vary the number of neighbors to pick from? (y/n) ") if flock.neighbor_select == 'n' or flock.neighbor_select == 't' or flock.neighbor_select == 'a' else 'n'
    if vary_pick == 'y':
        print("Minimum number of neighbors to pick from: " + str(min_pick))
    max_pick = int(input("Maximum number of neighbors to pick from: ")) + 1 if vary_pick == 'y' else min_pick + 1
    pick_step = int(input("Neighbors picked from step size: ")) if vary_pick == 'y' else 1
    vary_frustration = input("Vary the frustration power? (y/n) ") if flock.frustration else 'n'
    min_frustration = flock.frustration_power
    if vary_frustration == 'y':
        print("Minimum frustration power: " + str(min_frustration))
    max_frustration = int(input("Maximum frustration power: ")) + 1 if vary_frustration == 'y' else min_frustration + 1
    frustration_step = 1
    runs = int(input("Number of times: ")) if mode == 'd' else 1
    max_value = ((range_max - range_min) / range_step) * (max_frustration - min_frustration) * ((max_neighbor - min_neighbor) / neighbor_step) * ((max_pick - min_pick) / pick_step) * runs
    bar = progressbar.ProgressBar(max_value=int(max_value))
    i = 0
    bar.start()
    for num_recalc in range(range_min, range_max, range_step):
        for num_neighbor in range(min_neighbor, max_neighbor, neighbor_step):
            for num_pick in range(min_pick, max_pick, pick_step):
                for frustration in range(int(min_frustration), int(max_frustration), frustration_step):
                    bar.update(i)
                    flock.calculate_flock_mates = num_recalc if range_min != 0 else 'N/A'
                    flock.num_neighbors = num_neighbor
                    flock.nearest_num = num_pick
                    flock.frustration_power = frustration
                    if mode == 'p':
                        display_plot(num_boids, save, single, fps, frames, False, func_args)
                    elif mode == 'd':
                        run_avg(runs, num_boids, num_iterations, func_args=func_args)
                    elif mode == 'o':
                        order_params_only(num_boids, num_iterations, func_args)
                    i += 1


def start():
    sys.setrecursionlimit(sys.getrecursionlimit() * 10)  # Prevents python from having issues in calculating sccs for large flocks
    config = input("Input file? ")
    with open(config, 'r') as file:
        option = file.readline()[:-1].split(" ")[0]  # p for plot, d for data
        single = file.readline()[:-1].split(" ")[0] if option == "p" else ""  # plot the whole flock, f, or just a boid and flock mates, s
        flock.frustration = bool(file.readline()[:-1].split(" ")[0])  # frustration, put something, periodic if blank/empty
        if flock.frustration:
            # Reflection method:
            # Position u-turn (pu)
            # Velocity u-turn (vu)
            # specular reflection (s)
            # alpha reflection (a)
            # fixed alpha (f)
            # pseudo-specular (p)
            # alpha angular (an)
            # beta angular (bn)
            # dual angular (dn)
            # constrained alpha angular (ca)
            # triangular alpha angular (ta)
            flock.reflection_type = file.readline()[:-1].split(" ")[0]
            flock.alpha = int(file.readline()[:-1].split(" ")[0]) if flock.reflection_type == 'f' else 0  # alpha for fixed alpha reflection
        num_boids = int(file.readline()[:-1].split(" ")[0])  # number of boids
        flock.num_neighbors = int(file.readline()[:-1].split(" ")[0])  # number of flock mates
        if flock.frustration:
            flock.frustration_power = float(file.readline()[:-1].split(" ")[0])  # the frustration power
        flock.nearest_num = int(file.readline()[:-1].split(" ")[0])  # number of neighbors to pick flock mates from
        # Neighbor method
        # standard (s)
        # uniform square (u)
        # non-uniform square (n)
        # single sine wave (random initial phase) (ss)
        # multiple sine waves (ms)
        # negative exp (ne)
        # cutoff negative exp (ce)
        flock.neighbor_type = file.readline()[:-1].split(" ")[0]
        # Neighbor selection method
        # nearest neighbors (n)
        # random (r)
        # topological (t)
        flock.neighbor_select = file.readline()[:-1].split(" ")[0] if flock.neighbor_type != 'a' else 'a'
        flock.group_size = int(file.readline()[:-1].split(" ")[0])
        func_args = []
        if flock.neighbor_type == 'a':
            flock.active_type = file.readline()[:-1].split(" ")[0]
            if flock.active_type == 'bc':
                flock.block_size = int(file.readline()[:-1].split(" ")[0])
                flock.num_topological = int(file.readline()[:-1].split(" ")[0])
            elif flock.active_type == 'rn':
                flock.random_int = int(file.readline()[:-1].split(" ")[0])
        if flock.neighbor_type == 'u':
            func_args.append(int(file.readline()[:-1].split(" ")[0]))  # number of waves
            func_args.append(float(file.readline()[:-1].split(" ")[0]))  # maximum
            func_args.append(float(file.readline()[:-1].split(" ")[0]))  # minimum
            func_args.append(float(file.readline()[:-1].split(" ")[0]))  # period
        elif flock.neighbor_type == 'n':
            num_waves = int(file.readline()[:-1].split(" ")[0])  # number of waves
            for i in range(0, num_waves):
                wave = []
                wave.append(float(file.readline()[:-1].split(" ")[0]))  # maximum
                wave.append(float(file.readline()[:-1].split(" ")[0]))  # minimum
                wave.append(float(file.readline()[:-1].split(" ")[0]))  # period
                wave.append(float(file.readline()[:-1].split(" ")[0]))  # initial phase
                func_args.append(wave)
        elif flock.neighbor_type == 'ss':
            func_args.append([float(file.readline()[:-1].split(" ")[0])])  # amplitude
            func_args.append([float(file.readline()[:-1].split(" ")[0])])  # angular velocity
            func_args.append([float(file.readline()[:-1].split(" ")[0])])  # offset
        elif flock.neighbor_type == 'ms':
            num_waves = int(file.readline()[:-1].split(" ")[0])  # number of waves
            func_args = [[], [], []]
            for i in range(0, num_waves):
                func_args[0].append(float(file.readline()[:-1].split(" ")[0]))  # amplitude
                func_args[1].append(float(file.readline()[:-1].split(" ")[0]))  # angular velocity
                func_args[2].append(float(file.readline()[:-1].split(" ")[0]))  # offset
        elif flock.neighbor_type == 'ce':
            func_args = float(file.readline()[:-1].split(" ")[0])  # cutoff
        elif flock.neighbor_type == 'ie':
            func_args = float(file.readline()[:-1].split(" ")[0])  # period
        elif flock.neighbor_type == 'ri':
            func_args.append(int(file.readline()[:-1].split(" ")[0]))  # lower
            func_args.append(int(file.readline()[:-1].split(" ")[0]))  # upper
        if flock.reflection_type == 'ca':
            flock.alpha_center = float(file.readline()[:-1].split(" ")[0])  # center
            flock.alpha_range = float(file.readline()[:-1].split(" ")[0])  # vary by
        elif flock.reflection_type == 'ta':
            flock.alpha_center = float(file.readline()[:-1].split(" ")[0])  # center
            flock.triangle_high = float(file.readline()[:-1].split(" ")[0])  # high
            flock.triangle_low = float(file.readline()[:-1].split(" ")[0])  # low of the triangle distribution
        if option == 'p':
            save = file.readline()[:-1].split(" ")[0]  # whether to save the animation
            multi = file.readline()[:-1].split(" ")[0] if save == 'y' else ""  # whether to batch the data
            flock.calculate_flock_mates = int(file.readline()[:-1].split(" ")[0]) if \
                (flock.neighbor_type == 's' or flock.neighbor_type == 'a') \
                and flock.neighbor_select != 't' else 'N/A'  # neighbors aren't recalculated for topological neighbors
            if save == 'y' and multi == 'y':
                fps = int(file.readline()[:-1].split(" ")[0])  # how many fps to make the video
                frames = int(file.readline()[:-1].split(" ")[0])  # how long to make the video, in seconds
                gen_data(num_boids, save, single, fps, frames, func_args, option, 0)
            else:
                fps = int(file.readline()[:-1].split(" ")[0])  # how many fps to make the video
                frames = int(file.readline()[:-1].split(" ")[0])  # how long to make the video, in seconds
                display_plot(num_boids, save, single, fps, frames, True, func_args)
        else:
            flock.calculate_flock_mates = int(file.readline()[:-1].split(" ")[0]) if \
                (flock.neighbor_type == 's' or flock.neighbor_type == 'a') \
                and flock.neighbor_select != 't' else 'N/A'  # neighbors aren't recalculated for topological neighbors
            num_iterations = int(file.readline()[:-1].split(" ")[0])
            gen_data(num_boids, 'n', 'f', 0, 0, func_args, option, num_iterations)
    finish_time = time.perf_counter()
    print("\nRuntime: " + str(finish_time - start_time))

#  cProfile.run('start()')
start()