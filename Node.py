from Edge import Edge


class Node:

    def __init__(self, boid, id):
        self.boid = boid
        self.id = id
        self.edges = []
        self.reverse = []
        self.visited = False
        self.boid.node = self

    def __str__(self):
        to_return = str(self.id) + " Edges: "
        for edge in self.edges:
            to_return += str(edge.go_to().id) + " "
        return to_return

    def add_edge(self, edge):
        if edge not in self.edges:
            self.edges.append(edge)
            edge.go_to().reverse.append(Edge(self, edge.function))

    def reset_edges(self):
        self.edges = []
        self.reverse = []


class ActiveNode(Node):
    def __init__(self, boid, id):
        Node.__init__(self, boid, id)
        self.active = []

    def reset_edges(self):
        Node.reset_edges(self)
        self.clear_active()

    def add_active(self, edge):
        if edge not in self.active:
            self.active.append(edge)
            edge.go_to().reverse[self.id - 1].value = 1

    def clear_active(self):
        for edge in self.active:
            edge.go_to().reverse[self.id - 1].value = 0
        self.active = []

    def toggle_active(self):
        for edge in self.active:
            edge.toggle()