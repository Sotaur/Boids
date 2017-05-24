class Edge:
    """
        node is a reference to the node that the edge points to. function is a Function object
        or subclass of Function that is used to hold the value
    """
    def __init__(self, node, function):
        self.to = node
        self.function = function
        self.value = self.function.update(0)

    def go_to(self):
        return self.to

    def update(self, time):
        self.value = self.function.update(time)
        return self.value

    def __str__(self):
        return str(str(self.value) + " " + str(self.to.id))


class ActiveEdge(Edge):
    """
    Similar to an edge but has a toggle rather than update, to switch between 2 given values.
    on is a boolean that determines if the edge is on, and will pass that through to the assigned function.
    Functions must be a subclass of ActiveFunction
    """
    def __init__(self, node, function):
        self.on = False
        function.toggle(self.on)
        Edge.__init__(self, node, function)

    def toggle(self):
        self.on = not self.on
        self.value = self.function.toggle(self.on)