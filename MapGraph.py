
class Intersection:

    CONDITIONS = ["UNKNOWN", "UNDRIVEN", "NONE"]

    def __init__(self, curr_heading):
        self.streets = dict.fromkeys(range(7), "UNKNOWN")
        self.streets[(curr_heading + 4) % 8] = "DRIVEN"
    
    def set_connection(self, heading, status):
        if status not in self.CONDITIONS:
            raise Exception("Intersection.set_connection: Invalid status")

class MapGraph:

    def __init__(self):
        self.graph = {}

    def driven_connection(self, location, heading):
        if curr in self.graph:
            self.graph[curr].set_connection(heading, "DRIVEN")
        else:
            self.graph[curr] = Intersection(heading)

    def no_connection(self, location, heading):
        self.graph[curr].set_connection(heading, "NONE")

    def contains(self, intersection):
        return intersection in self.graph
