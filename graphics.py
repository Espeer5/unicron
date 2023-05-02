import matplotlib.pyplot as plt
from mapGraph import MapGraph, Intersection

class Visualizer:

    def __init__(self, graph, streets):
        self.graph = graph
        self.streets = streets
        self.x = []
        self.y = []
        self.x_edges = []
        self.y_edges = []

    def find_intersections(self):
        for intersection in self.graph:
            coords = intersection.get_location()
            self.x.append(intersection[0])
            self.y.append(intersection[1])

    def find_edges(self):
        for street in self.streets:
            self.x_edges.append(street[0][0], street[1][0])
            self.y_edges.append(street[0][1], street[1][1])      

    def show(self):
        self.find_intersections()
        self.find_edges()
        plt.plot(self.x, self.y, 'go')
        for i in range(len(self.x_edges)):
            plt.plot([self.x_edges[i], self.y_edges[i])
        plt.title("Normstorm Map")
        plt.show()

    def exit(self):
        plt.close()
        

if __name__ == "__main__":

    tool = Visualizer(graph)
    tool.show()
