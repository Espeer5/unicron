"""
This is the graphics module for displaying a track map explored by
the robot for ME/CS/EE 129 Spring '23. Maps are displayed as plots
of graphs using Matplotlib.

Authors: Edward Speer, Garrett Knuf
Date: 5/2
"""

import matplotlib.pyplot as plt
from mapping.MapGraph import MapGraph, Intersection

class Visualizer:
    """
    The vizualizer object takes in a MapGraph object and displays
    a representation of it using Matplotlib

    Inputs: graph: A MapGraph object
    """

    def __init__(self, graph):
        self.graph = graph

    def find_intersections(self):
        """
        Forms arrays of the x and y values of each intersection
        in order to be input into matplotlib plotting functions
        """
        x = []
        y = []
        intersection_explored = []
        for intersection in self.graph:
            coords = intersection.get_location()
            x.append(-coords[0])
            y.append(coords[1])
            intersection_explored.append(intersection.is_explored())
        return (x, y, intersection_explored)

    def find_edges(self):
        """
        Forms arrays of x and y values representing the streets
        in the mapGraph in order to be plotted
        """
        x_edges = []
        y_edges = []
        for intersection in self.graph:
            start = intersection.get_location()
            for conn in self.graph.get_graph()[intersection]:
                conn_loc = conn.get_location()
                x_edges.append([-start[0], -conn_loc[0]])
                y_edges.append([start[1], conn_loc[1]])
        return (x_edges, y_edges)

    def show(self):
        """
        Produces a matplotlib plot of the current state 
        of the MapGraph object input into the Visualizer.
        Fully explored intersections are shown in green,
        with all others shown in blue.
        """
        x, y, inter_exp = self.find_intersections()
        x_edges, y_edges = self.find_edges()
        for i in range(len(x_edges)):
            plt.plot(x_edges[i], y_edges[i], 'r')
        for i in range(len(x)):
            if inter_exp[i]:
                plt.plot(x[i], y[i], 'go')
            else:
                plt.plot(x[i], y[i], 'bo')
        plt.title("Normstorm Map")
        plt.show()

    def create_path(self, start, path):
        heading_map = {0:(0, 1), 1:(1, 1), 2:(1, 0), 3:(1, -1),
                4:(0, -1), 5:(-1, -1), 6:(-1, 0), 7:(-1, 1)}
        pth_x = [start[0]]
        pth_y = [start[1]]
        pth_l_x = []
        pth_l_y = []
        for i in range(len(path)):
            pth_x.append(pth_x[len(pth_x) - 1] + heading_map[path[i]][0])
            pth_y.append(pth_y[len(pth_y) - 1] + heading_map[path[i]][1])
            pth_l_x.append((-pth_x[i - 1], -pth_x[i]))
            pth_l_y.append((pth_y[i - 1], pth_y[i]))
        i = len(path)
        pth_l_x.append((-pth_x[i - 1], -pth_x[i]))
        pth_l_y.append((pth_y[i - 1], pth_y[i]))
        return (pth_x, pth_y, pth_l_x, pth_l_y)

    def show_path(self, start, path):
        x, y, inter_exp = self.find_intersections()
        x_edges, y_edges = self.find_edges()
        for i in range(len(x_edges)):
            plt.plot(x_edges[i], y_edges[i], 'r')
        for i in range(len(x)):
            if inter_exp[i]:
                plt.plot(x[i], y[i], 'go')
            else:
                plt.plot(x[i], y[i], 'ko')
        pth_x, pth_y, l_x, l_y = self.create_path(start, path)
        for i in range(len(pth_x)):
            plt.plot(-pth_x[i], pth_y[i], 'bo')
        for i in range(len(l_x)):
            plt.plot(l_x[i], l_y[i], 'b')
        plt.title("Normstorm Map")
        plt.show()

    def exit(self):
        """
        Closes out the graphics window opened by visualizer.show()
        """
        plt.close()
        
