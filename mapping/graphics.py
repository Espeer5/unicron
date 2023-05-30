"""
This is the graphics module for displaying a track map explored by
the robot for ME/CS/EE 129 Spring '23. Maps are displayed as plots
of graphs using Matplotlib.

Authors: Edward Speer, Garrett Knuf
Date: 5/2
"""

import matplotlib.pyplot as plt
from constants import heading_map, BLK, invert_h_map

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
            x.append(coords[0])
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
        blockages = []
        for intersection in self.graph:
            start = intersection.get_location()
            for conn in self.graph.get_graph()[intersection]:
                conn_loc = conn.get_location()
                x_edges.append([start[0], conn_loc[0]])
                y_edges.append([start[1], conn_loc[1]])
                inters1 = self.graph.get_intersection((start[0], start[1]))
                inters2 = self.graph.get_intersection((conn_loc[0], conn_loc[1]))
                loc2 = inters1.get_location()
                loc1 = inters2.get_location()
                relative_loc = (loc2[0] - loc1[0], loc2[1] - loc1[1])
                heading = invert_h_map[relative_loc]
                if inters2.check_blockage(heading) == BLK:
                    blockages.append(True)
                else:
                    blockages.append(False)
        return (x_edges, y_edges, blockages)

    def show(self):
        """
        Produces a matplotlib plot of the current state 
        of the MapGraph object input into the Visualizer.
        Fully explored intersections are shown in green,
        with all others shown in blue.
        """
        x, y, inter_exp = self.find_intersections()
        x_edges, y_edges, blockages = self.find_edges()
        for i in range(len(x_edges)):
            if blockages[i]:
                plt.plot(x_edges[i], y_edges[i], 'y')
            else:
                plt.plot(x_edges[i], y_edges[i], 'r')
        for i in range(len(x)):
            if inter_exp[i]:
                plt.plot(x[i], y[i], 'go')
            else:
                plt.plot(x[i], y[i], 'bo')
        plt.title("Normstorm Map")
        plt.savefig("map.png")
        #plt.show()

    def create_path(self, start, path):
        """ Takes in a path planned by a Djikstra object and generates the data 
        needed to overlay a plot of the path on the Visualizer graph

        Inputs: start - the location of the beginning of the path
                path - the path generated from Djikstra
        """
        pth_x = [start[0]]
        pth_y = [start[1]]
        pth_l_x = []
        pth_l_y = []
        for i in range(len(path)):
            pth_x.append(pth_x[len(pth_x) - 1] + heading_map[path[i]][0])
            pth_y.append(pth_y[len(pth_y) - 1] + heading_map[path[i]][1])
            pth_l_x.append((pth_x[i - 1], pth_x[i]))
            pth_l_y.append((pth_y[i - 1], pth_y[i]))
        i = len(path)
        pth_l_x.append((pth_x[i - 1], pth_x[i]))
        pth_l_y.append((pth_y[i - 1], pth_y[i]))
        return (pth_x, pth_y, pth_l_x, pth_l_y)

    def show_path(self, start, path):
        """ Show the plot of the graph in the vizualizer, overlayed with a 
        path planned by a Djikstra object

        Arguments: start - the starting location of the path
                   path - the path generated by the Djikstra object
        """
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
            plt.plot(pth_x[i], pth_y[i], 'bo')
        for i in range(len(l_x)):
            plt.plot(l_x[i], l_y[i], 'b')
        plt.title("Normstorm Map")
        plt.show()

    def exit(self):
        """
        Closes out the graphics window opened by visualizer.show()
        """
        plt.close()
        
