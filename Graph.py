from scipy.spatial import distance
import time

# This class creates a node used when sampling random points
class Node:
    def __init__(self, _coordinates):
        self.coordinates = _coordinates
        self.connected = []
        self.parent = None

    # This overwrites the equivalance operator for nodes comparing the coordinates
    def __eq__(self, other):
        return self.coordinates == other

    # This is a hash operator that is used by a dictionary when implementing
    # dijkstras algorithm
    def __hash__(self):
        return hash((self.coordinates[0], self.coordinates[1], self.coordinates[2]))


# This class creates a graph that is used to store the nodes and used to traverse
# the world space from the start point to the end point
class Graph:
    def __init__(self):
        self.nodes = []
        self.num_of_nodes = 0
        self.parent = None

    # This function adds a node to the graph while incrementing the number of nodes
    # in the graph by 1
    def add_node(self, _node):
        _node.ID = self.num_of_nodes
        self.num_of_nodes += 1
        self.nodes.append(_node)

    # This function returns true if there is already an edge between the two
    # nodes that are passed to it
    def edge_exists(self, node1, node2):
        if node1 in node2.connected and node2 in node1.connected:
            return True
        else:
            return False

    # This function adds an edge between 2 nodes in the graph
    def add_edge(self, node1, node2):
        node1.connected.append(node2)
        node2.connected.append(node1)

    # This function returns the distance between 2 points in the graph
    def get_distance(self, node1, node2):
        return distance.euclidean(node1, node2)

    # This function returns an array of neighboring nodes that are within a
    # specified distance from the given node.
    def get_neighbors(self, current_node, num_neighbors, distance):
        neighbors = []

        # If there are less than the number of neighbors passed to the function
        # then the function will timeout and return the neighbors it has found
        timeout = 5 # seconds
        start_time = time.time()
        while len(neighbors) < num_neighbors and time.time() <= (start_time + timeout):
            for neighbor in self.nodes:
                if neighbor != current_node:
                    if self.get_distance(current_node.coordinates, neighbor.coordinates) <= distance:
                        neighbors.append(neighbor)
        return neighbors

    # This is dijkstras algorithm for finding the shortest path in a graph
    def dijkstra(self, start, goal):
        fringe = {}

        for node in self.nodes:
            fringe[node] = 1000000
        fringe[start] = 0

        while not fringe == False:
            node = min(fringe, key=fringe.get)

            if node == goal:
                break

            for neighbor in node.connected:
                alt = fringe[node] + self.get_distance(node.coordinates, neighbor.coordinates)
                if fringe.get(neighbor) != None:
                    if alt < fringe[neighbor]:
                        fringe[neighbor] = alt
                        neighbor.parent = node

            del fringe[node]

        path_stack = []

        while node != start:
            path_stack.append(node)
            node = node.parent

        path_stack.append(start)

        return path_stack
