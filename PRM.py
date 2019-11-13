import numpy as np
from tqdm import tqdm
import math
import random
from scipy.spatial import distance
from Graph import Node, Graph

# This Function takes 2 nodes as arguments and calculates if there is a
# collision free path between the first node and the second node
def collision_free(world, node1, node2):
    delta = 3

    x0 = node1.coordinates[0]
    y0 = node1.coordinates[1]
    z0 = node1.coordinates[2]

    x1 = node2.coordinates[0]
    y1 = node2.coordinates[1]
    z1 = node2.coordinates[2]

    v = [x1-x0,y1-y0,z1-z0]
    magnitude = math.sqrt(math.pow(v[0],2) + math.pow(v[1],2) + math.pow(v[2],2))

    u = [v[0]/magnitude, v[1]/magnitude, v[2]/magnitude]
    Px = [x0 + (u[0]*delta), y0 + (u[1]*delta), z0 + (u[2]*delta)]

    # This loop checks if points along the line touch an obstacle
    while get_dist(node1.coordinates, Px) < get_dist(node1.coordinates, node2.coordinates):
        if world[math.ceil(Px[0]), math.ceil(Px[1]), math.ceil(Px[2])] == -1:
            return False
        delta = delta*2
        Px = [x0 + (u[0]*delta), y0 + (u[1]*delta), z0 + (u[2]*delta)]

    return True

# Returns the distance between 2 nodes
def get_dist(node1, node2):
    return distance.euclidean(node1, node2)

# This function is used while reading in the configuration space
def format_array(array):
    new_array = []
    for x in array.split():
        new_array.append(int(x))
    return new_array

# This function is used while creating the spheres in the configuration space
def sphere_distance(x, y, z, x_center, y_center, z_center):
    x1 = math.pow((x - x_center), 2)
    y1 = math.pow((y - y_center), 2)
    z1 = math.pow((z - z_center), 2)
    return (x1 + y1 + z1)

# This function adds a cube to the world space
def add_cube(world, x, y, z, x_center, y_center, z_center, length):
    j = 0
    k = 0
    for i in range(x):
        for j in range(y):
            for k in range(z):
                if i >= abs(x_center - math.ceil((length/2))) and i <= abs(x_center + math.ceil((length/2))):
                    if j >= abs(y_center - math.ceil((length/2))) and j <= abs(y_center + math.ceil((length/2))):
                        if k >= abs(z_center - math.ceil((length/2))) and k <= abs(z_center + math.ceil((length/2))):
                            world[i,j,k] = -1

# This function adds a sphere to the configuration space
def add_sphere(world, x, y, z, x_center, y_center, z_center, radius):
    j = 0
    k = 0
    for i in range(x):
        for j in range(y):
            for k in range(z):
                if sphere_distance(i,j,k,x_center,y_center,z_center) <= math.pow(radius, 2):
                    world[i,j,k] = -1

# This function reads in data from a file and creates the world space with
# obstacles
def create_world_space(file):

    global start
    global goal
    global x_upper
    global y_upper
    global z_upper

    # All data from the file are read in and stored in the proper variable array
    with open(file, 'r') as f:
        data = f.readlines()

        x_axis = format_array(data[0])
        y_axis = format_array(data[1])
        z_axis = format_array(data[2])
        cube1 = format_array(data[3])
        cube2 = format_array(data[4])
        sphere1 = format_array(data[5])
        sphere2 = format_array(data[6])
        start = format_array(data[7])
        goal = format_array(data[8])

    # These are used to keep track of the upper bounds on the axises
    x_upper = x_axis[1] - x_axis[0] + 1
    y_upper = y_axis[1] - y_axis[0] + 1
    z_upper = z_axis[1] - z_axis[0] + 1

    # A world space is created using a 3D numpy array full of 0s
    world = np.zeros([x_upper,y_upper,z_upper],dtype=np.int)

    # Cubes are added to the world space as -1
    add_cube(world,x_upper,y_upper,z_upper,cube1[0],cube1[1],cube1[2],cube1[3]+1)
    add_cube(world,x_upper,y_upper,z_upper,cube2[0],cube2[1],cube2[2],cube2[3]+1)

    # Spheres are added to the world space as -1
    add_sphere(world,x_upper,y_upper,z_upper,sphere1[0],sphere1[1],sphere1[2],sphere1[3]+1)
    add_sphere(world,x_upper,y_upper,z_upper,sphere2[0],sphere2[1],sphere2[2],sphere2[3]+1)

    # Initializing the goal node in the space as 2
    world[goal[0],goal[1],goal[2]] = 2

    return world

# PRM samples random points in the world space, connects them in a graph, and
# then uses dijkstra's algorithm to find the shortest path from start to goal
def PRM(world):

    # Number of nodes
    N = 2000
    # Number of neighbors per node
    k = 5
    # Maximum distance for a neighbor
    d = 15

    # The graph is created here and the start and goal nodes are created
    Roadmap = Graph()
    start_node = Node(start)
    goal_node = Node(goal)

    # Adding start and goal nodes to the graph
    Roadmap.add_node(start_node)
    Roadmap.add_node(goal_node)

    # Samples the world space for N nodes
    while len(Roadmap.nodes) != N:

        # Sampling random points on the graph
        x = random.randint(0, x_upper - 1)
        y = random.randint(0, y_upper - 1)
        z = random.randint(0, z_upper - 1)

        # Adding sampled points to the graph
        if world[x,y,z] != -1 and [x,y,z] not in Roadmap.nodes:
            Roadmap.add_node(Node([x,y,z]))

    # Links all the nodes in the graph
    print("\nCreating and Linking all Nodes:")
    for node in tqdm(Roadmap.nodes):
        neighbors = Roadmap.get_neighbors(node, k, d)
        for neighbor in neighbors:
            if collision_free(world, node, neighbor):
                if Roadmap.edge_exists(node, neighbor) == False:
                    Roadmap.add_edge(node, neighbor)

    # Here dijkstras algorithm is called on the graph returning a stack holding
    # the path from the start to the goal node
    print("\nFinding Shortest Path From Start to Goal....")
    return Roadmap.dijkstra(start_node, goal_node)

# The user is prompted to enter the name of the file to read data from
file_name = input("Please enter the file name you would like to create the world from: ")

# The world space is created here
world_space = create_world_space(file_name)

# Here PRM is called on the graph and path is returned and output to a file
path = PRM(world_space)

# An output file is created and the data is written to it
with open('PRM_output.txt', 'w') as f:
    while path:
        node = path.pop()
        f.write('%d,%d,%d\n' % (node.coordinates[0],node.coordinates[1],node.coordinates[2]))
print("\nA path has been written to the file: PRM_output.txt")
