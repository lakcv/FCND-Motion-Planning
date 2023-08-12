from enum import Enum
from queue import PriorityQueue
import numpy as np

from scipy.spatial import Voronoi
from bresenham import bresenham

import networkx as nx
import numpy.linalg as LA



def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Initialize an empty list for Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]

        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges, int(north_min), int(east_min)


def prepare_graph(file_name_str, TARGET_ALTITUDE, SAFETY_DISTANCE):
    # Read in obstacle map
    data = np.loadtxt(file_name_str, delimiter=',', dtype='Float64', skiprows=2)

    # I'll use a graph method
    grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

    # Create a graph with the weight of the edges
    # set to the Euclidean distance between the points
    w_edges = [(p1, p2, LA.norm(np.array(p2) - np.array(p1))) for p1, p2 in edges]
    G = nx.Graph()
    G.add_weighted_edges_from(w_edges)

    return data, grid, G, north_offset, east_offset

def a_star_graph(G, h, start_n, goal_n):
    paths = []
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start_n))
    visited = set(start_n)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start_n:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal_n:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in G[current_node]:
                w = G[current_node][next_node]['weight']
                # get the tuple repre.values())[0]['weight']sentation
                branch_cost = current_cost + G[current_node][next_node]['weight']
                queue_cost = branch_cost + h(next_node, goal_n)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, next_node)
                    queue.put((queue_cost, next_node))
                else:
                    if branch_cost < branch[next_node][0]:
                        branch[next_node] = (branch_cost, current_node, next_node)
                        queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal_n
        path_cost = branch[n][0]
        path.append(goal_n)
        while branch[n][1] != start_n:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])

    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


def find_start_goal_in_graph(G, start, goal):
    # TODO: find start and goal on skeleton

    idx = np.argmin([((start[0] - x) ** 2 + (start[1] - y) ** 2) for x, y in G.nodes])
    node_start = list(G.nodes)[idx]

    idx = np.argmin([((goal[0] - x) ** 2 + (goal[1] - y) ** 2) for x, y in G.nodes])
    node_goal = list(G.nodes)[idx]
    return node_start, node_goal





def path_pruning(grid, waypoints, north_offset, east_offset):
    if len(waypoints) < 3:
        return waypoints

    waypoints_rest = waypoints[1:]
    remain_waypoints = []
    for p in waypoints_rest[-1:0:-1]:
        remain_waypoints.insert(0, p)
        if not is_hit(grid, waypoints[0], p,north_offset ,east_offset):
            pruned_path = [waypoints[0]] + path_pruning(grid, remain_waypoints, north_offset, east_offset)
            return pruned_path
    pruned_path = [waypoints[0]] + path_pruning(grid, waypoints[1:], north_offset, east_offset)
    return pruned_path


def is_hit(grid, p1, p2, north_offset ,east_offset):
    cells = list(bresenham(int(p1[0]-north_offset), int(p1[1]-east_offset), int(p2[0]-north_offset), int(p2[1]-east_offset)))
    hit = False

    for c in cells:
        # First check if we're off the map
        if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
            hit = True
            break
        # Next check if we're in collision
        if grid[c[0], c[1]] == 1:
            hit = True
            break

    return hit
