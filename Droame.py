import pygame
import heapq
import csv

# Define the grid and drone objects
GRID_WIDTH = 20
GRID_HEIGHT = 20
DRONE_SIZE = 11

class Drone:
    def __init__(self, start, end, start_time):
        self.start = start
        self.end = end
        self.start_time = start_time
        self.path = []

    def __lt__(self, other):
        return self.start_time < other.start_time

# Create a 2D grid
grid = [[0 for x in range(GRID_WIDTH)] for y in range(GRID_HEIGHT)]

# Define the A* search algorithm
def astar(start, end, drones, start_time):
    # Define the heuristic function (Manhattan distance)
    def heuristic(node):
        return abs(node[0] - end[0]) + abs(node[1] - end[1])

    # Define the cost function
    def cost(node, time):
        # Add a penalty for each node that is occupied by another drone at that time step
        for drone in drones:
            if drone != None and drone.start_time <= time and node == drone.path[time - drone.start_time]:
                return float('inf')
        return 1

    # Define the neighbors function
    def neighbors(node):
        neighbors = []
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                # Skip the current node
                if dx == 0 and dy == 0:
                    continue
                x = node[0] + dx
                y = node[1] + dy
                # Check if the neighbor is within the grid
                if x >= 0 and x < GRID_WIDTH and y >= 0 and y < GRID_HEIGHT:
                    neighbors.append((x, y))
        return neighbors

    # Initialize the start and end nodes
    start_node = start
    end_node = end

    # Initialize the open and closed sets
    open_set = []
    closed_set = set()

    # Add the start node to the open set
    heapq.heappush(open_set, (heuristic(start_node), 0, start_node))

    # Initialize the came_from and g_score dictionaries
    came_from = {}
    g_score = {}
    came_from[start_node] = None
    g_score[start_node] = 0

    # Run the A* search algorithm
    while len(open_set) > 0:
        # Pop the node with the lowest f-score from the open set
        f_score, time, current_node = heapq.heappop(open_set)

        # Check if the
