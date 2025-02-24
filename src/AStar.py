import networkx as nx
import heapq
import matplotlib.pyplot as plt

def heuristic(node, goal):
    heuristic_values = {
        'A': 5,
        'B': 4,
        'C': 4,
        'D': 3,
        'E': 2,
        'F': 3,
        'G': 1,
        'H': 0
    }
    return heuristic_values.get(node, float('inf'))


def a_star(graph, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    g_scores = {node: float('inf') for node in graph.nodes}
    g_scores[start] = 0
    came_from = {}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            total_cost = g_scores[goal]
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], total_cost

        for neighbor in graph.neighbors(current):
            weight = graph.edges[current, neighbor].get('weight', 1)
            tentative_g = g_scores[current] + weight
            if tentative_g < g_scores[neighbor]:
                came_from[neighbor] = current
                g_scores[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score, neighbor))

    return None, float('inf')




