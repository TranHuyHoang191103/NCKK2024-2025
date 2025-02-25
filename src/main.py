import networkx as nx
import AStar
import DStar
import IDAstar
import BidirectionAStar
file_path = 'graph/inp.txt'  

graph = nx.DiGraph()
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
with open(file_path, 'r') as file:
    for line in file:
        parts = line.strip().split('\t')  
        if len(parts) == 3:
            start, end, weight = parts[0], parts[1], int(parts[2])
            graph.add_edge(start, end, weight=weight)

cost,meeting = BidirectionAStar.bidirectional_a_star(graph, 'A', 'H')
print("Đường đi ngắn nhất từ A đến H:",cost,meeting)
print(nx.astar_path(graph,'A','H',heuristic=heuristic))
print(IDAstar.ida_star('A','H',graph,IDAstar.heuristic))
print(DStar.d_star(graph,'A','H'))