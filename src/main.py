import networkx as nx
import AStar
file_path = 'graph/inp.txt'  

graph = nx.DiGraph()

with open(file_path, 'r') as file:
    for line in file:
        parts = line.strip().split('\t')  
        if len(parts) == 3:
            start, end, weight = parts[0], parts[1], int(parts[2])
            graph.add_edge(start, end, weight=weight)

shortest_path,cost = AStar.a_star(graph, 'A', 'H')
print("Đường đi ngắn nhất từ A đến H:", shortest_path,cost)