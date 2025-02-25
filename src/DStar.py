import heapq
from collections import defaultdict

def d_star(graph, start_node, goal_node, obstacle_updates=None):
    """
    Run D* Lite algorithm on a NetworkX graph.
    
    Parameters:
    - graph: NetworkX DiGraph object with weighted edges
    - start_node: Starting node
    - goal_node: Goal node
    - obstacle_updates: List of tuples (from_node, to_node, new_cost) representing discovered obstacles
    
    Returns:
    - Tuple containing (shortest_path, total_cost)
    """
    # Initialize the D* Lite algorithm
    dstar = DStarLite()
    
    # Convert NetworkX graph to our internal representation
    for u, v, data in graph.edges(data=True):
        dstar.add_edge(u, v, data['weight'])
    
    # Initialize and plan
    dstar.initialize(start_node, goal_node)
    path = dstar.replan()
    
    # Process obstacle updates if provided
    if obstacle_updates:
        for from_node, to_node, new_cost in obstacle_updates:
            dstar.current = from_node  # Simulate being at the node where the obstacle is discovered
            path = dstar.update_edge_cost(from_node, to_node, new_cost)
    
    # Calculate total cost
    total_cost = 0
    if path:
        for i in range(len(path) - 1):
            total_cost += dstar.graph[path[i]][path[i+1]]
    
    return path, total_cost

class DStarLite:
    def __init__(self):
        self.graph = defaultdict(dict)
        self.start = None
        self.goal = None
        self.km = 0  # Accumulated cost offset
        self.U = []  # Priority queue
        self.rhs = defaultdict(lambda: float('inf'))
        self.g = defaultdict(lambda: float('inf'))
        self.current = None  # Current position of the agent
        
    def add_edge(self, from_node, to_node, cost):
        """Add an edge to the graph."""
        self.graph[from_node][to_node] = cost
    
    def h(self, s, s_goal):
        """Heuristic function (simple straight-line distance approximation)."""
        # In a real-world scenario, you might use Euclidean distance
        # Since we're using a graph with no coordinates, we'll use a simple heuristic
        return 0  # Consistent heuristic, turns this into Dijkstra's algorithm
    
    def calculate_key(self, s):
        """Calculate the key for a vertex."""
        if self.g[s] > self.rhs[s]:
            return (self.rhs[s] + self.h(s, self.goal) + self.km, self.rhs[s])
        else:
            return (self.g[s] + self.h(s, self.goal) + self.km, self.g[s])
    
    def update_vertex(self, u):
        """Update vertex in the priority queue."""
        if u != self.goal:
            # Calculate minimum rhs value from successors
            self.rhs[u] = float('inf')
            for successor, cost in self.graph[u].items():
                self.rhs[u] = min(self.rhs[u], cost + self.g[successor])
        
        # Remove u from the priority queue if it exists
        for i, (_, vertex) in enumerate(self.U):
            if vertex == u:
                self.U.pop(i)
                heapq.heapify(self.U)
                break
        
        # If g and rhs are inconsistent, add u to the priority queue
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self.calculate_key(u), u))
    
    def compute_shortest_path(self):
        """Compute the shortest path from start to goal."""
        while (len(self.U) > 0 and 
              (self.U[0][0] < self.calculate_key(self.start) or 
               self.rhs[self.start] != self.g[self.start])):
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u)
            
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for predecessor in self.get_predecessors(u):
                    self.update_vertex(predecessor)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for predecessor in self.get_predecessors(u):
                    self.update_vertex(predecessor)
    
    def get_predecessors(self, node):
        """Get all predecessors of a node."""
        predecessors = []
        for potential_pred in self.graph:
            if node in self.graph[potential_pred]:
                predecessors.append(potential_pred)
        return predecessors
    
    def initialize(self, start, goal):
        """Initialize the D* Lite algorithm."""
        self.start = start
        self.goal = goal
        self.current = start  # Initialize current position to start
        self.km = 0
        self.U = []
        self.g = defaultdict(lambda: float('inf'))
        self.rhs = defaultdict(lambda: float('inf'))
        self.rhs[goal] = 0
        heapq.heappush(self.U, (self.calculate_key(goal), goal))
    
    def replan(self):
        """Replan the path if needed."""
        self.compute_shortest_path()
        return self.extract_path()
    
    def extract_path(self):
        """Extract the path from start to goal."""
        if self.g[self.start] == float('inf'):
            return None  # No path exists
        
        path = [self.start]
        current = self.start
        
        while current != self.goal:
            min_cost = float('inf')
            next_node = None
            
            for successor, cost in self.graph[current].items():
                if cost + self.g[successor] < min_cost:
                    min_cost = cost + self.g[successor]
                    next_node = successor
            
            if next_node is None:
                return None  # No path exists
            
            path.append(next_node)
            current = next_node
        
        return path
    
    def update_edge_cost(self, from_node, to_node, new_cost):
        """Update the cost of an edge."""
        self.km += self.h(self.start, self.goal)
        self.start = self.current  # Update start to current position
        
        # Update the edge cost
        old_cost = self.graph[from_node][to_node]
        self.graph[from_node][to_node] = new_cost
        
        # Update affected vertices
        self.update_vertex(from_node)
        self.update_vertex(to_node)
        
        # Recompute shortest path
<<<<<<< HEAD
        return self.replan()
=======
        return self.replan()
>>>>>>> 6816a064c91d931900e326faf7983f1277fd2c0f
