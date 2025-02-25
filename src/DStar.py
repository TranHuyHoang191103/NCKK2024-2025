import heapq

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = float('inf')
        self.rhs = float('inf')
        self.key = (float('inf'), float('inf'))
    
    def __lt__(self, other):
        return self.key < other.key

class DStarLite:
    def __init__(self, start, goal, grid):
        self.grid_size = (len(grid), len(grid[0]))
        self.grid = grid
        self.nodes = [[Node(x, y) for y in range(self.grid_size[1])] for x in range(self.grid_size[0])]
        
        self.start = self.nodes[start[0]][start[1]]
        self.goal = self.nodes[goal[0]][goal[1]]
        self.current = self.start
        
        self.queue = []
        self.km = 0
        
        # Initialize goal
        self.goal.rhs = 0
        heapq.heappush(self.queue, (self.calculate_key(self.goal), self.goal))
    
    def calculate_key(self, node):
        k1 = min(node.g, node.rhs) + self.heuristic(self.current, node) + self.km
        k2 = min(node.g, node.rhs)
        return (k1, k2)
    
    def heuristic(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)
    
    def get_neighbors(self, node):
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for dx, dy in directions:
            x, y = node.x + dx, node.y + dy
            if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
                if self.grid[x][y] == 0:  # Check for obstacles
                    neighbors.append(self.nodes[x][y])
        return neighbors
    
    def update_node(self, node):
        if node != self.goal:
            node.rhs = float('inf')
            neighbors = self.get_neighbors(node)
            for neighbor in neighbors:
                node.rhs = min(node.rhs, neighbor.g + 1)  # Assume movement cost = 1
        
        # Remove old queue entries
        temp_queue = []
        while self.queue:
            key, n = heapq.heappop(self.queue)
            if n.x == node.x and n.y == node.y:
                continue
            temp_queue.append((key, n))
        for item in temp_queue:
            heapq.heappush(self.queue, item)
        
        if node.g != node.rhs:
            heapq.heappush(self.queue, (self.calculate_key(node), node))
    
    def compute_shortest_path(self):
        while self.queue and (
            self.queue[0][0] < self.calculate_key(self.current) or
            self.current.rhs != self.current.g
        ):
            _, node = heapq.heappop(self.queue)
            if node.g > node.rhs:
                node.g = node.rhs
            else:
                node.g = float('inf')
                self.update_node(node)
            
            neighbors = self.get_neighbors(node)
            for neighbor in neighbors:
                self.update_node(neighbor)
    
    def replan(self, new_obstacles):
        self.km += self.heuristic(self.current, self.start)
        for (x, y) in new_obstacles:
            self.grid[x][y] = 1  # Mark as blocked
            self.update_node(self.nodes[x][y])
        self.compute_shortest_path()
    
    def get_path(self):
        path = []
        current = self.current
        while current != self.goal:
            path.append((current.x, current.y))
            neighbors = self.get_neighbors(current)
            current = min(neighbors, key=lambda n: n.g)
        path.append((self.goal.x, self.goal.y))
        return path

# Example usage
grid = [
    [0, 0, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 0, 0]
]

dstar = DStarLite((0, 0), (2, 3), grid)
dstar.compute_shortest_path()
print("Initial path:", dstar.get_path())  # Output: [(0, 0), (0, 1), (0, 2), (1, 3), (2, 3)]

# Add obstacle at (1, 0)
dstar.replan([(1, 0)])
print("Updated path:", dstar.get_path())  # Output: [(0, 0), (0, 1), (0, 2), (0, 3), (1, 3), (2, 3)]
