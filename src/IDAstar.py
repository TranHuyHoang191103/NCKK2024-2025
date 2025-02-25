def ida_star(start_state, goal_state, graph, heuristic_func):
    def search(path, g, bound):
        state = path[-1]
        f = g + heuristic_func(state, goal_state)
        if f > bound:
            return f
        if state == goal_state:
            return path.copy(), g  # Trả về đường đi và tổng chi phí
        min_bound = float('inf')
        for next_state in graph.neighbors(state):
            if next_state not in path:  # Tránh lặp lại trạng thái
                cost = graph.edges[state, next_state]['weight']
                path.append(next_state)
                result = search(path, g + cost, bound)
                if isinstance(result, tuple):  # Tìm thấy đường đi
                    return result
                if result < min_bound:
                    min_bound = result
                path.pop()  # Quay lui (backtrack)
        return min_bound

    bound = heuristic_func(start_state, goal_state)
    path = [start_state]
    while True:
        result = search(path, 0, bound)
        if isinstance(result, tuple):  # Đã tìm thấy đường đi
            return result  # Trả về đường đi và tổng chi phí
        if result == float('inf'):  # Không tìm thấy đường đi
            return None, None
        bound = result  # Cập nhật giới hạn mới
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
