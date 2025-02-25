import networkx as nx
import heapq


def heuristic(node, goal):
    heuristic_values = {
        'A': 5, 'B': 4, 'C': 4, 'D': 3,
        'E': 2, 'F': 3, 'G': 1, 'H': 0
    }
    return heuristic_values.get(node, float('inf'))


def bidirectional_a_star(graph, start, goal):
    # Open lists (priority queues): lưu tuple (f_score, node)
    open_forward = []
    open_backward = []
    heapq.heappush(open_forward, (heuristic(start, goal), start))
    heapq.heappush(open_backward, (heuristic(goal, start), goal))

    # Closed sets: lưu các nút đã được mở rộng
    closed_forward = {}
    closed_backward = {}

    # G-score cho mỗi hướng
    g_forward = {node: float('inf') for node in graph.nodes}
    g_backward = {node: float('inf') for node in graph.nodes}
    g_forward[start] = 0
    g_backward[goal] = 0

    # Maps truy vết đường đi (came_from)
    came_from_forward = {}
    came_from_backward = {}

    meeting_node = None
    best_cost = float('inf')

    while open_forward and open_backward:
        # --- Mở rộng phía tiến ---
        f_score_f, current_forward = heapq.heappop(open_forward)
        closed_forward[current_forward] = True

        # Nếu nút này đã được mở rộng từ phía lùi thì cập nhật đường chung
        if current_forward in closed_backward:
            total_cost = g_forward[current_forward] + g_backward[current_forward]
            if total_cost < best_cost:
                best_cost = total_cost
                meeting_node = current_forward

        # Thư giãn (relax) các nút kề theo hướng tiến
        for neighbor in graph.neighbors(current_forward):
            weight = graph.edges[current_forward, neighbor].get('weight', 1)
            tentative = g_forward[current_forward] + weight
            if tentative < g_forward[neighbor]:
                g_forward[neighbor] = tentative
                came_from_forward[neighbor] = current_forward
                f_score = tentative + heuristic(neighbor, goal)
                heapq.heappush(open_forward, (f_score, neighbor))

        # --- Mở rộng phía lùi ---
        f_score_b, current_backward = heapq.heappop(open_backward)
        closed_backward[current_backward] = True

        if current_backward in closed_forward:
            total_cost = g_forward[current_backward] + g_backward[current_backward]
            if total_cost < best_cost:
                best_cost = total_cost
                meeting_node = current_backward

        # Ở hướng lùi, duyệt theo các nút có cung hướng về current_backward (predecessors)
        for neighbor in graph.predecessors(current_backward):
            weight = graph.edges[neighbor, current_backward].get('weight', 1)
            tentative = g_backward[current_backward] + weight
            if tentative < g_backward[neighbor]:
                g_backward[neighbor] = tentative
                came_from_backward[neighbor] = current_backward
                f_score = tentative + heuristic(neighbor, start)
                heapq.heappush(open_backward, (f_score, neighbor))

        # Điều kiện dừng: nếu chi phí tốt nhất đã tìm được nhỏ hơn hoặc bằng tổng f-score nhỏ nhất của 2 hướng
        if open_forward and open_backward:
            if best_cost <= open_forward[0][0] + open_backward[0][0]:
                break

    if meeting_node is None:
        return None, float('inf')

    # Tái tạo đường đi từ start đến meeting_node (hướng tiến)
    path_forward = []
    node = meeting_node
    while node != start:
        path_forward.append(node)
        node = came_from_forward[node]
    path_forward.append(start)
    path_forward.reverse()

    # Tái tạo đường đi từ meeting_node đến goal (hướng lùi)
    path_backward = []
    node = meeting_node
    while node != goal:
        node = came_from_backward[node]
        path_backward.append(node)

    full_path = path_forward + path_backward
    return full_path, best_cost


