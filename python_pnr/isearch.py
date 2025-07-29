from .node import Node, Point
import heapq

class ISearch:
    def __init__(self, sub_map):
        self.sub_map = sub_map

    def search(self, start: Node, goal: Node, occupied_nodes=None):
        # 标准A*实现
        if occupied_nodes is None:
            occupied_nodes = set()
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            for neighbor in self.get_neighbors(current, occupied_nodes):
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    def heuristic(self, node1, node2):
        if node2 is None:
            return 0  # 当goal为None时，返回0作为启发式值
        return abs(node1.i - node2.i) + abs(node1.j - node2.j)

    def get_neighbors(self, node, occupied_nodes=None):
        neighbors = []
        if occupied_nodes is None:
            occupied_nodes = set()
        for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
            ni, nj = node.i + di, node.j + dj
            if self.sub_map.is_traversable(ni, nj) and (ni, nj) not in occupied_nodes:
                neighbors.append(self.sub_map.get_node(ni, nj))
        return neighbors

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path 