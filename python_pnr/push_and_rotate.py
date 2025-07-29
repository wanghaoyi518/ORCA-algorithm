from .actor_set import ActorSet
from .mapf_config import MAPFConfig
from .mapf_search_result import MAPFSearchResult
from .sub_map import SubMap
from .isearch import ISearch
from .node import Node, Point, ActorMove
from .utils import CN_INFINITY
import time
from collections import deque, defaultdict
import copy

def debug_log(msg):
    with open("pnr_debug.log", "a") as f:
        f.write(msg + "\n")
    print(msg)

class PushAndRotate:
    def __init__(self, search=None):
        self.search = search  # ISearch 实例
        self.result = MAPFSearchResult()
        self.agents_moves = []
        self.agents_paths = []

    def clear(self):
        self.agents_paths.clear()
        self.agents_moves.clear()

    def move_along_path(self, actor_set, aid, path):
        # 辅助函数：沿A*路径逐步推进，每一步都写入agents_moves
        for idx in range(1, len(path)):
            prev = path[idx-1]
            cur = path[idx]
            for a in actor_set:
                if a.id == aid and a.current.x == prev.i and a.current.y == prev.j:
                    # 计算增量移动
                    di = cur.i - prev.i
                    dj = cur.j - prev.j
                    # 存储增量移动，与C++保持一致
                    self.agents_moves.append(ActorMove(di, dj, a.id))
                    # 更新agent位置
                    a.current.x = cur.i
                    a.current.y = cur.j
                    break

    def clear_node(self, sub_map: SubMap, actor_set: ActorSet, node: Node, occupied_nodes=None):
        debug_log(f"[clear_node] node: ({node.i},{node.j}), occupied={occupied_nodes}")
        if occupied_nodes is None:
            occupied_nodes = set()
        candidates = []
        for i in range(sub_map.height):
            for j in range(sub_map.width):
                n = Node(i, j)
                if not sub_map.is_traversable(i, j):
                    continue
                if n in occupied_nodes:
                    continue
                if n == node:
                    continue
                if any(a.current.x == n.i and a.current.y == n.j for a in actor_set):
                    continue
                candidates.append(n)
        debug_log(f"[clear_node] 可行目标点: {[ (n.i, n.j) for n in candidates ]}")
        for goal in candidates:
            dijkstra = ISearch(sub_map)
            occupied_nodes = set((a.current.x, a.current.y) for a in actor_set)
            occupied_nodes.discard((node.i, node.j))
            occupied_nodes.discard((goal.i, goal.j))
            path = dijkstra.search(node, goal, occupied_nodes)
            debug_log(f"[clear_node] 尝试目标: ({goal.i},{goal.j})，A*路径: {[ (n.i, n.j) for n in path ] if path else '无路径'}")
            if path and len(path) > 1:
                # 只移动一步
                prev = path[-2]
                cur = path[-1]
                for a in actor_set:
                    if a.current.x == cur.i and a.current.y == cur.j:
                        debug_log(f"[clear_node] move actor {a.id} from ({cur.i},{cur.j}) to ({prev.i},{prev.j})")
                        # 用A*路径逐步推进
                        self.move_along_path(actor_set, a.id, path[-2:])
                        break
                if prev in occupied_nodes:
                    debug_log(f"[clear_node] prev ({prev.i},{prev.j}) 已被占用，递归让路")
                    if not self.clear_node(sub_map, actor_set, prev, occupied_nodes):
                        return False
                debug_log(f"[clear_node] return True (moved one step)")
                return True
        debug_log(f"[clear_node] 所有目标均失败, return False")
        return False

    def push(self, sub_map: SubMap, actor_set: ActorSet, from_node: Node, to_node: Node, occupied_nodes=None):
        debug_log(f"[push] from ({from_node.i},{from_node.j}) to ({to_node.i},{to_node.j}), occupied={occupied_nodes}")
        if occupied_nodes is None:
            occupied_nodes = set()
        if to_node in occupied_nodes:
            debug_log(f"[push] to_node {to_node.i},{to_node.j} in occupied_nodes, return False")
            return False
        if not sub_map.is_traversable(to_node.i, to_node.j):
            debug_log(f"[ERROR] push推进到障碍物: ({to_node.i},{to_node.j})，忽略该移动")
            return False
        blocking_actor = None
        for a in actor_set:
            if a.current.x == to_node.i and a.current.y == to_node.j:
                blocking_actor = a
                break
        if blocking_actor is None:
            dijkstra = ISearch(sub_map)
            occupied_nodes = set((a.current.x, a.current.y) for a in actor_set)
            occupied_nodes.discard((from_node.i, from_node.j))
            occupied_nodes.discard((to_node.i, to_node.j))
            path = dijkstra.search(from_node, to_node, occupied_nodes)
            debug_log(f"[push] 无阻挡A*路径: {[ (n.i, n.j) for n in path ] if path else '无路径'}")
            if not path or len(path) < 2:
                debug_log(f"[push] 无法找到合法路径，return False")
                return False
            # 用A*路径逐步推进
            self.move_along_path(actor_set, [a.id for a in actor_set if a.current.x == from_node.i and a.current.y == from_node.j][0], path)
            debug_log(f"[push] return True (no blocking actor, moved step by step)")
            return True
        debug_log(f"[push] blocking_actor: {blocking_actor.id} at ({to_node.i},{to_node.j})")
        inserted = False
        if from_node not in occupied_nodes:
            occupied_nodes.add(from_node)
            inserted = True
        can_clear = self.clear_node(sub_map, actor_set, to_node, occupied_nodes)
        debug_log(f"[push] clear_node result: {can_clear}")
        if inserted:
            occupied_nodes.remove(from_node)
        if not can_clear:
            debug_log(f"[push] clear_node failed, return False")
            return False
        # 只移动一步
        self.move_along_path(actor_set, [a.id for a in actor_set if a.current.x == from_node.i and a.current.y == from_node.j][0], [from_node, to_node])
        debug_log(f"[push] return True (after clear_node, moved one step)")
        return True

    def multipush(self, sub_map: SubMap, actor_set: ActorSet, first: Node, second: Node, to: Node, path):
        if len(path) > 1 and path[1].i == second.i and path[1].j == second.j:
            first, second = second, first
            path = path[1:]
        prev_node = second
        for idx in range(len(path)-1):
            cur_node = path[idx]
            next_node = path[idx+1]
            occupied_nodes = {prev_node, cur_node}
            if not sub_map.is_traversable(next_node.i, next_node.j):
                debug_log(f"[ERROR] multipush推进到障碍物: ({next_node.i},{next_node.j})，忽略该移动")
                return False
            blocking = any(a.current.x == next_node.i and a.current.y == next_node.j for a in actor_set)
            if blocking:
                if not self.clear_node(sub_map, actor_set, next_node, occupied_nodes):
                    return False
            # 用A*路径逐步推进
            self.move_along_path(actor_set, [a.id for a in actor_set if a.current.x == cur_node.i and a.current.y == cur_node.j][0], [cur_node, next_node])
            self.move_along_path(actor_set, [a.id for a in actor_set if a.current.x == prev_node.i and a.current.y == prev_node.j][0], [prev_node, cur_node])
            prev_node = cur_node
        return True

    def rotate(self, sub_map: SubMap, actor_set: ActorSet, q_path, cycle_beg):
        """Improved rotate function with complex case handling"""
        size = len(q_path) - cycle_beg
        
        # Try simple rotation first
        for i in range(cycle_beg, len(q_path)):
            empty = not any(a.current.x == q_path[i].i and a.current.y == q_path[i].j for a in actor_set)
            if empty:
                for j in range(size-1):
                    from_idx = cycle_beg + (i - cycle_beg - j - 1 + size) % size
                    to_idx = cycle_beg + (i - cycle_beg - j + size) % size
                    if not sub_map.is_traversable(q_path[to_idx].i, q_path[to_idx].j):
                        debug_log(f"[ERROR] rotate推进到障碍物: ({q_path[to_idx].i},{q_path[to_idx].j})，忽略该移动")
                        return False
                    self.move_along_path(actor_set, [a.id for a in actor_set if a.current.x == q_path[from_idx].i and a.current.y == q_path[from_idx].j][0], [q_path[from_idx], q_path[to_idx]])
                return True
        
        # If simple rotation fails, try complex case handling
        cycle_nodes = set(q_path[cycle_beg:])
        
        for i in range(cycle_beg, len(q_path)):
            cycle_nodes.discard(q_path[i])
            
            # Find agent at current position
            first_agent_id = None
            for agent in actor_set:
                if agent.current.x == q_path[i].i and agent.current.y == q_path[i].j:
                    first_agent_id = agent.id
                    break
            
            if first_agent_id is None:
                continue
            
            beg_size = len(self.agents_moves)
            
            # Try to clear the current position
            if self.clear_node(sub_map, actor_set, q_path[i], cycle_nodes):
                end_size = len(self.agents_moves)
                
                # Find second agent (previous in cycle)
                second_agent_index = cycle_beg + (i - cycle_beg - 1 + size) % size
                second_agent_id = None
                for agent in actor_set:
                    if agent.current.x == q_path[second_agent_index].i and agent.current.y == q_path[second_agent_index].j:
                        second_agent_id = agent.id
                        break
                
                if second_agent_id is None:
                    cycle_nodes.add(q_path[i])
                    continue
                
                # Move second agent to current position
                for agent in actor_set:
                    if agent.id == second_agent_id:
                        agent.current.x = q_path[i].i
                        agent.current.y = q_path[i].j
                        break
                
                # Get current position of first agent
                cur_position = None
                for agent in actor_set:
                    if agent.id == first_agent_id:
                        cur_position = Node(agent.current.x, agent.current.y)
                        break
                
                if cur_position is None:
                    cycle_nodes.add(q_path[i])
                    continue
                
                # Try swap operation
                if self.swap(sub_map, actor_set, q_path[i], cur_position):
                    # Complete the rotation
                    for j in range(size-1):
                        from_idx = cycle_beg + (i - cycle_beg - j - 2 + size) % size
                        to_idx = cycle_beg + (i - cycle_beg - j - 1 + size) % size
                        if any(a.current.x == q_path[from_idx].i and a.current.y == q_path[from_idx].j for a in actor_set):
                            self.move_along_path(actor_set, [a.id for a in actor_set if a.current.x == q_path[from_idx].i and a.current.y == q_path[from_idx].j][0], [q_path[from_idx], q_path[to_idx]])
                    
                    # Reverse the moves from clear_node operation
                    self.reverse(beg_size, end_size, first_agent_id, second_agent_id, actor_set)
                    return True
            
            cycle_nodes.add(q_path[i])
        
        return False

    def reverse(self, beg_size, end_size, first_id, second_id, actor_set: ActorSet):
        # 还原C++ reverse逻辑
        for i in range(end_size-1, beg_size-1, -1):
            move = self.agents_moves[i]
            id = move.id  # 现在move是ActorMove对象
            if id == first_id:
                id = second_id
            elif id == second_id:
                id = first_id
            for a in actor_set:
                if a.id == id:
                    # 反向移动
                    from_node = a.current
                    to_node = Node(from_node.i - move.di, from_node.j - move.dj)
                    a.current = to_node
                    # 存储反向的增量移动
                    self.agents_moves.append(ActorMove(-move.di, -move.dj, a.id))
        # 注意：此处简化，需根据C++逻辑补全

    def solve(self, sub_map: SubMap, config: MAPFConfig, actor_set: ActorSet):
        # 完全还原C++ solve主流程，包括优先级排序和循环检测
        # 优先级比较器
        def comparator(id1, id2):
            subgraph1 = getattr(actor_set.get_actor_by_id(id1), 'subgraph', -1)
            subgraph2 = getattr(actor_set.get_actor_by_id(id2), 'subgraph', -1)
            
            if subgraph1 != subgraph2:
                if subgraph1 == -1 or (hasattr(self, 'priorities') and subgraph2 in self.priorities.get(subgraph1, set())):
                    return False
                elif subgraph2 == -1 or (hasattr(self, 'priorities') and subgraph1 in self.priorities.get(subgraph2, set())):
                    return True
            return id1 < id2
        
        # 检查是否为多边形地图
        is_polygon = True
        for i in range(sub_map.height):
            for j in range(sub_map.width):
                if sub_map.grid[i][j] == 0:  # 可通行
                    # 计算度数（简化版）
                    degree = 0
                    for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
                        ni, nj = i+di, j+dj
                        if 0<=ni<sub_map.height and 0<=nj<sub_map.width and sub_map.grid[ni][nj] == 0:
                            degree += 1
                    if degree != 2:
                        is_polygon = False
                        break
            if not is_polygon:
                break
        
        not_finished = set(a.id for a in actor_set)
        finished = set()
        finished_positions = set()
        q_path_nodes = set()
        q_path = []
        
        cur_agent_id = -1
        steps = 0
        
        while not_finished and steps < config.max_steps:
            steps += 1

            # 交换位置冲突检测开始
            # 收集所有agent的当前位置和下一个位置（如果有路径）
            agent_positions = {}
            agent_next_positions = {}
            for a in actor_set:
                agent_positions[a.id] = (int(a.current.x), int(a.current.y))
                # 预测下一步（如果有规划路径且未到终点）
                if hasattr(a, 'planned_path') and a.planned_path is not None:
                    idx = 0
                    # 找到当前所在路径点
                    for i, n in enumerate(a.planned_path):
                        if (int(a.current.x), int(a.current.y)) == (n.i, n.j):
                            idx = i
                            break
                    # 取下一个点
                    if idx + 1 < len(a.planned_path):
                        agent_next_positions[a.id] = (a.planned_path[idx + 1].i, a.planned_path[idx + 1].j)
            # 检查所有agent对，是否有交换位置型冲突
            for id1, pos1 in agent_positions.items():
                for id2, pos2 in agent_positions.items():
                    if id1 >= id2:
                        continue
                    next1 = agent_next_positions.get(id1)
                    next2 = agent_next_positions.get(id2)
                    if next1 is not None and next2 is not None:
                        if pos1 == next2 and pos2 == next1:
                            debug_log(f"检测到Agent {id1} 和 Agent {id2} 发生交换位置型碰撞: {pos1} <-> {pos2}")
                            return False
            # 交换位置冲突检测结束

            if cur_agent_id == -1:
                # 按优先级选择下一个agent - 简化逻辑
                cur_agent_id = min(not_finished)
            
            debug_log(f"步骤 {steps}: 选择Agent {cur_agent_id}, not_finished={not_finished}")
            
            cur_agent = actor_set.get_actor_by_id(cur_agent_id)
            if cur_agent_id not in not_finished:
                debug_log(f"错误: Agent {cur_agent_id} 不在 not_finished 中!")
                return False
            not_finished.remove(cur_agent_id)
            
            # 搜索路径 - 修复坐标转换
            path = self.search.search(
                Node(int(cur_agent.current.x), int(cur_agent.current.y)),
                Node(int(cur_agent.goal.x), int(cur_agent.goal.y)),
                (lambda occ, s, g: occ - {(s.i, s.j), (g.i, g.j)})(set((a.current.x, a.current.y) for a in actor_set), Node(int(cur_agent.current.x), int(cur_agent.current.y)), Node(int(cur_agent.goal.x), int(cur_agent.goal.y)))
            )
            debug_log(f"A*路径 (Agent {cur_agent_id}): {[ (n.i, n.j) for n in path ] if path else '无路径'}")
            if not path or len(path) < 2:
                return False
            
            # 处理路径
            q_path.append(path[0])
            q_path_nodes.add(path[0])
            
            for idx in range(len(path)-1):
                current_node = path[idx]
                next_node = path[idx+1]
                
                # 检查循环
                if next_node in q_path_nodes:
                    cycle_beg = len(q_path) - 1
                    while cycle_beg >= 0 and q_path[cycle_beg] != next_node:
                        cycle_beg -= 1
                    
                    self.rotate(sub_map, actor_set, q_path, cycle_beg)
                    
                    # 清理循环部分
                    while len(q_path) > cycle_beg:
                        last_node = q_path.pop()
                        q_path_nodes.remove(last_node)
                        # 处理finished_positions
                        for a in actor_set:
                            if a.current.x == last_node.i and a.current.y == last_node.j:
                                if a.id in finished:
                                    finished_positions.add(last_node)
                                break
                else:
                    # 尝试push，失败则swap
                    occupied_set = finished_positions if is_polygon else set()
                    if not self.push(sub_map, actor_set, current_node, next_node, occupied_set):
                        if not self.swap(sub_map, actor_set, current_node, next_node):
                            return False
                        # 更新finished_positions
                        for a in actor_set:
                            if a.current.x == current_node.i and a.current.y == current_node.j and a.id in finished:
                                finished_positions.discard(next_node)
                                finished_positions.add(current_node)
                                break
                # 主循环推进时同步current并写入agents_moves（修正点）
                for a in actor_set:
                    if a.id == cur_agent_id and a.current.x == current_node.i and a.current.y == current_node.j:
                        # 计算增量移动
                        di = next_node.i - current_node.i
                        dj = next_node.j - current_node.j
                        # 存储增量移动
                        self.agents_moves.append(ActorMove(di, dj, a.id))
                        # 更新agent位置
                        a.current.x = next_node.i
                        a.current.y = next_node.j
                        break
                q_path.append(next_node)
                q_path_nodes.add(next_node)
            
            finished.add(cur_agent_id)
            finished_positions.add(Node(int(cur_agent.goal.x), int(cur_agent.goal.y)))
            
            # 处理q_path中的剩余agent
            cur_agent_id = -1
            while q_path:
                last_node = q_path[-1]
                for a in actor_set:
                    if a.current.x == last_node.i and a.current.y == last_node.j:
                        goal_node = Node(int(a.goal.x), int(a.goal.y))
                        if a.id not in not_finished and last_node != goal_node:
                            # 检查目标位置是否空闲
                            goal_occupied = any(oa.current.x == goal_node.i and oa.current.y == goal_node.j for oa in actor_set)
                            if not goal_occupied:
                                # 禁止直接赋值到goal，只允许逐步推进
                                # 需要用A*路径逐步推进到goal
                                dijkstra = ISearch(sub_map)
                                occupied_nodes = set((a.current.x, a.current.y) for a in actor_set)
                                occupied_nodes.discard((last_node.i, last_node.j))
                                occupied_nodes.discard((goal_node.i, goal_node.j))
                                path = dijkstra.search(last_node, goal_node, occupied_nodes)
                                if path and len(path) > 1:
                                    self.move_along_path(actor_set, a.id, path)
                                    finished_positions.discard(last_node)
                                    finished_positions.add(goal_node)
                                # 如果已经在goal或无法到达，不做任何补写
                            else:
                                # 找到占用目标位置的agent
                                for oa in actor_set:
                                    if oa.current.x == goal_node.i and oa.current.y == goal_node.j:
                                        cur_agent_id = oa.id
                                        debug_log(f"  重新选择Agent {cur_agent_id} (占用目标位置)")
                                        if cur_agent_id not in not_finished:
                                            not_finished.add(cur_agent_id)
                                            debug_log(f"  将Agent {cur_agent_id} 重新添加到 not_finished")
                                        break
                                break
                        break
                if cur_agent_id != -1:
                    break
                q_path_nodes.remove(last_node)
                q_path.pop()
        
        # 生成路径
        self.get_parallel_paths(actor_set, config)
        
        self.result.steps = steps
        return len(not_finished) == 0

    def clear(self, sub_map: SubMap, actor_set: ActorSet, first: Node, second: Node):
        """Implement the missing clear function from C++ version"""
        # Get successors of first node
        successors = []
        for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
            ni, nj = first.i + di, first.j + dj
            if sub_map.is_traversable(ni, nj):
                successors.append(Node(ni, nj))
        
        # Find unoccupied successors
        unoccupied = []
        for node in successors:
            if not any(a.current.x == node.i and a.current.y == node.j for a in actor_set):
                unoccupied.append(node)
        
        if len(unoccupied) >= 2:
            return True
        
        # Try to clear occupied successors
        forbidden = {first, second}
        forbidden.update(unoccupied)
        
        for node in successors:
            if node not in unoccupied and node != second:
                if self.clear_node(sub_map, actor_set, node, forbidden):
                    if len(unoccupied) >= 1:
                        return True
                    unoccupied.append(node)
                    forbidden.add(node)
        
        if not unoccupied:
            return False
        
        # Try complex clearing strategies
        free_neigh = unoccupied[0]
        for node in successors:
            if node != second and node != free_neigh:
                # Strategy 1: Clear both nodes
                cur_size = len(self.agents_moves)
                new_actor_set = copy.deepcopy(actor_set)
                if self.clear_node(sub_map, new_actor_set, node, {first, second}):
                    if self.clear_node(sub_map, new_actor_set, free_neigh, {first, second, node}):
                        # Update original actor_set
                        for i, agent in enumerate(actor_set):
                            agent.current.x = new_actor_set[i].current.x
                            agent.current.y = new_actor_set[i].current.y
                        return True
                    else:
                        # Rollback moves
                        self.agents_moves = self.agents_moves[:cur_size]
                break
        
        # Strategy 2: Move and clear
        for node in successors:
            if node != second and node != free_neigh:
                cur_size = len(self.agents_moves)
                new_actor_set = copy.deepcopy(actor_set)
                
                # Move first to free_neigh, second to first
                for agent in new_actor_set:
                    if agent.current.x == first.i and agent.current.y == first.j:
                        agent.current.x = free_neigh.i
                        agent.current.y = free_neigh.j
                    elif agent.current.x == second.i and agent.current.y == second.j:
                        agent.current.x = first.i
                        agent.current.y = first.j
                
                if self.clear_node(sub_map, new_actor_set, node, {first, second}):
                    if self.clear_node(sub_map, new_actor_set, second, {first, second, node}):
                        # Update original actor_set
                        for i, agent in enumerate(actor_set):
                            agent.current.x = new_actor_set[i].current.x
                            agent.current.y = new_actor_set[i].current.y
                        return True
                    else:
                        # Rollback moves
                        self.agents_moves = self.agents_moves[:cur_size]
                break
        
        # Strategy 3: Final attempt
        second_agent_id = None
        for agent in actor_set:
            if agent.current.x == second.i and agent.current.y == second.j:
                second_agent_id = agent.id
                break
        
        if second_agent_id is None:
            return False
        
        if not self.clear_node(sub_map, actor_set, second, {first}):
            return False
        
        # Move first to second
        for agent in actor_set:
            if agent.current.x == first.i and agent.current.y == first.j:
                agent.current.x = second.i
                agent.current.y = second.j
                break
        
        # Get second's new position
        second_position = None
        for agent in actor_set:
            if agent.id == second_agent_id:
                second_position = Node(agent.current.x, agent.current.y)
                break
        
        if second_position is None:
            return False
        
        if not self.clear_node(sub_map, actor_set, free_neigh, {first, second, second_position}):
            return False
        
        # Complex final moves
        for node in successors:
            if node != second and node != free_neigh:
                # Move node to first, first to free_neigh, second to first, second_position to second
                for agent in actor_set:
                    if agent.current.x == node.i and agent.current.y == node.j:
                        agent.current.x = first.i
                        agent.current.y = first.j
                    elif agent.current.x == first.i and agent.current.y == first.j:
                        agent.current.x = free_neigh.i
                        agent.current.y = free_neigh.j
                    elif agent.current.x == second.i and agent.current.y == second.j:
                        agent.current.x = first.i
                        agent.current.y = first.j
                    elif agent.current.x == second_position.i and agent.current.y == second_position.j:
                        agent.current.x = second.i
                        agent.current.y = second.j
                
                return self.clear_node(sub_map, actor_set, free_neigh, {first, second, node})
        
        return False

    def swap(self, sub_map: SubMap, actor_set: ActorSet, first: Node, second: Node):
        """Improved swap function matching C++ implementation"""
        # Get agent IDs
        first_agent_id = None
        second_agent_id = None
        for agent in actor_set:
            if agent.current.x == first.i and agent.current.y == first.j:
                first_agent_id = agent.id
            elif agent.current.x == second.i and agent.current.y == second.j:
                second_agent_id = agent.id
        
        if first_agent_id is None or second_agent_id is None:
            return False
        
        # Define goal condition: find nodes with degree >= 3
        def is_goal(start, cur, sub_map, actor_set):
            # Calculate degree (number of traversable neighbors)
            degree = 0
            for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
                ni, nj = cur.i + di, cur.j + dj
                if sub_map.is_traversable(ni, nj):
                    degree += 1
            return degree >= 3
        
        # Search for exchange node
        search_result = self.search.search(first, None, set())
        while search_result:
            # Find a node that satisfies goal condition
            exchange_node = None
            for node in search_result:
                if is_goal(first, node, sub_map, actor_set):
                    exchange_node = node
                    break
            
            if exchange_node is None:
                break
            
            # Try multipush to exchange node
            beg_size = len(self.agents_moves)
            new_actor_set = copy.deepcopy(actor_set)
            
            if self.multipush(sub_map, new_actor_set, first, second, exchange_node, search_result):
                # Find which agent is at exchange node
                exchange_agent_id = None
                neigh_agent_id = None
                for agent in new_actor_set:
                    if agent.current.x == exchange_node.i and agent.current.y == exchange_node.j:
                        exchange_agent_id = agent.id
                    elif agent.current.x == first.i and agent.current.y == first.j:
                        neigh_agent_id = agent.id
                
                if exchange_agent_id is None or neigh_agent_id is None:
                    break
                
                # Determine which agent is the neighbor
                if exchange_agent_id == first_agent_id:
                    neigh_agent_id = second_agent_id
                else:
                    neigh_agent_id = first_agent_id
                
                # Get neighbor's position
                neigh_node = None
                for agent in new_actor_set:
                    if agent.id == neigh_agent_id:
                        neigh_node = Node(agent.current.x, agent.current.y)
                        break
                
                if neigh_node and self.clear(sub_map, new_actor_set, exchange_node, neigh_node):
                    # Update original actor_set
                    for i, agent in enumerate(actor_set):
                        agent.current.x = new_actor_set[i].current.x
                        agent.current.y = new_actor_set[i].current.y
                    
                    end_size = len(self.agents_moves)
                    self.exchange(sub_map, actor_set, exchange_node, neigh_node)
                    self.reverse(beg_size, end_size, first_agent_id, second_agent_id, actor_set)
                    return True
            
            # Try next search result
            search_result = self.search.search(first, None, set())
        
        return False

    def exchange(self, sub_map: SubMap, actor_set: ActorSet, first: Node, second: Node):
        """Improved exchange function matching C++ implementation"""
        # Find successors of first node
        successors = []
        for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
            ni, nj = first.i + di, first.j + dj
            if sub_map.is_traversable(ni, nj):
                successors.append(Node(ni, nj))
        
        # Find free neighbors
        free_neigh = []
        for node in successors:
            if not any(a.current.x == node.i and a.current.y == node.j for a in actor_set):
                free_neigh.append(node)
        
        if len(free_neigh) < 2:
            return False
        
        # Execute 6-step exchange sequence as in C++ version
        # Step 1: Move first to free_neigh[0]
        for agent in actor_set:
            if agent.current.x == first.i and agent.current.y == first.j:
                agent.current.x = free_neigh[0].i
                agent.current.y = free_neigh[0].j
                break
        
        # Step 2: Move second to first
        for agent in actor_set:
            if agent.current.x == second.i and agent.current.y == second.j:
                agent.current.x = first.i
                agent.current.y = first.j
                break
        
        # Step 3: Move first to free_neigh[1]
        for agent in actor_set:
            if agent.current.x == free_neigh[0].i and agent.current.y == free_neigh[0].j:
                agent.current.x = free_neigh[1].i
                agent.current.y = free_neigh[1].j
                break
        
        # Step 4: Move free_neigh[0] to first
        for agent in actor_set:
            if agent.current.x == free_neigh[0].i and agent.current.y == free_neigh[0].j:
                agent.current.x = first.i
                agent.current.y = first.j
                break
        
        # Step 5: Move first to second
        for agent in actor_set:
            if agent.current.x == first.i and agent.current.y == first.j:
                agent.current.x = second.i
                agent.current.y = second.j
                break
        
        # Step 6: Move free_neigh[1] to first
        for agent in actor_set:
            if agent.current.x == free_neigh[1].i and agent.current.y == free_neigh[1].j:
                agent.current.x = first.i
                agent.current.y = first.j
                break
        
        return True

    def get_subgraphs(self, sub_map: SubMap, actor_set: ActorSet):
        # 完全还原C++ getSubgraphs逻辑
        # 这里需要为每个Node和Actor维护subgraph/connected_component等属性
        # Python实现需在Node/ActorSet中动态添加这些属性
        from collections import deque, defaultdict
        height, width = sub_map.height, sub_map.width
        grid = sub_map.grid
        node_to_subgraph = {}
        subgraph_id = 0
        visited = set()
        for i in range(height):
            for j in range(width):
                if grid[i][j] == 0 and (i, j) not in visited:
                    # BFS标记连通分量
                    queue = deque()
                    queue.append((i, j))
                    visited.add((i, j))
                    while queue:
                        ci, cj = queue.popleft()
                        node_to_subgraph[(ci, cj)] = subgraph_id
                        for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
                            ni, nj = ci+di, cj+dj
                            if 0<=ni<height and 0<=nj<width and grid[ni][nj]==0 and (ni, nj) not in visited:
                                queue.append((ni, nj))
                                visited.add((ni, nj))
                    subgraph_id += 1
        # 为每个actor和节点分配subgraph属性
        for a in actor_set:
            pos = (a.current.x, a.current.y)  # 使用Point的x,y属性
            a.subgraph = node_to_subgraph.get(pos, -1)
        self.node_to_subgraph = node_to_subgraph

    def assign_to_subgraphs(self, sub_map: SubMap, actor_set: ActorSet):
        # 完全还原C++ assignToSubgraphs逻辑
        # 统计每个subgraph中的actor数量
        from collections import defaultdict
        subgraph_actor_count = defaultdict(int)
        for a in actor_set:
            if hasattr(a, 'subgraph'):
                subgraph_actor_count[a.subgraph] += 1
        self.subgraph_actor_count = subgraph_actor_count
        # 可根据需要为每个节点/actor分配更多属性

    def get_priorities(self, sub_map: SubMap, actor_set: ActorSet):
        # 完全还原C++ getPriorities逻辑
        # 这里用一个dict维护subgraph之间的优先级关系
        from collections import defaultdict
        priorities = defaultdict(set)
        for a in actor_set:
            if hasattr(a, 'goal') and hasattr(a, 'subgraph'):
                goal_pos = (a.goal.x, a.goal.y)  # 使用Point的x,y属性
                goal_subgraph = self.node_to_subgraph.get(goal_pos, -1)
                if goal_subgraph != a.subgraph and goal_subgraph != -1 and a.subgraph != -1:
                    priorities[a.subgraph].add(goal_subgraph)
        self.priorities = priorities

    def get_component(self, actor_set: ActorSet, start_edge, edge_stack, components):
        # 严格还原C++ getComponent逻辑
        # start_edge: (Node, Node)
        # edge_stack: list of (Node, Node)
        # components: list of set(Node)
        component = set()
        while edge_stack:
            cur_edge = edge_stack.pop()
            component.add(cur_edge[0])
            component.add(cur_edge[1])
            if cur_edge == start_edge:
                break
        if len(component) <= 2:
            return
        # 为component内所有节点分配同一个subgraph编号
        subgraph_num = len(components)
        for node in component:
            # 这里假设node为Node对象
            setattr(node, 'subgraph', subgraph_num)
        components.append(component)

    def combine_node_subgraphs(self, actor_set: ActorSet, components, subgraph_node, subgraph_num):
        # 严格还原C++ combineNodeSubgraphs逻辑
        # 合并所有与subgraph_node相连且subgraph编号不同的component
        # subgraph_node: Node对象
        # subgraph_num: int
        # components: list of set(Node)
        # 先找出所有需要合并的subgraph编号
        to_merge = []
        for idx, comp in enumerate(components):
            if subgraph_node in comp and idx != subgraph_num:
                to_merge.append(idx)
        # 合并所有component到subgraph_num
        for idx in to_merge:
            for node in components[idx]:
                setattr(node, 'subgraph', subgraph_num)
                components[subgraph_num].add(node)
            components[idx].clear()

    def get_reachable_nodes_count(self, sub_map: SubMap, actor_set: ActorSet, start: Node, condition, occupied_nodes):
        # 完全还原C++ getReachableNodesCount逻辑
        from collections import deque
        visited = set()
        queue = deque()
        queue.append(start)
        count = 0
        while queue:
            node = queue.popleft()
            if node in visited:
                continue
            visited.add(node)
            if condition(start, node, sub_map, actor_set):
                count += 1
            for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
                ni, nj = node.i+di, node.j+dj
                if 0<=ni<sub_map.height and 0<=nj<sub_map.width and sub_map.grid[ni][nj]==0:
                    nnode = Node(ni, nj)
                    if nnode not in visited and nnode not in occupied_nodes:
                        queue.append(nnode)
        return count

    def get_paths(self, actor_set: ActorSet):
        # 完全还原C++ getPaths逻辑
        self.agents_paths = [[] for _ in range(len(actor_set))]
        
        # 初始化每个agent的起始位置（与C++版本一致）
        agent_positions = []
        for i, a in enumerate(actor_set):
            # 使用初始位置作为起点
            start_pos = Node(a.current.x, a.current.y)
            agent_positions.append(start_pos)
            self.agents_paths[i].append(start_pos)
        
        # 按时间顺序处理每个移动（与C++版本一致）
        for move in self.agents_moves:
            aid, to_node = move
            # 更新移动的agent的位置（模拟C++的增量移动）
            for idx, a in enumerate(actor_set):
                if a.id == aid:
                    agent_positions[idx] = to_node
                    break
            
            # 为所有agent添加当前位置到路径（与C++版本完全一致）
            for idx, pos in enumerate(agent_positions):
                self.agents_paths[idx].append(pos)
        
        return self.agents_paths

    def get_parallel_paths(self, actor_set: ActorSet, config: MAPFConfig):
        # 严格按照C++ getParallelPaths逻辑实现
        agent_count = len(actor_set)
        agents_positions = [[] for _ in range(agent_count)]
        agent_ind = [0] * agent_count
        nodes_occupations = {}  # Node -> list of agent indices
        node_ind = {}  # Node -> current index in occupation list
        
        # 初始化路径和位置
        self.agents_paths = [[] for _ in range(agent_count)]
        for i, a in enumerate(actor_set):
            # 使用agent的起始位置
            start_position = Node(a.start.x, a.start.y)
            agents_positions[i].append(start_position)
            self.agents_paths[i].append(start_position)
            if start_position not in nodes_occupations:
                nodes_occupations[start_position] = []
                node_ind[start_position] = 0
            nodes_occupations[start_position].append(i)  # 存储agent索引，与C++一致
        
        # 处理每个移动（使用增量移动）
        debug_log(f"处理 {len(self.agents_moves)} 个移动")
        for move in self.agents_moves:
            # move现在是ActorMove对象，move.id是agent ID
            # 在C++中，move.id直接用作索引，所以agent ID必须从0开始连续
            agent_idx = move.id  # 直接使用agent ID作为索引
            
            if 0 <= agent_idx < agent_count:
                cur = agents_positions[agent_idx][-1]
                # 计算新位置（增量移动）
                new_pos = Node(cur.i + move.di, cur.j + move.dj)
                debug_log(f"Agent {move.id} (idx={agent_idx}): ({cur.i},{cur.j}) + ({move.di},{move.dj}) = ({new_pos.i},{new_pos.j})")
                
                if new_pos not in nodes_occupations:
                    nodes_occupations[new_pos] = []
                    node_ind[new_pos] = 0
                
                # 暂时禁用重复位置移除逻辑，直接添加新位置
                agents_positions[agent_idx].append(new_pos)
                nodes_occupations[new_pos].append(agent_idx)  # 存储agent索引
            else:
                debug_log(f"警告: Agent ID {move.id} 超出范围")
        
        # 打印每个agent的位置序列
        for i, a in enumerate(actor_set):
            debug_log(f"Agent {a.id} (idx={i}) 位置序列: {[(p.i, p.j) for p in agents_positions[i]]}")
        
        # 重新实现并行路径生成，更接近C++逻辑
        debug_log(f"开始并行路径生成，agent_count={agent_count}")
        for i, a in enumerate(actor_set):
            debug_log(f"Agent {a.id} (idx={i}) 位置数量: {len(agents_positions[i])}")
        
        # 初始化路径
        self.agents_paths = [[] for _ in range(agent_count)]
        for i in range(agent_count):
            self.agents_paths[i].append(agents_positions[i][0])  # 添加起始位置
        
        # 并行路径生成
        finished = [False] * agent_count
        agent_ind = [0] * agent_count  # 每个agent在agents_positions中的当前位置
        
        # 重新构建nodes_occupations和node_ind，用于并行移动检测
        nodes_occupations = {}  # Node -> list of agent indices
        node_ind = {}  # Node -> current index in occupation list
        
        # 初始化起始位置的occupations
        for i in range(agent_count):
            start_pos = agents_positions[i][0]
            if start_pos not in nodes_occupations:
                nodes_occupations[start_pos] = []
                node_ind[start_pos] = 0
            nodes_occupations[start_pos].append(i)
        
        while True:
            has_moved = [False] * agent_count
            for i in range(agent_count):
                if has_moved[i] or finished[i]:
                    continue
                
                # 检查是否可以移动到下一个位置
                if agent_ind[i] + 1 < len(agents_positions[i]):
                    next_node = agents_positions[i][agent_ind[i] + 1]
                    current_node = agents_positions[i][agent_ind[i]]
                    
                    # 检查是否可以移动（基于C++逻辑的简化版本）
                    can_move = True
                    if next_node in nodes_occupations:
                        # 检查是否有其他agent在目标位置
                        for j in nodes_occupations[next_node]:
                            if j != i and not finished[j]:
                                can_move = False
                                break
                    
                    if can_move:
                        # 可以移动
                        has_moved[i] = True
                        # 更新occupations
                        if current_node in nodes_occupations:
                            nodes_occupations[current_node].remove(i)
                        if next_node not in nodes_occupations:
                            nodes_occupations[next_node] = []
                        nodes_occupations[next_node].append(i)
                        
                        agent_ind[i] += 1
                        self.agents_paths[i].append(agents_positions[i][agent_ind[i]])
                        if agent_ind[i] == len(agents_positions[i]) - 1:
                            finished[i] = True
                    else:
                        # 不能移动，重复当前位置
                        self.agents_paths[i].append(agents_positions[i][agent_ind[i]])
                else:
                    # 已经到达终点
                    self.agents_paths[i].append(agents_positions[i][agent_ind[i]])
                    finished[i] = True
            
            # 检查是否还有agent移动
            if not any(has_moved):
                break
        
        # 写入结果
        for i, a in enumerate(actor_set):
            debug_log(f"Agent {a.id} 最终路径: {[(p.i, p.j) for p in self.agents_paths[i]]}")
            # 转换为Point对象
            point_path = [Point(p.i, p.j) for p in self.agents_paths[i]]
            self.result.add_path(a.id, point_path)
        
        return self.agents_paths

    def start_search(self, sub_map: SubMap, config: MAPFConfig, actor_set: ActorSet):
        self.result = MAPFSearchResult()
        self.search = ISearch(sub_map)
        start_time = time.time()
        # 还原C++主入口完整流程
        self.get_subgraphs(sub_map, actor_set)
        self.assign_to_subgraphs(sub_map, actor_set)
        self.get_priorities(sub_map, actor_set)
        success = self.solve(sub_map, config, actor_set)
        self.result.success = success
        self.result.runtime = time.time() - start_time
        return self.result 