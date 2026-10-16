"""Sokoban solver: tìm kiếm theo CÚ ĐẨY (push), dùng A* có trọng số.

Ý tưởng:
  1. Mỗi node tìm kiếm = 1 cú đẩy thùng. Robot đi tới chỗ đứng đẩy bằng BFS
     (chi phí cú đẩy = số bước đi + 1).
  2. Tính trước "ô chết" (thùng nằm đó thì không bao giờ về được đích) bằng BFS ngược từ ô đích.
  3. Heuristic = ghép cặp thùng-đích có tổng khoảng cách đẩy nhỏ nhất (có cache).
  4. Tìm xong thì PHÁT LẠI các nước đi qua successors() để có chuỗi parent hợp lệ.

Chạy thử và in đường đi:
    py solution.py 19          # giải bài số 19 trong PROBLEMS, in từng bước
    py solution.py 19 map      # in thêm bản đồ sau mỗi bước
"""
import heapq
import sys
import time
from collections import deque, namedtuple
from itertools import count

from sokoban import sokoban_goal_state
FAST_W = 3                  # lượt đầu: W lớn -> ra lời giải nhanh 
REFINE_W = (2.5, 2, 1.5, 1)   # các lượt tinh chỉnh: W nhỏ dần (W = 1 là tối ưu)
REFINE = 3                  # số giây tối đa để tinh chỉnh (0 = chỉ lấy lượt nhanh)

DIRECTIONS = (("up", 0, -1), ("right", 1, 0), ("down", 0, 1), ("left", -1, 0))
INFINITY = 10 ** 9

# Một node tìm kiếm: vị trí robot, vị trí thùng, node cha, các nước đi của cú đẩy này
# (mỗi nước đi là (chỉ số robot, tên hướng))
Node = namedtuple("Node", "robots boxes parent moves")


def solve(initial_state, timebound=120):
    #Trả về SokobanState đích (có chuỗi parent hợp lệ), hoặc False nếu hết giờ / vô nghiệm.
    if timebound <= 0:
        return False
    deadline = time.time() + timebound
    if sokoban_goal_state(initial_state):
        return initial_state

    width, height = initial_state.width, initial_state.height
    obstacles, storage = initial_state.obstacles, initial_state.storage

    def is_open(cell):
        #Ô nằm trong bàn và không phải tường
        return 0 <= cell[0] < width and 0 <= cell[1] < height and cell not in obstacles
        #BFS ngược: khoảng cách đẩy
    # Từ mỗi ô đích, kéo thùng lùi lại. Ô nào chạm tới được thì thùng ở đó có thể về đích.
    storage_list = sorted(storage)
    push_distance = []                       # push_distance[k][ô] = số cú đẩy tới đích thứ k
    for goal in storage_list:
        distance = {goal: 0}
        queue = deque([goal])
        while queue:
            box_cell = queue.popleft()
            for _, step_x, step_y in DIRECTIONS:
                before = (box_cell[0] - step_x, box_cell[1] - step_y)   # thùng trước khi bị đẩy
                pusher = (before[0] - step_x, before[1] - step_y)       # chỗ robot đứng
                if before not in distance and is_open(before) and is_open(pusher):
                    distance[before] = distance[box_cell] + 1
                    queue.append(before)
        push_distance.append(distance)

    # cost_to_goals[ô] = (số cú đẩy tới đích 0, đích 1, ...). Ô không có trong bảng = ô chết.
    cost_to_goals = {}
    for x in range(width):
        for y in range(height):
            row = tuple(d.get((x, y), INFINITY) for d in push_distance)
            if min(row) < INFINITY:
                cost_to_goals[(x, y)] = row
    alive_cells = cost_to_goals

    if any(box not in alive_cells for box in initial_state.boxes):
        return False                         # có thùng ở ô chết ngay từ đầu: vô nghiệm

    #Heuristic: ghép cặp thùng - đích
    goal_count = len(storage_list)
    heuristic_cache = {}

    def estimate_remaining(boxes):
        """Tổng khoảng cách đẩy nhỏ nhất khi mỗi thùng chiếm một ô đích riêng."""
        cached = heuristic_cache.get(boxes)
        if cached is not None:
            return cached
        # quy hoạch động: best_by_mask[mask] = chi phí nhỏ nhất khi các đích trong mask đã dùng
        best_by_mask = {0: 0}
        for box in boxes:
            row = cost_to_goals[box]
            next_best = {}
            for mask, total in best_by_mask.items():
                for goal in range(goal_count):
                    if not (mask >> goal) & 1:
                        new_mask, new_total = mask | (1 << goal), total + row[goal]
                        if new_total < next_best.get(new_mask, INFINITY):
                            next_best[new_mask] = new_total
            best_by_mask = next_best
        result = min(best_by_mask.values()) if best_by_mask else INFINITY
        heuristic_cache[boxes] = result
        return result

    #Phát hiện deadlock (thùng kẹt vĩnh viễn)
    def is_frozen(box, boxes, visited):
        #Thùng có bị kẹt vĩnh viễn (không đẩy được theo cả 2 trục) không?
        visited = visited | {box}
        for step_x, step_y in ((1, 0), (0, 1)):
            side_a = (box[0] - step_x, box[1] - step_y)
            side_b = (box[0] + step_x, box[1] + step_y)
            if not is_open(side_a) or not is_open(side_b):
                continue                                    # tường chặn trục này
            if side_a not in alive_cells and side_b not in alive_cells:
                continue                                    # cả 2 phía là ô chết
            a_blocked = side_a in boxes and (side_a in visited or is_frozen(side_a, boxes, visited))
            b_blocked = side_b in boxes and (side_b in visited or is_frozen(side_b, boxes, visited))
            if a_blocked or b_blocked:
                continue                                    # thùng kề bên cũng bị kẹt
            return False                                    # trục này vẫn còn đi được
        return True

    def creates_deadlock(pushed_to, boxes):
        #Sau khi đẩy thùng tới pushed_to: có thùng nào NGOÀI đích bị kẹt vĩnh viễn không?
        x, y = pushed_to
        for cell in ((x, y), (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if cell in boxes and cell not in storage and is_frozen(cell, boxes, frozenset()):
                return True
        return False

    #BFS đi bộ của một robot
    def walk_tree(start, blocked):
        """BFS từ start. Trả về {ô: (ô trước đó, tên hướng)} cho mọi ô robot tới được."""
        came_from = {start: None}
        queue = deque([start])
        while queue:
            cell = queue.popleft()
            for direction_name, step_x, step_y in DIRECTIONS:
                neighbor = (cell[0] + step_x, cell[1] + step_y)
                if neighbor not in came_from and is_open(neighbor) and neighbor not in blocked:
                    came_from[neighbor] = (cell, direction_name)
                    queue.append(neighbor)
        return came_from

    def path_to(came_from, target):
        #Dựng danh sách hướng đi từ điểm xuất phát tới target.
        directions = []
        while came_from[target] is not None:
            target, direction_name = came_from[target]
            directions.append(direction_name)
        directions.reverse()
        return directions

    #A* trên các cú đẩy
    def search(weight, upper_bound, time_limit):
        """A* với f = g + weight * h. Bỏ mọi nhánh không thể tốt hơn upper_bound.
        Trả về (tổng bước, node đích) hoặc None nếu hết giờ / không tìm được."""
        tie_breaker = count()
        start = Node(initial_state.robots, initial_state.boxes, None, ())
        open_heap = [(weight * estimate_remaining(start.boxes), 0, next(tie_breaker), 0, start)]
        best_cost = {(tuple(sorted(start.robots)), start.boxes): 0}

        while open_heap:
            if time.time() >= time_limit:
                return None
            _, _, _, cost_so_far, node = heapq.heappop(open_heap)
            robots, boxes = node.robots, node.boxes
            if cost_so_far > best_cost[(tuple(sorted(robots)), boxes)]:
                continue                                    # bản ghi cũ, đã có đường rẻ hơn
            if all(box in storage for box in boxes):
                return cost_so_far, node

            robot_cells = set(robots)
            reachable = [walk_tree(r, boxes | obstacles | (robot_cells - {r})) for r in robots]

            for box in boxes:
                for direction_name, step_x, step_y in DIRECTIONS:
                    box_target = (box[0] + step_x, box[1] + step_y)
                    if box_target not in alive_cells or box_target in boxes or box_target in robot_cells:
                        continue
                    pusher_cell = (box[0] - step_x, box[1] - step_y)    # chỗ robot phải đứng
                    new_boxes = (boxes - {box}) | {box_target}
                    if creates_deadlock(box_target, new_boxes):
                        continue
                    h_value = estimate_remaining(new_boxes)

                    for i in range(len(robots)):                        # thử từng robot
                        if pusher_cell not in reachable[i]:
                            continue
                        walking = path_to(reachable[i], pusher_cell)
                        new_cost = cost_so_far + len(walking) + 1       # đi bộ + 1 cú đẩy
                        if new_cost + h_value >= upper_bound:
                            continue                                    # không thể thắng lời giải đã có
                        new_robots = robots[:i] + (box,) + robots[i + 1:]
                        state_key = (tuple(sorted(new_robots)), new_boxes)
                        if new_cost < best_cost.get(state_key, INFINITY):
                            best_cost[state_key] = new_cost
                            move_list = tuple((i, d) for d in walking) + ((i, direction_name),)
                            heapq.heappush(open_heap, (
                                new_cost + weight * h_value, h_value, next(tie_breaker),
                                new_cost, Node(new_robots, new_boxes, node, move_list)))
        return None

    # Lượt 1: W lớn để có lời giải nhanh (phải xong trong timebound)
    first_start = time.time()
    found = search(FAST_W, INFINITY, deadline)
    if found is None:
        return False
    best_total, best_node = found

    # Các lượt sau: giảm W để rút ngắn lời giải.
    # Bài dễ (lượt đầu xong nhanh) -> tinh chỉnh ngắn; bài khó -> tối đa REFINE giây.
    refine_budget = min(REFINE, max(0.2, 20 * (time.time() - first_start)))
    refine_deadline = min(deadline, time.time() + refine_budget)
    for weight in REFINE_W:
        result = search(weight, best_total, refine_deadline)
        if result is not None and result[0] < best_total:
            best_total, best_node = result
        if time.time() >= refine_deadline:
            break
    return replay(initial_state, best_node)

#cấy ni thầy yêu cầu truy ngược về nên giữ nhé
def replay(initial_state, goal_node):
    #Chạy lại chuỗi nước đi qua successors() của giáo viên -> parent/action/gval thật.
    moves_per_push = []
    node = goal_node
    while node is not None:
        moves_per_push.append(node.moves)
        node = node.parent
    current = initial_state
    for moves in reversed(moves_per_push):
        for robot_index, direction_name in moves:
            wanted_action = "%d %s" % (robot_index, direction_name)
            current = next(s for s in current.successors() if s.action == wanted_action)
    return current



#in đường đi, đoạn ni k liên quan đến code, cmay bỏ qua cũng được   
def get_path(final_state):
    """Danh sách các state từ trạng thái đầu tới đích (đi ngược theo parent)."""
    states = []
    while final_state is not None:
        states.append(final_state)
        final_state = final_state.parent
    states.reverse()
    return state_string


def print_path(final_state, show_map=False):
    """In từng bước: robot nào đi hướng nào, bước nào là ĐẨY thùng. show_map=True in thêm bản đồ."""
    states = get_path(final_state)
    print("Tổng số bước: %d" % final_state.gval)
    if show_map:
        print(states[0].state_string())
    for step_number, (before, after) in enumerate(zip(states, states[1:]), start=1):
        pushed = after.boxes != before.boxes
        robot_index, direction_name = after.action.split()
        note = "  <-- ĐẨY THÙNG" if pushed else ""
        print("Bước %3d: robot %s đi %-5s%s" % (step_number, robot_index, direction_name, note))
        if show_map:
            print(after.state_string())