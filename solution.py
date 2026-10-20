"""Sokoban solver: tìm kiếm theo CÚ ĐẨY (push), dùng A* có trọng số.

Ý tưởng:
  1. Mỗi node tìm kiếm = 1 cú đẩy thùng. Robot đi tới chỗ đứng đẩy bằng BFS
     (chi phí cú đẩy = số bước đi + 1).
  2. Tính trước "ô chết" (thùng nằm đó thì không bao giờ về được đích) bằng BFS ngược từ ô đích.
  3. Heuristic = ghép cặp thùng-đích có tổng khoảng cách đẩy nhỏ nhất (thuật toán Hungarian, có cache).
  4. Nếu chế độ nhanh bế tắc (vd. robot chắn đường nhau) thì bật thêm nước "nhường chỗ"
     (robot đi tới một ô khác mà không đẩy gì) để đảm bảo không bỏ sót lời giải.
  5. Tìm xong thì PHÁT LẠI các nước đi qua successors() để có chuỗi parent hợp lệ.
"""
import heapq
import time
from collections import deque, namedtuple
from itertools import count

from sokoban import sokoban_goal_state

# ---------------------------------------------------------------- cấu hình
FIRST_W = (3, 8, 25)        # lượt đầu: thử W = 3, chưa ra thì tăng dần W (càng tham lam càng nhanh)
FIRST_SLICE = 2.0           # W đầu tiên chỉ được thử tối đa chừng này giây (bản đồ lớn thì bỏ sớm)
REFINE_W = (2.5, 2, 1.5, 1,0) # các lượt tinh chỉnh: W nhỏ dần (W = 1 là tối ưu)
REFINE = 3                  # số giây tối đa để tinh chỉnh (0 = chỉ lấy lượt nhanh)
REFINE_MIN = 0.5            # ngân sách tinh chỉnh tối thiểu (giây) cho bài dễ
LIGHT_W = (2, 1)            # các lượt cuối có thêm nước nhường chỗ
MAX_STATES = 1_500_000      # giới hạn số trạng thái lưu trong 1 lượt tìm (chống tràn bộ nhớ)

DIRECTIONS = (("up", 0, -1), ("right", 1, 0), ("down", 0, 1), ("left", -1, 0))
INFINITY = 10 ** 9
TIMEOUT = "timeout"         # search() hết giờ
EXHAUSTED = "exhausted"     # search() duyệt hết mà không thấy lời giải

# Một node tìm kiếm: vị trí robot, vị trí thùng, node cha, các nước đi của bước này
# (mỗi nước đi là (chỉ số robot, tên hướng))
Node = namedtuple("Node", "robots boxes parent moves")


def min_cost_matching(rows):
    """Hungarian: ghép mỗi hàng (thùng) với 1 cột (đích) khác nhau, tổng chi phí nhỏ nhất.
    rows: ma trận n x m (n <= m). Độ phức tạp O(n^2 * m), không phụ thuộc 2^m."""
    n, m = len(rows), len(rows[0])
    u, v = [0] * (n + 1), [0] * (m + 1)
    column_owner, way = [0] * (m + 1), [0] * (m + 1)
    for i in range(1, n + 1):
        column_owner[0] = i
        j0 = 0
        min_slack = [INFINITY * 4] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0, delta, j1 = column_owner[j0], INFINITY * 4, 0
            for j in range(1, m + 1):
                if not used[j]:
                    cur = rows[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < min_slack[j]:
                        min_slack[j], way[j] = cur, j0
                    if min_slack[j] < delta:
                        delta, j1 = min_slack[j], j
            for j in range(m + 1):
                if used[j]:
                    u[column_owner[j]] += delta
                    v[j] -= delta
                else:
                    min_slack[j] -= delta
            j0 = j1
            if column_owner[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            column_owner[j0] = column_owner[j1]
            j0 = j1
    return -v[0]


def solve(initial_state, timebound=120):
    #Trả về SokobanState đích (có chuỗi parent hợp lệ), hoặc False nếu hết giờ / vô nghiệm.
    if timebound <= 0:
        return False
    deadline = time.time() + timebound
    if sokoban_goal_state(initial_state):
        return initial_state

    width, height = initial_state.width, initial_state.height
    obstacles, storage = initial_state.obstacles, initial_state.storage

    if len(initial_state.boxes) > len(storage):
        return False                         # nhiều thùng hơn ô đích: vô nghiệm

    def is_open(cell):
        #Ô nằm trong bàn và không phải tường
        return 0 <= cell[0] < width and 0 <= cell[1] < height and cell not in obstacles

    # ---------------------------------------- BFS ngược: khoảng cách đẩy
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

    # ---------------------------------------- Heuristic: ghép cặp thùng - đích
    heuristic_cache = {}

    def estimate_remaining(boxes):
        #Tổng khoảng cách đẩy nhỏ nhất khi mỗi thùng chiếm một ô đích riêng.
        cached = heuristic_cache.get(boxes)
        if cached is not None:
            return cached
        result = min_cost_matching([cost_to_goals[box] for box in boxes]) if boxes else 0
        if result >= INFINITY:
            result = INFINITY                # không ghép được: trạng thái vô vọng
        heuristic_cache[boxes] = result
        return result

    # ---------------------------------------- Phát hiện deadlock (thùng kẹt vĩnh viễn)
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

    # ---------------------------------------- BFS đi bộ của một robot
    def walk_tree(start, blocked):
        #BFS từ start. Trả về {ô: (ô trước đó, tên hướng)} cho mọi ô robot tới được.
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

    # ---------------------------------------- A* trên các cú đẩy
    def search(weight, upper_bound, time_limit, allow_relocation=False, light_relocation=False):
        """A* với f = g + weight * h. Bỏ mọi nhánh không thể tốt hơn upper_bound.
        allow_relocation=True: thêm nước "robot đi sang ô khác mà không đẩy" (để nhường chỗ).
        light_relocation=True: chỉ cho robot đang chắn ô đích của một cú đẩy bước sang ô gần đó.
        Trả về (tổng bước, node đích), hoặc TIMEOUT, hoặc EXHAUSTED."""
        tie_breaker = count()
        start = Node(initial_state.robots, initial_state.boxes, None, ())
        open_heap = [(weight * estimate_remaining(start.boxes), 0, next(tie_breaker), 0, start)]
        best_cost = {(tuple(sorted(start.robots)), start.boxes): 0}

        def push_node(new_cost, h_value, new_robots, new_boxes, parent, move_list):
            state_key = (tuple(sorted(new_robots)), new_boxes)
            if new_cost < best_cost.get(state_key, INFINITY):
                best_cost[state_key] = new_cost
                heapq.heappush(open_heap, (new_cost + weight * h_value, h_value, next(tie_breaker),
                                           new_cost, Node(new_robots, new_boxes, parent, move_list)))

        while open_heap:
            if time.time() >= time_limit or len(best_cost) > MAX_STATES:
                return TIMEOUT
            _, _, _, cost_so_far, node = heapq.heappop(open_heap)
            robots, boxes = node.robots, node.boxes
            if cost_so_far > best_cost[(tuple(sorted(robots)), boxes)]:
                continue                                    # bản ghi cũ, đã có đường rẻ hơn
            if all(box in storage for box in boxes):
                return cost_so_far, node

            robot_cells = set(robots)
            reachable = [walk_tree(r, boxes | obstacles | (robot_cells - {r})) for r in robots]

            blocked_targets = set()                             # ô đích của cú đẩy đang bị robot đứng chắn
            for box in boxes:
                for direction_name, step_x, step_y in DIRECTIONS:
                    box_target = (box[0] + step_x, box[1] + step_y)
                    if box_target not in alive_cells or box_target in boxes:
                        continue
                    if box_target in robot_cells:
                        blocked_targets.add(box_target)
                        continue
                    pusher_cell = (box[0] - step_x, box[1] - step_y)    # chỗ robot phải đứng
                    new_boxes = (boxes - {box}) | {box_target}
                    if creates_deadlock(box_target, new_boxes):
                        continue
                    h_value = estimate_remaining(new_boxes)
                    if h_value >= INFINITY:
                        continue

                    for i in range(len(robots)):                        # thử từng robot
                        if pusher_cell not in reachable[i]:
                            continue
                        walking = path_to(reachable[i], pusher_cell)
                        new_cost = cost_so_far + len(walking) + 1       # đi bộ + 1 cú đẩy
                        if new_cost + h_value >= upper_bound:
                            continue                                    # không thể thắng lời giải đã có
                        new_robots = robots[:i] + (box,) + robots[i + 1:]
                        move_list = tuple((i, d) for d in walking) + ((i, direction_name),)
                        push_node(new_cost, h_value, new_robots, new_boxes, node, move_list)

            if light_relocation and blocked_targets and not allow_relocation:        # nhường chỗ nhẹ: robot chắn đường đi ra gần đó
                h_value = estimate_remaining(boxes)
                for i in range(len(robots)):
                    if robots[i] not in blocked_targets:
                        continue
                    for cell in reachable[i]:
                        if cell == robots[i]:
                            continue
                        walking = path_to(reachable[i], cell)
                        if len(walking) > 2:
                            continue
                        new_cost = cost_so_far + len(walking)
                        if new_cost + h_value >= upper_bound:
                            continue
                        new_robots = robots[:i] + (cell,) + robots[i + 1:]
                        move_list = tuple((i, d) for d in walking)
                        push_node(new_cost, h_value, new_robots, boxes, node, move_list)

            if allow_relocation:                                # robot đi sang ô khác, không đẩy
                h_value = estimate_remaining(boxes)
                for i in range(len(robots)):
                    for cell in reachable[i]:
                        if cell == robots[i]:
                            continue
                        walking = path_to(reachable[i], cell)
                        new_cost = cost_so_far + len(walking)
                        if new_cost + h_value >= upper_bound:
                            continue
                        new_robots = robots[:i] + (cell,) + robots[i + 1:]
                        move_list = tuple((i, d) for d in walking)
                        push_node(new_cost, h_value, new_robots, boxes, node, move_list)
        return EXHAUSTED

    # Lượt đầu: thử các W tăng dần. W đầu được tối đa FIRST_SLICE giây, W giữa được nửa thời gian còn lại,
    # W cuối được phần còn lại. Lượt nào ra lời giải là dùng.
    first_start = time.time()
    found = TIMEOUT
    for position, weight in enumerate(FIRST_W):
        remaining = deadline - time.time()
        if position == 0:
            slice_end = time.time() + min(FIRST_SLICE, remaining / 4)
        elif position < len(FIRST_W) - 1:
            slice_end = time.time() + remaining / 2
        else:
            slice_end = deadline
        found = search(weight, INFINITY, slice_end)
        if found != TIMEOUT:
            break
    if found == EXHAUSTED:                   # bế tắc: thử lại có thêm nước "nhường chỗ"
        found = search(FIRST_W[0], INFINITY, deadline, allow_relocation=True)
    if found in (TIMEOUT, EXHAUSTED):
        return False
    best_total, best_node = found

    # Các lượt sau: giảm W để rút ngắn lời giải.
    # Bài dễ (lượt đầu xong nhanh) -> tinh chỉnh ngắn; bài khó -> tối đa REFINE giây.
    refine_budget = min(REFINE, max(REFINE_MIN, 100 * (time.time() - first_start)))
    refine_deadline = min(deadline, time.time() + refine_budget)
    # Các lượt đầu: tinh chỉnh thường. Các lượt cuối: có thêm nước nhường chỗ nhẹ (chỉ chạy
    # nếu còn thời gian, thường là bản đồ nhỏ).
    passes = [(w, False) for w in REFINE_W] + [(w, True) for w in LIGHT_W]
    for weight, light in passes:
        result = search(weight, best_total, refine_deadline, light_relocation=light)
        if isinstance(result, tuple) and result[0] < best_total:
            best_total, best_node = result
        if time.time() >= refine_deadline:
            break
    return replay(initial_state, best_node)


def replay(initial_state, goal_node):
    #Chạy lại chuỗi nước đi qua successors() của giáo viên -> parent/action/gval thật.
    moves_per_node = []
    node = goal_node
    while node is not None:
        moves_per_node.append(node.moves)
        node = node.parent
    current = initial_state
    for moves in reversed(moves_per_node):
        for robot_index, direction_name in moves:
            wanted_action = "%d %s" % (robot_index, direction_name)
            current = next(s for s in current.successors() if s.action == wanted_action)
    return current


# in đường đi (không liên quan chấm điểm)
def get_path(final_state):
    #Danh sách các state từ trạng thái đầu tới đích (đi ngược theo parent).
    states = []
    while final_state is not None:
        states.append(final_state)
        final_state = final_state.parent
    states.reverse()
    return states


def print_path(final_state, show_map=False):
    #In từng bước: robot nào đi hướng nào, bước nào là ĐẨY thùng. show_map=True in thêm bản đồ.
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