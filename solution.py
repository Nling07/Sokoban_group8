"""Student entry point. See README.md for the API and scoring rules."""
"""Sokoban solver: A* có trọng số trên từng nước đi (successors của giáo viên).

Bố cục file:
  1. Hằng số
  2. Ô chết (deadlock tĩnh): góc chết + đoạn tường chết theo cột/hàng
  3. Heuristic: bảng chi phí đẩy thùng -> ghép thùng-đích -> cộng chi phí robot đi tới
  4. solve: vòng lặp A*, f = g + WEIGHT * h
"""
import time                                     # đo thời gian để dừng khi hết timebound
import heapq                                    # heap (hàng đợi ưu tiên) cho A* và Dijkstra
from itertools import count                     # bộ đếm tăng dần làm tie-breaker trong heap
from sokoban import sokoban_goal_state          # hàm của giáo viên: kiểm tra state đã thắng chưa

# ----------------------------------------------------------------------
# 1. Hằng số
# ----------------------------------------------------------------------
INF = 10**6                                     # "vô cực": thùng không thể tới đích
DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))       # 4 hướng đẩy: phải, trái, xuống, lên (dx, dy)
WEIGHT = 3                                      # trọng số của h: lớn -> tìm nhanh hơn, lời giải kém tối ưu hơn


# ----------------------------------------------------------------------
# 2. Ô chết (deadlock tĩnh) - tính 1 lần từ bản đồ ban đầu
# ----------------------------------------------------------------------
def make_blocked(state):
    """Trả về hàm blocked(i, j): ô (i, j) là tường hoặc nằm ngoài bản đồ."""
    width = state.width                         # chiều rộng bản đồ
    height = state.height                       # chiều cao bản đồ

    def blocked(i, j):
        return (i < 0 or i >= width            # ngoài biên trái/phải
                or j < 0 or j >= height        # ngoài biên trên/dưới
                or (i, j) in state.obstacles)  # hoặc là vật cản

    return blocked


def find_corner_cells(state, blocked):
    """Ô góc chết: không phải đích, bị chặn cả theo chiều dọc lẫn chiều ngang."""
    corners = set()                             # tập ô góc chết
    for x in range(state.width):                # duyệt mọi cột
        for y in range(state.height):           # duyệt mọi hàng
            if (x, y) in state.storage:         # ô đích: thùng vào đây là xong, không chết
                continue
            elif (x, y) in state.obstacles:     # ô tường: không có thùng nào đứng đây
                continue

            up = blocked(x, y - 1)              # phía trên bị chặn?
            down = blocked(x, y + 1)            # phía dưới bị chặn?
            left = blocked(x - 1, y)            # bên trái bị chặn?
            right = blocked(x + 1, y)           # bên phải bị chặn?

            if (up or down) and (left or right):    # kẹt cả trục dọc lẫn trục ngang = góc
                corners.add((x, y))             # thùng vào góc này không đẩy ra được nữa
    return corners


def scan_line(state, line, corners, blocked, side_a, side_b):
    """Quét 1 đường thẳng (cột hoặc hàng). Hai góc chết liên tiếp, giữa chúng không có đích
    và các ô ở giữa cùng sát 1 bức tường -> mọi ô ở giữa đều là ô chết.
    side_a / side_b = (dx, dy) của 2 phía vuông góc với đường (vd cột: trái/phải)."""
    found = set()                               # các ô chết tìm được trên đường này
    temp_a = set()                              # ô ứng viên đang sát tường phía a
    temp_b = set()                              # ô ứng viên đang sát tường phía b
    start = False                               # đã gặp góc chết mở đầu chưa

    for x, y in line:                           # đi dọc theo đường
        if (x, y) in state.storage:             # gặp ô đích: đoạn này có chỗ cứu thùng
            temp_a.clear()                      # bỏ ứng viên phía a
            temp_b.clear()                      # bỏ ứng viên phía b
            start = False                       # huỷ đoạn đang xét
            continue
        elif (x, y) in state.obstacles:         # gặp tường: bỏ qua ô này
            continue
        elif (x, y) in corners:                 # gặp góc chết
            if not start:                       # góc đầu tiên -> bắt đầu 1 đoạn
                start = True
            else:                               # góc thứ hai -> chốt đoạn
                found |= temp_a | temp_b        # mọi ứng viên trong đoạn là ô chết
                start = False                   # kết thúc đoạn
            continue

        if start:                               # đang ở giữa 2 góc: kiểm tra ô này có sát tường không
            if blocked(x + side_a[0], y + side_a[1]):       # sát tường phía a
                temp_a.add((x, y))              # thêm vào ứng viên phía a
                temp_b.clear()                  # đổi phía tường thì ứng viên phía b không còn hợp lệ
            elif blocked(x + side_b[0], y + side_b[1]):     # sát tường phía b
                temp_b.add((x, y))              # thêm vào ứng viên phía b
                temp_a.clear()                  # bỏ ứng viên phía a
            else:                               # ô này không sát tường nào -> thùng có thể thoát ra
                start = False                   # huỷ đoạn
                temp_a.clear()
                temp_b.clear()
    return found


def blocked_space(state):
    """Tập mọi ô chết của bản đồ = góc chết + đoạn chết theo cột + đoạn chết theo hàng."""
    blocked = make_blocked(state)               # hàm kiểm tra ô bị chặn
    corners = find_corner_cells(state, blocked) # các góc chết

    dead_cols = set()                           # ô chết nằm trên các cột
    for x in range(state.width):                # mỗi cột là 1 đường thẳng đi từ trên xuống
        column = [(x, y) for y in range(state.height)]
        dead_cols |= scan_line(state, column, corners, blocked, (-1, 0), (1, 0))  # 2 phía: trái / phải

    dead_rows = set()                           # ô chết nằm trên các hàng
    for y in range(state.height):               # mỗi hàng là 1 đường thẳng đi từ trái sang phải
        row = [(x, y) for x in range(state.width)]
        dead_rows |= scan_line(state, row, corners, blocked, (0, -1), (0, 1))     # 2 phía: trên / dưới

    return corners | dead_cols | dead_rows     # gộp tất cả


# ----------------------------------------------------------------------
# 3. Heuristic: xử lý từng thùng một
# ----------------------------------------------------------------------
def turn(a, b):
    """Số bước robot phải đi thêm khi đổi hướng đẩy từ a sang b."""
    if a == b:                                  # đẩy tiếp cùng hướng
        return 0                                # robot chỉ bước theo thùng, không tốn thêm
    if DIRS[a][0] == -DIRS[b][0] and DIRS[a][1] == -DIRS[b][1]:     # hai hướng ngược nhau
        return 4                                # đổi chiều ngược: vòng quanh thùng
    return 2                                    # rẽ vuông góc


def build_push_cost_tables(storages, free):
    """Với mỗi đích j: ctg[j][(ô thùng, hướng đẩy cuối)] = số bước (đẩy + đi vòng) tối thiểu
    để đưa thùng từ ô đó tới đích j. Chỉ xét tường, bỏ qua thùng và robot khác."""
    tables = []                                 # tables[j] là bảng của đích j
    for s in storages:                          # mỗi đích 1 bảng
        dist = {}                               # dist[(ô, hướng đẩy cuối)] = chi phí nhỏ nhất
        pq = []                                 # hàng đợi ưu tiên cho Dijkstra
        for di in range(4):                     # thùng đã ở đích thì hướng đẩy cuối nào cũng được
            dist[(s, di)] = 0                   # chi phí 0
            pq.append((0, s, di))               # đưa vào hàng đợi
        heapq.heapify(pq)                       # biến list thành heap
        while pq:                               # Dijkstra NGƯỢC từ đích (kéo thùng ngược lại)
            d, n, dn = heapq.heappop(pq)        # lấy trạng thái có chi phí nhỏ nhất
            if d > dist[(n, dn)]:               # bản ghi cũ, đã có đường tốt hơn
                continue
            c = (n[0] - DIRS[dn][0], n[1] - DIRS[dn][1])    # ô thùng đứng trước khi bị đẩy
            b = (c[0] - DIRS[dn][0], c[1] - DIRS[dn][1])    # ô robot đứng để đẩy
            if not free(c) or not free(b):      # thiếu chỗ cho thùng hoặc robot -> không đẩy được
                continue
            for dl in range(4):                 # cú đẩy trước đó (hướng dl) có thể là hướng nào
                nd = d + 1 + turn(dl, dn)       # +1 cú đẩy, + chi phí robot đi vòng nếu đổi hướng
                if nd < dist.get((c, dl), INF): # tìm được đường rẻ hơn tới (c, dl)
                    dist[(c, dl)] = nd          # cập nhật
                    heapq.heappush(pq, (nd, c, dl))     # đưa vào hàng đợi để mở rộng tiếp
        tables.append(dist)                     # lưu bảng của đích này
    return tables


def build_box_tables(W, H, free, storages, ctg):
    """P[j][ô]: chi phí đưa thùng ở ô đó tới đích j (chưa tính robot đi tới chỗ đẩy).
    starts[ô]: danh sách (ô robot cần đứng để đẩy lần đầu, chi phí tốt nhất theo hướng đó)."""
    nS = len(storages)                          # số đích
    P = [dict() for _ in range(nS)]             # mỗi đích 1 dict ô -> chi phí
    starts = {}                                 # ô -> các lựa chọn đẩy đầu tiên
    for x in range(W):                          # duyệt mọi ô của bản đồ
        for y in range(H):
            c = (x, y)                          # ô đang xét (thùng đứng ở đây)
            if not free(c):                     # tường thì bỏ qua
                continue
            opts = []                           # các lựa chọn (chỗ robot đứng, chi phí) cho ô này
            for di in range(4):                 # thử cú đẩy đầu tiên theo hướng di
                n = (c[0] + DIRS[di][0], c[1] + DIRS[di][1])    # ô thùng tới sau cú đẩy đầu
                b = (c[0] - DIRS[di][0], c[1] - DIRS[di][1])    # ô robot phải đứng để đẩy
                if not free(n) or not free(b):  # không đẩy được theo hướng này
                    continue
                best = INF                      # chi phí tốt nhất cho hướng di
                for j in range(nS):             # thử từng đích
                    v = ctg[j].get((n, di))     # chi phí từ n (vừa đẩy theo di) tới đích j
                    if v is None:               # từ n không tới được đích j
                        continue
                    v += 1                      # cộng chính cú đẩy đầu tiên c -> n
                    if v < P[j].get(c, INF):    # rẻ hơn mức đã biết của (c, đích j)
                        P[j][c] = v             # cập nhật
                    if v < best:                # rẻ hơn mức tốt nhất của hướng di
                        best = v
                if best < INF:                  # có ít nhất 1 đích tới được theo hướng này
                    opts.append((b, best))      # ghi lại chỗ robot đứng và chi phí
            if opts:                            # ô này có thể đẩy đi được
                starts[c] = opts
            for j in range(nS):                 # nếu ô c chính là đích j
                if c == storages[j]:
                    P[j][c] = 0                 # thùng đã ở đích: chi phí 0
    return P, starts


def make_box_cost(P, nS):
    """Trả về hàm box_cost(boxes): ghép mỗi thùng với 1 đích riêng, tổng chi phí nhỏ nhất."""
    cache = {}                                  # cache theo tập thùng

    def box_cost(boxes):
        v = cache.get(boxes)                    # tra cache
        if v is not None:
            return v                            # đã tính rồi thì trả luôn
        dp = {0: 0}                             # dp[mask] = chi phí nhỏ nhất khi các đích trong mask đã có thùng
        for bx in boxes:                        # gán lần lượt từng thùng
            nd = {}                             # bảng dp mới sau khi gán thùng bx
            for mask, cst in dp.items():        # duyệt các cách gán hiện có
                for j in range(nS):             # thử gán thùng bx cho đích j
                    if mask >> j & 1:           # đích j đã bị thùng khác chiếm
                        continue
                    d = P[j].get(bx)            # chi phí từ ô bx tới đích j
                    if d is None:               # không tới được đích j
                        continue
                    nm = mask | (1 << j)        # mask mới: đánh dấu đích j đã dùng
                    val = cst + d               # tổng chi phí mới
                    if val < nd.get(nm, INF):   # chỉ giữ chi phí nhỏ nhất cho mỗi mask
                        nd[nm] = val
            dp = nd                             # chuyển sang thùng kế tiếp
            if not dp:                          # không còn cách gán nào -> vô nghiệm
                break
        v = min(dp.values()) if dp else INF     # chi phí gán tốt nhất (INF nếu không gán được)
        cache[boxes] = v                        # lưu cache
        return v

    return box_cost


def make_heuristic(state):
    """Dựng bảng chi phí theo bản đồ, trả về hàm h(state) dùng cho A*."""
    W, H, obs = state.width, state.height, state.obstacles  # kích thước và tường

    def free(p):                                # ô p nằm trong bản đồ và không phải tường
        return 0 <= p[0] < W and 0 <= p[1] < H and p not in obs

    storages = sorted(state.storage)            # danh sách đích theo thứ tự cố định (chỉ số j)
    nS = len(storages)                          # số đích

    ctg = build_push_cost_tables(storages, free)    # bảng Dijkstra ngược cho từng đích
    P, starts = build_box_tables(W, H, free, storages, ctg)     # chi phí theo ô + chỗ robot đứng đẩy đầu
    box_cost = make_box_cost(P, nS)             # hàm ghép thùng-đích
    storage_set = state.storage                 # tập đích để kiểm tra nhanh

    def h(st):
        bc = box_cost(st.boxes)                 # chi phí đẩy thùng (ghép tối ưu)
        if bc >= INF:                           # có thùng không thể tới đích
            return INF
        if bc == 0:                             # mọi thùng đã ở đích
            return 0
        extra = 0                               # chi phí robot đi tới chỗ đẩy của từng thùng chưa vào đích
        for bx in st.boxes:                     # duyệt từng thùng
            if bx in storage_set:               # thùng đã ở đích: bỏ qua
                continue
            opts = starts[bx]                   # các chỗ robot có thể đứng để đẩy thùng này lần đầu
            pmin = min(o[1] for o in opts)      # chi phí đẩy nhỏ nhất (đã nằm trong bc rồi)
            reach = min(abs(rx - b[0]) + abs(ry - b[1]) + cost     # Manhattan từ robot tới chỗ đứng + chi phí đẩy
                        for rx, ry in st.robots for b, cost in opts)   # thử mọi robot và mọi chỗ đứng
            extra += reach - pmin               # chỉ cộng phần dôi ra là quãng robot phải đi tới
        return bc + extra                       # h = chi phí đẩy + chi phí robot đi tới

    return h


# ----------------------------------------------------------------------
# 4. Tìm kiếm: A* có trọng số cố định, f = g + WEIGHT * h
# ----------------------------------------------------------------------
def solve(initial_state, timebound=120):
    t0 = time.time()                            # mốc bắt đầu để tính thời gian
    if sokoban_goal_state(initial_state):       # đã ở trạng thái thắng sẵn
        return initial_state

    dead_pol = blocked_space(initial_state)     # tập ô chết, tính 1 lần
    h = make_heuristic(initial_state)           # dựng hàm heuristic
    h0 = h(initial_state)                       # h của state ban đầu
    if h0 >= INF:                               # có thùng không thể tới đích
        return False                            # vô nghiệm

    tie = count()                               # bộ đếm để heap không phải so sánh state khi f, h bằng nhau
    L = [(WEIGHT * h0, h0, next(tie), initial_state)]   # heap: (f, h, tie, state); f = WEIGHT*h vì g=0
    best_g = {initial_state.hashable_state(): 0}        # best_g[state] = g nhỏ nhất đã biết

    while L:                                    # lặp tới khi hết state để mở rộng
        if time.time() - t0 >= timebound:       # hết thời gian
            return False

        _, _, _, state = heapq.heappop(L)       # lấy state có f nhỏ nhất
        if sokoban_goal_state(state):           # state thắng
            return state

        if state.gval > best_g.get(state.hashable_state(), float("inf")):
            continue                            # đã có đường tới state này tốt hơn -> bỏ bản cũ

        for child in state.successors():        # duyệt các nước đi hợp lệ của giáo viên
            if any(b in dead_pol for b in child.boxes):     # có thùng đứng trên ô chết
                continue                        # bỏ nhánh này
            key = child.hashable_state()        # khoá nhận diện state con
            if child.gval < best_g.get(key, float("inf")):  # đường tới child rẻ hơn mức đã biết
                hc = h(child)                   # heuristic của child
                if hc >= INF:                   # child vô nghiệm
                    continue
                best_g[key] = child.gval        # cập nhật g tốt nhất
                heapq.heappush(L, (child.gval + WEIGHT * hc, hc, next(tie), child))  # đưa vào heap, f = g + W*h

    return False                                # heap cạn mà chưa thấy lời giải