"""Sokoban solver: tim kiem theo CU DAY (push), A* co trong so.

Y tuong:
  1. Moi node = 1 cu day thung. Robot di toi cho dung bang BFS (chi phi = so buoc di + 1).
  2. Tinh truoc "o chet" (thung khong the keo toi dich) bang BFS nguoc tu o dich.
  3. h = gan cap thung-dich chi phi nho nhat (khoang cach day that, co cache).
  4. Tim xong thi PHAT LAI cac nuoc di qua successors() de co chuoi parent hop le.
"""
import heapq
import time
from collections import deque
from itertools import count

from sokoban import sokoban_goal_state

FAST_W = 3                              # luot dau: nhanh
REFINE_W = (2.5, 2, 1.5, 1)                # cac luot tinh chinh (W=1: toi uu)
REFINE = 3                              # so giay toi da de tinh chinh (dat 0 = chi lay luot nhanh)
DIRS = (("up", 0, -1), ("right", 1, 0), ("down", 0, 1), ("left", -1, 0))
INF = 10 ** 9


def solve(initial_state, timebound=120):
    if timebound <= 0:
        return False
    end = time.time() + timebound
    init = initial_state
    if sokoban_goal_state(init):
        return init

    wd, ht, obs, sto = init.width, init.height, init.obstacles, init.storage

    def free(c):
        return 0 <= c[0] < wd and 0 <= c[1] < ht and c not in obs

    # khoang cach day tu moi o toi dich (BFS nguoc); o khong co = o chet
    pdist = {}                           # list khoang cach toi tung dich
    sto_list = sorted(sto)
    for k, s in enumerate(sto_list):
        dist = {s: 0}
        q = deque([s])
        while q:
            b = q.popleft()
            for _, dx, dy in DIRS:
                a = (b[0] - dx, b[1] - dy)           # thung truoc khi bi day
                p = (a[0] - dx, a[1] - dy)           # cho robot dung
                if a not in dist and free(a) and free(p):
                    dist[a] = dist[b] + 1
                    q.append(a)
        pdist[k] = dist
    cost = {}                            # tuple chi phi toi tung dich
    for x in range(wd):
        for y in range(ht):
            row = tuple(pdist[k].get((x, y), INF) for k in range(len(sto_list)))
            if min(row) < INF:
                cost[(x, y)] = row
    alive = cost                         # o song = co trong cost

    # heuristic = gan cap nho nhat (DP bitmask), cache theo tap thung
    m = len(sto_list)
    hcache = {}

    def h(boxes):
        r = hcache.get(boxes)
        if r is not None:
            return r
        rows = [cost[b] for b in boxes]
        dp = {0: 0}
        for row in rows:
            nd = {}
            for mask, v in dp.items():
                for j in range(m):
                    if not mask >> j & 1:
                        nm, nv = mask | 1 << j, v + row[j]
                        if nv < nd.get(nm, INF):
                            nd[nm] = nv
            dp = nd
        r = min(dp.values()) if dp else INF
        hcache[boxes] = r
        return r

    def frozen(b, boxes, seen):
        # thung b co bi ket vinh vien (khong day duoc theo ca 2 truc) khong?
        seen = seen | {b}
        for dx, dy in ((1, 0), (0, 1)):
            n1, n2 = (b[0] - dx, b[1] - dy), (b[0] + dx, b[1] + dy)
            if not free(n1) or not free(n2):
                continue                                  # tuong chan truc nay
            if n1 not in alive and n2 not in alive:
                continue                                  # ca 2 phia la o chet
            if (n1 in boxes and (n1 in seen or frozen(n1, boxes, seen))) or \ (n2 in boxes and (n2 in seen or frozen(n2, boxes, seen))):
                continue                                  # thung ke ben bi ket
            return False                                  # truc nay con di duoc
        return True

    def deadlock(t, boxes):
        # sau khi day thung toi t: co thung nao NGOAI dich bi ket vinh vien khong
        for c in ((t[0], t[1]), (t[0] + 1, t[1]), (t[0] - 1, t[1]),
                  (t[0], t[1] + 1), (t[0], t[1] - 1)):
            if c in boxes and c not in sto and frozen(c, boxes, frozenset()):
                return True
        return False

    if any(b not in alive for b in init.boxes):
        return False                     # co thung o o chet tu dau: vo nghiem

    # BFS di bo cua 1 robot (thung, tuong, robot khac la vat can)
    def bfs(start, blocked):
        prev = {start: None}
        q = deque([start])
        while q:
            c = q.popleft()
            for name, dx, dy in DIRS:
                n = (c[0] + dx, c[1] + dy)
                if n not in prev and free(n) and n not in blocked:
                    prev[n] = (c, name)
                    q.append(n)
        return prev

    def walk(prev, cell):
        out = []
        while prev[cell] is not None:
            cell, name = prev[cell]
            out.append(name)
        out.reverse()
        return out

    # A* tren cac cu day; ub = can tren (tong buoc cua loi giai tot nhat da co)
    def astar(w, ub, deadline):
        tie = count()
        r0, b0 = init.robots, init.boxes
        root = (r0, b0, None, ())                 # (robots, boxes, node_cha, nuoc_di)
        heap = [(w * h(b0), 0, next(tie), 0, root)]
        best = {(tuple(sorted(r0)), b0): 0}
        while heap:
            if time.time() >= deadline:
                return None
            _, _, _, g, node = heapq.heappop(heap)
            robots, boxes = node[0], node[1]
            if g > best[(tuple(sorted(robots)), boxes)]:
                continue
            if all(b in sto for b in boxes):
                return g, node
            rset = set(robots)
            reach = [bfs(r, boxes | obs | (rset - {r})) for r in robots]
            for b in boxes:
                for name, dx, dy in DIRS:
                    t = (b[0] + dx, b[1] + dy)
                    if t not in alive or t in boxes or t in rset:
                        continue
                    p = (b[0] - dx, b[1] - dy)    # cho robot phai dung de day
                    nb = (boxes - {b}) | {t}
                    if deadlock(t, nb):
                        continue
                    hv = h(nb)
                    for i in range(len(robots)):
                        if p not in reach[i]:
                            continue
                        steps = walk(reach[i], p)
                        ng = g + len(steps) + 1
                        if ng + hv >= ub:         # khong the tot hon loi giai da co
                            continue
                        nr = robots[:i] + (b,) + robots[i + 1:]
                        k = (tuple(sorted(nr)), nb)
                        if ng < best.get(k, INF):
                            best[k] = ng
                            mv = tuple((i, s) for s in steps) + ((i, name),)
                            heapq.heappush(heap, (ng + w * hv, hv, next(tie), ng,
                                                  (nr, nb, node, mv)))
        return None

    # luot 1: W lon de co loi giai nhanh (phai xong trong timebound)
    t0 = time.time()
    found = astar(FAST_W, INF, end)
    if found is None:
        return False
    best_g, best_node = found
    # cac luot sau: giam W de rut ngan loi giai. Bai de (luot dau xong nhanh) -> tinh chinh ngan
    budget = min(REFINE, max(0.2, 20 * (time.time() - t0)))
    refine_end = min(end, time.time() + budget)
    for w in REFINE_W:
        res = astar(w, best_g, refine_end)
        if res is not None and res[0] < best_g:
            best_g, best_node = res
        if time.time() >= refine_end:
            break
    return replay(init, best_node)


def replay(init, node):
    """Dung lai chuoi nuoc di qua successors() cua giao vien -> parent/action/gval that."""
    chain = []
    while node is not None:
        chain.append(node[3])
        node = node[2]
    cur = init
    for mv in reversed(chain):
        for i, name in mv:
            want = "%d %s" % (i, name)
            cur = next(s for s in cur.successors() if s.action == want)
    return cur