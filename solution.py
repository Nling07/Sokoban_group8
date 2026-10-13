"""A* solver for the supplied Sokoban state-space framework."""

import heapq
import itertools
import time

from sokoban import (DOWN, LEFT, RIGHT, UP, SokobanState,
                     sokoban_goal_state)


def _minimum_assignment_cost(costs):
    """Return the minimum rectangular assignment cost, or None if impossible."""
    row_count = len(costs)
    if row_count == 0:
        return 0
    column_count = len(costs[0])
    if row_count > column_count:
        return None

    infinity = 10**9
    row_potential = [0] * (row_count + 1)
    column_potential = [0] * (column_count + 1)
    matched_row = [0] * (column_count + 1)
    previous_column = [0] * (column_count + 1)

    for row in range(1, row_count + 1):
        matched_row[0] = row
        current_column = 0
        minimum_slack = [infinity] * (column_count + 1)
        used = [False] * (column_count + 1)

        while True:
            used[current_column] = True
            current_row = matched_row[current_column]
            delta = infinity
            next_column = 0

            for column in range(1, column_count + 1):
                if used[column]:
                    continue
                reduced_cost = (costs[current_row - 1][column - 1]
                                - row_potential[current_row]
                                - column_potential[column])
                if reduced_cost < minimum_slack[column]:
                    minimum_slack[column] = reduced_cost
                    previous_column[column] = current_column
                if minimum_slack[column] < delta:
                    delta = minimum_slack[column]
                    next_column = column

            if delta >= infinity:
                return None

            for column in range(column_count + 1):
                if used[column]:
                    row_potential[matched_row[column]] += delta
                    column_potential[column] -= delta
                else:
                    minimum_slack[column] -= delta

            current_column = next_column
            if matched_row[current_column] == 0:
                break

        while True:
            next_column = previous_column[current_column]
            matched_row[current_column] = matched_row[next_column]
            current_column = next_column
            if current_column == 0:
                break

    total_cost = 0
    for column in range(1, column_count + 1):
        if matched_row[column]:
            value = costs[matched_row[column] - 1][column - 1]
            if value >= infinity:
                return None
            total_cost += value
    return total_cost


def solve(initial_state, timebound=120, _scan_all_boxes=False):
    """Solve with A* over pushes; walking between pushes is a macro edge."""
    try:
        budget = float(timebound)
    except (TypeError, ValueError):
        return False
    if not budget > 0:
        return False

    deadline = time.monotonic() + budget
    if sokoban_goal_state(initial_state):
        return initial_state

    width = initial_state.width
    height = initial_state.height
    obstacles = initial_state.obstacles
    storage = tuple(initial_state.storage)
    initial_boxes = tuple(sorted(initial_state.boxes))
    if len(initial_boxes) > len(storage):
        return False

    def in_bounds(point):
        return 0 <= point[0] < width and 0 <= point[1] < height

    directions = ((0, -1, UP), (1, 0, RIGHT),
                   (0, 1, DOWN), (-1, 0, LEFT))

    reverse_push_distances = []
    for goal in storage:
        if time.monotonic() >= deadline:
            return False
        distances = {goal: 0}
        frontier = [goal]
        head = 0
        while head < len(frontier):
            box_position = frontier[head]
            head += 1
            next_distance = distances[box_position] + 1
            for dx, dy, _ in directions:
                previous_box = (box_position[0] - dx, box_position[1] - dy)
                robot_stance = (box_position[0] - 2 * dx,
                                box_position[1] - 2 * dy)
                if (in_bounds(previous_box) and in_bounds(robot_stance)
                        and previous_box not in obstacles
                        and robot_stance not in obstacles
                        and previous_box not in distances):
                    distances[previous_box] = next_distance
                    frontier.append(previous_box)
        reverse_push_distances.append(distances)

    # A box square is live only if at least one goal can be reached by
    # reverse-pushing through static floor. Every other square is a dead square.
    live_squares = set().union(*(set(item) for item in reverse_push_distances))
    if any(box not in live_squares for box in initial_boxes):
        return False

    heuristic_cache = {}
    approach_cache = {}

    def heuristic(robots, boxes):
        if boxes.issubset(storage):
            return 0
        boxes = tuple(sorted(boxes))
        cached = heuristic_cache.get(boxes)
        if cached is False:
            return None
        if cached is None:
            if not boxes:
                push_cost = 0
                stances = ()
            else:
                costs = []
                for box in boxes:
                    costs.append([
                        reverse_push_distances[index].get(box, 10**9)
                        for index in range(len(storage))
                    ])
                push_cost = _minimum_assignment_cost(costs)
                if push_cost is None:
                    heuristic_cache[boxes] = False
                    return None

                possible_stances = set()
                for box in boxes:
                    for dx, dy, _ in directions:
                        stance = (box[0] - dx, box[1] - dy)
                        destination = (box[0] + dx, box[1] + dy)
                        if (in_bounds(stance) and in_bounds(destination)
                                and stance not in obstacles
                                and destination not in obstacles
                                and destination not in boxes
                                and destination in live_squares):
                            possible_stances.add(stance)
                stances = tuple(possible_stances)
            cached = (push_cost, stances)
            heuristic_cache[boxes] = cached

        if cached is None:
            return None
        push_cost, stances = cached
        if not stances:
            return None

        # Before the first push, boxes cannot move. BFS therefore treats the
        # current boxes as obstacles. Other robots are ignored, which keeps
        # this an optimistic lower bound if they must move out of the way.
        approach_key = (tuple(robots), boxes)
        approach_cost = approach_cache.get(approach_key)
        if approach_cost is None:
            blocked = obstacles.union(boxes)
            approach_cost = None
            for robot in robots:
                distances = {robot: 0}
                frontier = [robot]
                head = 0
                while head < len(frontier):
                    point = frontier[head]
                    head += 1
                    distance = distances[point]
                    if point in stances:
                        approach_cost = (distance if approach_cost is None
                                         else min(approach_cost, distance))
                        break
                    if (approach_cost is not None
                            and distance >= approach_cost):
                        break
                    for dx, dy, _ in directions:
                        neighbor = (point[0] + dx, point[1] + dy)
                        if (in_bounds(neighbor) and neighbor not in blocked
                                and neighbor not in distances):
                            distances[neighbor] = distance + 1
                            frontier.append(neighbor)
            approach_cache[approach_key] = approach_cost
        if approach_cost is None:
            return None
        return push_cost + approach_cost

    initial_robots = tuple(initial_state.robots)
    initial_boxes_set = frozenset(initial_boxes)
    initial_h = heuristic(initial_robots, initial_boxes_set)
    if initial_h is None or time.monotonic() >= deadline:
        return False

    def state_key(robots, boxes):
        # Robot tuple order is an API detail; the robots have identical rules.
        return (tuple(sorted(robots)), tuple(sorted(boxes)))

    def has_local_2x2_deadlock(boxes, affected):
        """Detect a solid 2x2 of walls/boxes containing an off-goal box."""
        occupied = obstacles.union(boxes)
        for ox in (affected[0] - 1, affected[0]):
            for oy in (affected[1] - 1, affected[1]):
                cells = ((ox, oy), (ox + 1, oy),
                         (ox, oy + 1), (ox + 1, oy + 1))
                blocked = all(
                    not in_bounds(cell) or cell in occupied for cell in cells
                )
                if blocked and any(
                        cell in boxes and cell not in storage for cell in cells):
                    return True
        return False

    def has_incremental_deadlock(boxes, changed_box):
        # The previous variant checked only the box moved by the last push.
        # This experiment also rescans each remaining box and its neighboring
        # 2x2 windows after every push, as requested.
        points = boxes if _scan_all_boxes else (changed_box,)
        return any(has_local_2x2_deadlock(boxes, point) for point in points)

    freeze_deadlock_cache = {}

    def has_freeze_deadlock(boxes):
        """Return True only for boxes proven immovable on both board axes."""
        cached = freeze_deadlock_cache.get(boxes)
        if cached is not None:
            return cached

        # Grow a set of permanently frozen boxes. Treating an unproven box as
        # movable makes this detector conservative: it may miss a deadlock,
        # but it will not reject a state just because a box is temporarily in
        # the way.
        frozen = set()

        def blocked_by_fixed(point):
            return (not in_bounds(point) or point in obstacles
                    or point in frozen)

        changed = True
        while changed:
            changed = False
            for box in boxes:
                if box in frozen:
                    continue

                def axis_is_blocked(dx, dy):
                    for sign in (-1, 1):
                        destination = (box[0] + sign * dx,
                                       box[1] + sign * dy)
                        stance = (box[0] - sign * dx,
                                  box[1] - sign * dy)
                        if not (blocked_by_fixed(destination)
                                or blocked_by_fixed(stance)):
                            return False
                    return True

                if axis_is_blocked(1, 0) and axis_is_blocked(0, 1):
                    frozen.add(box)
                    changed = True

        deadlocked = any(box not in storage for box in frozen)
        freeze_deadlock_cache[boxes] = deadlocked
        return deadlocked

    if any(has_local_2x2_deadlock(initial_boxes_set, box)
           for box in initial_boxes_set):
        return False
    if has_freeze_deadlock(initial_boxes_set):
        return False

    def make_solution(goal_node):
        macro_segments = []
        node = goal_node
        while node[3] is not None:
            macro_segments.append(node[4])
            node = node[3]
        current = initial_state
        for robot_start, direction_indices in reversed(macro_segments):
            try:
                robot_index = current.robots.index(robot_start)
            except ValueError:
                return False
            for direction_index in direction_indices:
                if time.monotonic() >= deadline:
                    return False
                dx, dy, direction = directions[direction_index]
                old_robots = current.robots
                old_boxes = current.boxes
                robot_position = old_robots[robot_index]
                next_position = (robot_position[0] + dx,
                                 robot_position[1] + dy)
                next_boxes = set(old_boxes)
                if next_position in next_boxes:
                    box_destination = (next_position[0] + dx,
                                       next_position[1] + dy)
                    next_boxes.remove(next_position)
                    next_boxes.add(box_destination)
                next_robots = list(old_robots)
                next_robots[robot_index] = next_position
                current = SokobanState(
                    str(robot_index) + " " + direction.name,
                    current.gval + 1,
                    current,
                    width,
                    height,
                    tuple(next_robots),
                    frozenset(next_boxes),
                    current.storage,
                    current.obstacles,
                )
        return current

    # A node is (robots, boxes, g, parent-node, primitive-action segment).
    # Search with canonical robot ordering to avoid symmetric duplicate states.
    initial_robots = tuple(sorted(initial_robots))
    root = (initial_robots, initial_boxes_set, initial_state.gval, None, ())
    sequence = itertools.count()
    # Weighted A* spends less time proving optimality and more time following
    # promising box configurations. The returned path remains legal, but may
    # be longer than the optimal path.
    heuristic_weight = 3.0
    frontier = [(initial_state.gval + heuristic_weight * initial_h, initial_h,
                 next(sequence), root)]
    best_cost = {state_key(initial_robots, initial_boxes_set): initial_state.gval}

    while frontier:
        if time.monotonic() >= deadline:
            return False
        _, _, _, node = heapq.heappop(frontier)
        robots, boxes, cost, _, _ = node
        key = state_key(robots, boxes)
        if cost != best_cost.get(key):
            continue
        if boxes.issubset(storage):
            return make_solution(node)

        blocked_boxes = obstacles.union(boxes)
        reachable_by_robot = []
        for robot_index, start in enumerate(robots):
            if time.monotonic() >= deadline:
                return False
            blocked = blocked_boxes.union(
                position for index, position in enumerate(robots)
                if index != robot_index
            )
            paths = {start: ()}
            queue = [start]
            head = 0
            while head < len(queue):
                if time.monotonic() >= deadline:
                    return False
                point = queue[head]
                head += 1
                for direction_index, (dx, dy, _) in enumerate(directions):
                    neighbor = (point[0] + dx, point[1] + dy)
                    if (in_bounds(neighbor) and neighbor not in blocked
                            and neighbor not in paths):
                        paths[neighbor] = paths[point] + (direction_index,)
                        queue.append(neighbor)
            reachable_by_robot.append(paths)

        candidates = []
        mobility = {box: 0 for box in boxes}
        min_goal_distance = {
            box: min(distances.get(box, 10**9)
                     for distances in reverse_push_distances)
            for box in boxes
        }
        for robot_index, paths in enumerate(reachable_by_robot):
            other_robots = {position for index, position in enumerate(robots)
                            if index != robot_index}
            for box in boxes:
                for direction_index, (dx, dy, _) in enumerate(directions):
                    stance = (box[0] - dx, box[1] - dy)
                    destination = (box[0] + dx, box[1] + dy)
                    if (stance not in paths or not in_bounds(destination)
                            or destination in obstacles
                            or destination in boxes
                            or destination in other_robots
                            or destination not in live_squares):
                        continue
                    mobility[box] += 1
                    candidates.append((
                        box, destination, robot_index, direction_index,
                        robots[robot_index],
                        paths[stance] + (direction_index,),
                    ))

        # Try goal-reaching pushes and flexible/near-goal boxes first when A*
        # has equal f/h priorities. Search completeness is unchanged.
        candidates.sort(key=lambda item: (
            item[1] not in storage,
            -mobility[item[0]],
            min_goal_distance[item[0]],
        ))

        for (box, destination, robot_index, _, robot_start,
             direction_indices) in candidates:
            if time.monotonic() >= deadline:
                return False
            next_boxes = set(boxes)
            next_boxes.remove(box)
            next_boxes.add(destination)
            next_boxes = frozenset(next_boxes)
            if (has_incremental_deadlock(next_boxes, destination)
                    or has_freeze_deadlock(next_boxes)):
                continue
            next_robots = list(robots)
            next_robots[robot_index] = box
            next_robots = tuple(sorted(next_robots))
            segment = (robot_start, direction_indices)
            next_cost = cost + len(direction_indices)
            next_key = state_key(next_robots, next_boxes)
            if next_cost >= best_cost.get(next_key, float("inf")):
                continue

            estimate = heuristic(next_robots, next_boxes)
            if estimate is None:
                continue
            best_cost[next_key] = next_cost
            next_node = (next_robots, next_boxes, next_cost, node, segment)
            heapq.heappush(frontier, (
                                      next_cost + heuristic_weight * estimate,
                                      estimate,
                                      next(sequence), next_node))

    return False
