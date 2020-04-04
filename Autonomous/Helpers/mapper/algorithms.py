import collections
import heapq
import numpy as np
import math
from functools import partial
import sys
from sympy import symbols, Eq, solve


class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return id not in self.walls

    def neighbors(self, id):
        (x, y) = id
        results = [(x-1, y+1), (x, y+1), (x+1, y+1),
                   (x+1, y), (x+1, y-1), (x, y-1), (x-1, y-1), (x-1, y),
                   (x-1, y+2), (x+1, y+2), (x+2, y+1), (x+2, y-1), (x+1, y-2), (x-1, y-2), (x-2, y-1), (x-2, y+1)]
        #results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0:
            pass #results.reverse()  # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results


class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        super().__init__(width, height)
        self.weights = {}

    def cost(self, from_node, to_node, cost_so_far, i):
        #return self.weights.get(to_node, 1)
        Co = cost_so_far.get(from_node, 1)
        Ci = cost_so_far.get(to_node, 1)
        weight = 1
        hd = self.weights.get(to_node, 1) - self.weights.get(from_node, 1)
        if i < 8:
            if i % 2 == 0:
                a = math.atan(hd/math.sqrt(2))
                x = math.sqrt(2 + hd**2) * ((Co + Ci)/2 + a*weight)
            else:
                a = math.atan(hd)
                x = math.sqrt(1 + hd**2) * ((Co + Ci)/2 + a*weight)
        else:
            n = list(super().neighbors(from_node))
            C1 = cost_so_far.get(n[i-8], 1)
            if i == 15:
                C2 = cost_so_far.get(n[0], 1)
            else:
                C2 = cost_so_far.get(n[i-7], 1)

            a = math.atan(hd/math.sqrt(5))
            x = math.sqrt(5 + hd**2) * ((Co + C1 + C2 + Ci) / 4 + a*weight)
        if x < 0:
            exit(34)
        return x


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for i, next in enumerate(graph.neighbors(current)):
            new_cost = cost_so_far[current] + \
                graph.cost(current, next, cost_so_far, i)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)  # optional
    path.reverse()  # optional
    return path


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for i, next in enumerate(graph.neighbors(current)):
            new_cost = cost_so_far[current] + \
                graph.cost(current, next, cost_so_far, i)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


if sys.version_info[0] >= 3:
    xrange = range


def pldist(point, start, end):
    if np.all(np.equal(start, end)):
        return np.linalg.norm(point - start)

    return np.divide(
        np.abs(np.linalg.norm(np.cross(end - start, start - point))),
        np.linalg.norm(end - start))


def rdp_rec(M, epsilon, dist=pldist):
    dmax = 0.0
    index = -1

    for i in xrange(1, M.shape[0]):
        d = dist(M[i], M[0], M[-1])

        if d > dmax:
            index = i
            dmax = d

    if dmax > epsilon:
        r1 = rdp_rec(M[:index + 1], epsilon, dist)
        r2 = rdp_rec(M[index:], epsilon, dist)

        return np.vstack((r1[:-1], r2))
    else:
        return np.vstack((M[0], M[-1]))


def _rdp_iter(M, start_index, last_index, epsilon, dist=pldist):
    stk = []
    stk.append([start_index, last_index])
    global_start_index = start_index
    indices = np.ones(last_index - start_index + 1, dtype=bool)

    while stk:
        start_index, last_index = stk.pop()

        dmax = 0.0
        index = start_index

        for i in xrange(index + 1, last_index):
            if indices[i - global_start_index]:
                d = dist(M[i], M[start_index], M[last_index])
                if d > dmax:
                    index = i
                    dmax = d

        if dmax > epsilon:
            stk.append([start_index, index])
            stk.append([index, last_index])
        else:
            for i in xrange(start_index + 1, last_index):
                indices[i - global_start_index] = False

    return indices


def rdp_iter(M, epsilon, dist=pldist, return_mask=False):
    mask = _rdp_iter(M, 0, len(M) - 1, epsilon, dist)

    if return_mask:
        return mask

    return M[mask]


def rdp(M, epsilon=0, dist=pldist, algo="iter", return_mask=False):
    if algo == "iter":
        algo = partial(rdp_iter, return_mask=return_mask)
    elif algo == "rec":
        if return_mask:
            raise NotImplementedError(
                "return_mask=True not supported with algo=\"rec\"")
        algo = rdp_rec

    if "numpy" in str(type(M)):
        return algo(M, epsilon, dist)

    return algo(np.array(M), epsilon, dist).tolist()


def calculate_position(landmarks, markers, prev_position, bypass=False):
    markers.sort(key=lambda x: x[1])
    A = list(filter(lambda x: x[0] == markers[0][0], landmarks))[0]
    B = list(filter(lambda x: x[0] == markers[1][0], landmarks))[0]
    AC = markers[0][1]
    BC = markers[1][1]
    x, y = symbols('x y', real=True)
    Cx, Cy = None, None

    eq1 = Eq((((A[1] - x)**2 + (A[2] - y)**2))-AC**2)
    eq2 = Eq((((B[1] - x)**2 + (B[2] - y)**2))-BC**2)
    sol1 = solve((eq1, eq2), (x, y), dict=True)

    if len(markers) > 2 and not bypass:
        D = list(filter(lambda x: x[0] == markers[2][0], landmarks))[0]
        DC = markers[2][1]

        eq3 = Eq((((D[1] - x)**2 + (D[2] - y)**2))-DC**2)
        sol2 = solve((eq2, eq3), (x, y), dict=True)
        
        for i in range(len(sol1)):
            ux, uy = sol1[i][x], sol1[i][y]
            for ii in range(len(sol2)):
                ix, iy = sol2[ii][x], sol2[ii][y]
                if (abs(ix-ux) < max([ix, ux]) * 0.5) and (abs(iy-uy) < max([ix, ux]) * 0.5):
                    d = np.array([[ix, ux], [iy, uy]])
                    Cx, Cy = np.average(d, axis=1)
                    break

        if Cx is None:
            return calculate_position(landmarks, markers, prev_position, True)

        return (Cx, Cy, markers[0])
    elif len(markers) > 1:
        distances = []
        ix, iy = prev_position
        for i in range(len(sol1)):
            ux, uy = sol1[i][x], sol1[i][y]
            distances.append(math.sqrt((ux-ix)**2 + (uy-iy)**2))

        Cx, Cy = sol1[distances.index(min(distances))][x], sol1[distances.index(min(distances))][y]

        if Cx is None:
            (Cx, Cy, markers[0]) = -1, -1, -1

        return (Cx, Cy, markers[0])
    else:
        return (-1, -1, -1)


def calculate_route(referance, waypoint, position, landmarks):
    A = waypoint
    B = list(filter(lambda x: x[0] == referance[0], landmarks))[0]
    (Cx, Cy) = position
    AB = math.sqrt(math.pow(A[1] - B[1], 2) + math.pow(A[2] - B[2], 2))
    AC = math.sqrt(math.pow(A[1] - Cx, 2) + math.pow(A[2] - Cy, 2))
    BC = referance[1]
    m = (B[2] - Cy) / (B[1] - Cx)
    b = Cy - (m * Cx)
    DISTANCE = AC

    min_angle = math.degrees(
        math.acos((math.pow(AC, 2) + math.pow(BC, 2) - math.pow(AB, 2))/(2*BC*AC)))
    if A[2] > ((m * A[1]) + b):
        if referance[2] > 0:
            ANGLE = (referance[2] - min_angle) % 360
        else:
            ANGLE = ((referance[2] % 360) + min_angle) % 360
    else:
        if referance[2] > 0:
            ANGLE = (referance[2] + min_angle) % 360
        else:
            ANGLE = ((referance[2] % 360) - min_angle) % 360

    return DISTANCE, -ANGLE % 360


def simplify_route(self, path, scale=1):
    simple_path = []
    for i in range(len(path)):
        M = [[(path[i][0] - scale, path[i][1] + scale), (path[i][0], path[i][1] + scale), (path[i][0] + scale, path[i][1] + scale)],
             [(path[i][0] - scale, path[i][1]), (path[i][0],
                                                 path[i][1]), (path[i][0] + scale, path[i][1])],
             [(path[i][0] - scale, path[i][1] - scale), (path[i][0], path[i][1] - scale), (path[i][0] + scale, path[i][1] - scale)]]

        # Convert matrix to binary
        total_found = 0
        in_sp = False
        for x in range(3):
            for y in range(3):
                if not in_sp:
                    in_sp = len([(xi, yi) for (
                        xi, yi) in simple_path if xi == M[x][y][0] and yi == M[x][y][1]]) > 0
                M[x][y] = len(
                    [(xi, yi) for (xi, yi) in path if xi == M[x][y][0] and yi == M[x][y][1]])
                total_found = total_found + M[x][y]

        if total_found < 3:
            simple_path.append((path[i][0], path[i][1]))
            continue
        if sum(M[1]) == 3 or sum(np.transpose(M)[1]) == 3:
            continue
        if in_sp:
            continue
        if total_found == 3 and (sum(M[1]) == 2 or sum(np.transpose(M)[1]) == 2):
            simple_path.append((path[i][0], path[i][1]))
        if total_found == 4:
            simple_path.append((path[i][0], path[i][1]))
    return simple_path
