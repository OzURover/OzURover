import numpy as np
from helpers.mapper.algorithms import (GridWithWeights, a_star_search,
                                       dijkstra_search, rdp, reconstruct_path)


class OptimalPath:

    def __init__(self):
        self.map = None

    def status(self):
        return self.map is not None

    def load_map(self, file, bound=3, scale=1):
        self.bound = bound
        topography = []
        z_min = 100
        self.x_max, self.y_max = 0, 0

        with open(file) as fp:
            fp.readline()
            line = fp.readline()
            while line:
                row = [float(x) for x in line.strip().split(",")]
                x, y, z = int(row[0]*scale), int(row[1]*scale), row[2]*scale

                if z < z_min:
                    z_min = z

                if x > self.x_max:
                    self.x_max = x

                if y > self.y_max:
                    self.y_max = y

                topography.append((x, y, z))
                line = fp.readline()

        self.map = GridWithWeights(self.x_max, self.y_max)
        self.map.walls = []
        for xi in range(self.x_max):
            for yi in range(self.y_max):
                if (xi < bound or xi > (self.x_max - bound)):
                    self.map.walls.append((xi, yi))
                elif (yi < bound or yi > (self.y_max - bound)):
                    self.map.walls.append((xi, yi))

        z_min = 0 if z_min > 0 else z_min
        self.map.weights = {(x, y): round(z + abs(z_min), 3)
                            for (x, y, z) in topography}
        return self.x_max, self.y_max, self.map

    def create_route(self, start, end, scale=1, astar=False):
        start, end = [start[0], start[1]], [end[0], end[1]]
        if self.map is None:
            raise Exception("Map file hasn't loaded!")

        if start[0] <= self.bound:
            start[0] = self.bound + 1
        if start[0] >= self.x_max - self.bound:
            start[0] = self.x_max - self.bound - 1
        if start[1] <= self.bound:
            start[1] = self.bound + 1
        if start[1] >= self.y_max - self.bound:
            start[1] = self.y_max - self.bound - 1

        if end[0] <= self.bound:
            end[0] = self.bound + 1
        if end[0] >= self.x_max - self.bound:
            end[0] = self.x_max - self.bound - 1
        if end[1] <= self.bound:
            end[1] = self.bound + 1
        if end[1] >= self.y_max - self.bound:
            end[1] = self.y_max - self.bound - 1

        start = (start[0], start[1])
        end = (end[0], end[1])

        if astar:
            path, _ = a_star_search(self.map, start, end)
        else:
            path, _ = dijkstra_search(self.map, start, end)
        path = reconstruct_path(path, start, end)
        path = [(x*scale, y*scale) for (x, y) in path]
        return path, rdp(path, epsilon=1.0)
