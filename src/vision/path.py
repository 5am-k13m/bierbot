import numpy as np
import heapq
import cv2

class AStarPlanner:
    def __init__(self, img, rectangles,
                 grid_step=4,
                 influence_radius=20,
                 penalty_weight=30):
        """
        img: cluster visualization image (H,W,3)
        rectangles: list of rectangles in IMAGE PIXELS (Nx4x2)
        """

        self.img = img
        self.rectangles = rectangles
        self.grid_step = grid_step
        self.influence_radius = influence_radius
        self.penalty_weight = penalty_weight

        self.h, self.w = img.shape[:2]
        self.cost_map = self._build_cost_map()

    def point_in_rect(self, p, rect):
        A, B, C, D = rect
        AB = B - A
        AD = D - A
        AP = p - A

        return (
            0 <= np.dot(AP, AB) <= np.dot(AB, AB) and
            0 <= np.dot(AP, AD) <= np.dot(AD, AD)
        )
    
    def _build_cost_map(self):
        cost = np.zeros((self.h, self.w), dtype=np.float32)

        # Hard obstacles
        obstacle_mask = np.zeros((self.h, self.w), dtype=np.uint8)
        for rect in self.rectangles:
            cv2.fillPoly(obstacle_mask, [rect.astype(np.int32)], 1)

        cost[obstacle_mask == 1] = np.inf

        # Soft repulsion using distance transform
        dist = cv2.distanceTransform(
            (1 - obstacle_mask).astype(np.uint8),
            cv2.DIST_L2,
            5
        )

        influence = dist < self.influence_radius
        cost[influence] += self.penalty_weight * (
            1 - dist[influence] / self.influence_radius
        ) ** 2

        return cost
    
    def _neighbors(self, p):
        for dx, dy in [
            (1,0), (-1,0), (0,1), (0,-1),
            (1,1), (1,-1), (-1,1), (-1,-1)
        ]:
            q = p[0] + dx*self.grid_step, p[1] + dy*self.grid_step
            if 0 <= q[0] < self.w and 0 <= q[1] < self.h:
                yield q

    def find_path(self, start, goal):
        start = tuple(map(int, start))
        goal = tuple(map(int, goal))

        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if heuristic(current, goal) < self.grid_step:
                return self._reconstruct(came_from, current)

            for nb in self._neighbors(current):
                if np.isinf(self.cost_map[nb[1], nb[0]]):
                    continue

                tentative = g[current] + heuristic(current, nb) + \
                            self.cost_map[nb[1], nb[0]]

                if nb not in g or tentative < g[nb]:
                    came_from[nb] = current
                    g[nb] = tentative
                    f = tentative + heuristic(nb, goal)
                    heapq.heappush(open_set, (f, nb))

        return None

    def _reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
    
    def draw_path(self, img, path, color=(0,0,255)):
        for p in path:
            cv2.circle(img, p, 1, color, -1)

