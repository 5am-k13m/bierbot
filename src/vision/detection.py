import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial import KDTree
from scipy.spatial import ConvexHull
import cv2
import time


# ------------------------------------------------------------
# Metrics
# ------------------------------------------------------------
def chamfer_distance(A, B):
    """
    A, B: (N, D) point clouds in the same dimensionality
    """
    tree = KDTree(B)
    d1 = tree.query(A)[0]

    tree = KDTree(A)
    d2 = tree.query(B)[0]

    return np.mean(d1) + np.mean(d2)


# ------------------------------------------------------------
# Cluster Detection + Tracking
# ------------------------------------------------------------
class ClusterDetection:
    def __init__(self,
                 eps=100,
                 min_samples=10,
                 match_radius=1000,
                 ema_alpha=0.25,
                 track_timeout=1.0):

        self.eps = eps
        self.min_samples = min_samples

        # 3D reference model (Nx3)
        self.roomba_pts = np.load("roomba_points.npy")

        # visualization bounds
        self.min_xy = 0
        self.max_xy = 0

        # persistent tracks
        self.tracks = {}
        self.next_track_id = 0

        self.match_radius = match_radius
        self.ema_alpha = ema_alpha
        self.track_timeout = track_timeout

        rng = np.random.default_rng(12345)
        self.colors = rng.integers(75, 175, size=(10000, 3))

        self.clusters = []


    # ------------------------------------------------------------
    # Clustering
    # ------------------------------------------------------------
    def detect_clusters(self, points_xy):

        if len(points_xy) == 0:
            self.clusters = []
            return [], np.array([])

        db = DBSCAN(
            eps=self.eps,
            min_samples=self.min_samples
        ).fit(points_xy)

        labels = db.labels_
        clusters = []

        for lbl in set(labels):
            if lbl == -1:
                continue

            mask = labels == lbl

            pts_xy = points_xy[mask]

            centroid_xy = pts_xy.mean(axis=0)

            rect = self.minimum_bounding_rectangle(pts_xy)

            # --- 3D Chamfer (centered) ---
            chamfer = chamfer_distance(
                pts_xy - centroid_xy,
                self.roomba_pts
            )

            hull = ConvexHull(pts_xy)
            hull_pts = pts_xy[hull.vertices]

            clusters.append({
                "points_xy": pts_xy,
                "centroid_xy": centroid_xy,
                "convex_hull": hull_pts,
                "rectangle": rect,
                "chamfer": chamfer
            })

        self.clusters = clusters
        return clusters, labels


    # ------------------------------------------------------------
    # Tracking + persistent scoring
    # ------------------------------------------------------------
    def update_tracks(self):
        now = time.time()
        used_tracks = set()

        for c in self.clusters:
            best_track = None
            best_dist = None

            for tid, t in self.tracks.items():
                d = np.linalg.norm(
                    c["centroid_xy"] - t["centroid_xy"]
                )
                if d < self.match_radius and (
                    best_dist is None or d < best_dist
                ):
                    best_dist = d
                    best_track = tid

            if best_track is None:
                tid = self.next_track_id
                self.next_track_id += 1
                self.tracks[tid] = {
                    "centroid_xy": c["centroid_xy"],
                    "score_ema": c["chamfer"],
                    "wins": 0,
                    "last_seen": now
                }
                c["track_id"] = tid
            else:
                t = self.tracks[best_track]
                t["centroid_xy"] = c["centroid_xy"]
                t["score_ema"] = (
                    self.ema_alpha * c["chamfer"]
                    + (1 - self.ema_alpha) * t["score_ema"]
                )
                t["last_seen"] = now
                c["track_id"] = best_track
                used_tracks.add(best_track)

        # prune dead tracks
        dead = [
            tid for tid, t in self.tracks.items()
            if now - t["last_seen"] > self.track_timeout
        ]
        for tid in dead:
            del self.tracks[tid]

        self.update_wins()


    def update_wins(self):
        if not self.tracks:
            return

        best = min(
            self.tracks.items(),
            key=lambda x: x[1]["score_ema"]
        )[0]
        self.tracks[best]["wins"] += 1


    # ------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------
    def minimum_bounding_rectangle(self, points):
        hull = ConvexHull(points)
        hull_pts = points[hull.vertices]

        min_area = np.inf
        best = None

        for i in range(len(hull_pts)):
            p1 = hull_pts[i]
            p2 = hull_pts[(i + 1) % len(hull_pts)]
            edge = p2 - p1
            angle = -np.arctan2(edge[1], edge[0])

            R = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle),  np.cos(angle)]
            ])

            rot = hull_pts @ R.T
            min_x, max_x = rot[:, 0].min(), rot[:, 0].max()
            min_y, max_y = rot[:, 1].min(), rot[:, 1].max()
            area = (max_x - min_x) * (max_y - min_y)

            if area < min_area:
                min_area = area
                best = (min_x, max_x, min_y, max_y, R)

        min_x, max_x, min_y, max_y, R = best
        rect = np.array([
            [max_x, min_y],
            [max_x, max_y],
            [min_x, max_y],
            [min_x, min_y]
        ])

        return rect @ R


    # ------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------
    def visualize_clusters(self, img_size=400, scale=0.025):
        floor_size = self.max_xy - self.min_xy
        canvas = int(max((floor_size * scale)) + 20)
        canvas = max(canvas, img_size)

        img = np.zeros((canvas, canvas, 3), dtype=np.uint8)

        for c in self.clusters:
            tid = c["track_id"]
            t = self.tracks[tid]

            color = tuple(map(int, self.colors[tid]))

            pts = ((c["points_xy"] - self.min_xy) * scale).astype(int)
            for x, y in pts:
                if 0 <= x < canvas and 0 <= y < canvas:
                    img[y, x] = color

            # --- Draw cluster convex hull ---
            hull = c["convex_hull"]  # (4,2) in world coords
            hull_img = ((hull - self.min_xy) * scale).astype(int)

            # Ensure shape is correct for OpenCV
            hull_img = hull_img.reshape((-1, 1, 2))

            cv2.polylines(
                img,
                [hull_img],
                isClosed=True,
                color=color,
                thickness=2
            )

            cx, cy = ((c["centroid_xy"] - self.min_xy) * scale).astype(int)
            if 0 <= cx < canvas and 0 <= cy < canvas:
                txt = f"{tid}  s={t['score_ema']:.2f}  w={t['wins']}"
                cv2.putText(
                    img, txt, (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2
                )

        return cv2.resize(
            img, (img_size, img_size),
            interpolation=cv2.INTER_NEAREST
        )
