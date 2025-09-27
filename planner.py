import heapq
import math

class AStarPlanner:
    
    def smooth_path(self, path, obstacles):
        """
        Takes the raw A* path and returns a smoothed path with fewer waypoints.
        Uses line-of-sight optimization to skip intermediate points.
        """
        if not path or len(path) < 3:
            return path

        def line_clear(p1, p2):
            """Check if straight line from p1 to p2 intersects any obstacle"""
            for obs in obstacles:
                # Treat obstacle as a circle with radius = size/2 + safety margin
                radius = obs.get('size', 20)/2 + self.robot_radius + 5  # Added 5px safety margin
                cx, cy = obs['x'], obs['y']

                x1, y1 = p1
                x2, y2 = p2

                # Calculate distance from line segment to circle center
                dx, dy = x2 - x1, y2 - y1
                if dx == 0 and dy == 0:
                    dist = math.hypot(cx - x1, cy - y1)
                    if dist <= radius:
                        return False
                    continue

                t = max(0, min(1, ((cx - x1) * dx + (cy - y1) * dy) / (dx*dx + dy*dy)))
                nearest_x = x1 + t * dx
                nearest_y = y1 + t * dy
                dist = math.hypot(nearest_x - cx, nearest_y - cy)

                if dist <= radius:
                    return False
            return True

        # Greedy smoothing: try to skip as many waypoints as possible
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            # Try to reach the farthest visible point
            j = len(path) - 1
            while j > i + 1:
                if line_clear(path[i], path[j]):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j

        return smoothed

    def adaptive_smooth_path(self, path, obstacles):
        """
        Advanced smoothing that creates larger steps when no obstacles are nearby.
        """
        if not path or len(path) < 2:
            return path

        def has_nearby_obstacles(point, radius=100):
            """Check if there are obstacles within radius of the point"""
            x, y = point
            for obs in obstacles:
                dist = math.hypot(obs['x'] - x, obs['y'] - y)
                if dist < radius:
                    return True
            return False

        def line_clear_extended(p1, p2):
            """More generous line clearing for open areas"""
            for obs in obstacles:
                radius = obs.get('size', 20)/2 + self.robot_radius
                cx, cy = obs['x'], obs['y']
                x1, y1 = p1
                x2, y2 = p2

                dx, dy = x2 - x1, y2 - y1
                if dx == 0 and dy == 0:
                    continue

                t = max(0, min(1, ((cx - x1) * dx + (cy - y1) * dy) / (dx*dx + dy*dy)))
                nearest_x = x1 + t * dx
                nearest_y = y1 + t * dy
                dist = math.hypot(nearest_x - cx, nearest_y - cy)

                if dist <= radius:
                    return False
            return True

        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            current_point = path[i]
            
            # If we're in an open area, try to make bigger jumps
            if not has_nearby_obstacles(current_point, radius=80):
                # Try to skip more waypoints in open areas
                max_skip = min(len(path) - 1, i + 8)  # Skip up to 8 waypoints
            else:
                # Be more conservative near obstacles
                max_skip = min(len(path) - 1, i + 3)  # Skip up to 3 waypoints
            
            j = max_skip
            while j > i + 1:
                if line_clear_extended(path[i], path[j]):
                    break
                j -= 1
            
            smoothed.append(path[j])
            i = j

        return smoothed

    def __init__(self, grid_size=50, robot_radius=18):  # Increased default grid_size
        self.grid_size = grid_size
        self.robot_radius = robot_radius 

    def _gridify(self, x, y):
        return (int(x // self.grid_size), int(y // self.grid_size))

    def _heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan(self, start, goal, obstacles, canvas_w=650, canvas_h=600):
        sx, sy = self._gridify(*start)
        gx, gy = self._gridify(*goal)

        ox = set()
        # Reduced inflation radius for larger grid
        inflation_radius = 1
        
        for ob in obstacles:
            cx, cy = self._gridify(ob['x'], ob['y'])
            
            for dx in range(-inflation_radius, inflation_radius + 1):
                for dy in range(-inflation_radius, inflation_radius + 1):
                    nx, ny = cx + dx, cy + dy
                    
                    if 0 <= nx < canvas_w//self.grid_size and 0 <= ny < canvas_h//self.grid_size:
                        ox.add((nx, ny))

        open_set = [(0, (sx, sy))]
        came_from = {}
        g_score = { (sx, sy): 0 }
        
        if (sx, sy) in ox or (gx, gy) in ox:
            print(f"Start ({sx},{sy}) or Goal ({gx},{gy}) is blocked after inflation.")
            return []

        # 8-way movement with proper diagonal costs
        motion = [
            (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == (gx, gy):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((sx, sy))
                path.reverse()
                
                # Convert grid path back to canvas coordinates
                canvas_path = [(px * self.grid_size + self.grid_size // 2, 
                               py * self.grid_size + self.grid_size // 2) for (px, py) in path]
                
                # Apply adaptive smoothing
                return self.adaptive_smooth_path(canvas_path, obstacles)

            for dx, dy, cost in motion:
                nx, ny = current[0]+dx, current[1]+dy
                
                if nx < 0 or ny < 0 or nx >= canvas_w//self.grid_size or ny >= canvas_h//self.grid_size:
                    continue
                
                if (nx, ny) in ox:
                    continue

                tentative_g = g_score[current] + cost
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    f_score = tentative_g + self._heuristic((nx, ny), (gx, gy))
                    heapq.heappush(open_set, (f_score, (nx, ny)))
                    came_from[(nx, ny)] = current

        return []