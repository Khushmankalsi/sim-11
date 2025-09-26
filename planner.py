import heapq
import math

class AStarPlanner:
    def __init__(self, grid_size=20, robot_radius=18):
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
        # Inflation radius is 1 cell (20px)
        inflation_radius = 1 
        
        for ob in obstacles:
            cx, cy = self._gridify(ob['x'], ob['y'])
            
            # Inflate the obstacle by 1 cell radius
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
            
        # --- START OF FIX: 8-way Movement with Diagonal Cost ---
        # (dx, dy, cost)
        motion = [
            (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1), # Cardinal moves (cost 1)
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414) # Diagonal moves (cost sqrt(2))
        ]
        # --- END OF FIX ---

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == (gx, gy):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((sx, sy))
                path.reverse()
                
                # Convert grid path back to canvas coordinates (center of grid cell)
                return [(px * self.grid_size + self.grid_size // 2, 
                         py * self.grid_size + self.grid_size // 2) for (px, py) in path]

            for dx, dy, cost in motion:
                nx, ny = current[0]+dx, current[1]+dy
                
                # Check bounds
                if nx < 0 or ny < 0 or nx >= canvas_w//self.grid_size or ny >= canvas_h//self.grid_size:
                    continue
                
                # Check for obstacle
                if (nx, ny) in ox:
                    continue

                tentative_g = g_score[current] + cost # Use correct cost (1 or 1.414)
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    f_score = tentative_g + self._heuristic((nx, ny), (gx, gy))
                    heapq.heappush(open_set, (f_score, (nx, ny)))
                    came_from[(nx, ny)] = current

        return [] # Path not found