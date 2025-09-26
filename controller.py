import asyncio
import websockets
import json
import requests
from planner import AStarPlanner

class RobotController:
    def __init__(self):
        self.robot_pos = (320, 300)
        self.goal = {"x": 550, "y": 80}
        # --- FIX: No hardcoded obstacles, will be fetched from server ---
        self.obstacles = []
        self.path = []
        self.ws = None
        self.planner = AStarPlanner(grid_size=20)
        self.server_url = "http://localhost:5001"
        self.is_collided = False
        self.goal_reached = False
        self.current_waypoint_index = 0

    async def _fetch_obstacles_from_server(self):
        """Fetches the current obstacle list from the server's API."""
        try:
            url = f"{self.server_url}/obstacles"
            r = requests.get(url)
            r.raise_for_status() # Raise an HTTPError for bad responses
            data = r.json()
            if 'obstacles' in data:
                self.obstacles = data['obstacles']
                print(f"Successfully fetched {len(self.obstacles)} obstacles from server.")
            else:
                print("Server response for obstacles was missing 'obstacles' key.")
                self.obstacles = []
        except requests.exceptions.RequestException as e:
            print(f"Failed to fetch obstacles from server: {e}")
            self.obstacles = []

    async def connect(self):
        uri = "ws://localhost:8080"
        async with websockets.connect(uri) as websocket:
            self.ws = websocket
            print("Connected to simulator")

            listen_task = asyncio.create_task(self._listen_for_feedback())
            act_task = asyncio.create_task(self._path_execution_loop())

            await asyncio.gather(listen_task, act_task)

    async def _listen_for_feedback(self):
        """Continuously listens for messages from the simulator (e.g., collisions)."""
        async for message in self.ws:
            data = json.loads(message)
            if data.get("type") == "goal_reached":
                print("🎯 Goal reached!")
                self.goal_reached = True
                break
            if data.get("type") == "collision":
                print("⚠️ Collision detected:", data)
                self.is_collided = True
                self.robot_pos = (data['robot_position']['x'], data['robot_position']['y'])
                self.move_to(*self.robot_pos)

    async def _path_execution_loop(self):
        """Handles path planning and execution in a continuous loop."""
        while not self.goal_reached:
            # Re-plan if path is empty (initial) OR if collision occurred
            if not self.path or self.is_collided:
                if self.is_collided:
                    print(f"Re-planning from collision point: {self.robot_pos}")
                    self.is_collided = False
                else:
                    print("Initial path planning...")
                
                # --- FIX: Fetch obstacles from the server before planning ---
                await self._fetch_obstacles_from_server()

                self.path = self.planner.plan(
                    self.robot_pos,
                    (self.goal["x"], self.goal["y"]),
                    self.obstacles
                )
                self.current_waypoint_index = 0

                if not self.path:
                    print("🛑 Cannot find a path. Retrying in 2 seconds...")
                    await asyncio.sleep(2)
                    continue

                print(f"Planned new path ({len(self.path)} waypoints): {self.path}")

            while self.current_waypoint_index < len(self.path):
                waypoint = self.path[self.current_waypoint_index]
                self.move_to(waypoint[0], waypoint[1])
                self.robot_pos = waypoint
                self.current_waypoint_index += 1

                await asyncio.sleep(0.5)

                if self.is_collided:
                    print("Path execution interrupted by collision. Re-plan initiated.")
                    break

            if not self.is_collided and not self.goal_reached:
                await asyncio.sleep(1.0)
                
        print("Path execution loop finished.")

    def move_to(self, x, y):
        url = f"{self.server_url}/move"
        payload = {"x": x, "y": y}
        try:
            r = requests.post(url, json=payload)
            r.raise_for_status() # Raise HTTPError for bad responses
            print(f"➡️ Moving to {payload}")
        except requests.exceptions.RequestException as e:
            print(f"❌ Failed to move: {e}")

if __name__ == "__main__":
    controller = RobotController()
    asyncio.run(controller.connect())
