# drone.py

import pybullet as p
import numpy as np
import math
import torch

from rl_agent import RLAgent

class Drone:
    def __init__(self, client, start_position, is_leader=False):
        self.client = client
        self.is_leader = is_leader
        self.is_collided = False
        self.speed = 0.1
        self.max_speed = 0.7
        self.perception_radius = 10.0  # Increased for better perception

        self.position = np.array(start_position, dtype=np.float32)
        self.velocity = np.random.uniform(-1, 1, size=3).astype(np.float32)
        self.velocity /= np.linalg.norm(self.velocity) + 1e-6  # Normalize

        self.yaw = 0.0
        self.pitch = 0.0

        # Initialize RL Agent
        if not self.is_leader:
            self.input_size = 15  # Adjusted based on observation space
            self.output_size = 3   # Velocity vector (dx, dy, dz)
            self.rl_agent = RLAgent(self.input_size, self.output_size)
            # Load pre-trained model if available
            self.rl_agent.load_model("drone_model.pth")

        # Drone visual and collision shapes
        self.drone_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName="triangle.obj",
            rgbaColor=[0.7, 0.3, 0.3, 1],
            meshScale=[1, 1, 1]
        )

        self.drone_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName="triangle.obj",
            meshScale=[1, 1, 1]
        )

        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.drone_id = p.createMultiBody(
            baseMass=10,
            baseVisualShapeIndex=self.drone_visual_shape,
            baseCollisionShapeIndex=self.drone_collision_shape,
            basePosition=start_position,
            baseOrientation=initial_orientation
        )

    def handle_input(self):
        """Handle input for the leader drone."""
        keys = p.getKeyboardEvents()
        LEFT_ARROW = 65361
        RIGHT_ARROW = 65363
        UP_ARROW = 65362
        DOWN_ARROW = 65364

        move_direction = np.array([0.0, 0.0, 0.0])

        if LEFT_ARROW in keys and keys[LEFT_ARROW] & p.KEY_IS_DOWN:
            move_direction[0] = -1.0
        if RIGHT_ARROW in keys and keys[RIGHT_ARROW] & p.KEY_IS_DOWN:
            move_direction[0] = 1.0
        if UP_ARROW in keys and keys[UP_ARROW] & p.KEY_IS_DOWN:
            move_direction[1] = 1.0
        if DOWN_ARROW in keys and keys[DOWN_ARROW] & p.KEY_IS_DOWN:
            move_direction[1] = -1.0

        if np.linalg.norm(move_direction) > 0:
            move_direction /= np.linalg.norm(move_direction)
            self.velocity = self.speed * move_direction

    def make_leader_color(self):
        """Change the color for the leader drone."""
        p.changeVisualShape(self.drone_id, -1, rgbaColor=[1, 0, 0, 1])

    def change_color(self, rgba_color):
        """Change the drone's color."""
        p.changeVisualShape(self.drone_id, -1, rgbaColor=rgba_color)

    def get_observation(self, drones, leader_drone, obstacles):
        """Get the observation vector for the RL agent."""
        # Relative position and velocity to the leader
        rel_pos_leader = leader_drone.position - self.position
        rel_vel_leader = leader_drone.velocity - self.velocity

        # Nearby drones
        nearby_drones = []
        for drone in drones:
            if drone is self or drone.is_collided:
                continue
            distance = np.linalg.norm(drone.position - self.position)
            if distance < self.perception_radius:
                nearby_drones.append(drone)

        # Sum of relative positions and velocities of nearby drones
        rel_pos_sum = np.zeros(3, dtype=np.float32)
        rel_vel_sum = np.zeros(3, dtype=np.float32)
        for drone in nearby_drones:
            rel_pos = drone.position - self.position
            rel_vel = drone.velocity - self.velocity
            rel_pos_sum += rel_pos
            rel_vel_sum += rel_vel

        # Nearby obstacles
        nearby_obstacles = []
        for obs_id in obstacles:
            obs_pos, _ = p.getBasePositionAndOrientation(obs_id)
            obs_pos = np.array(obs_pos)
            distance = np.linalg.norm(obs_pos - self.position)
            if distance < self.perception_radius:
                nearby_obstacles.append(obs_pos - self.position)

        # Sum of relative positions to obstacles
        rel_obs_pos_sum = np.zeros(3, dtype=np.float32)
        for obs_rel_pos in nearby_obstacles:
            rel_obs_pos_sum += obs_rel_pos

        # Normalize observations
        observation = np.concatenate([
            rel_pos_leader / (np.linalg.norm(rel_pos_leader) + 1e-6),
            rel_vel_leader / (np.linalg.norm(rel_vel_leader) + 1e-6),
            rel_pos_sum / (len(nearby_drones) + 1e-6),
            rel_vel_sum / (len(nearby_drones) + 1e-6),
            rel_obs_pos_sum / (len(nearby_obstacles) + 1e-6),
        ])

        return observation.astype(np.float32)

    def update(self, drones, leader_drone, obstacles, skip=False):
        """Update the drone's position and orientation."""
        if self.is_leader:
            self.handle_input()
        else:
            # Get observation
            obs = self.get_observation(drones, leader_drone, obstacles)
            obs_tensor = torch.from_numpy(obs)

            # Get action from RL agent
            with torch.no_grad():
                action = self.rl_agent(obs_tensor).numpy()

            # Update velocity
            self.velocity = action
            speed = np.linalg.norm(self.velocity)
            if speed > self.max_speed:
                self.velocity = (self.velocity / speed) * self.max_speed

        # Update position
        self.position += self.velocity * 0.1  # Time step

        # Update orientation based on velocity
        if np.linalg.norm(self.velocity) > 1e-6:
            self.yaw = math.atan2(self.velocity[1], self.velocity[0])
            self.pitch = math.atan2(self.velocity[2], np.linalg.norm(self.velocity[:2]))

        # Check for collisions
        self.check_collision(drones, obstacles)

        # Update in PyBullet
        quat = p.getQuaternionFromEuler([0, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.drone_id, self.position.tolist(), quat)

    def check_collision(self, drones, obstacles):
        """Check for collisions with other drones and obstacles."""
        # Check with obstacles
        for obs_id in obstacles:
            obs_pos, _ = p.getBasePositionAndOrientation(obs_id)
            obs_pos = np.array(obs_pos)
            distance = np.linalg.norm(obs_pos - self.position)
            if distance < 1.0:  # Collision radius
                self.is_collided = True
                print(f"Drone {self.drone_id} collided with obstacle.")
                return

        # Check with other drones
        for drone in drones:
            if drone is self or drone.is_collided:
                continue
            distance = np.linalg.norm(drone.position - self.position)
            if distance < 1.0:  # Collision radius
                self.is_collided = True
                print(f"Drone {self.drone_id} collided with another drone.")
                return

    def train_model(self, experiences):
        """Train the RL model using collected experiences."""
        if not experiences:
            return

        batch_obs, batch_actions, batch_rewards, batch_next_obs = zip(*experiences)

        batch_obs = torch.tensor(batch_obs, dtype=torch.float32)
        batch_actions = torch.tensor(batch_actions, dtype=torch.float32)
        batch_rewards = torch.tensor(batch_rewards, dtype=torch.float32)
        batch_next_obs = torch.tensor(batch_next_obs, dtype=torch.float32)

        # Predicted actions
        pred_actions = self.rl_agent(batch_obs)

        # Compute loss
        loss = self.rl_agent.criterion(pred_actions, batch_actions)

        # Backpropagation
        self.rl_agent.optimizer.zero_grad()
        loss.backward()
        self.rl_agent.optimizer.step()
