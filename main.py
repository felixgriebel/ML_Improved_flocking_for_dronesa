# environment.py
import random
import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from robotics import Drone
import torch

class Environment:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(cameraDistance=15, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        p.setGravity(0, 0, -9.8)

        self.plane_id = p.loadURDF("plane.urdf")

        self.num_drones = 20
        self.drones = []
        for i in range(self.num_drones):
            start_pos = [np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(5, 10)]
            is_leader = (i == 0)
            drone = Drone(self.client, start_position=start_pos, is_leader=is_leader)
            if is_leader:
                drone.make_leader_color()
            else:
                drone.change_color(rgba_color=[0, 0, 1, 1])
            self.drones.append(drone)

        self.fixed_objects = []
        self.add_fixed_objects()

        # Experience buffer for training
        self.experience_buffer = []
        self.max_buffer_size = 1000

    def add_fixed_objects(self):
        # Define positions and sizes for the floating objects
        positions = [
            [5, 5, 8],
            [-5, -5, 7],
            [10, -10, 9],
            [-20, 8, 15],
            [10, 15, 1],
        ]

        sizes = [0.5, 1.0, 0.5, 2.0, 4.0]

        for pos, size in zip(positions, sizes):
            visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=size, rgbaColor=[0, 1, 0, 1])
            collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=size)

            fixed_object_id = p.createMultiBody(
                baseMass=0,  # Mass of 0 makes it static
                baseCollisionShapeIndex=collision_shape_id,
                baseVisualShapeIndex=visual_shape_id,
                basePosition=pos,
            )
            self.fixed_objects.append(fixed_object_id)

    def run(self):
        t = 0.0
        while True:
            p.stepSimulation()

            leader_drone = self.drones[0]

            # Copy drones to avoid modification during iteration
            copy_drones = self.drones[:]
            if t % 0.1 == 0:
                for drone in copy_drones:
                    if drone.is_collided:
                        self.drones.remove(drone)
                        continue

                    if not drone.is_leader:
                        # Collect experience
                        obs = drone.get_observation(self.drones, leader_drone, self.fixed_objects)
                        action = drone.velocity.copy()
                        drone.update(self.drones, leader_drone, self.fixed_objects)
                        reward = self.compute_reward(drone, leader_drone)
                        next_obs = drone.get_observation(self.drones, leader_drone, self.fixed_objects)

                        experience = (obs, action, reward, next_obs)
                        self.experience_buffer.append(experience)

                        if len(self.experience_buffer) > self.max_buffer_size:
                            self.experience_buffer.pop(0)
                    else:
                        drone.update(self.drones, leader_drone, self.fixed_objects)
            else:
                for drone in copy_drones:
                    if drone.is_collided:
                        self.drones.remove(drone)
                        continue
                    drone.update(self.drones, leader_drone, self.fixed_objects)

            # Training the RL model periodically
            if t % 1.0 == 0 and self.experience_buffer:
                print("Training RL model...")
                batch_size = min(32, len(self.experience_buffer))
                #experiences = np.random.choice(self.experience_buffer, batch_size, replace=False)
                experiences = random.sample(self.experience_buffer, batch_size)
                for drone in self.drones:
                    if not drone.is_leader and not drone.is_collided:
                        drone.train_model(experiences)
                # Save the model
                drone.rl_agent.save_model("drone_model.pth")

            # Update camera
            cam_target = leader_drone.position
            cam_distance = 20
            cam_yaw = np.degrees(leader_drone.yaw)
            cam_pitch = -30

            p.resetDebugVisualizerCamera(cameraDistance=cam_distance,
                                         cameraYaw=cam_yaw,
                                         cameraPitch=cam_pitch,
                                         cameraTargetPosition=cam_target)

            t += 0.01
            time.sleep(1 / 240)

    def compute_reward(self, drone, leader_drone):
        """Compute the reward for the drone."""
        # Reward for following the leader
        distance_to_leader = np.linalg.norm(drone.position - leader_drone.position)
        reward = -distance_to_leader

        # Penalty for collision
        if drone.is_collided:
            reward -= 100

        return reward

if __name__ == "__main__":
    env = Environment()
    env.run()
