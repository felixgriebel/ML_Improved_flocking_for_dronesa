import pybullet as p
import numpy as np
import math

class Drone:
    def __init__(self, client, start_position, is_leader=False):
        self.client = client
        
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
        
        # Corrected initial orientation: rotate 90 degrees around the z-axis
        initial_orientation = p.getQuaternionFromEuler([0, 0, 1])
        
        self.drone_id = p.createMultiBody(
            baseMass=1,
            baseVisualShapeIndex=self.drone_visual_shape,
            baseCollisionShapeIndex=self.drone_collision_shape,
            basePosition=start_position,
            baseOrientation=initial_orientation
        )

        self.is_leader = is_leader
        self.speed = 0.1  # Forward speed
        self.turn_rate = 0.01  # Radians per frame
        self.pitch_rate = 0.01  # Radians per frame
        self.position = np.array(start_position)
        self.yaw = 0  # Initial yaw (facing positive x-axis)
        self.pitch = 0  # Initial pitch
        self.velocity = np.zeros(3)
        self.max_speed = 1
        self.perception_radius = 5.0

    def update(self, drones):
        if self.is_leader:
            self.handle_input()
            
            # Calculate direction based on yaw and pitch
            direction = np.array([
                -math.cos(self.pitch) * math.sin(self.yaw),
                math.cos(self.pitch) * math.cos(self.yaw),
                math.sin(self.pitch)
            ])
            
            # Update velocity
            self.velocity = self.speed * direction
        else:
            # Follower behavior (adjust as needed)
            leader_altitude = drones[0].position[2]
            altitude_difference = leader_altitude - self.position[2]
            self.velocity[2] = np.clip(altitude_difference, -self.max_speed, self.max_speed)

        # Update position
        self.position += self.velocity

        # Update orientation
        orientation = p.getQuaternionFromEuler([self.pitch, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.drone_id, self.position.tolist(), orientation)

        # Collision detection
        self.detect_collision_with_floor()
        self.detect_collision_with_drones(drones)

    def handle_input(self):
        keys = p.getKeyboardEvents()

        # Yaw control
        if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
            self.yaw += self.turn_rate
        if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            self.yaw -= self.turn_rate

        # Pitch control
        if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
            self.pitch = min(self.pitch + self.pitch_rate, math.pi/2)
        if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
            self.pitch = max(self.pitch - self.pitch_rate, -math.pi/2)

        # Altitude control
        if ord('r') in keys and keys[ord('r')] & p.KEY_IS_DOWN:
            #self.position[2] += self.speed
            self.speed +=0.01
            if self.speed>self.max_speed:
                self.speed = self.max_speed
            if self.speed<0.1:
                self.speed = 0.1
        if ord('f') in keys and keys[ord('f')] & p.KEY_IS_DOWN:
            #self.position[2] -= self.speed
            self.speed -=0.001
            if self.speed>self.max_speed:
                self.speed = self.max_speed
            # if self.speed<0.01:
            #     self.speed = 0.1

    # Other methods remain unchanged
    def detect_collision_with_floor(self):
        if self.position[2] < 0.1:
            self.position[2] = 0.1
            self.velocity[2] = 0

    def detect_collision_with_drones(self, drones):
        for other in drones:
            if other == self:
                continue
            distance = np.linalg.norm(other.position - self.position)
            if distance < 0.2:
                # Simple collision response
                self.velocity *= -1
                other.velocity *= -1

    def change_color(self, rgba_color):
        p.changeVisualShape(self.drone_id, -1, rgbaColor=rgba_color)





    # def detect_nearby_objects(self, radius):
    #     drone_pos, _ = p.getBasePositionAndOrientation(self.drone_id)
        
    #     aabb_min = [pos - radius for pos in drone_pos]
    #     aabb_max = [pos + radius for pos in drone_pos]
        
    #     object_ids = p.getOverlappingObjects(aabb_min, aabb_max)
        
    #     nearby_objects = []
    #     for obj_id, _ in object_ids:
    #         if obj_id != self.drone_id:  # Exclude the drone itself
    #             obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
    #             distance = np.linalg.norm(np.array(obj_pos) - np.array(drone_pos))
                
    #             if distance <= radius:
    #                 vector = np.array(obj_pos) - np.array(drone_pos)
    #                 nearby_objects.append((obj_id, distance, vector))

    #     return nearby_objects

    # def get_position_and_orientation(self):
    #     return p.getBasePositionAndOrientation(self.drone_id)

    # def set_position_and_orientation(self, position, orientation):
    #     p.resetBasePositionAndOrientation(self.drone_id, position, orientation)

    # def move_forward(self, distance):
    #     pos, orn = self.get_position_and_orientation()
    #     direction = p.getMatrixFromQuaternion(orn)[0:3]  # Extract forward vector
    #     new_pos = [pos[i] + direction[i] * distance for i in range(3)]
    #     self.set_position_and_orientation(new_pos, orn)

    # def roll(self, angle):
    #     pos, orn = self.get_position_and_orientation()
    #     rotation = p.getQuaternionFromEuler([angle, 0, 0])
    #     new_orn = p.multiplyQuaternions(orn, rotation)
    #     self.set_position_and_orientation(pos, new_orn)

    # def yaw(self, angle):
    #     pos, orn = self.get_position_and_orientation()
    #     rotation = p.getQuaternionFromEuler([0, 0, angle])
    #     new_orn = p.multiplyQuaternions(orn, rotation)
    #     self.set_position_and_orientation(pos, new_orn)



    # def visualize_detection_sphere(self, radius, lifetime=0):
    #     drone_pos = self.get_position()
    #     p.addUserDebugLine([drone_pos[0] - radius, drone_pos[1], drone_pos[2]], 
    #                        [drone_pos[0] + radius, drone_pos[1], drone_pos[2]], 
    #                        [1, 0, 0], lineWidth=2, lifeTime=lifetime)
    #     p.addUserDebugLine([drone_pos[0], drone_pos[1] - radius, drone_pos[2]], 
    #                        [drone_pos[0], drone_pos[1] + radius, drone_pos[2]], 
    #                        [0, 1, 0], lineWidth=2, lifeTime=lifetime)
    #     p.addUserDebugLine([drone_pos[0], drone_pos[1], drone_pos[2] - radius], 
    #                        [drone_pos[0], drone_pos[1], drone_pos[2] + radius], 
    #                        [0, 0, 1], lineWidth=2, lifeTime=lifetime)

    # def get_position(self):
    #         pos, _ = p.getBasePositionAndOrientation(self.drone_id)
    #         return pos

    # def set_position(self, position):
    #     p.resetBasePositionAndOrientation(self.drone_id, position, p.getQuaternionFromEuler([0, 0, 0]))


    # def move_forward(self, distance):
    #     current_pos = self.get_position()
    #     new_pos = [current_pos[0], current_pos[1] + distance, current_pos[2]]
    #     self.set_position(new_pos)