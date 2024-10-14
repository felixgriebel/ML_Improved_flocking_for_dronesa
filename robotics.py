import pybullet as p
import os
import math
import numpy as np

class Drone:
    def __init__(self, client, start_position, is_leader=False):
        self.client = client
        
        # current_dir = os.path.dirname(os.path.abspath(__file__))
        # obj_path = os.path.join(current_dir, "triangle.obj")
        
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
        
        self.drone_id = p.createMultiBody(
            baseMass=1,
            baseVisualShapeIndex=self.drone_visual_shape,
            baseCollisionShapeIndex=self.drone_collision_shape,
            basePosition=start_position,#[0, 0, 0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 1])  # No rotation needed now
        )

        self.is_leader = is_leader
        self.speed = 0.1  # Forward speed
        self.turn_rate = 0.01  # Radians per frame
        self.climb_rate = 0.05  # Vertical speed
        self.position = start_position
        self.orientation = [0, 0, 0, 1]  # Quaternion
        self.yaw = 0.0  # Yaw angle in radians
        self.velocity = np.zeros(3)
        self.max_speed = 0.4
        self.perception_radius = 5.0




    def update(self, drones):
        if self.is_leader:
            self.handle_input()
            # Move forward in the direction of current yaw
            direction = np.array([
                np.cos(self.yaw),
                np.sin(self.yaw),
                0.0
            ])
            # Update horizontal velocity components
            self.velocity[0] = self.speed * direction[0]
            self.velocity[1] = self.speed * direction[1]
            # Vertical velocity is handled in handle_input()
        else:
            #self.flocking(drones)

            # Followers adjust altitude to match leader
            leader_altitude = drones[0].position[2]
            altitude_difference = leader_altitude - self.position[2]
            self.velocity[2] = np.clip(altitude_difference, -self.max_speed, self.max_speed)

        # Update position
        self.position += self.velocity

        # Update orientation
        self.orientation = p.getQuaternionFromEuler([0, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.drone_id, self.position.tolist(), self.orientation)

        # Collision detection
        self.detect_collision_with_floor()
        self.detect_collision_with_drones(drones)

    def handle_input(self):
        keys = p.getKeyboardEvents()
        # Reset vertical velocity each frame
        self.velocity[2] = 0.0

        # Rotation control
        if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
            self.yaw += self.turn_rate
        if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            self.yaw -= self.turn_rate

        # Vertical movement
        if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
            self.velocity[2] = self.climb_rate
        elif ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
            self.velocity[2] = -self.climb_rate





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

    def detect_distance(self, objects):
        distances = []
        for obj_id in objects:
            obj_pos = np.array(p.getBasePositionAndOrientation(obj_id)[0])
            distance = np.linalg.norm(self.position - obj_pos)
            distances.append(distance)
        return distances

    def limit_speed(self):
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = self.velocity / speed * self.max_speed

    def set_magnitude(self, vector, magnitude):
        current_magnitude = np.linalg.norm(vector)
        if current_magnitude > 0:
            return vector / current_magnitude * magnitude
        else:
            return vector





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