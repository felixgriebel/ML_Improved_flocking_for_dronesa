import pybullet as p
import os
import math
import numpy as np

class Drone:
    def __init__(self, client):
        self.client = client
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        obj_path = os.path.join(current_dir, "triangle.obj")
        
        self.drone_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=obj_path,
            rgbaColor=[0.7, 0.3, 0.3, 1],
            meshScale=[1, 1, 1]
        )
        
        self.drone_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=obj_path,
            meshScale=[1, 1, 1]
        )
        
        self.drone_id = p.createMultiBody(
            baseMass=1,
            baseVisualShapeIndex=self.drone_visual_shape,
            baseCollisionShapeIndex=self.drone_collision_shape,
            basePosition=[0, 0, 1],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])  # No rotation needed now
        )

    def change_color(self, rgba_color):
            p.changeVisualShape(self.drone_id, -1, rgbaColor=rgba_color)

    def detect_nearby_objects(self, radius):
        drone_pos, _ = p.getBasePositionAndOrientation(self.drone_id)
        
        aabb_min = [pos - radius for pos in drone_pos]
        aabb_max = [pos + radius for pos in drone_pos]
        
        object_ids = p.getOverlappingObjects(aabb_min, aabb_max)
        
        nearby_objects = []
        for obj_id, _ in object_ids:
            if obj_id != self.drone_id:  # Exclude the drone itself
                obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
                distance = np.linalg.norm(np.array(obj_pos) - np.array(drone_pos))
                
                if distance <= radius:
                    vector = np.array(obj_pos) - np.array(drone_pos)
                    nearby_objects.append((obj_id, distance, vector))

        return nearby_objects

    def get_position_and_orientation(self):
        return p.getBasePositionAndOrientation(self.drone_id)

    def set_position_and_orientation(self, position, orientation):
        p.resetBasePositionAndOrientation(self.drone_id, position, orientation)

    def move_forward(self, distance):
        pos, orn = self.get_position_and_orientation()
        direction = p.getMatrixFromQuaternion(orn)[0:3]  # Extract forward vector
        new_pos = [pos[i] + direction[i] * distance for i in range(3)]
        self.set_position_and_orientation(new_pos, orn)

    def roll(self, angle):
        pos, orn = self.get_position_and_orientation()
        rotation = p.getQuaternionFromEuler([angle, 0, 0])
        new_orn = p.multiplyQuaternions(orn, rotation)
        self.set_position_and_orientation(pos, new_orn)

    def yaw(self, angle):
        pos, orn = self.get_position_and_orientation()
        rotation = p.getQuaternionFromEuler([0, 0, angle])
        new_orn = p.multiplyQuaternions(orn, rotation)
        self.set_position_and_orientation(pos, new_orn)



    def visualize_detection_sphere(self, radius, lifetime=0):
        drone_pos = self.get_position()
        p.addUserDebugLine([drone_pos[0] - radius, drone_pos[1], drone_pos[2]], 
                           [drone_pos[0] + radius, drone_pos[1], drone_pos[2]], 
                           [1, 0, 0], lineWidth=2, lifeTime=lifetime)
        p.addUserDebugLine([drone_pos[0], drone_pos[1] - radius, drone_pos[2]], 
                           [drone_pos[0], drone_pos[1] + radius, drone_pos[2]], 
                           [0, 1, 0], lineWidth=2, lifeTime=lifetime)
        p.addUserDebugLine([drone_pos[0], drone_pos[1], drone_pos[2] - radius], 
                           [drone_pos[0], drone_pos[1], drone_pos[2] + radius], 
                           [0, 0, 1], lineWidth=2, lifeTime=lifetime)

    def get_position(self):
            pos, _ = p.getBasePositionAndOrientation(self.drone_id)
            return pos

    def set_position(self, position):
        p.resetBasePositionAndOrientation(self.drone_id, position, p.getQuaternionFromEuler([0, 0, 0]))


    def move_forward(self, distance):
        current_pos = self.get_position()
        new_pos = [current_pos[0], current_pos[1] + distance, current_pos[2]]
        self.set_position(new_pos)