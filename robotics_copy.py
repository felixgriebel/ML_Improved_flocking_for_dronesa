import pybullet as p
import os
import math
import numpy as np

class Drone:
    def __init__(self, client):

        self.position = np.array([1,0,0])
        self.speed = 0.01



        self.client = client
        
        # Get the current script's directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Construct the full path to the .obj file
        obj_path = os.path.join(current_dir, "triangle.obj")
        
        # Load the mesh for the drone with color
        self.drone_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=obj_path,
            rgbaColor=[0.7, 0.3, 0.3, 1],  # Red-ish color
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
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )

    def change_color(self, rgba_color):
        p.changeVisualShape(self.drone_id, -1, rgbaColor=rgba_color)

    def detect_nearby_objects(self, radius):
        drone_pos, _ = p.getBasePositionAndOrientation(self.drone_id)
        
        # Get all objects in the scene
        objects = p.getCollisionShapeData(self.drone_id, -1)
        nearby_objects = []

        for obj in objects:
            obj_id = obj[0]
            if obj_id != self.drone_id:  # Exclude the drone itself
                obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
                distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(drone_pos, obj_pos)))
                
                if distance <= radius:
                    nearby_objects.append((obj_id, distance))

        return nearby_objects

    def get_position(self):
        self.position, _ = p.getBasePositionAndOrientation(self.drone_id)
        return self.position

    def set_position(self, position):
        p.resetBasePositionAndOrientation(self.drone_id, position, p.getQuaternionFromEuler([0, 0, 0]))


# def get_close_objects(self, radius=5.0):
#         """
#         Get positions of objects that are within a certain radius from the drone.
#         """
#         objects_in_radius = []
#         if self.drone_id is not None:
#             drone_position = self.get_position()
#             # Iterate over all objects in the environment (assuming we know their IDs)
#             for obj_id in range(p.getNumBodies()):
#                 if obj_id != self.drone_id:  # Avoid comparing the drone itself
#                     obj_position, _ = p.getBasePositionAndOrientation(obj_id)
#                     distance = np.linalg.norm(np.array(obj_position) - drone_position)
#                     if distance < radius:
#                         objects_in_radius.append({"id": obj_id, "position": obj_position, "distance": distance})
#         return objects_in_radius

    def move(self, direction=[1, 0, 0]):
        """
        Move the drone in a specific direction by its speed.
        Direction is a vector (e.g., [1, 0, 0] for moving along the x-axis).
        """
        normalized_direction = np.array(direction) / np.linalg.norm(direction)  # Normalize the direction
        self.position += self.speed * normalized_direction
        if self.drone_id is not None:
            p.resetBasePositionAndOrientation(self.drone_id, self.position.tolist(), [0, 0, 0, 1])

    def update_position(self):
        """
        Update the drone's position in the PyBullet environment.
        """
        if self.drone_id is not None:
            p.resetBasePositionAndOrientation(self.drone_id, self.position.tolist(), [0, 0, 0, 1])
