import pybullet as p
import pybullet_data
import time
import math
from robotics import Drone

class Environment:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.plane_id = p.loadURDF("plane.urdf")
        
        self.drone = Drone(self.client)
        
        # Add some objects to the environment
        self.cube_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        p.createMultiBody(baseMass=1, baseCollisionShapeIndex=self.cube_id, basePosition=[3, 3, 1])

    def run(self):
        detection_radius = 5
        self.drone.visualize_detection_sphere(detection_radius)

        t = 0
        while True:
            p.stepSimulation()
            
            # Detect nearby objects
            nearby_objects = self.drone.detect_nearby_objects(radius=detection_radius)
            if nearby_objects:
                for obj_id, distance, vector in nearby_objects:
                    print(f"Object {obj_id} detected:")
                    print(f"  Distance: {distance:.2f}")
                    print(f"  Vector: [{vector[0]:.2f}, {vector[1]:.2f}, {vector[2]:.2f}]")
            
            # Move the drone forward
            self.drone.move_forward(0.01)
            
            # Apply rolling motion
            roll_angle = math.sin(t) * 0.1  # Small rolling motion
            self.drone.roll(roll_angle)
            
            # Apply yawing motion
            yaw_angle = math.cos(t) * 0.05  # Small yawing motion
            self.drone.yaw(yaw_angle)
            
            t += 0.01
            time.sleep(1/240)

if __name__ == "__main__":
    env = Environment()
    env.run()