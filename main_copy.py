import pybullet as p
import pybullet_data
import time
from robotics import Drone

class Environment:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.plane_id = p.loadURDF("plane.urdf")
        
        self.drone = Drone(self.client)
        
        # Add some objects to the environment
        self.sphere_id = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
        p.createMultiBody(baseMass=1, baseCollisionShapeIndex=self.sphere_id, basePosition=[3, 3, 1])

    def run(self):
        for _ in range(10000):
            p.stepSimulation()
            
            # Example: Detect nearby objects
            nearby_objects = self.drone.detect_nearby_objects(radius=5)
            if nearby_objects:
                print(f"Nearby objects: {nearby_objects}")
            
            # Example: Move the drone
            current_pos = self.drone.get_position()
            #new_pos = [current_pos[0] + 0.01, current_pos[1], current_pos[2]]
            #self.drone.set_position(new_pos)
            self.drone.move()
            
            time.sleep(1/240)

if __name__ == "__main__":
    env = Environment()
    env.run()