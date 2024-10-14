import pybullet as p
import pybullet_data
import time
import numpy as np
from drone import Drone

def main():
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetDebugVisualizerCamera(cameraDistance=15, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
    p.setGravity(0, 0, -9.8)

    # Load plane
    planeId = p.loadURDF("plane.urdf")

    # Create drones
    num_drones = 20
    drones = []
    for i in range(num_drones):
        start_pos = [np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(1, 3)]
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[0, 0, 1, 1])
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.1)
        mass = 1
        base_orientation = [0, 0, 0, 1]
        drone_id = p.createMultiBody(mass, collision_shape_id, visual_shape_id, start_pos, base_orientation)
        is_leader = (i == 0)
        if is_leader:
            # Change leader color to red
            p.changeVisualShape(drone_id, -1, rgbaColor=[1, 0, 0, 1])
        drone = Drone(drone_id, is_leader)
        drones.append(drone)

    # Simulation loop
    while True:
        p.stepSimulation()

        # Update drones
        for drone in drones:
            drone.update(drones)

        # Update camera to follow the leader drone
        leader_drone = drones[0]
        cam_target = leader_drone.position.tolist()
        cam_distance = 5
        cam_pitch = -30

        # Adjust camera yaw to be behind the leader drone
        cam_yaw = np.degrees(leader_drone.yaw) + 180

        p.resetDebugVisualizerCamera(cameraDistance=cam_distance,
                                     cameraYaw=cam_yaw,
                                     cameraPitch=cam_pitch,
                                     cameraTargetPosition=cam_target)

        time.sleep(1./240.)

    # Disconnect from simulation
    p.disconnect()

if __name__ == "__main__":
    main()
