import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from robotics import Drone

#TODO: ändern zu nur im gewissen bereich sind die nahen boids
#TODO: andern zu leader immer enthalten
#TODO: https://people.ece.cornell.edu/land/courses/ece4760/labs/s2021/Boids/Boids.html




class Environment:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(cameraDistance=15, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        p.setGravity(0, 0, -9.8)

        self.plane_id = p.loadURDF("plane.urdf")
        
        #self.drone = Drone(self.client)
        
        self.num_drones = 100
        self.drones = []
        for i in range(self.num_drones):
            start_pos = [np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(10, 20)]
            # visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[0, 0, 1, 1])
            # collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.1)
            
            is_leader = (i == 0)
            drone = Drone(self.client,start_position=start_pos,is_leader=is_leader)
            if is_leader:
                # Change leader color to red
                drone.make_leader_color()
            else:
                drone.change_color(rgba_color=[0, 0, 1, 1])
            self.drones.append(drone)



    def run(self):
        # detection_radius = 5
        # self.drone.visualize_detection_sphere(detection_radius)

        t = 0
        while True:
            p.stepSimulation()
            
            leader_drone = self.drones[0]

            copy_drones = self.drones[:]
            if t% 0.1 == 0:
                for drone in copy_drones:
                    drone.update(copy_drones, leader_drone)
                    if drone.is_collided:
                        print("remove")
                        self.drones.remove(drone)
            else:
                for drone in copy_drones:
                    drone.update(copy_drones, leader_drone,skip=True)
                    if drone.is_collided:
                        print("remove")
                        self.drones.remove(drone)
                
            # for drone in self.drones:
            #     drone.update(self.drones, leader_drone)
                
            
            
            if leader_drone.is_collided:
                for i in self.drones:
                    if not i.is_collided:
                        leader_drone = i
                        i.is_leader = True
                        i.make_leader_color()
                        break

            border = 50
            for i in self.drones:
                pos = i.position
                if pos[0]>border:
                    pos[0]-=(2*border)
                elif pos[0]< -border:
                    pos[0]+=(2*border)
                if pos[1]>border:
                    pos[1]-=(2*border)
                elif pos[1]< -border:
                    pos[1]+=(2*border)
                i.position = pos
            # # Detect nearby objects
            # nearby_objects = self.drone.detect_nearby_objects(radius=detection_radius)
            # if nearby_objects:
            #     for obj_id, distance, vector in nearby_objects:
            #         print(f"Object {obj_id} detected:")
            #         print(f"  Distance: {distance:.2f}")
            #         print(f"  Vector: [{vector[0]:.2f}, {vector[1]:.2f}, {vector[2]:.2f}]")
            
            # Move the drone forward
            #self.drone.move_forward(0.01)
            
            # # Apply rolling motion
            # roll_angle = math.sin(t) * 0.1  # Small rolling motion
            # self.drone.roll(roll_angle)
            
            # # Apply yawing motion
            # yaw_angle = math.cos(t) * 0.05  # Small yawing motion
            # self.drone.yaw(yaw_angle)

            cam_target = leader_drone.position#.tolist()
            cam_distance = 8
            # cam_pitch = -30

            # Adjust camera yaw to be behind the leader drone
            cam_yaw = np.degrees(leader_drone.yaw) #- 90
            cam_pitch = np.degrees(leader_drone.pitch)
            
            p.resetDebugVisualizerCamera(cameraDistance=cam_distance,
                                        cameraYaw=cam_yaw,
                                        cameraPitch=cam_pitch,
                                        cameraTargetPosition=cam_target)
            
            t += 0.01
            time.sleep(1/240)

if __name__ == "__main__":
    env = Environment()
    env.run()

