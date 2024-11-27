import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from robotics import Drone
import datetime 


#TODO: ändern zu nur im gewissen bereich sind die nahen boids
#TODO: andern zu leader immer enthalten
#TODO: https://people.ece.cornell.edu/land/courses/ece4760/labs/s2021/Boids/Boids.html




class Environment:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(cameraDistance=15, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        p.setGravity(0, 0, -9.8)#-9.8

        self.plane_id = p.loadURDF("plane.urdf")
        
        #self.drone = Drone(self.client)
        
        self.num_drones = 120
        self.drones = []
        for i in range(self.num_drones):
            start_pos = [np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(5, 10)]
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
        self.fixed_objects = []
        self.add_fixed_objects()

    def add_fixed_objects(self):
        # Define positions for the floating objects
        positions = [
            [5, 5, 8],
            [-5, -5, 7],
            [10, -10, 9],
            [-20, 8, 15],
            [10, 15, 1],
            [0, 0, 30],
            [-8, 15, 10],
            [25, -15, 20],
            [-30, 30, 5],
            [40, -40, 35],
            [-45, 20, 18],
            [15, -35, 25],
            [30, 10, 12],
            [-50, -30, 8],
            [20, -25, 16],
            [10, 40, 3],
            [-15, 25, 22],
            [0, -45, 20],
            [35, -10, 15],
            [-25, -40, 5],
        ]
        
        sizes = [
            0.5,
            1.0,
            0.5,
            2.0,
            4.0,
            1.5,
            0.5,
            3.0,
            4.5,
            2.5,
            1.0,
            4.0,
            3.5,
            2.0,
            4.5,
            1.5,
            0.75,
            3.25,
            2.75,
            4.0
        ]
        count=0
        # Define a visual and collision shape for cubes
        for pos in positions:
            if count<10:
                visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[sizes[count], sizes[count], sizes[count]], rgbaColor=[0, 1, 0, 1])
                collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[sizes[count],sizes[count], sizes[count]])
            else:
                visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=sizes[count], rgbaColor=[0, 1, 0, 1])
                collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sizes[count])

            count+=1
            
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


