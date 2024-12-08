import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from drone_vanilla import Drone as Drone_0
from drone_1 import Drone as Drone_1
from drone_2 import Drone as Drone_2
from drone_3 import Drone as Drone_3
import visualize_tests as vt
import leader_print as lp

#Basic vectors knowledge: https://people.ece.cornell.edu/land/courses/ece4760/labs/s2021/Boids/Boids.html



class Environment:
    def __init__(self, numdro = 30, drone_version=0, numberObjects = 1500, graphical = False):
        self.seed = 42
        np.random.seed(42)
        if p.getConnectionInfo()['isConnected'] == 0:
            if graphical:
                self.client = p.connect(p.GUI)
            else:
                self.client = p.connect(p.DIRECT)
        else:
            self.close()
            if graphical:
                self.client = p.connect(p.GUI)
            else:
                self.client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(cameraDistance=15, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        p.setGravity(0, 0, -9.8)#-9.8

        self.plane_id = p.loadURDF("plane.urdf")
        
        #self.drone = Drone(self.client)
        
        self.num_drones = numdro
        self.drones = []
        for i in range(self.num_drones):
            start_pos = [np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(5, 10)]
            # visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[0, 0, 1, 1])
            # collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.1)
            
            is_leader = (i == 0)

            if drone_version==1:
                drone = Drone_1(self.client,start_position=start_pos,is_leader=is_leader)
            elif drone_version == 2:
                drone = Drone_2(self.client,start_position=start_pos,is_leader=is_leader)
            elif drone_version == 3:
                drone = Drone_3(self.client,start_position=start_pos,is_leader=is_leader)
            else:
                drone = Drone_0(self.client,start_position=start_pos,is_leader=is_leader)

            if is_leader:
                # Change leader color to red
                #drone.make_leader_color()
                drone.change_color(rgba_color=[1, 0, 0, 1])
                
            else:
                drone.change_color(rgba_color=[0, 0, 1, 1])
            self.drones.append(drone)
        self.fixed_objects = []
        self.add_fixed_objects(numberObjects)



    def add_fixed_objects(self,numObj):
        visual_shape_id = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[800, 400, 0.1],  # Very thin box to represent the plane
            rgbaColor=[0.7, 0.7, 0.7, 1]  # Light grey color
        )
        collision_shape_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[800, 400, 0.1]  # Matches visual extents
        )
        fixed_object_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[700, 0, 0],  # Position the plane at z = 0
        )
        self.fixed_objects.append(fixed_object_id)
        np.random.seed(42)
        # Define positions for the meteorite field
        positions = []
        for _ in range(numObj):  # Adjust the number of meteorites
            x = np.random.uniform(20, 1200)
            y = np.random.uniform(-40, 40)
            z = np.random.uniform(0, 40)
            positions.append([10 + x, y, z])  # Offset by 10.0 in the x direction

        # Random sizes for the meteorites
        sizes = np.random.uniform(0.5, 3.0, len(positions))

        # Create meteorites
        for pos, size in zip(positions, sizes):
            visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=size, rgbaColor=[0, 1.0, 0, 1])#rgbaColor=[0.5, 0.5, 0.5, 1])
            collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=size)
            fixed_object_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_shape_id,
                baseVisualShapeIndex=visual_shape_id,
                basePosition=pos,
            )
            self.fixed_objects.append(fixed_object_id)
        
        np.random.seed(self.seed)


    # def add_fixed_objects(self):
    #     # Define random positions and sizes
    #     for _ in range(300):  # Adjust the number of objects
    #         x = np.random.uniform(-150, 150)
    #         y = np.random.uniform(-150, 150)
    #         z = np.random.uniform(0, 60)
    #         size = np.random.uniform(1.0, 5.0)

    #         shape_type = np.random.choice([p.GEOM_SPHERE, p.GEOM_BOX, p.GEOM_CYLINDER])
    #         if shape_type == p.GEOM_SPHERE:
    #             visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=size, rgbaColor=[0.4, 0.4, 0.4, 1])
    #             collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=size)
    #         elif shape_type == p.GEOM_BOX:
    #             visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.6, 0.3, 0.3, 1])
    #             collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
    #         else:  # p.GEOM_CYLINDER
    #             visual_shape_id = p.createVisualShape(p.GEOM_CYLINDER, radius=size, length=size * 2, rgbaColor=[0.3, 0.3, 0.7, 1])
    #             collision_shape_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=size, height=size * 2)

    #         fixed_object_id = p.createMultiBody(
    #             baseMass=0,
    #             baseCollisionShapeIndex=collision_shape_id,
    #             baseVisualShapeIndex=visual_shape_id,
    #             basePosition=[x, y, z],
    #         )
    #         self.fixed_objects.append(fixed_object_id)

    def seed(self, seed=None):
        self.seed=seed
        np.random.seed(seed)
    
    def close(self):
        p.disconnect()




# ! not important -------------------------------------------------------------------------------------
    def run(self):
        leader_drone = self.drones[0]

        leader_drone.bp = lp.print3

        cnt = 0
        living_drones = []
        comp_time = []

        # ! leader dist
        leader_dist = []

        # * freeze funct
        # frozen = False
        # camera_target = [0, 0, 0]
        # camera_distance = 30.0

        while (not leader_drone.leader_ended):





            # * freeze funct
            # keys = p.getKeyboardEvents()
            # if 106 in keys and keys[106] & p.KEY_IS_DOWN:
            #     frozen= True
            #     all_body_ids = [p.getBodyUniqueId(i) for i in range(p.getNumBodies())]

            #     # Get the IDs of the drones
            #     drone_ids = [drone.drone_id for drone in self.drones]

            #     # Remove objects that are not drones
            #     for body_id in all_body_ids:
            #         if body_id not in drone_ids:
            #             p.removeBody(body_id)
            # if 107 in keys and keys[107] & p.KEY_IS_DOWN:
            #     frozen= False
            # if 49 in keys and keys[49] & p.KEY_IS_DOWN:
            #     p.resetDebugVisualizerCamera(
            #         cameraDistance=camera_distance,
            #         cameraYaw=0,
            #         cameraPitch=0,
            #         cameraTargetPosition=camera_target
            #     )
            # # Check if the '2' key is pressed
            # if 50 in keys and keys[50] & p.KEY_IS_DOWN:
            #     p.resetDebugVisualizerCamera(
            #         cameraDistance=camera_distance,
            #         cameraYaw=90,
            #         cameraPitch=0,
            #         cameraTargetPosition=camera_target
            #     )
            # # Check if the '3' key is pressed
            # if 51 in keys and keys[51] & p.KEY_IS_DOWN:
            #     p.resetDebugVisualizerCamera(
            #             cameraDistance=camera_distance,
            #             cameraYaw=0,
            #             cameraPitch=-98.99,
            #             cameraTargetPosition=camera_target
            #         )
            # if frozen:
            #     continue

            cnt += 1
            
            # ! for test completeness we continue the test
            if leader_drone.is_collided:
                living_drones.append(-1)
                comp_time.append(-1.0)
                continue

            p.stepSimulation()
            
            

            timetaken =0.0
            timediv = len(self.drones)

            copy_drones = self.drones#[:]
            for drone in copy_drones:
                start_time =time.perf_counter()
                drone.update(copy_drones, leader_drone)
                end_time =time.perf_counter()
                timetaken +=((end_time - start_time) * 1000)

            for drone in copy_drones:
                if drone.is_collided:
                    self.drones.remove(drone)
                    p.removeBody(drone.drone_id)

            # ! here did leader
            temp_lead_dist = 0.0
            for drone in self.drones:
                to_leader = leader_drone.position - drone.position
                temp_lead_dist += np.linalg.norm(to_leader)
            if len(self.drones)>1:
                temp_lead_dist = temp_lead_dist/(len(self.drones)-1)
            else:
                temp_lead_dist = -1
            

            leader_dist.append(temp_lead_dist)

            
            comp_time.append((timetaken/timediv))
            living_drones.append(len(self.drones))
            
            # * end when the leader is dead
            if leader_drone.is_collided:
                break

            # * Camera stuff
            cam_target = leader_drone.position
            cam_distance = 5
            cam_pitch = np.degrees(leader_drone.pitch)
            cam_yaw_offset = -90  # Offset to ensure the camera looks at the back of the drone
            cam_yaw = np.degrees(leader_drone.yaw) + cam_yaw_offset
            p.resetDebugVisualizerCamera(cameraDistance=cam_distance,
                                        cameraYaw=cam_yaw,
                                        cameraPitch=cam_pitch,
                                        cameraTargetPosition=cam_target)
            
            #time.sleep(1/240)
        vt.plot_columns_adjacent_no_space(30,living_drones[:-1])
        vt.plot_time_series_with_avg(comp_time[:-1])
        vt.plot_leader_dist(leader_dist[:-1])    

if __name__ == "__main__":
    # use 0 as vanilla
    # use 1 for first RL
    # use 2 for second RL
    # use 3 for third RL
    env = Environment(drone_version=3,numberObjects=750,graphical=False)
    ser_1 =env.run()
