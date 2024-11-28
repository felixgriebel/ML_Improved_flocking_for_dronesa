import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from robotics_orig import Drone
import datetime 


#TODO: ändern zu nur im gewissen bereich sind die nahen boids
#TODO: andern zu leader immer enthalten
#TODO: https://people.ece.cornell.edu/land/courses/ece4760/labs/s2021/Boids/Boids.html




class Environment:
    def __init__(self):
        self.seed = 42
        np.random.seed(42)

        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(cameraDistance=15, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        p.setGravity(0, 0, -9.8)#-9.8

        self.plane_id = p.loadURDF("plane.urdf")
        
        #self.drone = Drone(self.client)
        
        self.num_drones = 30
        self.drones = []
        for i in range(self.num_drones):
            start_pos = [np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(5, 10)]
            # visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[0, 0, 1, 1])
            # collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.1)
            
            is_leader = (i == 0)
            drone = Drone(self.client,start_position=start_pos,is_leader=is_leader)
            if is_leader:
                # Change leader color to red
                #drone.make_leader_color()
                drone.change_color(rgba_color=[1, 0, 0, 1])
                
            else:
                drone.change_color(rgba_color=[0, 0, 1, 1])
            self.drones.append(drone)
        self.fixed_objects = []
        self.add_fixed_objects()

    # def add_fixed_objects(self):
    #     # Define positions for the floating objects
    #     positions = [
    #         [5, 5, 8],
    #         [-5, -5, 7],
    #         [10, -10, 9],
    #         [-20, 8, 15],
    #         [10, 15, 1],
    #         [0, 0, 30],
    #         [-8, 15, 10],
    #         [25, -15, 20],
    #         [-30, 30, 5],
    #         [40, -40, 35],
    #         [-45, 20, 18],
    #         [15, -35, 25],
    #         [30, 10, 12],
    #         [-50, -30, 8],
    #         [20, -25, 16],
    #         [10, 40, 3],
    #         [-15, 25, 22],
    #         [0, -45, 20],
    #         [35, -10, 15],
    #         [-25, -40, 5],
    #     ]
        
    #     sizes = [
    #         0.5,
    #         1.0,
    #         0.5,
    #         2.0,
    #         4.0,
    #         1.5,
    #         0.5,
    #         3.0,
    #         4.5,
    #         2.5,
    #         1.0,
    #         4.0,
    #         3.5,
    #         2.0,
    #         4.5,
    #         1.5,
    #         0.75,
    #         3.25,
    #         2.75,
    #         4.0
    #     ]
    #     count=0
    #     # Define a visual and collision shape for cubes
    #     for pos in positions:
    #         if count<10:
    #             visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[sizes[count], sizes[count], sizes[count]], rgbaColor=[0, 1, 0, 1])
    #             collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[sizes[count],sizes[count], sizes[count]])
    #         else:
    #             visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=sizes[count], rgbaColor=[0, 1, 0, 1])
    #             collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sizes[count])
    #         count+=1
            
    #         fixed_object_id = p.createMultiBody(
    #             baseMass=0,  # Mass of 0 makes it static
    #             baseCollisionShapeIndex=collision_shape_id,
    #             baseVisualShapeIndex=visual_shape_id,
    #             basePosition=pos,
    #         )
    #         self.fixed_objects.append(fixed_object_id)
    
    
    
    
    
    def add_fixed_objects(self):
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
        for _ in range(1500):  # Adjust the number of meteorites
            x = np.random.uniform(20, 1200)
            y = np.random.uniform(-40, 40)
            z = np.random.uniform(0, 40)
            positions.append([10 + x, y, z])  # Offset by 10.0 in the x direction

        # Random sizes for the meteorites
        sizes = np.random.uniform(0.5, 3.0, len(positions))

        # Create meteorites
        for pos, size in zip(positions, sizes):
            visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=size, rgbaColor=[0.5, 0.5, 0.5, 1])
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
    #     visual_shape_id = p.createVisualShape(
    #         p.GEOM_BOX,
    #         halfExtents=[400, 400, 0.1],  # Very thin box to represent the plane
    #         rgbaColor=[0.7, 0.7, 0.7, 1]  # Light grey color
    #     )
    #     collision_shape_id = p.createCollisionShape(
    #         p.GEOM_BOX,
    #         halfExtents=[400, 400, 0.1]  # Matches visual extents
    #     )
    #     fixed_object_id = p.createMultiBody(
    #         baseMass=0,
    #         baseCollisionShapeIndex=collision_shape_id,
    #         baseVisualShapeIndex=visual_shape_id,
    #         basePosition=[0, 0, 0],  # Position the plane at z = 0
    #     )
    #     self.fixed_objects.append(fixed_object_id)
        
    #     corridor_length = 200  # Extended length of the corridor
    #     corridor_width = 23
    #     corridor_height = 23

    #     # Define walls and ceiling
    #     wall_positions = [
    #         [corridor_length / 2, -corridor_width / 2, corridor_height / 2],
    #         [corridor_length / 2, corridor_width / 2, corridor_height / 2],
    #     ]
    #     ceiling_position = [corridor_length / 2, 0, corridor_height]

    #     # Add walls
    #     for pos in wall_positions:
    #         visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[corridor_length / 2, 0.5, corridor_height / 2], rgbaColor=[0.7, 0.7, 0.7, 1])
    #         collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[corridor_length / 2, 0.5, corridor_height / 2])
    #         fixed_object_id = p.createMultiBody(
    #             baseMass=0,
    #             baseCollisionShapeIndex=collision_shape_id,
    #             baseVisualShapeIndex=visual_shape_id,
    #             basePosition=pos,
    #         )
    #         self.fixed_objects.append(fixed_object_id)

    #     # Add ceiling
    #     visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[corridor_length / 2, corridor_width / 2, 0.5], rgbaColor=[0.7, 0.7, 0.7, 1])
    #     collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[corridor_length / 2, corridor_width / 2, 0.5])
    #     fixed_object_id = p.createMultiBody(
    #         baseMass=0,
    #         baseCollisionShapeIndex=collision_shape_id,
    #         baseVisualShapeIndex=visual_shape_id,
    #         basePosition=ceiling_position,
    #     )
    #     self.fixed_objects.append(fixed_object_id)

    #     # Add varied bars across the corridor
    #     for x in np.arange(20, corridor_length, 10.0):  # Bars at intervals of 10.0
    #         bar_type = np.random.choice(["horizontal", "vertical"])  # Randomly choose bar type
    #         rotation_angle = np.random.uniform(0, math.pi / 6)  # Random small rotation for variation

    #         if bar_type == "horizontal":
    #             # Bar from wall to wall, with small rotation
    #             bar_position = [x, 0, np.random.uniform(0.5, corridor_height - 0.5)]
    #             half_extents = [0.5, corridor_width / 2, 0.5]
    #             orientation = p.getQuaternionFromEuler([0, 0, rotation_angle])  # Rotate around the Z-axis
    #         elif bar_type == "vertical":
    #             # Bar from top to bottom
    #             bar_position = [x, np.random.uniform(-corridor_width / 2 + 1, corridor_width / 2 - 1), corridor_height / 2]
    #             half_extents = [0.5, 0.5, corridor_height / 2]
    #             orientation = p.getQuaternionFromEuler([rotation_angle, 0, 0])  # Rotate around the X-axis

    #         # Create the bar
    #         visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0, 0, 0, 1])
    #         collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    #         fixed_object_id = p.createMultiBody(
    #             baseMass=0,
    #             baseCollisionShapeIndex=collision_shape_id,
    #             baseVisualShapeIndex=visual_shape_id,
    #             basePosition=bar_position,
    #             baseOrientation=orientation,
    #         )
    #         self.fixed_objects.append(fixed_object_id)






    # def add_fixed_objects(self):
    #     corridor_length = 50  # Length of the corridor
    #     corridor_width = 20
    #     corridor_height = 20

    #     # Define walls and ceiling
    #     wall_positions = [
    #         [10, -corridor_width / 2, corridor_height / 2],
    #         [10, corridor_width / 2, corridor_height / 2]
    #     ]
    #     ceiling_position = [10, 0, corridor_height]

    #     for pos in wall_positions:
    #         visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[corridor_length / 2, 0.5, corridor_height / 2], rgbaColor=[0.7, 0.7, 0.7, 1])
    #         collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[corridor_length / 2, 0.5, corridor_height / 2])
    #         fixed_object_id = p.createMultiBody(
    #             baseMass=0,
    #             baseCollisionShapeIndex=collision_shape_id,
    #             baseVisualShapeIndex=visual_shape_id,
    #             basePosition=pos,
    #         )
    #         self.fixed_objects.append(fixed_object_id)

    #     visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[corridor_length / 2, corridor_width / 2, 0.5], rgbaColor=[0.7, 0.7, 0.7, 1])
    #     collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[corridor_length / 2, corridor_width / 2, 0.5])
    #     fixed_object_id = p.createMultiBody(
    #         baseMass=0,
    #         baseCollisionShapeIndex=collision_shape_id,
    #         baseVisualShapeIndex=visual_shape_id,
    #         basePosition=ceiling_position,
    #     )
    #     self.fixed_objects.append(fixed_object_id)

    #     # Add bars
    #     for x in np.arange(10, 10 + corridor_length, 5.0):
    #         for y in [-corridor_width / 2, corridor_width / 2]:
    #             bar_position = [x, y, corridor_height / 2]
    #             visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, corridor_height / 2], rgbaColor=[0, 0, 0, 1])
    #             collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, corridor_height / 2])
    #             fixed_object_id = p.createMultiBody(
    #                 baseMass=0,
    #                 baseCollisionShapeIndex=collision_shape_id,
    #                 baseVisualShapeIndex=visual_shape_id,
    #                 basePosition=bar_position,
    #             )
    #             self.fixed_objects.append(fixed_object_id)

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

















# ! not important -------------------------------------------------------------------------------------
    def run(self):

        t = 0.0
        while True:
            p.stepSimulation()
            
            
            leader_drone = self.drones[0]
            
            copy_drones = self.drones#[:]
            if t% 0.1 == 0:
                for drone in copy_drones:
                    drone.update(copy_drones, leader_drone)
                    if drone.is_collided:
                        #print("remove")
                        self.drones.remove(drone)
            else:
                for drone in copy_drones:
                    drone.update(copy_drones, leader_drone,skip=True)
                    if drone.is_collided:
                        #print("remove")
                        self.drones.remove(drone)
            
            
            if leader_drone.is_collided:
                return
                #continue
                for i in self.drones:
                    if not i.is_collided:
                        leader_drone = i
                        i.is_leader = True
                        i.make_leader_color()
                        break

            cam_target = leader_drone.position
            cam_distance = 5

            # Adjust camera yaw to be behind the leader drone
            cam_pitch = np.degrees(leader_drone.pitch)
            cam_yaw_offset = -90  # Offset to ensure the camera looks at the back of the drone
            cam_yaw = np.degrees(leader_drone.yaw) + cam_yaw_offset
            p.resetDebugVisualizerCamera(cameraDistance=cam_distance,
                                        cameraYaw=cam_yaw,
                                        cameraPitch=cam_pitch,
                                        cameraTargetPosition=cam_target)
            
            t += 0.01
            time.sleep(1/240)
    

# if __name__ == "__main__":
#     env = Environment()
#     env.run()



