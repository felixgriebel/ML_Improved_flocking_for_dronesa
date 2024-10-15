import pybullet as p
import numpy as np
import math

class Drone:
    def __init__(self, client, start_position, is_leader=False):
        self.client = client
        
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
        
        # Initial orientation
        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        self.drone_id = p.createMultiBody(
            baseMass=100,
            baseVisualShapeIndex=self.drone_visual_shape,
            baseCollisionShapeIndex=self.drone_collision_shape,
            basePosition=start_position,
            baseOrientation=initial_orientation
        )

        self.is_leader = is_leader
        self.is_collided = False
        self.speed = 0.1  # Forward speed
        self.turn_rate = 0.05  # Radians per frame
        self.position = np.array(start_position)
        self.yaw = 0  # Initial yaw
        self.pitch = 0  # Initial pitch
        self.direction = np.array([1, 0, 0])  # Initial direction (normalized)
        self.velocity = np.zeros(3)
        self.max_speed = 1
        self.perception_radius = 5.0    

    def update(self, drones, leader_drone):
        if self.is_collided:
            return
        if self.is_leader:
            self.handle_input()
            self.velocity = self.speed * self.direction

        else:
            pass
            tuptuper = self.get_closest_drones(drones)
            velo = np.array([0.0,0.0,0.0])
            velo += self.get_alignment(tuptuper,drones=drones)
            velo += self.get_cohesion(tuptuper)
            velo += self.get_seperation(tuptuper)
            velo += self.get_leader(leader=leader_drone)
            
            self.direction= velo/np.linalg.norm(velo)
            
            self.speed = (self.speed+self.avg_speed(tuptuper))/2
            
            
            self.velocity = self.speed * self.direction

            v_x, v_z, v_y = self.direction
            self.yaw = math.atan2(v_x, v_z)  # Yaw based on XZ plane
            self.pitch = math.atan2(v_y, math.sqrt(v_x**2 + v_z**2))  # Pitch based on vertical direction

        # Update position
        self.position += self.velocity

        # Update orientation
        orientation = p.getQuaternionFromEuler([self.pitch, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.drone_id, self.position.tolist(), orientation)

        # Collision detection
        self.detect_collision_with_floor()
        self.detect_collision_with_drones(drones)

    def handle_input(self):
        keys = p.getKeyboardEvents()
        
        LEFT_ARROW = 65295
        RIGHT_ARROW = 65296
        UP_ARROW = 65297
        DOWN_ARROW = 65298
        
        
        if LEFT_ARROW in keys and keys[LEFT_ARROW] & p.KEY_IS_DOWN:
            self.yaw = (self.yaw + self.turn_rate) % (2 * math.pi)
        if RIGHT_ARROW in keys and keys[RIGHT_ARROW] & p.KEY_IS_DOWN:
            self.yaw = (self.yaw - self.turn_rate) % (2 * math.pi)

        # Pitch control (full 360 degrees)
        if UP_ARROW  in keys and keys[UP_ARROW] & p.KEY_IS_DOWN:
            self.pitch = (self.pitch + self.turn_rate) % (2 * math.pi)
        if DOWN_ARROW in keys and keys[DOWN_ARROW] & p.KEY_IS_DOWN:
            self.pitch = (self.pitch - self.turn_rate) % (2 * math.pi)


        # Yaw control (full 360 degrees)
        # if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        #     self.yaw = (self.yaw + self.turn_rate) % (2 * math.pi)
        # if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        #     self.yaw = (self.yaw - self.turn_rate) % (2 * math.pi)

        # # Pitch control (full 360 degrees)
        # if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        #     self.pitch = (self.pitch + self.turn_rate) % (2 * math.pi)
        # if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
        #     self.pitch = (self.pitch - self.turn_rate) % (2 * math.pi)

        # Update direction based on yaw and pitch
        self.direction = np.array([
            -math.cos(self.pitch) * math.sin(self.yaw),
            math.cos(self.pitch) * math.cos(self.yaw),
            math.sin(self.pitch)
        ])

        # Speed control
        if ord('r') in keys and keys[ord('r')] & p.KEY_IS_DOWN:
            self.speed = min(self.speed + 0.01, self.max_speed)
        if ord('f') in keys and keys[ord('f')] & p.KEY_IS_DOWN:
            self.speed = max(self.speed - 0.01, 0.1)

    def detect_collision_with_floor(self):
        if self.position[2] < 0.1:
            self.is_collided = True
            self.velocity *= 0
            self.direction *= 0

    def detect_collision_with_drones(self, drones):
        for other in drones:
            if other == self:
                continue
            distance = np.linalg.norm(other.position - self.position)
            if distance < 0.1:
                self.is_collided = True
                self.velocity *= 0
                self.direction *= 0

    def change_color(self, rgba_color):
        p.changeVisualShape(self.drone_id, -1, rgbaColor=rgba_color)


    def get_closest_drones(self, all_drones, num_closest=20):
        """
        Get the positions of the num_closest drones to this drone.
        
        :param all_drones: List of all Drone objects in the simulation
        :param num_closest: Number of closest drones to return (default 5)
        :return: List of tuples (drone_id, position, distance) for the closest drones
        """
        distances = []
        for drone in all_drones:
            if drone != self:  # Don't include self in the list
                distance = np.linalg.norm(drone.position - self.position)
                distances.append((drone.drone_id, drone.position, distance,drone.velocity, drone.speed))
        
        # Sort by distance and return the num_closest drones
        distances.sort(key=lambda x: x[2])
        return distances[:num_closest]
    
    
    
    # def set_direction(self, new_direction):
    #     # Ensure the new direction is normalized
    #     self.direction = new_direction / np.linalg.norm(new_direction)

    # def get_orientation(self):
    #     return p.getQuaternionFromEuler([self.pitch, 0, self.yaw])

    # def set_orientation(self, orientation):
    #     euler = p.getEulerFromQuaternion(orientation)
    #     self.pitch = euler[0]
    #     self.yaw = euler[2]
    #     self.update_direction_from_orientation()

    # def update_direction_from_orientation(self):
    #     print(self.pitch)
    #     print(self.yaw)
    #     print()
    #     self.direction = np.array([
    #         (math.cos(self.pitch) * math.cos(self.yaw)),
    #         math.sin(self.pitch),
    #         (math.cos(self.pitch) * math.sin(self.yaw))
    #     ])
        
    def get_seperation(self, tuplelist, neighbour_num = 10):
        vector_sum=np.array([0.0,0.0,0.0])
        maxer =  np.array(([t[2] for t in tuplelist[:neighbour_num]])).max()
        for i in tuplelist[:neighbour_num]:
            vector_sum+= (i[1]*(maxer-i[2]))
        return vector_sum/np.linalg.norm(vector_sum)
            
    
    def get_alignment(self, tuplelist, drones, neighbour_num = 10):
        dir_sum = np.array([0.0,0.0,0.0])
        matchingfactor = 1.0
        for i in tuplelist[:neighbour_num]:
            dir_sum+= i[3] * matchingfactor
        if np.linalg.norm(dir_sum) == 0:
            return np.array([0.0,0.0,0.0])
        return dir_sum/np.linalg.norm(dir_sum)
    
    def get_cohesion(self, tuplelist):
        vector_sum=np.array([0.0,0.0,0.0])
        for i in tuplelist:
            vector_sum+= i[1]
        return vector_sum/np.linalg.norm(vector_sum)
    
    
    def get_leader(self,leader):
        dist = np.linalg.norm(leader.position - self.position)
        vec = leader.position-self.position
        return (vec/np.linalg.norm(vec)) * math.sqrt(math.sqrt(dist))

    def avg_speed(self, tuplelist, neighbour_num = 10):
        summe = 0.0
        for i in tuplelist[:neighbour_num]:
            summe+= i[4]
        return summe/neighbour_num


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