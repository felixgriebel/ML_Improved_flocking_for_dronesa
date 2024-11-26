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
            baseMass=10,
            baseVisualShapeIndex=self.drone_visual_shape,
            baseCollisionShapeIndex=self.drone_collision_shape,
            basePosition=start_position,
            baseOrientation=initial_orientation
        )

        self.is_leader = is_leader
        self.is_collided = False
        self.speed = 0.1  # Forward speed
        self.turn_rate = 0.01  # Radians per frame
        self.position = np.array(start_position)
        self.yaw = 0  # Initial yaw
        self.pitch = 0  # Initial pitch
        self.direction = np.array([1.0, 0.0, 0.0])  # Initial direction (normalized)
        self.velocity = np.array([1.0, 1.0, 0.0])
        self.max_speed = 0.7
        self.perception_radius = 5.0

        self.seperation_threshold = 3.0
        self.seperationfactor = 0.001
        self.cohesion_threshold = 20.0
        self.centeringfactor = 0.005
        self.alignment_threshold = 10.0
        self.matchingfactor = 0.3

        self.obstacle_avoidance_radius = 5.0
        

    def update(self, drones, leader_drone, skip = False):
        if self.is_collided:
            return
        if self.is_leader:
            self.handle_input()
            self.velocity = self.speed * self.direction
            
        else:
            if not skip:
                tuptuper = self.get_closest_drones(drones)
                velo = np.array([0.0,0.0,0.0])
                velo += self.get_alignment(tuptuper,drones=drones)
                velo += self.get_cohesion(tuptuper)
                velo += self.get_seperation(tuptuper)
                velo += self.get_leader(leader=leader_drone)*0.01
                self.direction += velo#/np.linalg.norm(velo)
                self.direction/=np.linalg.norm(self.direction)
                self.calculate_avoidance_vector()
            
            self.velocity = self.speed * self.direction

            v_x, v_z, v_y = self.direction
            self.yaw = math.atan2(v_x, v_z)  # Yaw based on XZ plane
            self.pitch = math.atan2(v_y, math.sqrt(v_x**2 + v_z**2))  # Pitch based on vertical direction

        self.position += self.velocity
        orientation = p.getQuaternionFromEuler([self.pitch, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.drone_id, self.position.tolist(), orientation)
        #Collision detection
        coool = self.collision_detected(drones)
        if coool == 1:
            self.is_collided = True
            self.velocity *= 0
            self.direction *= 0
        if coool == 2:
            self.is_collided = True
            self.velocity *= 0
            self.direction *= 0

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
            

    def change_color(self, rgba_color):
        p.changeVisualShape(self.drone_id, -1, rgbaColor=rgba_color)
        
    def make_leader_color(self):
        self.drone_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName="triangle_reversed.obj",
            rgbaColor=[0.7, 0.3, 0.3, 1],
            meshScale=[1, 1, 1]
        )
        
        self.drone_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName="triangle_reversed.obj",
            meshScale=[1, 1, 1]
        )
        self.drone_id = p.createMultiBody(
            baseMass=10,
            baseVisualShapeIndex=self.drone_visual_shape,
            baseCollisionShapeIndex=self.drone_collision_shape,
            basePosition=self.position,
            baseOrientation=p.getQuaternionFromEuler([self.pitch, 0, self.yaw])
        )
        self.change_color(rgba_color=[1, 0, 0, 1])


    def get_closest_drones(self, all_drones, num_closest=20):
        max_thresh = max(self.alignment_threshold,self.cohesion_threshold,self.seperation_threshold)
        distances = []
        for drone in all_drones:
            if drone != self:  # Don't include self in the list
                distance = np.linalg.norm(drone.position - self.position)
                if distance<max_thresh:
                    distances.append((drone.drone_id, drone.position, distance, drone.velocity, drone.speed))
        
        # Sort by distance and return the num_closest drones
        distances.sort(key=lambda x: x[2])
        return distances
    
    



    # ! BOID ALGO METHODS:-----------------------------------------------------------------------------------------------------------------------

    def get_seperation(self, tuplelist, neighbour_num = 10):
        vector_sum=np.array([0.0,0.0,0.0])
        
        for i in tuplelist:
            if i[2]>self.seperation_threshold:
                break
            vector_sum+= (self.position - i[1])
        
        vector_sum = vector_sum*self.seperationfactor
        return vector_sum
    
    def get_alignment(self, tuplelist, drones, neighbour_num = 10):
        dir_sum = np.array([0.0,0.0,0.0])

        counter = 0
        for i in tuplelist:
            if i[2]>self.alignment_threshold:
                break
            dir_sum+= i[3]
            counter +=1

        if counter == 0:
            return np.array([0.0,0.0,0.0])
        dir_sum = dir_sum/counter

        dir_sum = (dir_sum-self.velocity)*self.matchingfactor
        return dir_sum
    
    def get_cohesion(self, tuplelist):
        vector_sum=np.array([0.0,0.0,0.0])

        counter = 0

        for i in tuplelist:
            if i[2]>self.cohesion_threshold:
                break
            vector_sum+= i[1]
            counter +=1
        
        if counter == 0:
            return np.array([0.0,0.0,0.0])
        vector_sum = vector_sum/counter
        vector_sum = (vector_sum-self.position)*self.centeringfactor

        return vector_sum#/np.linalg.norm(vector_sum)
    
    
    def get_leader(self,leader):
        dist = np.linalg.norm(leader.position - self.position)
        vec = leader.position-self.position
        return (vec)#/np.linalg.norm(vec))# * math.sqrt(math.sqrt(dist))

    def avg_speed(self, tuplelist,leader,  neighbour_num = 10):
        summe = leader.speed
        for i in tuplelist[:neighbour_num]:
            summe+= i[4]
        return summe/(neighbour_num+1)






    # ! Collision detection and avoidance:--------------------------------------------------------------------------------------------------------



    def calculate_avoidance_vector(self):
        """
        Dynamically fetch obstacles from the environment and calculate a vector to avoid collisions.
        
        Returns:
        - avoidance_vector: A numpy array representing the avoidance direction.
        """
        # Initialize the avoidance vector
        avoidance_vector = np.array([0.0, 0.0, 0.0])

        # Get the drone's current position
        drone_pos = self.position

        # Check for obstacles within the radius
        aabb_min = drone_pos - self.obstacle_avoidance_radius
        aabb_max = drone_pos + self.obstacle_avoidance_radius

        overlapping_objects = p.getOverlappingObjects(aabb_min.tolist(), aabb_max.tolist())
        
        for obj_id, _ in overlapping_objects or []:
            # Avoid self
            if obj_id == self.drone_id:
                continue

            # Get obstacle position
            obstacle_pos, _ = p.getBasePositionAndOrientation(obj_id)
            obstacle_pos = np.array(obstacle_pos)

            # Calculate distance and direction
            distance = np.linalg.norm(drone_pos - obstacle_pos)
            if distance < self.obstacle_avoidance_radius:
                direction_away = drone_pos - obstacle_pos
                avoidance_strength = (self.obstacle_avoidance_radius - distance) / self.obstacle_avoidance_radius
                avoidance_vector += direction_away / np.linalg.norm(direction_away) * avoidance_strength

        # Normalize the avoidance vector if it has magnitude
        if np.linalg.norm(avoidance_vector) > 0:
            avoidance_vector = avoidance_vector / np.linalg.norm(avoidance_vector)

        return avoidance_vector




    def collision_detected(self, drones):
        #Returns:
        #- 0: No collision
        #- 1: Collision with the floor or fixed object
        #- 2: Collision with another drone
        
        # Check collision with the floor
        if self.position[2] < 0.1:
            return 1  # Collision with the floor

        # Check collision with fixed objects
        aabb_min = self.position - 0.0005
        aabb_max = self.position + 0.0005
        overlapping_objects = p.getOverlappingObjects(aabb_min.tolist(), aabb_max.tolist())

        for obj_id, _ in overlapping_objects or []:
            if obj_id != self.drone_id:  # Exclude the drone itself
                return 1  # Collision with fixed object

        # Check collision with other drones
        for other in drones:
            if other == self:
                continue
            distance = np.linalg.norm(other.position - self.position)
            if distance < 0.0005:  # Threshold for collision with another drone
                return 2  # Collision with another drone

        # No collision
        return 0
