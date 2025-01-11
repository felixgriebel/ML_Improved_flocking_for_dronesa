import pybullet as p
import numpy as np
import math
import leader_print as leader_bp
from stable_baselines3 import PPO


class Drone:
    def __init__(self, client, start_position, is_leader=False):
        self.client = client
        
        self.drone_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName="./object/triangle.obj",
            rgbaColor=[0.7, 0.3, 0.3, 1],
            meshScale=[1, 1, 1]
        )
        
        self.drone_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName="./object/triangle.obj",
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
        self.collision_reason = 0
        self.speed = 0.1  # Forward speed
        self.turn_rate = 0.01  # Radians per frame
        self.position = np.array(start_position)
        self.yaw = 0  # Initial yaw
        self.pitch = 0  # Initial pitch
        self.direction = np.array([1.0, 0.0, 0.0])  # Initial direction (normalized)
        self.velocity = np.array([1.0, 1.0, 0.0])
        self.max_speed = 0.7

        self.perception_radius = 5.0

        self.seperation_threshold = self.perception_radius
        self.cohesion_threshold = self.perception_radius
        self.alignment_threshold = self.perception_radius
        self.obstacle_avoidance_radius = self.perception_radius


        self.seperationfactor = 0.3#0.1
        self.centeringfactor = 0.4#0.0005
        self.matchingfactor = 0.5#0.01
        self.avoidancefactor = 0.7#0.2
        self.followfactor = 0.6#0.2

        self.model = None
        # ! Prediction functionality
        self.model = PPO.load("./models/RL_env_2_1_combi")
        
        
        #only important for the leader to remember the step in blueprint
        self.leader_counter = 0
        self.leader_ended = False
        self.leaderbp = leader_bp.meteorite




    def handle_input(self):
        if self.leader_counter >= len(self.leaderbp):
            self.leader_ended = True
            return
        act = self.leaderbp[self.leader_counter]
        if act == 4:
            self.yaw = (self.yaw + self.turn_rate) % (2 * math.pi)
        elif act == 2:
            self.yaw = (self.yaw - self.turn_rate) % (2 * math.pi)
        elif act == 1:
            self.pitch = (self.pitch + self.turn_rate) % (2 * math.pi)
        elif act == 3:
            self.pitch = (self.pitch - self.turn_rate) % (2 * math.pi)
        if act == 7:
            print("cc: ",self.leader_counter)
        self.direction = np.array([
            math.cos(self.pitch) * math.cos(self.yaw),
            math.cos(self.pitch) * math.sin(self.yaw),
            math.sin(self.pitch)
        ])
        self.leader_counter+=1
    
    def change_color(self, rgba_color):
        p.changeVisualShape(self.drone_id, -1, rgbaColor=rgba_color)

    
    # ! FOR STEERING IT MANUALLY--------------------------------------------------------------------------
    # def handle_input(self):
    #     keys = p.getKeyboardEvents()
        
    #     LEFT_ARROW = 65295
    #     RIGHT_ARROW = 65296
    #     UP_ARROW = 65297
    #     DOWN_ARROW = 65298
        
    #     makenone = True

    #     if LEFT_ARROW in keys and keys[LEFT_ARROW] & p.KEY_IS_DOWN:
    #         self.yaw = (self.yaw + self.turn_rate) % (2 * math.pi)
    #         print("4,", end =" ")
    #         makenone = False
    #     if RIGHT_ARROW in keys and keys[RIGHT_ARROW] & p.KEY_IS_DOWN:
    #         self.yaw = (self.yaw - self.turn_rate) % (2 * math.pi)
    #         print("2,", end =" ")
    #         makenone = False

    #     # Pitch control (full 360 degrees)
    #     if UP_ARROW  in keys and keys[UP_ARROW] & p.KEY_IS_DOWN:
    #         self.pitch = (self.pitch + self.turn_rate) % (2 * math.pi)
    #         print("1,", end =" ")
    #         makenone = False
    #     if DOWN_ARROW in keys and keys[DOWN_ARROW] & p.KEY_IS_DOWN:
    #         self.pitch = (self.pitch - self.turn_rate) % (2 * math.pi)
    #         print("3,", end =" ")
    #         makenone = False
    #     if makenone:
    #         print("0,", end =" ")

    #     # Update direction based on yaw and pitch
    #     # self.direction = np.array([
    #     #     -math.cos(self.pitch) * math.sin(self.yaw),
    #     #     math.cos(self.pitch) * math.cos(self.yaw),
    #     #     math.sin(self.pitch)
    #     # ])
    #     self.direction = np.array([
    #         math.cos(self.pitch) * math.cos(self.yaw),
    #         math.cos(self.pitch) * math.sin(self.yaw),
    #         math.sin(self.pitch)
    #     ])

    #     # Speed control
    #     if ord('r') in keys and keys[ord('r')] & p.KEY_IS_DOWN:
    #         self.speed = min(self.speed + 0.01, self.max_speed)
    #     if ord('f') in keys and keys[ord('f')] & p.KEY_IS_DOWN:
    #         self.speed = max(self.speed - 0.01, 0.1)
            
        
    #! ==================================================================================================













    def update(self, drones, leader_drone):
        if self.is_collided:
            return
        
        if self.is_leader:
            self.handle_input()
            self.velocity = self.speed * self.direction
            
        else:
            tuptuper = self.get_closest_drones(drones)
            to_leader = leader_drone.position - self.position
            distance_to_leader = np.linalg.norm(to_leader)
            velo = np.array([0.0,0.0,0.0])
            alivec = self.get_alignment(tuptuper,drones=drones)
            cohevec = self.get_cohesion(tuptuper)
            sepavec = self.get_seperation(tuptuper)
            leadvec = self.get_leader(leader=leader_drone)
            avoidvec = self.calculate_avoidance_vector()
            if self.model != None:
                obs = np.concatenate((
                    [distance_to_leader],
                    alivec,
                    cohevec,
                    sepavec,
                    leadvec,
                    avoidvec
                ))
                action, _ = self.model.predict(obs, deterministic=True)
                self.alignment_threshold = action[0]
                self.centeringfactor = action[1]
                self.seperationfactor = action[2]
                self.followfactor = action[3]
                self.avoidancefactor = action[4]

            velo += (alivec*self.matchingfactor)
            velo += (cohevec*self.centeringfactor)
            velo += (sepavec*self.seperationfactor)
            velo += (leadvec*self.followfactor)
            velo+= (avoidvec*self.avoidancefactor)






            self.direction += velo
            
            self.direction/=np.linalg.norm(self.direction)
            
            self.velocity = self.speed * self.direction

            v_x, v_y, v_z = self.direction
            self.yaw = math.atan2(v_z, v_x)  # Yaw based on XZ plane
            self.pitch = math.atan2(v_y, math.sqrt(v_x**2 + v_z**2))  # Pitch based on vertical direction
            

        # * Collision detection
        coool = self.collision_detected(drones)
        if coool == 1:
            self.is_collided = True
            self.collision_reason=1
            self.velocity *= 0
            self.direction *= 0
            self.position = (-1.0,-1.0,-1.0)
        if coool == 2:
            self.is_collided = True
            self.collision_reason=2
            self.velocity *= 0
            self.direction *= 0
            self.position = (-1.0,-1.0,-1.0)


        self.position += self.velocity
        orientation = p.getQuaternionFromEuler([self.pitch, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.drone_id, self.position.tolist(), orientation)



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
        
        vector_sum = vector_sum
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

        dir_sum = (dir_sum-self.velocity)
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
        vector_sum = (vector_sum-self.position)

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
        if self.position[2] < 0.0001:
            #print("id: ",obj_id)
            return 1  # Collision with the floor

        # Check collision with fixed objects
        aabb_min = self.position - 0.0001
        aabb_max = self.position + 0.0001
        overlapping_objects = p.getOverlappingObjects(aabb_min.tolist(), aabb_max.tolist())


        for obj_id, _ in overlapping_objects or []:
            if obj_id != self.drone_id:  # Exclude the drone itself
                return 1

        # Check collision with other drones
        for other in drones:
            if other == self:
                continue
            distance = np.linalg.norm(other.position - self.position)
            if distance < 0.0001:  # Threshold for collision with another drone
                return 2  # Collision with another drone

        # No collision
        return 0