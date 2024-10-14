import pybullet as p
import numpy as np

class Drone:
    def __init__(self, drone_id, is_leader=False):
        self.drone_id = drone_id
        self.is_leader = is_leader
        self.speed = 0.1  # Forward speed
        self.turn_rate = 0.01  # Radians per frame
        self.climb_rate = 0.05  # Vertical speed
        self.position = np.array(p.getBasePositionAndOrientation(drone_id)[0])
        self.orientation = [0, 0, 0, 1]  # Quaternion
        self.yaw = 0.0  # Yaw angle in radians
        self.velocity = np.zeros(3)
        self.max_speed = 0.2
        self.perception_radius = 5.0

    def update(self, drones):
        if self.is_leader:
            self.handle_input()
            # Move forward in the direction of current yaw
            direction = np.array([
                np.cos(self.yaw),
                np.sin(self.yaw),
                0.0
            ])
            # Update horizontal velocity components
            self.velocity[0] = self.speed * direction[0]
            self.velocity[1] = self.speed * direction[1]
            # Vertical velocity is handled in handle_input()
        else:
            self.flocking(drones)
            # Followers adjust altitude to match leader
            leader_altitude = drones[0].position[2]
            altitude_difference = leader_altitude - self.position[2]
            self.velocity[2] = np.clip(altitude_difference, -self.max_speed, self.max_speed)

        # Update position
        self.position += self.velocity

        # Update orientation
        self.orientation = p.getQuaternionFromEuler([0, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.drone_id, self.position.tolist(), self.orientation)

        # Collision detection
        self.detect_collision_with_floor()
        self.detect_collision_with_drones(drones)

    def handle_input(self):
        keys = p.getKeyboardEvents()
        # Reset vertical velocity each frame
        self.velocity[2] = 0.0

        # Rotation control
        if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
            self.yaw += self.turn_rate
        if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            self.yaw -= self.turn_rate

        # Vertical movement
        if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
            self.velocity[2] = self.climb_rate
        elif ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
            self.velocity[2] = -self.climb_rate

    def flocking(self, drones):
        # Followers steer towards the leader while avoiding collisions
        alignment = self.align(drones)
        cohesion = self.cohesion(drones)
        separation = self.separation(drones)

        # Adjust weights as needed
        alignment_weight = 1.0
        cohesion_weight = 1.0
        separation_weight = 1.5

        steering = (alignment * alignment_weight +
                    cohesion * cohesion_weight +
                    separation * separation_weight)

        self.velocity += steering
        self.limit_speed()

    def align(self, drones):
        steering = np.zeros(3)
        total = 0
        avg_vector = np.zeros(3)
        for other in drones:
            if other == self:
                continue
            distance = np.linalg.norm(other.position - self.position)
            if distance < self.perception_radius:
                avg_vector += other.velocity
                total += 1
        if total > 0:
            avg_vector /= total
            avg_vector = self.set_magnitude(avg_vector, self.max_speed)
            steering = avg_vector - self.velocity
        return steering

    def cohesion(self, drones):
        steering = np.zeros(3)
        total = 0
        center_of_mass = np.zeros(3)
        for other in drones:
            if other == self:
                continue
            distance = np.linalg.norm(other.position - self.position)
            if distance < self.perception_radius:
                center_of_mass += other.position
                total += 1
        if total > 0:
            center_of_mass /= total
            direction = center_of_mass - self.position
            steering = self.set_magnitude(direction, self.max_speed)
            steering -= self.velocity
        return steering

    def separation(self, drones):
        steering = np.zeros(3)
        total = 0
        for other in drones:
            if other == self:
                continue
            distance = np.linalg.norm(other.position - self.position)
            if distance < self.perception_radius / 2:
                diff = self.position - other.position
                if distance > 0:
                    diff /= distance
                steering += diff
                total += 1
        if total > 0:
            steering /= total
            steering = self.set_magnitude(steering, self.max_speed)
            steering -= self.velocity
        return steering

    def detect_collision_with_floor(self):
        if self.position[2] < 0.1:
            self.position[2] = 0.1
            self.velocity[2] = 0

    def detect_collision_with_drones(self, drones):
        for other in drones:
            if other == self:
                continue
            distance = np.linalg.norm(other.position - self.position)
            if distance < 0.2:
                # Simple collision response
                self.velocity *= -1
                other.velocity *= -1

    def detect_distance(self, objects):
        distances = []
        for obj_id in objects:
            obj_pos = np.array(p.getBasePositionAndOrientation(obj_id)[0])
            distance = np.linalg.norm(self.position - obj_pos)
            distances.append(distance)
        return distances

    def limit_speed(self):
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = self.velocity / speed * self.max_speed

    def set_magnitude(self, vector, magnitude):
        current_magnitude = np.linalg.norm(vector)
        if current_magnitude > 0:
            return vector / current_magnitude * magnitude
        else:
            return vector
