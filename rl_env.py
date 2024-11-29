import  gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
from robotics_orig import Drone
from main_orig import Environment
from stable_baselines3 import PPO

class DroneGymEnv(gym.Env):
    def __init__(self, num_drones=30):
        super(DroneGymEnv, self).__init__()
        
        self.env = Environment()
        self.num_drones = num_drones
        self.drones = self.env.drones[:num_drones]

        # Action space: 5 continuous factors [0.0, 1.0]
        self.action_space = spaces.Box(low=0.0, high=1.0, shape=(5,), dtype=np.float32)

        # Observation space: State vector for each drone
        state_size = 5  # Per drone (number of drones, avg distance, etc.)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(num_drones, state_size), dtype=np.float32
        )
        
        # Initialize leader drone
        self.leader_drone = self.drones[0]

        self.collided_drones =[]

    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
            self.env.seed(seed)
        # Reset PyBullet environment
        self.env.__init__()  # Reinitialize environment
        self.drones = self.env.drones[:self.num_drones]
        self.leader_drone = self.drones[0]
        obs = self._get_observation()
        return obs, {}
    

    def step(self, action):
        # Apply action: Update each drone's behavior factors
        factor = action
        for drone in self.drones:
            if drone == self.leader_drone:
                continue
            if drone in self.collided_drones:
                continue
            drone.matchingfactor = factor[0]  # Alignment
            drone.centeringfactor = factor[1]  # Centering
            drone.seperationfactor = factor[2]  # Separation
            drone.avoidancefactor = factor[3]
            drone.followfactor = factor[4]

        # Step simulation
        collisions_prev = self.collided_drones.copy()
        p.stepSimulation()
        copy_drones = self.drones#[:]
        for drone in copy_drones:
            drone.update(copy_drones, self.leader_drone)
            if drone.is_collided:
                self.collided_drones.append(drone)

        # Compute reward 
        reward = 20.0  # Placeholder for a constant reward
        collision_fact = 0
        for i in self.collided_drones:
            if i in collisions_prev:
                continue
            collision_fact+=i.collision_reason
        reward -= (collision_fact*20.0)


        for drone in self.drones:
            if drone == self.leader_drone or drone in self.collided_drones:
                continue

            # Vector to leader
            to_leader = self.leader_drone.position - drone.position

            distance_to_leader = np.linalg.norm(to_leader)

            reward+= (20.0-distance_to_leader)

            # to_leader /= distance_to_leader  # Normalize

            # # Compare direction vector
            # direction = drone.direction
            # cos_angle = np.dot(direction, to_leader)  # Cosine of the angle between vectors
            # angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # Convert to angle in radians
            # angle_deg = np.degrees(angle)  # Convert to degrees

            # # Add to reward based on angle to leader
            # if angle_deg <= 45:
            #     reward += 1.0
            # elif angle_deg <= 90:
            #     reward += 0.5
            # elif angle_deg <= 135:
            #     reward -= 0.5
            # else:
            #     reward -= 1.0
        
        
        # Check for termination (e.g., collision of leader drone)
        #! 12422 is the end of the leader blueprint
        terminated = self.leader_drone.is_collided or (self.leader_drone.leader_counter>12422)
        truncated = len(self.collided_drones) > (len(self.drones) / 2)
        # Get next state
        obs = self._get_observation()        
        return obs, reward, terminated, truncated, {}

    def _get_observation(self):
        obs = []
        for drone in self.drones:
            #TODO: in theory exclude leader
            # State vector: drones in perception radius, average distance, etc.
            close_drones = drone.get_closest_drones(self.drones)
            num_drones = len(close_drones)
            avg_distance = np.mean([d[2] for d in close_drones]) if num_drones > 0 else 0
            
            obstacle_count = len(p.getOverlappingObjects(
                drone.position - drone.obstacle_avoidance_radius,
                drone.position + drone.obstacle_avoidance_radius
            ) or [])
            avg_obstacle_distance = np.mean([
                np.linalg.norm(np.array(p.getBasePositionAndOrientation(o[0])[0]) - drone.position)
                for o in p.getOverlappingObjects(
                    drone.position - drone.obstacle_avoidance_radius,
                    drone.position + drone.obstacle_avoidance_radius
                ) or []
            ]) if obstacle_count > 0 else 0
            leader_distance = np.linalg.norm(drone.position - self.leader_drone.position)

            obs.append([num_drones, avg_distance, obstacle_count, avg_obstacle_distance, leader_distance])
        return np.array(obs)

    def render(self, mode='human'):
        pass  # Visualization handled by PyBullet GUI

    def close(self):
        p.disconnect()



if __name__ == "__main__":
    env = DroneGymEnv()

    newModel = True
    if newModel:
        model = PPO("MlpPolicy", env,n_steps=1000, verbose=1,n_epochs=50,batch_size=50)
    else:
        model = PPO.load("drone_rl_model")

    model.learn(total_timesteps=1000)
    model.save("drone_rl_model_1000_50epoch")
