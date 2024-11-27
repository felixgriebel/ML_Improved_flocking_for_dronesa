import gym
from gym import spaces
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

    def reset(self):
        # Reset PyBullet environment
        self.env.__init__()  # Reinitialize environment
        self.drones = self.env.drones[:self.num_drones]
        self.leader_drone = self.drones[0]
        return self._get_observation()

    def step(self, action):
        # Apply action: Update each drone's behavior factors
        for drone, factor in zip(self.drones, action):
            drone.matchingfactor = factor[0]  # Alignment
            drone.centeringfactor = factor[1]  # Centering
            drone.seperationfactor = factor[2]  # Separation
            #TODO: add other factors and check if these ones are correct
            # TODO: change perception radius
            # Remaining factors affect leader-following and avoidance
            # Add these interactions here if necessary for RL optimization.

        # Step simulation
        p.stepSimulation()
        for drone in self.drones:
            drone.update(self.drones, self.leader_drone)

        # Compute reward (constant for now)
        reward = 1.0  # Placeholder for a constant reward

        # Check for termination (e.g., collision of leader drone)
        done = self.leader_drone.is_collided

        # Get next state
        obs = self._get_observation()
        
        return obs, reward, done, {}

    def _get_observation(self):
        obs = []
        for drone in self.drones:
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
    # env = Environment()
    # env.run()      
    env = DroneGymEnv()

    newModel = False
    if newModel:
        model = PPO("MlpPolicy", env, verbose=1)
    else:
        model = PPO.load("MlpPolicy", env, )
    model.learn(total_timesteps=5000)
    model.save("drone_rl_model")
