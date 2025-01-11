import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
from main_orig import Environment
from stable_baselines3 import PPO
import leader_print as blueprints

class NonZeroBox(spaces.Box):
    def sample(self):
        sample = super().sample()
        while np.linalg.norm(sample) < 1e-6:  # Avoid zero vector or near-zero
            sample = super().sample()
        return sample

class DroneGymEnv(gym.Env):
    def __init__(self, num_drones=30, numberObjects = 1500, bp=0):
        super(DroneGymEnv, self).__init__()
        
        self.env = Environment(drone_version=1,numberObjects=numberObjects)
        self.num_drones = num_drones
        self.drones = self.env.drones[:num_drones]

        # Action space: 3 floats for new direction vector [0.0, 1.0]
        #self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.action_space = NonZeroBox(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Observation space: Structured for each drone
        max_neighbors = 10
        max_objects = 5
        state_size = 1 + 3 + 3 + (max_neighbors * 4) + (max_objects * 4)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(state_size,), dtype=np.float32
        )
        
        self.leader_drone = self.drones[0]
        self.collided_drones = []
        self.current_drone_idx = 0

        if bp==1:
            self.leader_drone.leaderbp = blueprints.print2
        elif bp == 4:
            self.leader_drone.leaderbp = blueprints.print4
        else:
            self.leader_drone.leaderbp = blueprints.meteorite


    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
            self.env.seed(seed)
        # ! changed here
        self.env.close()
        #p.disconnect()
        self.env.__init__()  # Reinitialize environment
        self.drones = self.env.drones[:self.num_drones]
        self.leader_drone = self.drones[0]
        self.collided_drones = []
        self.current_drone_idx = 0
        obs = self._get_observation(self.drones[self.current_drone_idx])
        return obs, {}

    def step(self, action):
    # Get the current drone to process
        while self.drones[self.current_drone_idx] in self.collided_drones:
                self.current_drone_idx = (self.current_drone_idx + 1) % self.num_drones
        drone = self.drones[self.current_drone_idx]
        if drone == self.leader_drone:
            drone.update(self.drones, self.leader_drone)
            # Skip processing if collided or it's the leader
            self.current_drone_idx = (self.current_drone_idx + 1) % self.num_drones
            # If we completed one round of all drones, step the simulation
            return self._get_observation(self.drones[self.current_drone_idx]), 0, False, False, {}
        
        # Process action: Set new direction
        action = np.clip(action, -1.0, 1.0)
        new_direction = action / np.linalg.norm(action) if np.linalg.norm(action) > 0 else np.array([1.0, 0.0, 0.0])
        if np.linalg.norm(action)!=0.0:
            drone.direction = new_direction
            drone.velocity = drone.direction * drone.max_speed
        # Update drone behavior
        drone.update(self.drones, self.leader_drone)
        # Check for collisions
        if drone.is_collided:
            self.collided_drones.append(drone)
            p.removeBody(drone.drone_id)

        # Compute reward
        reward = self._compute_reward(drone, action)

        # Termination conditions
        terminated = self.leader_drone.is_collided or (self.leader_drone.leader_ended)
        truncated = len(self.collided_drones) > (len(self.drones) /2)

        # Move to the next drone
        self.current_drone_idx = (self.current_drone_idx + 1) % self.num_drones
        while self.drones[self.current_drone_idx] in self.collided_drones:
            self.current_drone_idx = (self.current_drone_idx + 1) % self.num_drones
        # Step the simulation after completing one round of all drones
        if self.current_drone_idx == 0:
            p.stepSimulation()

        obs = self._get_observation(self.drones[self.current_drone_idx])
        return obs, reward, terminated, truncated, {}


    def _get_observation(self, drone):
        max_neighbors = 10
        max_objects = 5

        # Leader information
        to_leader = self.leader_drone.position - drone.position
        distance_to_leader = np.linalg.norm(to_leader)
        direction_to_leader = to_leader / distance_to_leader if distance_to_leader > 0 else np.zeros(3)

        # Drone's own state
        own_velocity = drone.velocity

        # Nearby drones
        neighbors = drone.get_closest_drones(self.drones)
        drone_features = []
        for neighbor in neighbors[:max_neighbors]:
            relative_position = neighbor[1] - drone.position
            distance = np.linalg.norm(relative_position)
            direction = relative_position / distance if distance > 0 else np.zeros(3)
            if not(np.any(np.isnan(distance)) or np.any(np.isinf(distance))) and not(np.any(np.isnan(direction)) or np.any(np.isinf(direction))):
                drone_features.extend([distance, *direction])
        while len(drone_features) < max_neighbors * 4:
            drone_features.extend([0, 0, 0, 0])

        # Nearby objects
        nearby_objects = []
        overlapping_objects = p.getOverlappingObjects(
            drone.position - drone.obstacle_avoidance_radius,
            drone.position + drone.obstacle_avoidance_radius
        ) or []
        for obj_id, _ in overlapping_objects[:max_objects]:
            obj_position, _ = p.getBasePositionAndOrientation(obj_id)
            relative_position = np.array(obj_position) - drone.position
            distance = np.linalg.norm(relative_position)
            direction = relative_position / distance if distance > 0 else np.zeros(3)
            if not(np.any(np.isnan(distance)) or np.any(np.isinf(distance))) and not(np.any(np.isnan(direction)) or np.any(np.isinf(direction))):
                nearby_objects.extend([distance, *direction])
        while len(nearby_objects) < max_objects * 4:
            nearby_objects.extend([0, 0, 0, 0])

        observation = np.concatenate((
            [distance_to_leader],
            direction_to_leader,
            own_velocity,
            drone_features,
            nearby_objects
        ))

        return observation

    def _compute_reward(self, drone, action):
        if drone.is_collided:
            if drone.collision_reason == 1:
                return -500.0
            else:
                return -1000.0
        # if np.linalg.norm(action)==0.0:
        # return -2000.0
        to_leader = self.leader_drone.position - drone.position
        distance_to_leader = np.linalg.norm(to_leader)
        reward = 20.0-distance_to_leader
        return reward


    def render(self, mode='human'):
        pass  # Visualization handled by PyBullet GUI

    def close(self):
        p.disconnect()


if __name__ == "__main__":

    env = DroneGymEnv(numberObjects=0,bp=0)
    model = PPO("MlpPolicy", env, n_steps=2048, verbose=1, n_epochs=20, batch_size=64)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=0,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=0,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)

    env = DroneGymEnv(numberObjects=250,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=250,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=250,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)

    env = DroneGymEnv(numberObjects=500,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=500,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=500,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    model.save(("./models/RL_env_1_1_small"))


    env = DroneGymEnv(numberObjects=1000,bp=0)
    model = PPO("MlpPolicy", env, n_steps=2048, verbose=1, n_epochs=20, batch_size=64)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1000,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1000,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)

    env = DroneGymEnv(numberObjects=1250,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1250,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1250,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)

    env = DroneGymEnv(numberObjects=1500,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1500,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1500,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    model.save(("./models/RL_env_1_1_big"))

    env = DroneGymEnv(numberObjects=0,bp=0)
    model = PPO("MlpPolicy", env, n_steps=2048, verbose=1, n_epochs=20, batch_size=64)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=0,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=0,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)

    env = DroneGymEnv(numberObjects=250,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=250,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=250,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)

    env = DroneGymEnv(numberObjects=500,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=500,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=500,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)


    env = DroneGymEnv(numberObjects=750,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=750,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=750,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)

    env = DroneGymEnv(numberObjects=1000,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1000,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1000,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)

    env = DroneGymEnv(numberObjects=1250,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1250,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1250,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)

    env = DroneGymEnv(numberObjects=1500,bp=0)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1500,bp=4)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    env = DroneGymEnv(numberObjects=1500,bp=2)
    model.set_env(env)
    model.learn(total_timesteps=104856)
    model.save(("./models/RL_env_1_1_combi"))














    # suffix = "model1"
    # newModel = True

    # n_steps=10000
    # n_epochs=70
    # batch_size=50
    # total_timesteps=150000

    # number_of_objects = 0
    # blueprint_used = 0

    # env = DroneGymEnv(numberObjects=number_of_objects,bp=blueprint_used)
    # if newModel:
    #     model = PPO("MlpPolicy", env, n_steps=n_steps, verbose=1, n_epochs=n_epochs, batch_size=batch_size)
    # else:
    #     model = PPO.load(("./models/RL_env_1_"+suffix), env, n_steps=n_steps, verbose=1, n_epochs=n_epochs, batch_size=batch_size)

    # model.learn(total_timesteps=total_timesteps)
    # model.save(("./models/RL_env_1_"+suffix))
