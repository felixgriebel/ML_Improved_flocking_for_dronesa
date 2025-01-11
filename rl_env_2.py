import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
from main_orig import Environment
from stable_baselines3 import PPO
import leader_print as blueprints

class DroneGymEnv(gym.Env):
    def __init__(self, num_drones=30, numberObjects = 1500, bp=0):
        super(DroneGymEnv, self).__init__()
        
        self.env = Environment(drone_version=2,numberObjects=numberObjects)
        self.num_drones = num_drones
        self.drones = self.env.drones[:num_drones]

        # Action space: 3 floats for new direction vector [0.0, 1.0]
        self.action_space = spaces.Box(low=0.0001, high=1.0, shape=(5,), dtype=np.float32)
        #self.action_space = NonZeroBox(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Observation space: Structured for each drone

        state_size = 16
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(state_size,), dtype=np.float32
        )
        
        self.leader_drone = self.drones[0]
        self.current_drone_idx = 0
        self.collidedidx= []
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
        self.current_drone_idx = 0
        obs = self._get_observation(self.drones[self.current_drone_idx])
        return obs, {}

    def step(self, action):
        # if self.current_drone_idx in self.collidedidx:
        #     self.current_drone_idx = (self.current_drone_idx + 1) % self.num_drones
        # Get the current drone to process
        drone = self.drones[self.current_drone_idx]
        if drone == self.leader_drone:
            drone.update(self.drones, self.leader_drone)
            # Skip processing if collided or it's the leader
            self.current_drone_idx = (self.current_drone_idx + 1) % len(self.drones)#self.num_drones
            return self._get_observation(self.drones[self.current_drone_idx]), 0, False, False, {}

        # Process action: Set new direction
        drone.alignmentfactor = action[0]
        drone.centeringfactor = action[1]
        drone.seperationfactor = action[2]
        drone.followfactor = action[3]
        drone.avoidancefactor = action[4]
        
        
        # Update drone behavior
        drone.update(self.drones, self.leader_drone)
        # Check for collisions
        
        if drone.is_collided:
            p.removeBody(drone.drone_id)
            self.drones.remove(drone)
            self.collidedidx.append(self.current_drone_idx)
            self.current_drone_idx-=1
            if drone.collision_reason==1:
                reward = -500.0
            else:
                reward = -1000.0 
        else:
            to_leader = self.leader_drone.position - drone.position
            distance_to_leader = np.linalg.norm(to_leader)
            reward = 20.0-distance_to_leader
        # Termination conditions
        terminated = self.leader_drone.is_collided or (self.leader_drone.leader_ended)
        truncated = len(self.drones) < (self.num_drones / 2)

        # Move to the next drone
        self.current_drone_idx = (self.current_drone_idx + 1) % len(self.drones)#self.num_drones
        # Step the simulation after completing one round of all drones
        if self.current_drone_idx == 0:
            p.stepSimulation()

        obs = self._get_observation(self.drones[self.current_drone_idx])
        return obs, reward, terminated, truncated, {}


    def _get_observation(self, drone):

        # Leader information
        tuptuper = drone.get_closest_drones(self.drones)
        to_leader = self.leader_drone.position - drone.position
        distance_to_leader = np.linalg.norm(to_leader)
        alivec = drone.get_alignment(tuptuper,self.drones)
        cohevec = drone.get_cohesion(tuptuper)
        sepavec = drone.get_seperation(tuptuper)
        leadvec = drone.get_leader(self.leader_drone)
        avoidvec = drone.calculate_avoidance_vector()

        observation = np.concatenate((
            [distance_to_leader],
            alivec,
            cohevec,
            sepavec,
            leadvec,
            avoidvec
        ))
        return observation


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
    model.save(("./models/RL_env_2_1_small"))


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
    model.save(("./models/RL_env_2_1_big"))

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
    model.save(("./models/RL_env_2_1_combi"))





    # suffix = "model1"
    # newModel = True

    # n_steps=10000
    # n_epochs=70
    # batch_size=50
    # total_timesteps=150000

    # number_of_objects = 1500
    # blueprint_used = 0

    # env = DroneGymEnv(numberObjects=number_of_objects,bp=blueprint_used)
    # if newModel:
    #     model = PPO("MlpPolicy", env, n_steps=n_steps, verbose=1, n_epochs=n_epochs, batch_size=batch_size)
    # else:
    #     model = PPO.load(("./models/RL_env_2_"+suffix), env, n_steps=n_steps, verbose=1, n_epochs=n_epochs, batch_size=batch_size)

    # model.learn(total_timesteps=total_timesteps)
    # model.save(("./models/RL_env_2_"+suffix))
