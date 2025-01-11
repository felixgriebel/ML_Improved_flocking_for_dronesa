# README

Fore more detailled info read : [Project Report](./REPORT_PAPER/Robots_project_report.pdf)



### Abstract

This project explores the development of a leader-based flocking algorithm for coordinating a robotic swarm in three-dimensional space. Inspired by biological behaviors, such as flocking in birds, the Boids algorithm was adapted to include reinforcement learning (RL) techniques for enhancing swarm adaptability and efficiency. The research aimed to address challenges such as dynamic obstacle avoidance and effective leader-following in constrained environments.

The simulation environment was developed using PyBullet, incorporating real-time interactions with drones and obstacles. Various approaches were implemented, starting with a baseline adaptation of the Boids algorithm, followed by centralized and decentralized RL methods. The decentralized RL models allowed individual drones to adjust their behaviors based on local conditions, demonstrating improved scalability and responsiveness compared to the centralized approach. Further refinements explored direct movement vector prediction, reducing reliance on pre-defined principles, and optimizing computational efficiency.

The results highlight the potential of combining traditional algorithms with machine learning to improve swarm performance in complex environments. Key insights include trade-offs between model complexity and real-world applicability, as well as the importance of diverse training scenarios for achieving robust behavior. This work contributes to advancing autonomous swarm robotics in applications like disaster response and exploration.








### Components

    drone scripts: include the functionality for drone behaviour. 
        dronee_vanilla: is the baseline algorithm (weights need to be manually changed and can be seen in the tab le in the report)
        drone_1: is the 3-vector prediction with high-dimensional prediction space
        drone_2: is the weight prediction 
        drone_3: is the improved 3-vector prediction with principle vectors as observation space
    
        The drone classes are similar and include similar code in major parts, for better readability, they are divided into different scripts. 
        To move the leader drone manually, the handle_input method need to be swapped with the commented one!
        To train a model, the user has to comment the line in the init method, loading the model "PPO.load(...)"

    main_orig: this script implements the pybullet simulation. 
        With the different parameters during the initialization, the user can change which drone version is used, the amount of obstacles in the space or whether the simulation uses the GUI.
        To run a simulation, the user will call the main method, which executes the run()-function. (make sure that the line that loads the model in the drone class is not commented)
    
    rl_env scripts: These scripts build the RL environment, using the main_orig and their respective drone class. to train a model, call the main function. (make sure that the line that loads the model in the drone class is commented)
        rl_env_1: is the 3-vector prediction with high-dimensional prediction space
        rl_env_2: is the weight prediction 
        rl_env_3: is the improved 3-vector prediction with principle vectors as observation space
    
    visualize_tests: implements the plots used in the run() method in main_orig.

    leader_print: includes 4 hard-coded action lists that were recorded from the user interaction

    models folder: includes the models (deleted for git) used for the report and should be the default folder to store the trained models

    object folder: includes the object description of the drones. (a cone)

    graphics folder: includes the graphics for the RL test results




### Prerequisites:
    Python 3.8+ (I used 3.12.6)

    Libraries:
        PyBullet
        Stable-Baselines3
        Gymnasium
        NumPy
        Matplotlib
        SciPy