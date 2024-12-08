# README

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

    models folder: includes the models used for the report and should be the default folder to store the trained models

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