import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet in GUI mode
physics_client = p.connect(p.GUI)

# Set up the environment (no ground plane)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Disable gravity for flying elements
p.setGravity(0, 0, 0)

# Parameters for flying elements
num_boids = 10  # Number of flying elements
boid_radius = 0.4  # Size of each boid
speed = 0.1  # Speed of boids
direction_change_interval = 100  # Number of simulation steps before direction change
boid_ids = []

# Create initial positions and velocities for boids
positions = np.random.uniform(low=-50, high=50, size=(num_boids, 3))
velocities = np.random.uniform(low=-1, high=1, size=(num_boids, 3))

# Create visual shapes for the boids (cylinders to indicate direction)
for i in range(num_boids):
    visual_shape_id = p.createVisualShape(p.GEOM_CYLINDER, 
                                          radius=boid_radius, 
                                          length=2*boid_radius,  # Make the cylinder longer to visualize direction
                                          rgbaColor=[0, 1, 0, 1])
    boid_id = p.createMultiBody(baseMass=1,
                                baseVisualShapeIndex=visual_shape_id,
                                basePosition=positions[i])
    boid_ids.append(boid_id)

# Function to update boid positions and orientations
def update_boid_positions():
    for i, boid_id in enumerate(boid_ids):
        # Get current position and orientation
        position, orientation = p.getBasePositionAndOrientation(boid_id)

        # Update position with velocity
        new_position = np.array(position) + velocities[i] * speed

        # Keep boids within a specific boundary (e.g., a box of size 100x100x100)
        for j in range(3):
            if new_position[j] > 50:
                new_position[j] = -50
            elif new_position[j] < -50:
                new_position[j] = 50

        # Calculate the new orientation of the boid to face the direction of its velocity
        forward_direction = velocities[i] / np.linalg.norm(velocities[i])  # Normalize velocity to get direction
        # Set orientation based on direction (as a quaternion)
        new_orientation = p.getQuaternionFromEuler([np.arctan2(forward_direction[1], forward_direction[0]), 0, 0])

        # Set the new position and orientation for the boid
        p.resetBasePositionAndOrientation(boid_id, new_position.tolist(), new_orientation)

        # Set camera to follow the first boid
        if i == 0:  # Adjust the camera to follow the first boid (you can change the index to follow other boids)
            camera_distance = 20  # Distance behind the boid
            camera_height = 1    # Height of the camera above the boid
            target_position = new_position.tolist()
            camera_position = [target_position[0] - forward_direction[0] * camera_distance,
                               target_position[1] - forward_direction[1] * camera_distance,
                               target_position[2] + camera_height]
            p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                         cameraYaw=np.degrees(np.arctan2(forward_direction[1], forward_direction[0])),
                                         cameraPitch=-10,  # Slight downward pitch
                                         cameraTargetPosition=target_position)

# Function to randomly adjust the direction of each boid
def update_boid_directions():
    global velocities
    # Introduce random directional changes
    random_changes = np.random.uniform(low=-0.1, high=0.1, size=(num_boids, 3))
    velocities += random_changes
    velocities = velocities / np.linalg.norm(velocities, axis=1)[:, None]  # Normalize to maintain consistent speed

# Run the simulation
step_count = 0
while True:
    # Step simulation (can be replaced with more complex physics if necessary)
    update_boid_positions()

    # Randomly change boid directions every `direction_change_interval` steps
    if step_count % direction_change_interval == 0:
        update_boid_directions()

    # Step through PyBullet simulation
    p.stepSimulation()

    # Sleep to slow down the simulation to a real-time speed
    time.sleep(0.01)
    step_count += 1

# Disconnect PyBullet when done
p.disconnect()
