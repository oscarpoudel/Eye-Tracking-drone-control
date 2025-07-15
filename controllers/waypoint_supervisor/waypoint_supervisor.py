from controller import Supervisor, Receiver, Emitter
import json
import math
import csv
from datetime import datetime

# Initialize Supervisor and timestep
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Initialize Receiver
receiver = supervisor.getDevice("receiver")
receiver.enable(timestep)
receiver.setChannel(2)  # Match the drone's emitter channel

# Initialize Emitter
emitter = supervisor.getDevice("emitter")
emitter.setChannel(1)  # Set a channel for sending waypoint data to the drone

# Waypoints
waypoint_defs = ['way_1', 'way_2', 'way_3', 'way_4']
waypoints = {}

# Access waypoint nodes
for waypoint_def in waypoint_defs:
    waypoint_node = supervisor.getFromDef(waypoint_def)
    if waypoint_node is None:
        print(f"Error: Node '{waypoint_def}' not found.")
    else:
        waypoints[waypoint_def] = waypoint_node

# Predefined positions for `way_1`
way_1_positions = [
    [-5.9 ,-1.52 ,1.49],
    [-7.8, -1.83, 1.27],
    [-7.0, -5.6, 1.0],
    [-2.0, -4.5, 1.5],
    [-1.31, -1.35, 0.425]
]

# Predefined rotations (axis + angle) for each position
way_1_rotations = [
    [1, 0, 0, math.radians(30)],
    [-0.774597, 0.447214, -0.447214, 1.82348],
    [0.8628559982596194, 0.35740699927911007, -0.35740699927911007, -1.7177753071795863],
    [-0.35740895267941514, 0.7409489018991128, 0.5685539247240339, -1.7177653071795866],
    [-1, 0, 0, -1.0472053071795866]
]

# Metrics tracking
way_1_index = 0  # Track the current position index of `way_1`
start_time = supervisor.getTime()  # Record the start time of the simulation
segment_start_time = start_time  # Start time for the current segment
control_inputs = {}  # Dictionary to track control input counts
failed_attempts = 0  # Counter for failed attempts (touching way_2, way_3, or way_4)
last_trajectory_log_time = start_time  # Last time trajectory was logged

# Open CSV files for continuous writing
current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
metrics_file = open(f"metrics_{current_time}.csv", mode='w', newline='')
trajectory_file = open(f"trajectory_{current_time}.csv", mode='w', newline='')

metrics_writer = csv.writer(metrics_file)
trajectory_writer = csv.writer(trajectory_file)

# Write headers to CSV files
metrics_writer.writerow([
    "Segment ID",
    "Completion Time (s)",
    "Failed Attempts",
    "Control Inputs",
    "Current Waypoint"
])
trajectory_writer.writerow(["Time (s)", "X", "Y", "Z", "Current Waypoint"])

# Log task metrics
def log_task_metrics(segment_time):
    global metrics_writer
    metrics_writer.writerow([
        way_1_index + 1,  # Segment ID
        segment_time,
        failed_attempts,
        sum(control_inputs.values()),
        way_1_index + 1  # Current waypoint
    ])

# Check collision with other waypoints
def check_collisions(drone_position):
    global failed_attempts
    for waypoint_def in ['way_2', 'way_3', 'way_4']:
        waypoint_position = waypoints[waypoint_def].getField('translation').getSFVec3f()
        if is_near(drone_position, waypoint_position, 0.25):  # Collision threshold
            failed_attempts += 1
            print(f"Drone collided with {waypoint_def}. Counted as a failed attempt.")

# Log drone trajectory every second
def log_trajectory(current_time, drone_position):
    global last_trajectory_log_time, trajectory_writer
    if current_time - last_trajectory_log_time >= 0.25:  # Log every 1 second
        trajectory_writer.writerow([current_time, *drone_position, way_1_index + 1])
        last_trajectory_log_time = current_time

drone_position = [0, 0, 0]

# Reposition and rotate waypoints
def reposition_and_rotate_waypoints():
    global way_1_index, segment_start_time

    # Log the metrics for the current segment
    current_time = supervisor.getTime()
    segment_time = current_time - segment_start_time
    log_task_metrics(segment_time)
    segment_start_time = current_time  # Reset segment start time

    if way_1_index < len(way_1_positions) - 1:
        way_1_index += 1  # Move to the next position
    else:
        # If the final waypoint is reached, close files and exit simulation
        print("Final waypoint reached. Closing files and exiting simulation.")
        metrics_file.close()
        trajectory_file.close()
        # supervisor.simulationQuit(0)  # End the simulation
        return  # Exit the function

    # Get the new position and rotation for `way_1`
    new_way_1_position = way_1_positions[way_1_index]
    rotation_params = way_1_rotations[way_1_index]

    # Get the current position of `way_1`
    way_1_node = waypoints['way_1']
    current_way_1_position = way_1_node.getField('translation').getSFVec3f()

    # Calculate the offset between the new and current positions of `way_1`
    offset = [
        new_way_1_position[0] - current_way_1_position[0],
        new_way_1_position[1] - current_way_1_position[1],
        new_way_1_position[2] - current_way_1_position[2]
    ]

    # Apply the offset and rotation to all waypoints
    for waypoint_def, waypoint_node in waypoints.items():
        current_position = waypoint_node.getField('translation').getSFVec3f()

        # Apply the offset
        new_position = [
            current_position[0] + offset[0],
            current_position[1] + offset[1],
            current_position[2] + offset[2]
        ]

        # Update the position
        waypoint_node.getField('translation').setSFVec3f(new_position)

        # Apply rotation directly to the `rotation` field
        waypoint_node.getField('rotation').setSFRotation(rotation_params)
        print(f"{waypoint_def} moved to {new_position} with rotation {rotation_params}.")

# Check proximity
def is_near(drone_pos, waypoint_pos, threshold=0.25):
    distance = sum((drone_pos[i] - waypoint_pos[i]) ** 2 for i in range(3)) ** 0.5
    return distance < threshold

while supervisor.step(timestep) != -1:
    # Get current simulation time
    current_time = supervisor.getTime()

    # Check for GPS data from the drone
    while receiver.getQueueLength() > 0:
        message = receiver.getString()
        data = json.loads(message)  # Parse the JSON message
        drone_position = data['position']  # Extract the drone's position
        control_inputs[data.get('input', 'unknown')] = control_inputs.get(data.get('input', 'unknown'), 0) + 1
        receiver.nextPacket()

        # Check if the drone is near `way_1`
        waypoint_1_position = waypoints['way_1'].getField('translation').getSFVec3f()
        if is_near(drone_position, waypoint_1_position):
            print("Drone reached way_1. Repositioning and rotating waypoints.")
            reposition_and_rotate_waypoints()

        # Check for collisions with other waypoints
        check_collisions(drone_position)

    # Log trajectory
    log_trajectory(current_time, drone_position)
