# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT Licenseeeeeeee

# Copyright (c) 2023 Bitcraze


"""
file: crazyflie_py_wallfollowing.py

Controls the crazyflie and implements a wall following method in webots in Python

Author:   Kimberly McGuire (Bitcraze AB)
"""

from controller import Robot, Receiver, Emitter
from controller import Keyboard

from math import cos, sin

from pid_controller import pid_velocity_fixed_height_controller
from wall_following import WallFollowing
import pandas as pd
import numpy as np
import json  # For decoding JSON data
 # For decoding Base64 messages


FLYING_ATTITUDE = 1

# Parameters for gaze assistance
GAZE_ASSISTANCE_ENABLED = False  # Toggle gaze assistance ON/OFF
GAZE_ASSISTANCE_STRENGTH = 0.25  # Strength of gaze assistance (0 to 1)

# Load gaze data from CSV
gaze_csv_file = r'study-eye-tracking_study__oscar_updated_-92c1787e-3185-4136-a3e6-7f3270ff8bc2-raw-gazes-denoised.csv'
df = pd.read_csv(gaze_csv_file)
gaze_data_column = 'test_raw_data'
gaze_data = eval(df[gaze_data_column].iloc[0])        

current_waypoint_index = 0
# waypoints = [
      # Waypoint 1
                    # ]
waypoint=[0,0,0]
# Define the start time from gaze data
time_start = 3200  # Replace with desired time in milliseconds
gaze_index = 0  

if __name__ == '__main__':

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    # Initialize Sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)
    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)
    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)
    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)
    speaker = robot.getDevice("speaker")
    
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    receiver.setChannel(1) 
    # Get keyboard
    
    keyboard = Keyboard()
    keyboard.enable(timestep)


    emitter = robot.getDevice("emitter")
    emitter.setChannel(2) 
    # Initialize variables

    past_x_global = 0
    past_y_global = 0
    past_time = 0
    first_time = True

    # Crazyflie velocity PID controller
    PID_crazyflie = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()

    height_desired = FLYING_ATTITUDE

    wall_following = WallFollowing(angle_value_buffer=0.01, reference_distance_from_wall=0.5,
                                   max_forward_speed=0.3, init_state=WallFollowing.StateWallFollowing.FORWARD)

    autonomous_mode = False

    print("\n")

    print("====== Controls =======\n\n")

    print(" The Crazyflie can be controlled from your keyboard!\n")
    print(" All controllable movement is in body coordinates\n")
    print("- Use the up, back, right and left button to move in the horizontal plane\n")
    print("- Use Q and E to rotate around yaw\n ")
    print("- Use W and S to go up and down\n ")
    print("- Press A to start autonomous mode\n")
    print("- Press D to disable autonomous mode\n")
    
    print("speaker_player")
    robot_start_time = None
    # Main loop:
    while robot.step(timestep) != -1:
        current_time = robot.getTime() * 1000 
        # print(gaze_data[gaze_index][2])
        while gaze_data[gaze_index][2] < time_start:
                # print(gaze_data[gaze_index][2])
                gaze_index += 1  # Advance to the relevant gaze data
            # if gaze_index >= len(gaze_data):
                # print("All gaze data is before time_start or finished.")
                # break  

        while receiver.getQueueLength() > 0:
            # Retrieve the message as a UTF-8 string
            message = receiver.getString()
            
            # Parse the JSON data
            waypoint_data = json.loads(message)
            
            # Print the received waypoint data
            for waypoint_id, position in waypoint_data.items():
                # print(f"Received position of {waypoint_id}: {position}")
                if waypoint_id == "way_1":
                    waypoint = position;
                    # print(waypoint)
            # Clear the receiver queue
            receiver.nextPacket()
        
        dt = robot.getTime() - past_time
        actual_state = {}

        if first_time:
            past_x_global = gps.getValues()[0]
            past_y_global = gps.getValues()[1]
            past_time = robot.getTime()
            first_time = False

        # Get sensor data
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global)/dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global)/dt
        altitude = gps.getValues()[2]
        
        
        #emits the current location
        current_position = gps.getValues()
        message = json.dumps({'position': current_position})
        emitter.send(message)  
        # Get body fixed velocities
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw

        # Initialize values
        desired_state = [0, 0, 0, 0]
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0
        
        # if current_waypoint_index < len(waypoints):
        # waypoint = waypoints[current_waypoint_index]
        x_wp, y_wp, z_wp = waypoint  # Unpack waypoint coordinates
        # print(waypoint)
        # Compute the distance to the waypoint
        distance_to_waypoint = ((x_global - x_wp)**2 + (y_global - y_wp)**2 + (altitude - z_wp)**2)**0.5

        # Threshold to consider waypoint reached
        threshold = 0.25

        if distance_to_waypoint < threshold:
            # print(f"Waypoint {current_waypoint_index + 1} reached at position "
                  # f"({x_global:.2f}, {y_global:.2f}, {altitude:.2f})")
            current_waypoint_index += 1
            forward_desired = 0
            sideways_desired = 0
            yaw_desired = 0
            height_desired = altitude
            print("Reached the final waypoint. Drone is hovering.")
            speaker.playSound(speaker, speaker, "../../sounds/beep_3.wav", 1.0, 1.0, 0.0, False)
        
        
        gaze_point = gaze_data[gaze_index]
        if (gaze_point[2]-time_start) <= current_time:
            gaze_x = gaze_point[0]*0.01  # Gaze X position (normalized, 0-100)
            gaze_y = gaze_point[1]*0.01  # Gaze Y position (normalized, 0-100)
            gaze_index += 1
        else:
            gaze_x, gaze_y = None, None  # No new gaze data
        
        print(gaze_x)
        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                forward_desired += 0.5
            elif key == Keyboard.DOWN:
                forward_desired -= 0.5
            elif key == Keyboard.RIGHT:
                sideways_desired -= 0.5
            elif key == Keyboard.LEFT:
                sideways_desired += 0.5
            elif key == ord('Q'):
                yaw_desired = + 1
            elif key == ord('E'):
                yaw_desired = - 1
            elif key == ord('W'):
                height_diff_desired = 0.1
            elif key == ord('S'):
                height_diff_desired = - 0.1
            elif key == ord('A'):
                if autonomous_mode is False:
                    autonomous_mode = True
                    print("Autonomous mode: ON")
            elif key == ord('D'):
                if autonomous_mode is True:
                    autonomous_mode = False
                    print("Autonomous mode: OFF")
            key = keyboard.getKey()
        
        # GAZE_ASSISTANCE_ENABLED=True
                # Apply gaze assistance if enabled
        if GAZE_ASSISTANCE_ENABLED and gaze_x is not None and gaze_y is not None:
            # Adjust yaw based on gaze X (centered at 50)
            yaw_desired += GAZE_ASSISTANCE_STRENGTH * (gaze_x - 50) / 50

            # Adjust pitch (desired forward tilt) based on gaze Y (centered at 50)
            forward_desired += GAZE_ASSISTANCE_STRENGTH * (50 - gaze_y) / 50
            
        # print(yaw_desired);
        # print(forward_desired);
        height_desired += height_diff_desired * dt
        # print(f"GPS Values: {gps.getValues()}")
        camera_data = camera.getImage()

        # get range in meters
        range_front_value = range_front.getValue() / 1000
        range_right_value = range_right.getValue() / 1000
        range_left_value = range_left.getValue() / 1000

        # Choose a wall following direction
        # if you choose direction left, use the right range value
        # if you choose direction right, use the left range value
        direction = WallFollowing.WallFollowingDirection.LEFT
        range_side_value = range_right_value

        # Get the velocity commands from the wall following state machine
        cmd_vel_x, cmd_vel_y, cmd_ang_w, state_wf = wall_following.wall_follower(
            range_front_value, range_side_value, yaw, direction, robot.getTime())

        if autonomous_mode:
            sideways_desired = cmd_vel_y
            forward_desired = cmd_vel_x
            yaw_desired = cmd_ang_w

        # PID velocity controller with fixed height
        motor_power = PID_crazyflie.pid(dt, forward_desired, sideways_desired,
                                        yaw_desired, height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)

        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global
