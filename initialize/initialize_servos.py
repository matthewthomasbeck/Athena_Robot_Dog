##################################################################################
# Copyright (c) 2024 Matthew Thomas Beck                                         #
#                                                                                #
# All rights reserved. This code and its associated files may not be reproduced, #
# modified, distributed, or otherwise used, in part or in whole, by any person   #
# or entity without the express written permission of the copyright holder,      #
# Matthew Thomas Beck.                                                           #
##################################################################################





############################################################
############### IMPORT / CREATE DEPENDENCIES ###############
############################################################


########## IMPORT DEPENDENCIES ##########

##### import necessary libraries #####

import logging # import logging for debugging
import math

##### import necessary functions #####

from initialize.initialize_maestro import * # import maestro initialization functions


########## CREATE DEPENDENCIES ##########

##### create maestro object #####

MAESTRO = createMaestroConnection() # create maestro connection

##### set dictionary of servos and their ranges #####

LEG_CONFIG = { # dictionary of leg configurations

    'FL': {'hip': {'servo': 3, 'FULL_BACK': 1236.50, 'FULL_FRONT': 1892.25, 'NEUTRAL': 1564.375, 'CUR_POS': 1564.375, 'DIR': 1},
           'upper': {'servo': 5, 'FULL_BACK': 1921.50, 'FULL_FRONT': 1266.00, 'NEUTRAL': 1593.75, 'CUR_POS': 1593.75, 'DIR': -1},
           'lower': {'servo': 4, 'FULL_BACK': 1872.75, 'FULL_FRONT': 1148.50, 'NEUTRAL': 1510.625, 'CUR_POS': 1510.625, 'DIR': -1}},

    'FR': {'hip': {'servo': 2, 'FULL_BACK': 1613.25, 'FULL_FRONT': 992.00, 'NEUTRAL': 1302.625, 'CUR_POS': 1302.625, 'DIR': -1},
           'upper': {'servo': 1, 'FULL_BACK': 1310.00, 'FULL_FRONT': 1921.50, 'NEUTRAL': 1615.75, 'CUR_POS': 1615.75, 'DIR': 1},
           'lower': {'servo': 0, 'FULL_BACK': 1231.75, 'FULL_FRONT': 2000.00, 'NEUTRAL': 1615.875, 'CUR_POS': 1615.875, 'DIR': 1}},

    'BL': {'hip': {'servo': 8, 'FULL_BACK': 1623.00, 'FULL_FRONT': 1036.00, 'NEUTRAL': 1329.5, 'CUR_POS': 1329.5, 'DIR': 1},
           'upper': {'servo': 7, 'FULL_BACK': 2000.00, 'FULL_FRONT': 1354.00, 'NEUTRAL': 1777.0, 'CUR_POS': 1777.0, 'DIR': -1},
           'lower': {'servo': 6, 'FULL_BACK': 2000.00, 'FULL_FRONT': 1138.75, 'NEUTRAL': 1669.375, 'CUR_POS': 1669.375, 'DIR': -1}},

    'BR': {'hip': {'servo': 11, 'FULL_BACK': 1261.00, 'FULL_FRONT': 1848.25, 'NEUTRAL': 1554.625, 'CUR_POS': 1554.625, 'DIR': -1},
           'upper': {'servo': 10, 'FULL_BACK': 1065.25, 'FULL_FRONT': 1701.50, 'NEUTRAL': 1283.375, 'CUR_POS': 1283.375, 'DIR': 1},
           'lower': {'servo': 9, 'FULL_BACK': 1221.75, 'FULL_FRONT': 2000.00, 'NEUTRAL': 1510.875, 'CUR_POS': 1510.875, 'DIR': 1}},
}

##### servo constraints #####

MAX_VELOCITY = 16383
MAX_ACCELERATION = 255
MAX_YAW_RATE = 9.52 # radians per second

##### leg dimensions #####

HIP_OFFSET = 5 # distance from hip axis to center of body
FEMUR = 11 # length of femur
TIBIA = 12.25 # length of tibia

##### robot dimensions #####

BODY_WIDTH = 10 # width of robot body from hip axis to hip axis
BODY_LENGTH = 20 # length of robot body from 'shoulder' axis to 'shoulder' axis (upper femur joint)
BODY_HEIGHT = 18 # height of robot body from ground to 'shoulder' axis





#############################################################
############### FUNDAMENTAL MOVEMENT FUNCTION ###############
#############################################################


########## INVERSE KINEMATICS ##########

def inverse_kinematics(target_x, target_y):

    d = math.sqrt(target_x ** 2 + target_y ** 2)

    # Prevent invalid d values
    if d > (FEMUR + TIBIA):
        d = FEMUR + TIBIA  # Max reach

    if d < abs(FEMUR - TIBIA):
        d = abs(FEMUR - TIBIA)  # Minimum reach

    # Solve for lower leg angle (theta2)
    cos_theta2 = (FEMUR ** 2 + TIBIA ** 2 - d ** 2) / (2 * FEMUR * TIBIA)
    theta2 = math.acos(cos_theta2)  # In radians

    # Solve for upper leg angle (theta1)
    theta1 = math.atan2(target_y, target_x) + math.atan2(
        TIBIA * math.sin(theta2), FEMUR + TIBIA * math.cos(theta2)
    )

    # Convert to degrees
    theta1 = math.degrees(theta1)
    theta2 = math.degrees(theta2)

    return theta1, theta2


########## CALCULATE INTENSITY ##########

def interpretIntensity(intensity): # function to interpret intensity

    ##### find speed and acceleration #####

    if intensity == 1 or intensity == 2:
        speed = int(((16383 / 5) / 10) * intensity)
        acceleration = int(((255 / 5) / 10) * intensity)
    elif intensity == 3 or intensity == 4:
        speed = int(((16383 / 4) / 10) * intensity)
        acceleration = int(((255 / 4) / 10) * intensity)
    elif intensity == 5 or intensity == 6:
        speed = int(((16383 / 3) / 10) * intensity)
        acceleration = int(((255 / 3) / 10) * intensity)
    elif intensity == 7 or intensity == 8:
        speed = int(((16383 / 2) / 10) * intensity)
        acceleration = int(((255 / 2) / 10) * intensity)
    else:
        speed = int((16383 / 10) * intensity)
        acceleration = int((255 / 10) * intensity)

    ##### return speed and acceleration #####

    return speed, acceleration # return movement parameters


########## MOVE A SINGLE SERVO ##########

def setTarget(channel, target, speed, acceleration): # function to set target position of a singular servo

    ##### move a servo to a desired position using its number and said position #####

    try: # attempt to move desired servo

        target = int(round(target * 4)) # convert target from microseconds to quarter-microseconds

        # ensure speed and acceleration are within valid ranges
        speed = max(0, min(16383, speed))
        acceleration = max(0, min(255, acceleration))

        # create speed command
        speed_command = bytearray([0x87, channel, speed & 0x7F, (speed >> 7) & 0x7F])
        MAESTRO.write(speed_command)

        # create acceleration command
        accel_command = bytearray([0x89, channel, acceleration & 0x7F, (acceleration >> 7) & 0x7F])
        MAESTRO.write(accel_command)

        # create and send target position command
        command = bytearray([0x84, channel, target & 0x7F, (target >> 7) & 0x7F])
        MAESTRO.write(command)

    except: # if movement failed...

        logging.error("ERROR (initialize_servos.py): Failed to move servo.\n") # print failure statement


########## MOVE LEG ##########

def moveLeg(leg_name, target_x, target_y, min_speed, min_acceleration):

    if leg_name not in LEG_CONFIG:
        logging.error(f"Invalid leg name: {leg_name}")
        return

    upper_servo_data = LEG_CONFIG[leg_name]['upper']
    lower_servo_data = LEG_CONFIG[leg_name]['lower']

    theta1, theta2 = inverse_kinematics(target_x, target_y)

    # Adjust angles based on the servo's rotation direction
    upper_direction = 1 if upper_servo_data['FULL_FRONT'] > upper_servo_data['FULL_BACK'] else -1
    lower_direction = 1 if lower_servo_data['FULL_FRONT'] > lower_servo_data['FULL_BACK'] else -1

    # Convert IK angles to servo positions with direction accounted for
    upper_new_pos = (
        upper_servo_data['NEUTRAL']
        + upper_direction * ((theta1 - 144.79) * (upper_servo_data['FULL_FRONT'] - upper_servo_data['FULL_BACK']) / 90) # 144.79
    )

    lower_new_pos = (
        lower_servo_data['NEUTRAL']
        + lower_direction * ((theta2 - 99.96) * (lower_servo_data['FULL_FRONT'] - lower_servo_data['FULL_BACK']) / 90) # 99.96
    )

    logging.info(f"Moving {leg_name} leg to ({target_x}, {target_y}) -> Upper: {theta1}°, Lower: {theta2}°")

    setTarget(upper_servo_data['servo'], upper_new_pos, min_speed, min_acceleration)
    setTarget(lower_servo_data['servo'], lower_new_pos, min_speed, min_acceleration)

    logging.info(f"Upper servo moved to {upper_new_pos} lower servo moved to {lower_new_pos}.\n")

    time.sleep(0.1)

    # Handle direction flipping after step is completed
    upper_servo_data['DIR'] = -1 if upper_servo_data['DIR'] == 1 else 1


########## DISABLE ALL SERVOS ##########

def disableAllServos(): # function to disable servos via code

    ##### make all servos go limp for easy reinitialization #####

    logging.debug("Attempting to disable all servos...\n") # print initialization statement

    try: # attempt to disable all servos

        for leg, joints in LEG_CONFIG.items(): # loop through each leg

            for joint, config in joints.items(): # loop through each joint

                servo = config['servo'] # get the servo number

                setTarget(servo,0xFFFF) # set target to 0 to disable the servo

                logging.info(f"Disabled servo {servo} ({leg} - {joint}).") # print success statement

        logging.info("\nSuccessfully disabled all servos.\n") # print success statement

    ##### exception incase failure to disable servos #####

    except: # if failure to disable any or all servos...

        logging.error("ERROR (initialize_servos.py): Failed to disable servo(s).\n") # print failure statement
