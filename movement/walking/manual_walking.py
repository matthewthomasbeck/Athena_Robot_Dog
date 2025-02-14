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

import time # import time library for time functions
import math # import math library for pi, used with elliptical movement
import logging # import logging for debugging

##### import necessary functions #####

import initialize.initialize_servos as initialize_servos # import servo logic functions


########## CREATE DEPENDENCIES ##########

##### define servos #####

upper_leg_servos = { # define upper leg servos

    "FL": initialize_servos.LEG_CONFIG['FL']['upper'],  # front left
    "FR": initialize_servos.LEG_CONFIG['FR']['upper'],  # front right
    "BL": initialize_servos.LEG_CONFIG['BL']['upper'],  # back left
    "BR": initialize_servos.LEG_CONFIG['BR']['upper'],  # back right
}

lower_leg_servos = { # define lower leg servos

    "FL": initialize_servos.LEG_CONFIG['FL']['lower'],  # front left
    "FR": initialize_servos.LEG_CONFIG['FR']['lower'],  # front right
    "BL": initialize_servos.LEG_CONFIG['BL']['lower'],  # back left
    "BR": initialize_servos.LEG_CONFIG['BR']['lower'],  # back right
}





#################################################
############### WALKING FUNCTIONS ###############
#################################################


########## CALCULATE INTENSITY ##########

def interpretIntensity(intensity, full_back, full_front): # function to interpret intensity

    ##### find intensity value to calculate arc later #####

    # find intensity by dividing the difference between full_back and full front,
    # converting to positive, dividing by 10, and multiplying by intensity
    arc_length = (abs(full_back - full_front) / 10) * intensity

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

    ##### return arc length speed and acceleration #####

    return arc_length, speed, acceleration # return movement parameters


########## WALK BACKWARD ##########

def walkBackward(intensity): # function to oscillate one servo

    ##### set vairables #####

    upper_arc_lengths = []  # Store all arc lengths for uniform movement distance
    speeds = []  # Store all speeds for uniform movement speed
    accelerations = []  # Store all accelerations for uniform movement acceleration

    ##### Find movement parameters #####
    for leg, upper_servo_data in initialize_servos.LEG_CONFIG.items(): # Loop through upper leg servos to get parameters with intensity
        upper_servo_data = initialize_servos.LEG_CONFIG[leg]['upper']
        full_back = upper_servo_data['FULL_BACK']  # Get full back position
        full_front = upper_servo_data['FULL_FRONT']  # Get full front position
        arc_length, speed, acceleration = interpretIntensity(intensity, full_back, full_front)  # Get movement parameters
        upper_arc_lengths.append(arc_length)  # Append arc length to list
        speeds.append(speed)  # Append speed to list
        accelerations.append(acceleration)  # Append acceleration to list
        upper_servo_data['MOVED'] = False

    min_upper_arc_length = min(upper_arc_lengths)  # Get minimum arc length
    min_speed = min(speeds)  # Get minimum speed
    min_acceleration = min(accelerations)  # Get minimum acceleration

    ##### move legs #####

    forwardFrontLeft(min_upper_arc_length, min_speed, min_acceleration)
    forwardBackRight(min_upper_arc_length, min_speed, min_acceleration)
    forwardFrontRight(min_upper_arc_length, min_speed, min_acceleration)
    forwardBackLeft(min_upper_arc_length, min_speed, min_acceleration)




########## WALK FORWARD ##########

def walkForward(intensity): # function to oscillate one servo

    ##### set vairables #####

    upper_arc_lengths = []  # Store all arc lengths for uniform movement distance
    speeds = []  # Store all speeds for uniform movement speed
    accelerations = []  # Store all accelerations for uniform movement acceleration

    ##### Find movement parameters #####
    for leg, upper_servo_data in initialize_servos.LEG_CONFIG.items(): # Loop through upper leg servos to get parameters with intensity
        upper_servo_data = initialize_servos.LEG_CONFIG[leg]['upper']
        full_back = upper_servo_data['FULL_BACK']  # Get full back position
        full_front = upper_servo_data['FULL_FRONT']  # Get full front position
        arc_length, speed, acceleration = interpretIntensity(intensity, full_back, full_front)  # Get movement parameters
        upper_arc_lengths.append(arc_length)  # Append arc length to list
        speeds.append(speed)  # Append speed to list
        accelerations.append(acceleration)  # Append acceleration to list
        upper_servo_data['MOVED'] = False

    min_upper_arc_length = min(upper_arc_lengths)  # Get minimum arc length
    min_speed = min(speeds)  # Get minimum speed
    min_acceleration = min(accelerations)  # Get minimum acceleration

    ##### move legs #####

    forwardFrontLeft(min_upper_arc_length, min_speed, min_acceleration)
    forwardBackRight(min_upper_arc_length, min_speed, min_acceleration)
    forwardFrontRight(min_upper_arc_length, min_speed, min_acceleration)
    forwardBackLeft(min_upper_arc_length, min_speed, min_acceleration)


# function to oscillate lower leg
def liftLowerLeg(servo_name, min_upper_arc_length, speed, acceleration):

    logging.debug(f"Lifting up via {min_upper_arc_length}...")

    initialize_servos.setTarget(servo_name, (-1 * min_upper_arc_length), speed, acceleration)

def neutralLowerLeg(servo_name, neutral_lower_leg, speed, acceleration):

    logging.debug(f"Stepping down via neutral {neutral_lower_leg}...")

    initialize_servos.setTarget(servo_name, neutral_lower_leg, speed, acceleration)

def lowerLowerLeg(servo_name, min_upper_arc_length, speed, acceleration):

    logging.debug(f"Planting foot via {min_upper_arc_length}...")

    initialize_servos.setTarget(servo_name, (1 * min_upper_arc_length), speed, acceleration)



##### MOVE INDIVIDUAL LEGS #####

def forwardFrontLeft(min_upper_arc_length, min_speed, min_acceleration):  # function to move individual leg

    upper_servo_data = upper_leg_servos["FL"]
    lower_servo_data = lower_leg_servos["FL"]

    full_back = upper_servo_data['FULL_BACK']
    full_front = upper_servo_data['FULL_FRONT']
    neutral_position = upper_servo_data['NEUTRAL']

    if full_back < full_front:
        max_limit = neutral_position + (min_upper_arc_length / 2)
        min_limit = neutral_position - (min_upper_arc_length / 2)
    else:
        min_upper_arc_length = (-1 * min_upper_arc_length)
        max_limit = neutral_position + (min_upper_arc_length / 2)
        min_limit = neutral_position - (min_upper_arc_length / 2)

    # Initialize movement direction
    if upper_servo_data['DIR'] == 0:
        upper_servo_data['DIR'] = 1  # Move forward first

    logging.info("Moving FL leg.\n")

    # Change direction at limits
    if upper_servo_data['DIR'] == 1: # if leg set to move forward...

        upper_new_pos = max_limit
        upper_servo_data['CUR_POS'] = upper_new_pos
        initialize_servos.LEG_CONFIG["FL"]['upper']['CUR_POS'] = upper_new_pos

        liftLowerLeg(  # Lift-up lower leg
            lower_servo_data['servo'],
            min_upper_arc_length,
            min_speed,
            min_acceleration
        )

        time.sleep(0.1)

        logging.debug(f"Swinging FL leg via {upper_new_pos}...")

        initialize_servos.setTarget(upper_servo_data['servo'], upper_new_pos, min_speed, min_acceleration)

        time.sleep(0.05)

        neutralLowerLeg(  # Touch down lower leg
            lower_servo_data['servo'],
            lower_servo_data['NEUTRAL'],
            min_speed,
            min_acceleration
        )

        time.sleep(0.05)

        upper_servo_data['DIR'] = -1  # Move backward next cycle

        logging.info("FL stepped forward.\n")

    elif upper_servo_data['DIR'] == -1: # if leg set to move backward...

        upper_new_pos = min_limit
        upper_servo_data['CUR_POS'] = upper_new_pos
        initialize_servos.LEG_CONFIG["FL"]['upper']['CUR_POS'] = upper_new_pos

        lowerLowerLeg(  # Touch down lower leg
            lower_servo_data['servo'],
            min_upper_arc_length,
            min_speed,
            min_acceleration
        )

        time.sleep(0.1)

        logging.debug(f"Pushing back FL leg via {upper_new_pos}...")

        initialize_servos.setTarget(upper_servo_data['servo'], upper_new_pos, min_speed, min_acceleration)

        time.sleep(0.05)

        logging.info("FL pushed backward.\n")

        upper_servo_data['DIR'] = 1  # Move forward next cycle

    upper_servo_data['MOVED'] = True


def forwardFrontRight(min_upper_arc_length, min_speed, min_acceleration):  # function to move individual leg

    upper_servo_data = upper_leg_servos["FR"]
    lower_servo_data = lower_leg_servos["FR"]

    full_back = upper_servo_data['FULL_BACK']
    full_front = upper_servo_data['FULL_FRONT']
    neutral_position = upper_servo_data['NEUTRAL']

    if full_back < full_front:
        max_limit = neutral_position + (min_upper_arc_length / 2)
        min_limit = neutral_position - (min_upper_arc_length / 2)
    else:
        min_upper_arc_length = (-1 * min_upper_arc_length)
        max_limit = neutral_position + (min_upper_arc_length / 2)
        min_limit = neutral_position - (min_upper_arc_length / 2)

    # Initialize movement direction
    if upper_servo_data['DIR'] == 0:
        upper_servo_data['DIR'] = -1  # Move backward first

    logging.info("Moving FR leg.\n")

    # Change direction at limits
    if upper_servo_data['DIR'] == 1: # if leg set to move forward...

        upper_new_pos = max_limit
        upper_servo_data['CUR_POS'] = upper_new_pos
        initialize_servos.LEG_CONFIG["FR"]['upper']['CUR_POS'] = upper_new_pos

        liftLowerLeg(  # Lift-up lower leg
            lower_servo_data['servo'],
            min_upper_arc_length,
            min_speed,
            min_acceleration
        )

        time.sleep(0.1)

        logging.debug(f"Swinging FR leg via {upper_new_pos}...")

        initialize_servos.setTarget(upper_servo_data['servo'], upper_new_pos, min_speed, min_acceleration)

        time.sleep(0.05)

        neutralLowerLeg(  # Touch down lower leg
            lower_servo_data['servo'],
            lower_servo_data['NEUTRAL'],
            min_speed,
            min_acceleration
        )

        time.sleep(0.05)

        upper_servo_data['DIR'] = -1  # Move backward next cycle

        logging.info("FR stepped forward.\n")

    elif upper_servo_data['DIR'] == -1: # if leg set to move backward...

        upper_new_pos = min_limit
        upper_servo_data['CUR_POS'] = upper_new_pos
        initialize_servos.LEG_CONFIG["FR"]['upper']['CUR_POS'] = upper_new_pos

        lowerLowerLeg(  # Touch down lower leg
            lower_servo_data['servo'],
            min_upper_arc_length,
            min_speed,
            min_acceleration
        )

        time.sleep(0.1)

        logging.debug(f"Pushing back FR leg via {upper_new_pos}...")

        initialize_servos.setTarget(upper_servo_data['servo'], upper_new_pos, min_speed, min_acceleration)

        time.sleep(0.05)

        logging.info("FR pushed backward.\n")

        upper_servo_data['DIR'] = 1

    upper_servo_data['MOVED'] = True


def forwardBackLeft(min_upper_arc_length, min_speed, min_acceleration):  # function to move individual leg

    upper_servo_data = upper_leg_servos["BL"]
    lower_servo_data = lower_leg_servos["BL"]

    full_back = upper_servo_data['FULL_BACK']
    full_front = upper_servo_data['FULL_FRONT']
    neutral_position = upper_servo_data['NEUTRAL']

    if full_back < full_front:
        max_limit = neutral_position + (min_upper_arc_length / 2)
        min_limit = neutral_position - (min_upper_arc_length / 2)
    else:
        min_upper_arc_length = (-1 * min_upper_arc_length)
        max_limit = neutral_position + (min_upper_arc_length / 2)
        min_limit = neutral_position - (min_upper_arc_length / 2)

    # Initialize movement direction
    if upper_servo_data['DIR'] == 0:
        upper_servo_data['DIR'] = -1  # Move backward first

    logging.info("Moving BL leg.\n")

    # Change direction at limits
    if upper_servo_data['DIR'] == 1: # if leg set to move forward...

        upper_new_pos = max_limit
        upper_servo_data['CUR_POS'] = upper_new_pos
        initialize_servos.LEG_CONFIG["BL"]['upper']['CUR_POS'] = upper_new_pos

        liftLowerLeg(  # Lift-up lower leg
            lower_servo_data['servo'],
            min_upper_arc_length,
            min_speed,
            min_acceleration
        )

        time.sleep(0.1)

        logging.debug(f"Swinging BL leg via {upper_new_pos}...")

        initialize_servos.setTarget(upper_servo_data['servo'], upper_new_pos, min_speed, min_acceleration)

        time.sleep(0.05)

        neutralLowerLeg(  # Touch down lower leg
            lower_servo_data['servo'],
            lower_servo_data['NEUTRAL'],
            min_speed,
            min_acceleration
        )

        time.sleep(0.05)

        upper_servo_data['DIR'] = -1  # Move backward next cycle

        logging.info("BL stepped forward.\n")

    elif upper_servo_data['DIR'] == -1: # if leg set to move backward...

        upper_new_pos = min_limit
        upper_servo_data['CUR_POS'] = upper_new_pos
        initialize_servos.LEG_CONFIG["BL"]['upper']['CUR_POS'] = upper_new_pos

        lowerLowerLeg(  # Touch down lower leg
            lower_servo_data['servo'],
            min_upper_arc_length,
            min_speed,
            min_acceleration
        )

        time.sleep(0.1)

        logging.debug(f"Pushing back BL leg via {upper_new_pos}...")

        initialize_servos.setTarget(upper_servo_data['servo'], upper_new_pos, min_speed, min_acceleration)

        time.sleep(0.05)

        logging.info("BL pushed backward.\n")

        upper_servo_data['DIR'] = 1

    upper_servo_data['MOVED'] = True


def forwardBackRight(min_upper_arc_length, min_speed, min_acceleration):  # function to move individual leg

    upper_servo_data = upper_leg_servos["BR"]
    lower_servo_data = lower_leg_servos["BR"]

    full_back = upper_servo_data['FULL_BACK']
    full_front = upper_servo_data['FULL_FRONT']
    neutral_position = upper_servo_data['NEUTRAL']

    if full_back < full_front:
        max_limit = neutral_position + (min_upper_arc_length / 2)
        min_limit = neutral_position - (min_upper_arc_length / 2)
    else:
        min_upper_arc_length = (-1 * min_upper_arc_length)
        max_limit = neutral_position + (min_upper_arc_length / 2)
        min_limit = neutral_position - (min_upper_arc_length / 2)

    # Initialize movement direction
    if upper_servo_data['DIR'] == 0:
        upper_servo_data['DIR'] = 1  # Move forward first

    logging.info("Moving BR leg.\n")

    # Change direction at limits
    if upper_servo_data['DIR'] == 1: # if leg set to move forward...

        upper_new_pos = max_limit
        upper_servo_data['CUR_POS'] = upper_new_pos
        initialize_servos.LEG_CONFIG["BR"]['upper']['CUR_POS'] = upper_new_pos

        liftLowerLeg(  # Lift-up lower leg
            lower_servo_data['servo'],
            min_upper_arc_length,
            min_speed,
            min_acceleration
        )

        time.sleep(0.1)

        logging.debug(f"Swinging BR leg via {upper_new_pos}...")

        initialize_servos.setTarget(upper_servo_data['servo'], upper_new_pos, min_speed, min_acceleration)

        time.sleep(0.05)

        neutralLowerLeg(  # Touch down lower leg
            lower_servo_data['servo'],
            lower_servo_data['NEUTRAL'],
            min_speed,
            min_acceleration
        )

        time.sleep(0.05)

        upper_servo_data['DIR'] = -1  # Move backward next cycle

        logging.info("BR stepped forward.\n")

    elif upper_servo_data['DIR'] == -1: # if leg set to move backward...

        upper_new_pos = min_limit
        upper_servo_data['CUR_POS'] = upper_new_pos
        initialize_servos.LEG_CONFIG["BR"]['upper']['CUR_POS'] = upper_new_pos

        lowerLowerLeg(  # Touch down lower leg
            lower_servo_data['servo'],
            min_upper_arc_length,
            min_speed,
            min_acceleration
        )

        time.sleep(0.1)

        logging.debug(f"Pushing back BR leg via {upper_new_pos}...")

        initialize_servos.setTarget(upper_servo_data['servo'], upper_new_pos, min_speed, min_acceleration)

        time.sleep(0.05)

        logging.info("BR pushed backward.\n")

        upper_servo_data['DIR'] = 1

    upper_servo_data['MOVED'] = True