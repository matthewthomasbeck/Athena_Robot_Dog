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
import os # import os library for system functions
import sys # import sys library for system functions
import logging # import logging for debugging

##### import necessary functions #####

import initialize.initialize_servos as initialize_servos # import servo logic functions





##########################################################
############### STANDING INPLACE MOVEMENTS ###############
##########################################################


########## NEUTRAL ##########

def neutralStandingPosition():

    logging.debug("Moving to neutral standing position...\n")

    try:

        logging.debug("Preparing legs...\n")

        # Create a dictionary to store the neutral positions dynamically
        new_positions = {}

        # Iterate over LEG_CONFIG to extract NEUTRAL positions
        for leg, joints in initialize_servos.LEG_CONFIG.items():
            for joint, config in joints.items():
                servo_id = config['servo']
                neutral_position = config['NEUTRAL']
                new_positions[servo_id] = neutral_position
                config['DIR'] = 0

                # Move servos to neutral positions
        for servo, position in new_positions.items():
            initialize_servos.setTarget(servo, position, speed=16383, acceleration=255)

        logging.debug("Updating LEG_CONFIG with new positions...\n")

        # Update CUR_POS for each servo in LEG_CONFIG
        for leg, joints in initialize_servos.LEG_CONFIG.items():
            for joint, config in joints.items():
                servo_id = config['servo']
                if servo_id in new_positions:
                    config['CUR_POS'] = new_positions[servo_id]

        time.sleep(0.1) # wait for servos to reach destination

        logging.info("Moved to neutral standing and updated LEG_CONFIG.\n")

    except Exception as e:
        logging.error(f"ERROR (standing_inplace.py): Failed to move to neutral standing position. {e}\n")