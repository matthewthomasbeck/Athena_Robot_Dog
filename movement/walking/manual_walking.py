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





#################################################
############### WALKING FUNCTIONS ###############
#################################################


########## CYCLE LEG ##########

def cycleFrontLeft(min_speed, min_acceleration):

    logging.info("Moving FL leg...\n")

    initialize_servos.moveLeg('FL', 7, 13, min_speed, min_acceleration)

    time.sleep(1)

    initialize_servos.moveLeg('FL', 0, 18, min_speed, min_acceleration)

    time.sleep(1)

    initialize_servos.moveLeg('FL', -5, 23, min_speed, min_acceleration)

    time.sleep(1)

    initialize_servos.moveLeg('FL', 0, 18, min_speed, min_acceleration)

    time.sleep(1)


########## WALK BACKWARD ##########

def walkBackward(intensity): # function to oscillate one servo

    ##### set variables #####

    speed, acceleration = initialize_servos.interpretIntensity(intensity)  # Get movement parameters

    ##### move legs #####

    cycleFrontLeft(speed, acceleration)

########## WALK FORWARD ##########

def walkForward(intensity): # function to oscillate one servo

    ##### set variables #####

    speed, acceleration = initialize_servos.interpretIntensity(intensity)  # Get movement parameters

    ##### move legs #####

    #moveLeg('FL',0, 18, min_speed, min_acceleration)