# -*- coding: utf-8 -*-
"""
Created on Sun Sep 06 22:17:54 2015

@author: mike
"""

import math
import numpy as np
VERTICAL_OFFSET = 85
    
def rotation(x, y, z, psi, theta):
    """ Converts from Vehicle-2 (yaw,pitch) reference frame to original frame
    Input x, y, z in the rotated frame, along with psi and theta
    returns the position in the original frame. Also handles the translation, since
    lidar center is not at the center of the pitch rotation servo axis
    Remember that Z is positive down.
    """
    pos = np.matrix((x, y, z)).T
    theta_rad = math.radians(theta)
    psi_rad = math.radians(psi)
    c_theta = math.cos(theta_rad)
    s_theta = math.sin(theta_rad)
    c_psi = math.cos(psi_rad)
    s_psi = math.sin(psi_rad)
    Yaw = np.matrix(((c_psi, -s_psi, 0),(s_psi, c_psi, 0),(0, 0, 1)))
    Pitch = np.matrix(((c_theta, 0, s_theta),(0, 1, 0),(-s_theta, 0, c_theta)))
    new_pos = (Yaw*Pitch)*pos
    new_x = float(new_pos[0]) - VERTICAL_OFFSET * math.sin(theta)
    new_y = float(new_pos[1])
    new_z = float(new_pos[2]) + VERTICAL_OFFSET * math.cos(theta) - VERTICAL_OFFSET
    return new_x, new_y, new_z
   