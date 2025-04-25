# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np

class QTrobotArmsSolver:

    def __init__(self):
        pass
                

    def calculate_right_arm_angles(self, xyz):
        x = xyz[0]
        y = xyz[1]
        z = xyz[2]
  
        # Given parameters
        d1 = 0.4  # translation from F0 to F1 along z-axis
        d2 = 0.12  # translation from F1 to F2 along z-axis
        d3 = 0.26  # translation from F2 to End-Effector along x-axis        
        z = z - d1
        y = y - d2
        # np.arctan2(y, x) where y = z - d1
        theta_1 = np.arctan2(z, x)        
        d_xz = np.sqrt(x**2 + z**2)
        theta_2 = -np.arctan2(d_xz, -y)
        theta_3 = np.radians(-7.0)

        # Clamp HeadYaw and HeadPitch to their limits
        theta_1 = np.clip(theta_1, np.radians(-140), np.radians(140))
        # theta_2 = np.clip(theta_2, np.radians(-75), np.radians(-5.0))        

        return [np.degrees(theta_1), np.degrees(theta_2), np.degrees(theta_3)]


    def calculate_left_arm_angles(self, xyz):
        x = xyz[0]
        y = xyz[1]
        z = xyz[2]
  
        # Given parameters
        d1 = 0.4  # translation from F0 to F1 along z-axis
        d2 = 0.12  # translation from F1 to F2 along z-axis
        d3 = 0.26  # translation from F2 to End-Effector along x-axis        
        z = z - d1
        y = y - d2
        # np.arctan2(y, x) where y = z - d1
        theta_1 = -np.arctan2(z, x)        
        d_xz = np.sqrt(x**2 + z**2)
        theta_2 = -np.arctan2(d_xz, y)
        theta_3 = np.radians(-7.0)

        # Clamp HeadYaw and HeadPitch to their limits
        theta_1 = np.clip(theta_1, np.radians(-140), np.radians(140))
        theta_2 = np.clip(theta_2, np.radians(-75), np.radians(-5.0))        

        return [np.degrees(theta_1), np.degrees(theta_2), np.degrees(theta_3)]

