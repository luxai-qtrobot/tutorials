# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np

class QTrobotHeadSolver:
    def __init__(self):
        pass


    def pixel_to_base(self, uv, depth, head_yaw, head_pitch):
        xyz_camera = self.pixel_to_camera(uv[0], uv[1], depth)  
        xyz_base = self.camera_to_base(xyz_camera, head_yaw, head_pitch)
        return [depth, xyz_base[1], xyz_base[2]]


    def pixel_to_camera(self, u, v, depth):
        # intrisnict params
        fx = 376.403076171875
        fy = 376.084228515625
        ppx = 314.64068603515625
        ppy = 255.57965087890625
        cam_offset_z = 0.09  # camera is 9cm ahead of the center of HeadPitch joint  

        u = u + (ppx-320)
        v = v + (ppy-240)

        # Convert pixel coordinates to camera frame coordinates
        x_camera = (u - ppx) * depth / fx  # x-axis of camera frame has oposit directon of 'u'
        y_camera = (v - ppy) * depth / fy
        z_camera = depth + cam_offset_z
        return [x_camera, y_camera, z_camera]


    def camera_to_base(self, xyz_camera, head_yaw, head_pitch):
        theta1 = np.radians(head_yaw)
        theta2 = np.radians(head_pitch)
        point_F3 = np.array(xyz_camera)

        T_0_3 = self._transformation_matrix(theta1, theta2)    
        # Transform the point to the base frame F0
        point_F0_homogeneous = np.dot(T_0_3, np.append(point_F3, 1))
        # Convert back to Cartesian coordinates    
        return point_F0_homogeneous[:3].tolist()


    def calculate_xyz(self, head_yaw, head_pitch, depth):
        # Camera position relative to base_link (height)
        camera_height = 0.6
        theta1 = np.radians(head_yaw)
        theta2 = np.radians(head_pitch)        
        x = depth * np.cos(theta1)
        y = depth * np.sin(theta1)
        z = camera_height - depth * np.sin(theta2)
        return [x, y, z]


    def calculate_head_angles(self, xyz):
        x = xyz[0]
        y = xyz[1]
        z = xyz[2]
        # Camera position relative to base_link (height)
        camera_height = 0.6
        # Calculate the HeadYaw angle
        head_yaw = np.arctan2(y, x)
        # Calculate the distance from the base_link to the target point in the x-y plane
        d_xy = np.sqrt(x**2 + y**2)
        # Calculate the HeadPitch angle
        head_pitch = np.arctan2(camera_height-z, d_xy) # head_pitch = np.arctan2(camera_height - z, d_xy)
        # Clamp HeadYaw and HeadPitch to their limits
        head_yaw = np.clip(head_yaw, np.radians(-60), np.radians(60))
        head_pitch = np.clip(head_pitch, np.radians(-25), np.radians(25))
        # Convert radians to degrees
        head_yaw_deg = np.degrees(head_yaw)
        head_pitch_deg = np.degrees(head_pitch)
        return [head_yaw_deg, head_pitch_deg]


    def _transformation_matrix(self, theta1, theta2):
        T_0_1 = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0.34],
                [0, 0, 0, 1]
            ])

        cos_theta1 = np.cos(theta1)
        sin_theta1 = np.sin(theta1)
        T_1_2 = np.array([
            [cos_theta1, 0, -sin_theta1, 0],
            [sin_theta1, 0, cos_theta1, 0],
            [0, -1, 0, 0.1],
            [0, 0, 0, 1]
        ])
        
        cos_theta2 = np.cos(theta2)
        sin_theta2 = np.sin(theta2)
        T_2_3 = np.array([            
            [0, -sin_theta2, cos_theta2, 0.16*sin_theta2],
            [0, cos_theta2, sin_theta2, -0.16*cos_theta2],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ])

        # print(theta1, theta2, T_2_3[:, -1])
        # Compute the final transformation matrix T^0_3 by matrix multiplication
        T_0_2 = np.dot(T_0_1, T_1_2)
        T_0_3 = np.dot(T_0_2, T_2_3)
        return  T_0_3