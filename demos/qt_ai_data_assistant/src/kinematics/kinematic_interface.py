# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import random
import numpy as np
import rospy
from threading import Lock
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# from qt_robot_interface.srv import emotion_look
from .head_solver import QTrobotHeadSolver
from .arms_solver import QTrobotArmsSolver

class QTrobotKinematicInterface:
    PARTS_HOME_POS = {       
       'head': [0.0, 0.0],
       'right_arm': [-90.0, -55.0, -35.0],       
       'left_arm': [90.0, -55.0, -35.0],       
    }

    LOOK_THRESHOULD_ANGLES = [10, 10]
    GAZE_THRESHOULD_ANGLES = [20, 20]

    def __init__(self):
        self.joints_state = None
        self.joints_state_lock = Lock()

        self.head_solver = QTrobotHeadSolver()
        self.arms_solver = QTrobotArmsSolver()        
    
        # create a publisher and subscribers and service clients 
        # self.gaze = rospy.ServiceProxy('/qt_robot/emotion/look', emotion_look)
        self.head = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=1)
        self.right_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=1)
        self.left_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=1)
        rospy.Subscriber('/qt_robot/joints/state', JointState, self._joint_state_callback)


        # wait for head publisher
        wtime_begin = rospy.get_time()
        while self.head.get_num_connections() == 0:
            if rospy.get_time() - wtime_begin > 5.0:
                raise Exception("Timeout while waiting '/qt_robot/head_position/command' publisher")                
            rospy.sleep(0.1)

        # wait for arm publisher
        wtime_begin = rospy.get_time()
        while self.right_pub.get_num_connections() == 0:
            if rospy.get_time() - wtime_begin > 5.0:
                raise Exception("Timeout while waiting '/qt_robot/right_arm_position/command' publisher")                
            rospy.sleep(0.1)

        # ensure the joint state data is received 
        wtime_begin = rospy.get_time()
        while self.joints_state == None:
            if rospy.get_time() - wtime_begin > 5.0:
                raise Exception("Timeout while waiting joint state values!")
            rospy.sleep(0.1)
                        
    def get_head_pos(self):
        self.joints_state_lock.acquire()
        state = self.joints_state
        head_yaw = state.position[state.name.index("HeadYaw")]
        head_pitch = state.position[state.name.index("HeadPitch")]
        self.joints_state_lock.release()
        return [head_yaw, head_pitch]

    def home(self, parts, sync=False):        
        pos = None
        for part in parts:            
            pos = QTrobotKinematicInterface.PARTS_HOME_POS.get(part)
            # print('homing', part, pos)
            if not pos:
                rospy.logerr(f"invalid part: {part}")
                continue
            self._move_part(part, pos, sync)


    def look_at_xyz(self, xyz, duration=0, sync=False, only_gaze=False):
        head_angles = self.head_solver.calculate_head_angles(xyz)
        self.joints_state_lock.acquire()
        state = self.joints_state
        head_yaw = state.position[state.name.index("HeadYaw")]
        head_pitch = state.position[state.name.index("HeadPitch")]
        self.joints_state_lock.release()     

        yaw_diff = int(head_angles[0] - head_yaw)
        pitch_diff = int(head_angles[1] - head_pitch)
        # xp = np.clip(yaw_diff, -self.GAZE_THRESHOULD_ANGLES[0], self.GAZE_THRESHOULD_ANGLES[0])
        # yp = np.clip(pitch_diff, -self.GAZE_THRESHOULD_ANGLES[1], self.GAZE_THRESHOULD_ANGLES[1])
        # self.gaze([xp, yp], [xp, yp], 0)
        if only_gaze or (abs(yaw_diff) < self.LOOK_THRESHOULD_ANGLES[0] and abs(pitch_diff) < self.LOOK_THRESHOULD_ANGLES[1]):
            return 
        
        if duration <= 0:
            self._move_part('head', head_angles, sync)
            return 
        
        self._move_part('head', head_angles, sync)
        rospy.sleep(duration)
        self._move_part('head', [head_yaw, head_pitch], sync)

    def pixel_to_base(self, uv, depth=1.0, head_pose=None):
        if depth < 0.1:
            depth = 0.1
        if head_pose:
            head_yaw = head_pose[0]
            head_pitch = head_pose[1]
        else:
            self.joints_state_lock.acquire()
            state = self.joints_state
            head_yaw = state.position[state.name.index("HeadYaw")]
            head_pitch = state.position[state.name.index("HeadPitch")]
            self.joints_state_lock.release()
        xyz = self.head_solver.pixel_to_base(uv, depth, 
                                             head_yaw,
                                             head_pitch)   
        return xyz      


    def look_at_pixel(self, uv, depth=1.0, duration=0, sync=False, only_gaze=False):
        xyz = self.pixel_to_base(uv, depth)
        self.look_at_xyz(xyz, duration, sync, only_gaze)


    def point_at_xyz(self, xyz, duration=0, sync=False):        
        y = xyz[1]
        if y < 0 :
            self.reach_right(xyz, duration, sync)
        else:
            self.reach_left(xyz, duration, sync)


    def point_at_pixel(self, uv, depth=1.0, duration=0, sync=False):
        xyz = self.pixel_to_base(uv, depth)
        self.point_at_xyz(xyz, duration, sync)
        

    def reach_right(self, xyz, duration=0, sync=False):        
        arm_angles = self.arms_solver.calculate_right_arm_angles(xyz)            
        if duration <= 0:                
            self._move_part('right_arm', arm_angles, sync)
            return 
        
        self.joints_state_lock.acquire()
        state = self.joints_state
        rsp = state.position[state.name.index("RightShoulderPitch")]
        rsr = state.position[state.name.index("RightShoulderRoll")]
        rer = state.position[state.name.index("RightElbowRoll")]
        self.joints_state_lock.release()
        self._move_part('right_arm', arm_angles, sync)
        rospy.sleep(duration)
        self._move_part('right_arm', [rsp, rsr, rer], sync)        


    def reach_left(self, xyz, duration=0, sync=False):
        arm_angles = self.arms_solver.calculate_left_arm_angles(xyz)            
        if duration <= 0:    
            self._move_part('left_arm', arm_angles, sync)
            return

        self.joints_state_lock.acquire()
        state = self.joints_state
        lsp = state.position[state.name.index("LeftShoulderPitch")]
        lsr = state.position[state.name.index("LeftShoulderRoll")]
        ler = state.position[state.name.index("LeftElbowRoll")]
        self.joints_state_lock.release()
        self._move_part('left_arm', arm_angles, sync)
        rospy.sleep(duration)
        self._move_part('left_arm', [lsp, lsr, ler], sync)        



    def _move_part(self, part, pos, sync=False):         
        if not part in ['head', 'right_arm', 'left_arm']:
            raise ValueError("part must be one of 'head', 'rigt_arm', 'left_arm'")        
        pos_ = [x + random.uniform(*(0.01, 0.1)) for x in pos]
        try:
            ref = Float64MultiArray()
            ref.data = pos_
            if part == 'right_arm':
                self.right_pub.publish(ref)
            elif part == 'head':
                self.head.publish(ref)
            elif part == 'left_arm':
                self.left_pub.publish(ref)
            if sync:
                self._wait_joints_reach(part, pos_)            
        except rospy.ROSInterruptException:
            rospy.logerr(f"could not publish motor commnad to {part}")


    def _wait_joints_reach(self, part, target_pos, timeout=5.0):
        if not part in ['head', 'right_arm', 'left_arm']:
            raise ValueError("part must be one of 'head', 'rigt_arm', 'left_arm'")
                
        t_start = rospy.get_time()
        current_pos = []
        while True:
            if rospy.get_time()-t_start > timeout: # timeout
                rospy.logwarn(f'_wait_joints_reach: timeout while reaching at {target_pos} vs {current_pos}')
                break             
            # read current joints position
            self.joints_state_lock.acquire()
            state = self.joints_state
            current_pos = []     
            if part == 'head':
                current_pos.append(state.position[state.name.index("HeadYaw")])
                current_pos.append(state.position[state.name.index("HeadPitch")])
            elif part == 'right_arm':
                current_pos.append(state.position[state.name.index("RightShoulderPitch")])
                current_pos.append(state.position[state.name.index("RightShoulderRoll")])
                current_pos.append(state.position[state.name.index("RightElbowRoll")])
            elif part == 'left_arm':
                current_pos.append(state.position[state.name.index("LeftShoulderPitch")])
                current_pos.append(state.position[state.name.index("LeftShoulderRoll")])
                current_pos.append(state.position[state.name.index("LeftElbowRoll")])                
            self.joints_state_lock.release()

            # check if current_pos is almost the same as target_pos            
            threshold = 3.0 # degree 
            reached = all(abs(c - t) < threshold for c, t in zip(current_pos, target_pos))
            if reached:
                break
            rospy.sleep(0.1)
                    
    def _joint_state_callback(self, msg):
        self.joints_state_lock.acquire()
        self.joints_state = msg
        self.joints_state_lock.release()
        

        