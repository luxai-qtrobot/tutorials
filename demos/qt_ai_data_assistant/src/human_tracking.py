# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import time
import copy
from threading import Lock
import numpy as np
from concurrent.futures import ThreadPoolExecutor, wait

from kinematics.kinematic_interface import QTrobotKinematicInterface
from human_presence_detection import HumanPresenceDetection


class HumanTracking:
    GAZE_TRACKING_RATE = 10 # herz
    HEAD_TRACKING_RATE = 2 # herz
    ABSENCE_MEMORY_DURATION = 600 # 10 minutes
    PRESENCE_TIME_THRESHOLD = 5 # 5 sec

    def __init__(self, human_detector=None):
        self.should_track = False
        self.persons = {}
        self.persons_lock = Lock()
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.executor_task = None
        self.prev_head_track_time = time.time()

        if human_detector:
            self.human_detector = human_detector
        else:
            self.ikin = QTrobotKinematicInterface()            
            self.human_detector = HumanPresenceDetection(kinematic_interface=self.ikin)
        self.human_detector.register_callback(self._on_presence)


    def _on_presence(self, persons):         
        self.persons_lock.acquire()
        for id, person in persons.items():             
            self.persons[id] = copy.deepcopy(person)
            self.persons[id]['last_seen'] = time.time()
        self.persons_lock.release()
        

    def _forget_absences(self):
        self.persons_lock.acquire()
        keys_to_remove = [id for id, data in self.persons.items() if time.time() - data['last_seen'] > self.ABSENCE_MEMORY_DURATION]
        for id in keys_to_remove:
            del self.persons[id]
        self.persons_lock.release()
    

    def _track_person(self, person): 
        if not person:
            # print('No person!')
            self.human_detector.kinematics.look_at_xyz([2.0, 0, 0.65])
            return 
                
        self._forget_absences()

        self.person_id = person['id']
        while self.should_track:                
            time.sleep(1/self.GAZE_TRACKING_RATE)

            self.persons_lock.acquire()
            cur_person = self.persons.get(self.person_id) 
            # print(cur_person)
            if cur_person:
                cur_person = copy.deepcopy(cur_person)
            self.persons_lock.release()            
            if cur_person: 
                #  if the person has been recently noticed in the image, trac the person
                if cur_person.get('face') and (time.time() - cur_person['last_seen'] < self.PRESENCE_TIME_THRESHOLD):
                    self.look_at_xyz(cur_person['face']['xyz'])                    
                    continue
                
                # we need to look for the person 
                closest_person = self._look_for_closest_person(cur_person)
                if not closest_person:
                    # print('No person!')
                    self.look_at_xyz([2.0, 0, 0.65])
                    break                
                self.person_id = closest_person['id']
        # reset the gaze on stop tracking
        # self.human_detector.kinematics.gaze([0,0], [0,0], 0.1)


    def get_current_person_id(self):
        return self.person_id
    
    def look_at_xyz(self, xyz):
        # self.human_detector.kinematics.look_at_xyz(xyz, only_gaze=True)
        # if time.time() - self.prev_head_track_time > 1/self.HEAD_TRACKING_RATE:
        #     self.human_detector.kinematics.look_at_xyz(xyz)
        #     self.prev_head_track_time = time.time()
        self.human_detector.kinematics.look_at_xyz(xyz)
        
    def _look_for_closest_person(self, person):
        xyz = person['face']['xyz'] if person.get('face') else person['voice']['xyz']
        self.human_detector.kinematics.look_at_xyz(xyz)
        time.sleep(1) # wait a second to update the presence list
        self.persons_lock.acquire()
        # serach for closest person         
        closest_person = None
        min_distance = np.inf
        for id, cur_person in self.persons.items():            
            if cur_person.get('face') and (time.time() - cur_person['last_seen'] < self.PRESENCE_TIME_THRESHOLD):
                cur_xyz = cur_person['face']['xyz']
                distance = np.linalg.norm(np.array(xyz) - np.array(cur_xyz))
                min_distance = distance if distance < min_distance else min_distance
                closest_person = cur_person
        # update the cur_person location in persons list
        if closest_person:
            self.persons[closest_person['id']] = closest_person
        self.persons_lock.release()
        return closest_person

    def track(self, person):
        # stop ongoing tracking
        self.untrack()        
        self.should_track = True        
        self.executor_task = self.executor.submit(self._track_person, person)        
     

    def track_by_id(self, person_id):
        # stop ongoing tracking
        self.untrack()        
        self.should_track = True        
        self.executor_task = self.executor.submit(self._track_person, self.persons.get(person_id))

    def untrack(self):
        self.should_track = False
        if self.executor_task:
            wait([self.executor_task])
            self.executor_task = None        
        # reset the gaze on stop tracking
        # self.human_detector.kinematics.gaze([0,0], [0,0], 0.1)



    