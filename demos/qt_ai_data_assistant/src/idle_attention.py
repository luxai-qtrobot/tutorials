# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import random
import rospy
import time
import copy
from threading import Lock, Thread

from kinematics.kinematic_interface import QTrobotKinematicInterface


class IdleAttention:    

    def __init__(self, attention_time=5, human_tracker=None):
        self.attention_time = attention_time
        self.human_tracker = human_tracker
        self.pay_attention = False
        self.persons = {}
        self.persons_lock = Lock()
        if self.human_tracker:
            self.ikin = self.human_tracker.human_detector.kinematics
            self.human_tracker.human_detector.register_callback(self._human_presence_callback)
        else:
            self.ikin = QTrobotKinematicInterface()
        self.processing_thread = Thread(target=self.process, daemon=True)        
        self.processing_thread.start()


    def _human_presence_callback(self, persons):
        self._forget_absences()
        self.persons_lock.acquire()        
        for id, person in persons.items():
            if not person.get('face'):
                continue
            self.persons[id] = copy.deepcopy(person)
            self.persons[id]['last_seen'] = time.time()
        self.persons_lock.release()
        


    def _forget_absences(self):
        self.persons_lock.acquire()
        keys_to_remove = [id for id, data in self.persons.items() if time.time() - data['last_seen'] > 2.0]
        for id in keys_to_remove:
            del self.persons[id]
        self.persons_lock.release()


    def process(self):
        while not rospy.is_shutdown():
            if not self.pay_attention:
                rospy.sleep(1)
                continue
            
            self._forget_absences()
            self.persons_lock.acquire()
            if self.persons and self.human_tracker:
                random_person = random.choice(list(self.persons.values()))
                # print(f"tracking {random_person.get('id')}...")
                self.human_tracker.track(random_person)
            else:                
                xyz = [2.0, random.uniform(-1, 1), 0.65 ]   
                # print(f'random look {xyz}...')
                self.ikin.look_at_xyz(xyz)
            self.persons_lock.release()
            rospy.sleep(self.attention_time)
            

    def start(self):
        self.stop()
        self.pay_attention = True
        print('idle attention sarted.')


    def stop(self):        
        self.pay_attention = False
        if self.human_tracker:
            self.human_tracker.untrack()
        print('idle attention stopped.')