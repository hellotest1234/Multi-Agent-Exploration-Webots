# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This controller gives to its robot the following behavior:
According to the messages it receives, the robot change its
behavior.
"""

from controller import AnsiCodes, Robot
from common import common_print
import random 
import math

class Enumerate(object):
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)


class Slave(Robot):
    timeStep = 32
    maxSpeed = 7
    
    motors = []
    distanceSensors = []


    def boundSpeed(self, speed):
        return max(-self.maxSpeed, min(self.maxSpeed, speed))
    def __init__(self):
        super(Slave, self).__init__()
        self.camera = self.getDevice('camera')
        self.camera.enable(4 * self.timeStep)
        self.camera.recognitionEnable(4*self.timeStep)
        self.emitter = self.getDevice('emitter')
        self.receiver = self.getDevice('receiver')
        self.receiver.enable(self.timeStep)
        self.motors.append(self.getDevice("left wheel motor"))
        self.motors.append(self.getDevice("right wheel motor"))
        self.motors[0].setPosition(float("inf"))
        self.motors[1].setPosition(float("inf"))
        self.motors[0].setVelocity(0.0) 
        self.motors[1].setVelocity(0.0)
        for dsnumber in range(0, 2):
            self.distanceSensors.append(self.getDevice('ds' + str(dsnumber)))
            self.distanceSensors[-1].enable(self.timeStep)
    
    def turn(self,amt,dirn):
        speeds = [0,0]
        sign = [-1 ,1]
        if dirn ==1:
            sign = [1,-1]
        speeds[0] = sign[0]*0.625*math.pi
        speeds[1] = sign[1]*0.625*math.pi
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        for i in range(amt):
            
            self.step(self.timeStep)
        
        speeds[0] = 0.0
        speeds[1] = 0.0
    def rand(self):
        speeds = [0,0]
        delta = self.distanceSensors[0].getValue() - self.distanceSensors[1].getValue()
        if random.random()<0.01:
            self.turn(random.randint(20,100),random.randint(0,1))
        else:
            speeds[0] = self.boundSpeed(random.uniform(0, self.maxSpeed)+ 0.01 * delta)
            speeds[1] = self.boundSpeed(random.uniform(0, self.maxSpeed)- 0.01*delta)
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        self.step(self.timeStep)
    
    def fw(self):
        speeds = [0,0]
        speeds[0] = 5
        speeds[1] = 5
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        self.step(self.timeStep)
    
    def run(self):
        
        while True:
            # chance = random.random()
            
            L,R = self.distanceSensors[0].getValue() ,self.distanceSensors[1].getValue()
            self.fw()
            if L>500 or R>500:
                self.turn(random.randint(1,100),random.randint(0,1))
            # if chance<0.01:
                # self.turn(random.randint(1,100),random.randint(0,1))        
            #self.rand()
            # Webots is about to quit.
            if self.step(self.timeStep) == -1:
                break


controller = Slave()
common_print('slave')
controller.run()