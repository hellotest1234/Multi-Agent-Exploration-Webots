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
This controller gives to its node the following behavior:
Listen the keyboard. According to the pressed key, send a
message through an emitter or handle the position of Robot1.
"""

from controller import Supervisor
from common import common_print
import json
import math 
import numpy as np
import csv


class Driver (Supervisor):
    timeStep = 64
    x = -0.3
    y = -0.1
    translation = [x, y, 0]
    time_interval = 0 # ADDED
    counter = 0 # ADDED
    target_list = [] # ADDED

    def __init__(self):
        super(Driver, self).__init__()
        self.emitter = self.getDevice('emitter')
        self.pos_emitter = self.getDevice('emitter')
        self.receiver = self.getDevice('receiver')
        self.receiver2 = self.getDevice('receiver2')
        self.receiver3 = self.getDevice('receiver3')
        self.receiver.enable(self.timeStep)
        self.receiver2.enable(self.timeStep)
        self.receiver3.enable(self.timeStep)
        

        # Define robot and sensor readings
        robot = [self.getFromDef('ROBOT1'), self.getFromDef('ROBOT2'), self.getFromDef('ROBOT3')]
        self.translationField = [robot[0].getField('translation') , robot[1].getField('translation'), robot[2].getField('translation')]
        self.rotationField = [robot[0].getField('rotation'), robot[1].getField('rotation'),robot[2].getField('rotation')]
        self.keyboard.enable(Driver.timeStep)
        self.keyboard = self.getKeyboard()
        #target = [self.getFromDef('TARGET'), self.getFromDef('TARGET2'), self.getFromDef('TARGET3')]
        #self.nameField = [target[0].getField('name'), target[1].getField('name'), target[2].getField('name')]
        self.M = np.full((4*4,3*4),0)

    def nearest(self, n):
        # Get current position
        curr = self.translationField[n].getSFVec3f()
        
        posx, posy = curr[0], curr[1]
    
        # Calculate the nearest grid cell indices
        grid_cell_size = 0.25
        offset_x, offset_y = 0.125, 0.125
        compx, compy = 0, 0
        
        # Adjust for the offset and compute the grid cell indices
        grid_x = int(round(max(0, (posx - offset_x) / grid_cell_size))) + compx
        grid_y = int(round(max(0, (posy - offset_y) / grid_cell_size))) + compy
    
        return grid_x, grid_y
    
    # Determine whether robot has reached the target position within a certain tolerance.  
    def tol(self,c,t):
        if abs(c[0]-t[0])+abs(c[1]-t[1])<0.5:
            return True
        return False
    
    # Run the controller
    def run(self):
        self.displayHelp()
        previous_message = ''

        
        i = 0
        # Main loop.
        while True:


            # Deal with the pressed keyboard key.
            k = self.keyboard.getKey()
            message = ''
            if k == ord('A'):
                message = 'avoid obstacles'
            elif k == ord('F'):
                message = 'move forward'
            elif k == ord('D'):
                message = 'move backward'
            elif k == ord('S'):
                message = 'stop'
            elif k == ord('T'):
                message = 'turn'
            elif k == ord('I'):
                self.displayHelp()
            elif k == ord('R'):
                message = 'random'
            elif k == ord('W'):
                message = 'wall follow'
            elif k == ord('O'):
                message = 'obstacle follow'
            elif k == ord('B'):
                message = 'sgba'
            elif k == ord('G'): # show position of ROBOT1
                print('ROBOT1 is located at (' + str(translationValues[0]) + ',' + str(translationValues[1]) + ')')


        #########################  SENDING OF MESSAGES VIA EMITTER  ##############################                 

            # Allows 'driver' to send new message to 'slave' robots
            if message != '' and message != previous_message:
                previous_message = message
                print('Please, ' + message)
                #self.emitter.send(message.encode('utf-8')) # Send a new message through the emitter device.


            # Send the position message if the counter reaches the desired time interval
            self.counter+=1
            if self.counter >= self.time_interval:
                self.counter = 0  # Reset the counter
                translationValues = self.translationField[0].getSFVec3f() # calculate global position
                rotationalValues = self.rotationField[0].getSFRotation()
                translationValues2 = self.translationField[1].getSFVec3f() # calculate global position
                rotationalValues2 = self.rotationField[1].getSFRotation()
                translationValues3 = self.translationField[2].getSFVec3f() # calculate global position
                rotationalValues3 = self.rotationField[2].getSFRotation()
                #target1 = self.nameField[0].getSFString()
                #target2 = self.nameField[1].getSFString()
                #target3 = self.nameField[2].getSFString()

                

            # combined message for robot 1
            combined_message = {
                "pose": {
                "x": translationValues[0],
                "y": translationValues[1],
                "theta": rotationalValues
            },
            "control": {
                "message": message
            },
            "pose2": {
                "x2": translationValues2[0],
                "y2": translationValues2[1],
                "theta2": rotationalValues2
            },
            "pose3": {
                "x3": translationValues3[0],
                "y3": translationValues3[1],
                "theta3": rotationalValues3
            },
            "target": {
                "target_list": self.target_list
            }}
            self.emitter.send(json.dumps(combined_message).encode('utf-8'))
            
            
        ####################################################################################


            # Receive the target id
            if self.receiver.getQueueLength() > 0: 
                self.receiver.setChannel(4)
                received_message = self.receiver.getString()
                if int(received_message) not in self.target_list:
                    self.target_list.append(int(received_message))
                    print('driver updated list: ', self.target_list) 

                    
            if self.receiver2.getQueueLength() > 0: 
                self.receiver2.setChannel(2)
                received_message = self.receiver2.getString()
                if int(received_message) not in self.target_list:
                    self.target_list.append(int(received_message))
                    print('driver updated list: ', self.target_list) 
            
            if self.receiver3.getQueueLength() > 0: 
                self.receiver3.setChannel(3)
                received_message = self.receiver3.getString()
                if int(received_message) not in self.target_list:
                    self.target_list.append(int(received_message))
                    print('driver updated list: ', self.target_list) 




            # if i%2==0: #simstep 2
            #     for j in range (len(self.translationField)):
            #         pos = self.nearest(j)
            #         self.M[pos[1],pos[0]]+=1
            # if i==2000: #2000 steps
            #     with open('coverage.csv', 'w', newline='') as csvfile:
            #         spamwriter = csv.writer(csvfile)
                    
            #         spamwriter.writerows(reversed(self.M))
            #         print('CSV written')
                            
            #     break
            
            # if i%100==0:
            #     print("Count is",str(i))
                
            # i+=1


            #print(self.timeStep)

            # Perform a simulation step, quit the loop when
            # Webots is about to quit.
            if self.step(self.timeStep) == -1:
                break
    
    # Display all user commands
    def displayHelp(self):
        print(
            'Commands:\n'
            ' I for displaying the commands\n'
            ' A for avoid obstacles\n'
            ' F for move forward\n'
            ' S for stop\n'
            ' T for turn\n'
            ' R for random\n'
            ' W for wall follow\n'
            ' O for obstacle follow\n'
            ' G for knowing the (x,y) position of ROBOT1'
        )


controller = Driver() # create instance of the controller
common_print('driver') # print info about controller
controller.run()
