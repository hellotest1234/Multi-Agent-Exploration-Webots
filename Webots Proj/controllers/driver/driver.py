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


class Driver (Supervisor):
    timeStep = 128
    x = -0.3
    y = -0.1
    translation = [x, y, 0]

    def __init__(self):
        super(Driver, self).__init__()
        self.emitter = self.getDevice('emitter')
        robot = [self.getFromDef('ROBOT1'),self.getFromDef('ROBOT2'),self.getFromDef('ROBOT2')]
        self.translationField = [robot[0].getField('translation'),robot[1].getField('translation'),robot[2].getField('translation')]
        self.keyboard.enable(Driver.timeStep)
        self.keyboard = self.getKeyboard()
        self.target = [1,1.5]
    
    def tol(self,c,t):
        if abs(c[0]-t[0])+abs(c[1]-t[1])<0.5:
            return True
        return False
    
    def run(self):
        self.displayHelp()
        previous_message = ''

        # Main loop.
        while True:
            # Deal with the pressed keyboard key.
            k = self.keyboard.getKey()
            message = ''
            if k == ord('A'):
                message = 'avoid obstacles'
            elif k == ord('F'):
                message = 'move forward'
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
            #elif self.tol(self.translationField[0].getSFVec3f(),self.target):
                #message = 'stop'
            #elif self.tol(self.translationField[1].getSFVec3f(),self.target):
                #message = 'stop'
            #elif self.tol(self.translationField[2].getSFVec3f(),self.target):
                #message = 'stop'
                
            elif k == ord('G'):
                translationValues = self.translationField[0].getSFVec3f()
                print('ROBOT1 is located at (' + str(translationValues[0]) + ',' + str(translationValues[1]) + ')')
            elif k == ord('R'):
                print('Teleport ROBOT1 at (' + str(self.x) + ',' + str(self.y) + ')')
                self.translationField.setSFVec3f(self.translation)

            # Send a new message through the emitter device.
            if message != '' and message != previous_message:
                previous_message = message
                print('Please, ' + message)
                self.emitter.send(message.encode('utf-8'))

            # Perform a simulation step, quit the loop when
            # Webots is about to quit.
            if self.step(self.timeStep) == -1:
                break

    def displayHelp(self):
        print(
            'Commands:\n'
            ' I for displaying the commands\n'
            ' A for avoid obstacles\n'
            ' F for move forward\n'
            ' S for stop\n'
            ' T for turn\n'
            ' R for positioning ROBOT1 at (-0.3,-0.1)\n'
            ' G for knowing the (x,y) position of ROBOT1'
        )


controller = Driver()
common_print('driver')
controller.run()
