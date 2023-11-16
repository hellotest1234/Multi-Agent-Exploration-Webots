from agent import Agent
from controller import AnsiCodes, Robot
from common import common_print
import sys
import math

class Enumerate(object):
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)


class Slave(Robot):
    timeStep = 32
    maxSpeed = 10.0
    motors = []
    distanceSensors = []
    
    def boundSpeed(self, speed):
        return max(-self.maxSpeed, min(self.maxSpeed, speed))
        
    def __init__(self,startx,starty,name):
        
        params = sys.argv
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
        self.agent = Agent(startx,starty,4,1,name)
        print(self.agent.name)
        for dsnumber in range(0, 2):
            self.distanceSensors.append(self.getDevice('ds' + str(dsnumber)))
            self.distanceSensors[-1].enable(self.timeStep)
        update_arr = []
        previous_message = ''
        for i in range(4):
            self.turnR()
            self.agent.turnR()
            updates = self.agent.view(self.distanceSensors[0].getValue(),self.distanceSensors[1].getValue())
            message = str(['Driver', updates,self.agent.name,self.agent.pose])
            self.talk(message,previous_message)
            previous_message = message
            update_arr.append(updates)
            #self.talk(['Driver',updates])

        message = str(['Done'])
        self.talk(message,previous_message)
        previous_message = message
        print('initialise agent complete')
    
    def listen(self):
        if self.receiver.getQueueLength() > 0:
            received = self.receiver.getString()
            received = eval(received)
            self.receiver.nextPacket()
            #print(received)
            return received
    
    def talk(self,message,previous_message):
        if message != '' and message != previous_message:
            previous_message = message
            #print('Please, ' + message)
            self.emitter.send(message)
    
    def forward(self):
        
        speeds = [0,0]
        speeds[0] = 6.25
        speeds[1] = 6.25
        Collide = False
        count = 0
        for i in range(50):
            self.motors[0].setVelocity(speeds[0])
            self.motors[1].setVelocity(speeds[1])
            self.step(self.timeStep)
            object= self.camera.getRecognitionObjects()
            for item in object:
                if item.getColors()[1]:
                 
                    if not Collide:
                        #print('LD',str(self.distanceSensors[0].getValue()))
                        #print('RD',str(self.distanceSensors[1].getValue()))
                        count+=1
                        if self.distanceSensors[0].getValue()<self.distanceSensors[1].getValue():
                            correction = 'L'
                            
                            self.turnL()
                            self.half_F()
                            self.turnR()
                            #self.half_F()
                        else:
                            correction = 'R'
                            self.turnR()
                            self.half_F()
                            self.turnL()
                            #self.half_F()
                    # for i in range(2):
                        # self.half_F()
                    # self.turnL()
                    # self.half_F()
                    
                    Collide = True
                    
        
        
        speeds[0] = 0.0
        speeds[1] = 0.0
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        #print('Count',str(count))
        return Collide
    
    def half_F(self):
        speeds = [0,0]
        speeds[0] = 6.25
        speeds[1] = 6.25
        
        for i in range(18):
            self.motors[0].setVelocity(speeds[0])
            self.motors[1].setVelocity(speeds[1])
            self.step(self.timeStep)
            self.found()
        
        speeds[0] = 0.0
        speeds[1] = 0.0
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        
    def turnL(self):
        speeds = [0,0]        
        speeds[0] = -0.625*math.pi
        speeds[1] = 0.625*math.pi
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        for i in range(50):
            
            self.step(self.timeStep)
            
        speeds[0] = 0.0
        speeds[1] = 0.0
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        
    def turnR(self):
        speeds = [0,0]
        speeds[0] = 0.625*math.pi
        speeds[1] = -0.625*math.pi
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        for i in range(50):
            
            self.step(self.timeStep)
            
        speeds[0] = 0.0
        speeds[1] = 0.0
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
    
    
        
    def found(self):
        object= self.camera.getRecognitionObjects()
        for item in object:
            if item.getColors()[0]:
                print('found')
    def stop(self):
        speeds = [0,0]
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])

    def run(self):
        previous_message = ''
        while True:
            
            msg =  self.listen()
            if msg:
                if msg[0]== "Agent":
                    route = msg[1][self.agent.name]
                    for i in route:
                        if i == 'f':
                            self.forward()
                            self.agent.move()    
                                
                        elif i =='r':
                            self.turnR()
                            self.agent.turnR()
                        else:
                            self.turnL()
                            self.agent.turnL()
                      
                        updates = self.agent.view(self.distanceSensors[0].getValue(),self.distanceSensors[1].getValue())
                        message = str(['Driver',updates,self.agent.name,str(self.agent.pose)])
                        self.talk(message,previous_message)
                        previous_message = message
                    dest = msg[2][self.agent.name]
                    self.agent.x = dest[0]
                    self.agent.y = dest[1]
                    self.agent.pose = dest[2]
                    if route ==[]:
                        for i in range(4):
                            self.turnR()
                            self.agent.turnR()
                            updates = self.agent.view(self.distanceSensors[0].getValue(),self.distanceSensors[1].getValue())
                            message = str(['Driver',updates,self.agent.name,str(self.agent.pose)])
                            self.talk(message,previous_message)
                            previous_message = message

                    message = str(['Done',self.agent.name])
                    
                    self.talk(message,previous_message)
                    previous_message = message
            else:
                self.stop()
                
            if self.step(self.timeStep) == -1:
                break

params = sys.argv
#print(params)
controller = Slave(float(params[1]),float(params[2]),params[3])
common_print('slave')
controller.run()  