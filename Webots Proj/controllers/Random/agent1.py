from agent import Agent
from controller import AnsiCodes, Robot
from common import common_print

class Enumerate(object):
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)


class Slave(Robot):
    Mode = Enumerate('STOP MOVE_FORWARD AVOIDOBSTACLES TURN RANDOM WALLFOLLOW')
    timeStep = 32
    maxSpeed = 10.0
    mode = Mode.AVOIDOBSTACLES
    motors = []
    distanceSensors = []
    desiredV = [1,0]

    def boundSpeed(self, speed):
        return max(-self.maxSpeed, min(self.maxSpeed, speed))
    def __init__(self):
        super(Slave, self).__init__()
        self.mode = self.Mode.WALLFOLLOW
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
        self.agent = Agent(0.125,0.125,8,1,'A1')
        for dsnumber in range(0, 2):
            self.distanceSensors.append(self.getDevice('ds' + str(dsnumber)))
            self.distanceSensors[-1].enable(self.timeStep)
    
    def listen(self):
        if self.receiver.getQueueLength() > 0:
            received = self.receiver.getString()
            received = eval(received)
        return received
    
    def talk(self,message):
        self.emitter.send(str(message).encode('utf-8'))
    
    def forward(self):
        speeds[0] = 6.25
        speeds[1] = 6.25
        
        for i in range(50):
            self.motors[0].setVelocity(speeds[0])
            self.motors[1].setVelocity(speeds[1])
            self.step(self.timeStep)
        speeds[0] = 0.0
        speeds[1] = 0.0
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
    def turnL(self):
                
        speeds[0] = 0.625*math.pi
        speeds[1] = -0.625*math.pi
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        for i in range(50):
            
            self.step(self.timeStep)
        L=self.distanceSensors[0].getValue()
        R=self.distanceSensors[1].getValue()
        speeds[0] = 0.0
        speeds[1] = 0.0
        
    def turnR(self):
        speeds[0] = -0.625*math.pi
        speeds[1] = 0.625*math.pi
        self.motors[0].setVelocity(speeds[0])
        self.motors[1].setVelocity(speeds[1])
        for i in range(50):
            
            self.step(self.timeStep)
        L=self.distanceSensors[0].getValue()
        R=self.distanceSensors[1].getValue()
        speeds[0] = 0.0
        speeds[1] = 0.0
        
    def run(self):
        
        while True:
            msg =  self.listen()
            if msg[0]== "Agent":
                route = msg[1][self.agent.name]
                for i in route:
                    if i == 'f':
                        self.forward()
                    elif i =='r':
                        self.turnR()
                    else:
                        self.turnL()
                    se
                for i in range(4):
                    self.turnR()
                    Map.update_map(self.view())
            
            movesCompleted = True
            if movesCompleted:
                nextMoves = M.plan_routes()
                self.talk(['Agent',nextMoves])
                movesCompleted = False
            done = 0
            while not movesCompleted:
                msg = self.listen()
                if msg == None:#COMPLETED
                    done+=1
                else:
                    if msg[0] =='Driver':
                        update = msg[1]
                        self.map.update_map(update)
                if done == 3:
                    movesCompleted = True
            
    
    

