
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
import json

class Enumerate(object): 
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)


class Slave(Robot):
   
    Mode = Enumerate('STOP MOVE_FORWARD MOVE_BACKWARD AVOIDOBSTACLES TURN RANDOM WALLFOLLOW OBSTACLEFOLLOW SGBA') # assign numbers to the names
    timeStep = 64
    maxSpeed = 5.0
    mode = Mode.AVOIDOBSTACLES
    motors = []
    distanceSensors = []
    desiredV = [1,0] # left wheel, right wheel
    prev_sensor_readings =[]; counter =0; # ADDED
    x_position = 0; y_position = 0 # ADDED
    robot_angle_in_deg = 0 # ADDED
    prev_position = [] # ADDED
    prev_x_position = 0; prev_y_position = 0 # ADDED
    target_angle = 0 # ADDED
    stuck_counter = 0 #ADDED
    random_x = random.uniform(-1, 1); 
    random_y = random.uniform(-1, 1) #ADDED
    position_list = [] #ADDED
    loop_counter = 0 #ADDED
    last_loop_position = [] #ADDED
    target_list = [] #ADDED

    def boundSpeed(self, speed):
        return max(-self.maxSpeed, min(self.maxSpeed, speed))
    
    def path_planner (self, pos_x, pos_y,increment_x, increment_y):
        updated_x = pos_x + increment_x
        updated_y = pos_y + increment_y
        return updated_x, updated_y
    # need a condition to check if the end has been reached 

    def euler_from_quaternion(self,l):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x = l[0]; y = l[1]; z = l[2]; w = l[3]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def euler_to_degrees(self, yaw):
        yaw = round(yaw,2)
        diff = 3.13 - 1.74
        increment = diff/179
        increment2 = diff/179
        if yaw == 3.14 or yaw == -3.14:
            degree = 180
        elif yaw == 1.73 or yaw == -1.73:
            degree = 0
        elif yaw > 1.73 and yaw < 3.14:
            degree = (yaw - 1.73)/increment
        elif yaw < -1.73 and yaw > -3.14:
            degree = (yaw + 3.14)/increment2 + 180

        if degree >= 360: 
            degree = 0 
        return degree

    def calculate_robot_angle_wrt_base(self, theta, prev_x,prev_y):
        diff_x = self.x_position - prev_x
        diff_y = self.y_position - prev_y
        abs_angle = math.atan2(abs(diff_y), abs(diff_x)) # in radians
        deg = abs(theta[3]/math.pi * 180)
        if diff_x > 0 and diff_y < 0: # 1st quad 
            deg = deg
        
        elif diff_x > 0 and diff_y > 0: # 2nd quad
            deg = 360 - deg
            
        elif diff_x < 0 and diff_y > 0: # 3rd quad
            deg = 360 - deg
            
        elif diff_x < 0 and diff_y < 0: # 4th quad
            deg = deg
           
        return deg

    def calculate_target_angle_wrt_base(self, intended_x, intended_y):
        # Calculate target angle relative to base frame
        diff_x = intended_x - self.x_position
        diff_y = intended_y - self.y_position
        abs_angle = math.atan2(abs(diff_y), abs(diff_x)) # in radians
        if diff_x > 0 and diff_y < 0: # 1st quad 
            target_angle = math.degrees(abs_angle)
        elif diff_x > 0 and diff_y > 0: # 2nd quad
            target_angle = 360 - math.degrees(abs_angle)
        elif diff_x < 0 and diff_y > 0: # 3rd quad
            target_angle = 180 + math.degrees(abs_angle)
        elif diff_x < 0 and diff_y < 0: # 4th quad
            target_angle = 180 - math.degrees(abs_angle)

        return target_angle
    
    def rotate_to_face_target(self, target_angle, speeds):
        # based on target angle, 
        if target_angle > 270 and target_angle <= 360: # 1st quad 
            target_angle = 360 - target_angle
            if  self.robot_angle_in_deg < target_angle:
                #rotate ccw
                speeds[0] = -self.maxSpeed / 5
                speeds[1] = self.maxSpeed / 5
            else:
                #rotate cw
                speeds[0] = self.maxSpeed / 5
                speeds[1] = -self.maxSpeed / 5

        elif target_angle > 180 and target_angle <= 270: # 2nd quad
            target_angle = 360 - target_angle
            if  self.robot_angle_in_deg < target_angle:
                #rotate ccw
                speeds[0] = -self.maxSpeed / 5
                speeds[1] = self.maxSpeed / 5
            else:
                #rotate cw
                speeds[0] = self.maxSpeed / 5
                speeds[1] = -self.maxSpeed / 5

        elif target_angle > 90 and target_angle <= 180: # 3rd quad
           
            if  self.robot_angle_in_deg < target_angle:
                # rotate cw
                speeds[0] = self.maxSpeed / 5
                speeds[1] = -self.maxSpeed / 5
            else: 
                #rotate ccw
                speeds[0] = -self.maxSpeed / 5
                speeds[1] = self.maxSpeed / 5

        elif target_angle > 0 and target_angle <= 90: # 4th quad
            if  self.robot_angle_in_deg < target_angle:
                # rotate cw
                speeds[0] = self.maxSpeed / 5
                speeds[1] = -self.maxSpeed / 5
            else: 
                #rotate ccw
                speeds[0] = -self.maxSpeed / 5
                speeds[1] = self.maxSpeed / 5

        return speeds

    def obstacle_circumnavigation(self,left_dist, right_dist, speeds, prev_sensor_readings):
        # If close to obstacle, start following it
        desired_dist = 5

        # Proportional gain
        Kp = -0.1

        # Implement proportional control
        error = desired_dist - min(left_dist, right_dist)
        correction = Kp * error

        # SCENARIO 1: obstacle on the right and too close
        if right_dist > left_dist and right_dist > desired_dist: 
            self.counter =0
            if right_dist > 400 and left_dist ==0:
                speeds[0] = self.maxSpeed/2
                speeds[1] = self.boundSpeed(self.maxSpeed/2 - correction)
            else:
                speeds[0] = self.maxSpeed/2
                speeds[1] = self.boundSpeed(self.maxSpeed/2 + correction)


        # SCENARIO 2: obstacle on the right and too far
        elif right_dist > left_dist and right_dist < desired_dist:   
            self.counter =0

            speeds[0] = self.maxSpeed/2
            speeds[1] = self.boundSpeed(self.maxSpeed/2 - correction)


        # SCENARIO 3: obstacle on the left and too close
        elif left_dist > right_dist and left_dist > desired_dist:
            self.counter =0
            
            if left_dist > 400 and right_dist ==0:
                speeds[0] = self.boundSpeed(self.maxSpeed/2 - correction) 
            else: 
                speeds[0] = self.boundSpeed(self.maxSpeed/2 + correction)
                speeds[1] = self.maxSpeed/2

            
        # SCENARIO 4: obstacle on the left and too far
        elif left_dist > right_dist and left_dist < desired_dist:
            self.counter =0
            
            speeds[0] = self.boundSpeed(self.maxSpeed/2 - correction)
            speeds[1] = self.maxSpeed/2


        # SCENARIO 5: Navigating (U-turn) a corner obstacle (edge of wall)
        elif right_dist ==0 and left_dist ==0:
            
            if prev_sensor_readings[0] != 0 and prev_sensor_readings[1] == 0: # left sensor detect
                self.counter +=1
                speeds[0] = self.boundSpeed(self.maxSpeed/2 + 2*correction)
                speeds[1] = self.boundSpeed(self.maxSpeed/2 - 2*correction)
                #print(self.boundSpeed(self.maxSpeed/2),self.boundSpeed(self.maxSpeed/2 - correction))
                
            elif prev_sensor_readings[0] == 0 and prev_sensor_readings[1] != 0: # right sensor detect
                self.counter +=1
                speeds[0] = self.boundSpeed(self.maxSpeed/2 - 2*correction)
                speeds[1] = self.boundSpeed(self.maxSpeed/2 + 2*correction)
                #print(speeds)   
            
            else:  # move forward
                speeds[0] = self.maxSpeed/2
                speeds[1] = self.maxSpeed/2

        return speeds

    def __init__(self):
        super(Slave, self).__init__()

        # mode
        self.mode = self.Mode.SGBA
        self.following_obstacle = True
        
        # camera
        self.camera = self.getDevice('camera')
        self.camera.enable(4 * self.timeStep)
        self.camera.recognitionEnable(4*self.timeStep)

        # receiver
        self.receiver = self.getDevice('receiver')
        self.receiver.enable(self.timeStep)
        self.pos_receiver = self.getDevice('receiver')
        self.pos_receiver.enable(self.timeStep)

        # emitter 
        self.emitter = self.getDevice('driver_emitter')

        # motors
        self.motors.append(self.getDevice("left wheel motor"))
        self.motors.append(self.getDevice("right wheel motor"))
        self.motors[0].setPosition(float("inf"))
        self.motors[1].setPosition(float("inf"))
        self.motors[0].setVelocity(0.0)
        self.motors[1].setVelocity(0.0)

        # distance sensors
        for dsnumber in range(0, 2):
            self.distanceSensors.append(self.getDevice('ds' + str(dsnumber)))
            self.distanceSensors[-1].enable(self.timeStep)
    
    

    def run(self):
        # infinite loop
        while True:
            
            # Check if the target is in front of the robot.
            object= self.camera.getRecognitionObjects()
            
            for obj in object:  
                #print('robot2 found: ',obj.getId())
                #print('robot2 current list: ',self.target_list)
                if obj.getId() not in self.target_list:   
                    self.target_list.append(obj.getId())
                    #print('found')
                    

                    found_target_message = obj.getId()
                    found_target_message = str(obj.getId()).encode('utf-8')
                    self.emitter.setChannel(2)
                    self.emitter.send(found_target_message)
                    #self.mode = self.Mode.STOP
                    
                        
                    #return 0 
                #else:
                    #print('target already found')
                    
          
            
            ######################## RECEIVING MESSAGES FROM THE RECEIVER ########################

            #  Listens for messages from the receiver & updates the mode accordingly
            if self.receiver.getQueueLength() > 0: 
                # robot 1
                received_message = self.receiver.getString() 
                combined_message = json.loads(received_message)  

                # Extract position and control messages
                position_message = combined_message["pose2"]
                control_message = combined_message["control"]
                target_message = combined_message["target"]
                

                # Handle the position message
                x_pos = position_message["x2"]; self.x_position = x_pos
                y_pos = position_message["y2"]; self.y_position = y_pos
                theta = position_message["theta2"];  self.theta = theta
                self.robot_angle_in_deg = abs(self.theta[3]/math.pi * 180)
                self.target_list = target_message["target_list"]
                
                
               
               
            
                
                # Handle the control message    
                message = control_message["message"]
                
                self.receiver.nextPacket() 
                if message != '':
                    print('I should ' + AnsiCodes.RED_FOREGROUND + message + AnsiCodes.RESET + '!')
                #print('I am at'+ AnsiCodes.RED_FOREGROUND + str(x_pos) + ','+ str(y_pos) + AnsiCodes.RESET + '!')
                #print(round(eul_angle[2],2))
                if message == 'avoid obstacles':
                    self.mode = self.Mode.AVOIDOBSTACLES
                elif message == 'move forward':
                    self.mode = self.Mode.MOVE_FORWARD
                elif message == 'move backward':
                    self.mode = self.Mode.MOVE_BACKWARD
                elif message == 'stop':
                    self.mode = self.Mode.STOP
                elif message == 'random':
                    self.mode = self.Mode.RANDOM
                elif message == 'turn':
                    self.mode = self.Mode.TURN
                elif message == 'wall follow':
                    self.mode = self.Mode.WALLFOLLOW
                elif message == 'obstacle follow':
                    self.mode = self.Mode.OBSTACLEFOLLOW
                elif message == 'sgba':
                    self.mode = self.Mode.SGBA
        ########################################################################################

            delta = self.distanceSensors[0].getValue() - self.distanceSensors[1].getValue() # difference between the two sensors
            speeds = [0.0, 0.0] # desired velocities
            

            # Send actuators commands according to the mode.
            if self.mode == self.Mode.AVOIDOBSTACLES:
                speeds[0] = self.boundSpeed(self.maxSpeed / 2 + 0.1 * delta)
                speeds[1] = self.boundSpeed(self.maxSpeed / 2 - 0.1*delta )
            elif self.mode == self.Mode.MOVE_FORWARD:
                speeds[0] = self.maxSpeed
                speeds[1] = self.maxSpeed
            elif self.mode == self.Mode.MOVE_BACKWARD:
                self.stuck_counter -=1
                speeds[0] = -self.maxSpeed
                speeds[1] = -self.maxSpeed
                if self.stuck_counter == 0:
                    self.mode = self.Mode.SGBA   
                    #print('switch to SGBA')
            elif self.mode == self.Mode.STOP:
                speeds[0] = 0
                speeds[1] = 0
                return
            elif self.mode == self.Mode.TURN: # clockwise direction
                speeds[0] = self.maxSpeed / 5
                speeds[1] = -self.maxSpeed / 5                
                print('target angle: ',self.target_angle)
                print('robot angle: ',self.robot_angle_in_deg)
                print('diff: ',abs(self.robot_angle_in_deg - self.target_angle))
            elif self.mode == self.Mode.RANDOM:
                # 20% chance to turn ranodmly
                if random.random()<0.2: 
                    speeds[0] = self.boundSpeed(random.uniform(-self.maxSpeed, self.maxSpeed))
                    speeds[1] = self.boundSpeed(random.uniform(-self.maxSpeed, self.maxSpeed))
                # 80% chance to turn randomly but also based on delta 
                else:
                    speeds[0] = self.boundSpeed(random.uniform(0, self.maxSpeed)+ 0.8 * delta)
                    speeds[1] = self.boundSpeed(random.uniform(0, self.maxSpeed)- 0.8*delta)
            elif self.mode == self.Mode.WALLFOLLOW:
                # Implement wall-following behavior.
                # Use sensor data to keep a consistent distance from the obstacle (wall).
                correction = 0
                left =  self.distanceSensors[0].getValue()
                right = self.distanceSensors[1].getValue()


                if left>0:
                    desired_distance = 2
                    #print(left)
                    error = left-desired_distance
                    
                    correction = 0.1 * error  # Adjust the correction factor as needed
                
                if left <= 0.1:
                    speeds[0] = self.boundSpeed(self.maxSpeed/2 + correction)
                    speeds[1] = self.boundSpeed(self.maxSpeed/2 + correction)
                else: 
                    speeds[0] = self.boundSpeed(self.maxSpeed/2 + correction)
                    speeds[1] = self.boundSpeed(self.maxSpeed/2 - correction)
            elif self.mode == self.Mode.OBSTACLEFOLLOW:
                
                #print('OBSTACLEFOLLOW MODE')
                # Get left and right sensor distances
                left_dist = self.distanceSensors[0].getValue()
                right_dist = self.distanceSensors[1].getValue()
                #print("left: " + str(left_dist) + " right: " + str(right_dist))

                # Proportional gain
                Kp = -0.1

                desired_dist = 5
                # Implement proportional control
                error = desired_dist - min(left_dist, right_dist)
                correction = Kp * error
            
                # stuck detection
                #print('stuck counter: ',self.stuck_counter)
                if abs(self.x_position - self.prev_position[0]) < 0.001 and abs(self.y_position - self.prev_position[1]) < 0.001:
                    self.stuck_counter+=1
                elif left_dist > 800 or right_dist > 800:
                    self.stuck_counter += 1
                else: 
                    self.stuck_counter = 0

                # SCENARIO 6: Stuck
                if self.stuck_counter  == 30:                        
                        self.mode = self.Mode.MOVE_BACKWARD
                        #print('moving backward')
                

                # SCENARIO 1: obstacle on the right and too close
                elif right_dist > left_dist and right_dist > desired_dist: 
                    self.counter =0
                    if right_dist > 400 and left_dist ==0:
                        speeds[0] = self.maxSpeed/2
                        speeds[1] = self.boundSpeed(self.maxSpeed/2 - correction)
                    else:
                        speeds[0] = self.maxSpeed/2
                        speeds[1] = self.boundSpeed(self.maxSpeed/2 + correction)


                # SCENARIO 2: obstacle on the right and too far
                elif right_dist > left_dist and right_dist < desired_dist:   
                    self.counter =0

                    speeds[0] = self.maxSpeed/2
                    speeds[1] = self.boundSpeed(self.maxSpeed/2 - correction)


                # SCENARIO 3: obstacle on the left and too close
                elif left_dist > right_dist and left_dist > desired_dist:
                    self.counter =0
                    # FOR SOME REASON, CCW DOES NOT WORK WELL 
                    if left_dist > 400 and right_dist ==0:
                        speeds[0] = self.boundSpeed(self.maxSpeed/2 + correction) # CHANGES MADE TO THE SIGN TO ALWAYS MAKE IT TURN CW
                        speeds[1] = self.maxSpeed/2
                    else: 
                        speeds[0] = self.boundSpeed(self.maxSpeed/2 - correction) # CHANGES MADE TO THE SIGN TO ALWAYS MAKE IT TURN CW
                        speeds[1] = self.maxSpeed/2

                    
                # SCENARIO 4: obstacle on the left and too far
                elif left_dist > right_dist and left_dist < desired_dist:
                    self.counter =0
                    
                    speeds[0] = self.boundSpeed(self.maxSpeed/2 - correction)
                    speeds[1] = self.maxSpeed/2


                # SCENARIO 5: Navigating (U-turn) a corner obstacle (edge of wall)
                elif right_dist ==0 and left_dist ==0:
                    
                    if self.prev_sensor_readings[0] != 0 and self.prev_sensor_readings[1] == 0: # left sensor detect
                        self.counter +=1
                        
                        speeds[0] = self.boundSpeed(self.maxSpeed/2 + 2*correction)
                        speeds[1] = self.boundSpeed(self.maxSpeed/2 - 2*correction)
                        #print(self.boundSpeed(self.maxSpeed/2),self.boundSpeed(self.maxSpeed/2 - correction))
                            
                    elif self.prev_sensor_readings[0] == 0 and self.prev_sensor_readings[1] != 0: # right sensor detect
                        self.counter +=1
                        speeds[0] = self.boundSpeed(self.maxSpeed/2 - 2*correction)
                        speeds[1] = self.boundSpeed(self.maxSpeed/2 + 2*correction)
                        #print(speeds)   
                    
                    else:  # move forward
                        speeds[0] = self.maxSpeed/2
                        speeds[1] = self.maxSpeed/2

                

                # Convert target angle to robot's coordinate frame 
                if  self.target_angle > 270 and  self.target_angle <= 360: # 1st quad 
                        target_angle1 = 360 -  self.target_angle
                elif  self.target_angle > 180 and  self.target_angle <= 270: # 2nd quad
                    target_angle1 = 360 -  self.target_angle
                elif  self.target_angle > 90 and  self.target_angle <= 180: # 3rd quad
                    target_angle1 = self.target_angle
                elif  self.target_angle > 0 and  self.target_angle <= 90: # 4th quad
                    target_angle1 =  self.target_angle

                #print('target angle: ',target_angle1)
                #print('robot angle: ',self.robot_angle_in_deg)
                #print('diff: ',abs(self.robot_angle_in_deg - target_angle1))
                #print('right: ',right_dist)
                if abs(self.robot_angle_in_deg - target_angle1) <=3 and left_dist == 0 and right_dist == 0: #and left_dist <= 5 and right_dist <= 5:
                    self.mode = self.Mode.SGBA   
                    #print('switch to SGBA')
                     
            elif self.mode == self.Mode.SGBA:
                #print('SGBA MODE')
                #initialize 
                
                # Get left and right sensor distances
                left_dist = self.distanceSensors[0].getValue()
                right_dist = self.distanceSensors[1].getValue()
                #print("left: " + str(left_dist) + " right: " + str(right_dist))
                #if self.prev_position !=[]:
                #    print('position diff: ', self.x_position-self.prev_position[0], self.y_position-self.prev_position[1])
               
                # Get the next the coordinate and angle the robot should head towards
                updated_x, updated_y = self.path_planner(self.x_position,self.y_position, self.random_x, self.random_y)
                self.target_angle = self.calculate_target_angle_wrt_base(updated_x, updated_y)
                #print(self.random_x, self.random_y)
                #print('target angle: ', self.target_angle)
                

             



                # Convert target angle to robot's coordinate frame
                if  self.target_angle > 270 and  self.target_angle <= 360: # 1st quad 
                    target_angle1 = 360 -  self.target_angle
                elif  self.target_angle > 180 and  self.target_angle <= 270: # 2nd quad
                    target_angle1 = 360 -  self.target_angle
                elif  self.target_angle > 90 and  self.target_angle <= 180: # 3rd quad
                    target_angle1 = self.target_angle
                elif  self.target_angle > 0 and  self.target_angle <= 90: # 4th quad
                    target_angle1 =  self.target_angle
                    

               
                #print('robot angle: ',self.robot_angle_in_deg)
                #print('target angle: ',target_angle1)

                
                # Rotate robot to face target
                diff_angle = self.robot_angle_in_deg - target_angle1
                #print('target angle: ',target_angle)
                #print('difference in angle: ',diff_angle)
                if abs(diff_angle) > 2: # not facing right direction
                    #print('wrong angle')
                    speeds = self.rotate_to_face_target(self.target_angle, speeds)

                elif abs(diff_angle) <= 3: # facing right direction 


                    # path is clear and move forward
                    if left_dist == 0 and right_dist == 0: # and any(item == 0 for item in self.prev_sensor_readings):   
                        #print('path clear')
                        speeds[0] = self.maxSpeed/2
                        speeds[1] = self.maxSpeed/2
                        

                        '''
                        This loop detection will not clear the loop counter. This is based on a straight line path 
                        that the robot takes. If the robot path happens to be within 0.1m of its previous path, it will 
                        add to the loop counter. Once a loop has been detected, it will reset the loop counter. This method
                        is based on the assumption that the robot is unable to keep track of CONSECUTIVE path points to determine
                        whether it is in a loop. 
                        '''

                        # Check for loop 
                        if  len(self.position_list) >= 2000:

                        
                            for index, i in enumerate(self.position_list):
                                #print('diff for x:' ,round(self.x_position - i[0],2))
                                #print('diff for y:' ,round(self.y_position - i[1],2))
                                if abs(self.x_position- i[0]) <= 0.25 and abs(self.y_position - i[1]) <= 0.25:
                                    #print('same position detected')
                                    #print('diff for x:' ,self.x_position - i[0])
                                    #print('diff for y:' ,self.y_position - i[1])
                                    self.position_list = self.position_list[index+1:]
                                    self.loop_counter += 1
                                    #print('position list length: ', len(self.position_list))
                                    print('loop counter: ', self.loop_counter)
                                    self.last_loop_position = [self.x_position, self.y_position]
                                    if self.loop_counter > 0:
                                        print('loop detected')
                                        self.random_x = random.uniform(-1,1); self.random_y = random.uniform(-1,1)
                                        self.loop_counter = 0
                                        #self.position_list = []
                                    break
                            
                                # elif abs(self.x_position - i[0]) > 0.1 and abs(self.y_position - i[1]) > 0.1: 
                                #     #self.position_list.pop(0)
                                #     if self.loop_counter != 0:
                                #         print('reset')
                                #         self.loop_counter = 0
                                    
                                    

                    # path not clear
                    else: 
                        #print('path not clear')
                        #print(left_dist,right_dist)

                        if left_dist > 0 or right_dist> 0: # obstacle detected, do wall-following  
                            #print('Switch to OBSTACLEFOLLOW')
                            
                            self.mode = self.Mode.OBSTACLEFOLLOW
                            self.motors[0].setVelocity(speeds[0])
                            self.motors[1].setVelocity(speeds[1])
                        
                        # elif left_dist > 0 and right_dist> 0: # obstacle detected, do wall-following  
                        #     print('Switch to OBSTACLEFOLLOW')
                            
                        #     self.mode = self.Mode.OBSTACLEFOLLOW
                        #     self.motors[0].setVelocity(speeds[0])
                        #     self.motors[1].setVelocity(speeds[1])
                               

                        # else: # move forward and closer to the obstacle 
                        #     print('Moving Forward')
                        #     speeds[0] = self.maxSpeed/2
                        #     speeds[1] = self.maxSpeed/2


                # Set motor speeds
                self.motors[0].setVelocity(speeds[0])
                self.motors[1].setVelocity(speeds[1])
                    
                 

            # Save previous sensor readings
            if self.counter == 0 or self.counter == 100:
                self.counter = 0
                self.prev_sensor_readings = [self.distanceSensors[0].getValue(), self.distanceSensors[1].getValue()] # ADDED
                #print(self.prev_sensor_readings)

            # Save previous position 
            self.prev_position = [self.x_position, self.y_position]  

            # Save positions for loop detection 
            self.position_list.append([self.x_position, self.y_position])
            if len(self.position_list) > 2000:
                self.position_list.pop(0)
            #print( len(self.position_list))

            # loop reset 
            if self.last_loop_position != []:
                if abs(self.x_position - self.last_loop_position[0]) > 2.5 and abs(self.y_position - self.last_loop_position[1]) > 2.5:
                    self.loop_counter = 0
                    self.position_list = []
                    print('loop reset')


            # Set motor speeds for other modes
            if self.mode != self.Mode.SGBA:
                self.motors[0].setVelocity(speeds[0])
                self.motors[1].setVelocity(speeds[1])
                #print(speeds)
           

            # Perform a simulation step, quit the loop when
            # Webots is about to quit.
            if self.step(self.timeStep) == -1:
                break


controller = Slave()
common_print('slave')
controller.run()