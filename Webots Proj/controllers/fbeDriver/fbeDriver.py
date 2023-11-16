import numpy as np
import itertools
import math
import heapq as pq
from controller import Supervisor
from common import common_print
from agent import Agent

class Map():
    def __init__(self,x,y,div,Agents):
        #initialise map x*y meters, with each step being 1/div meters
        #unknown - 0.5, open- 0.0, wall 1.0
        self.M = np.full((y*div,x*div),0.5)
        
        
        self.Agents = Agents
        #frontiers are areas which are open that are adjacent to unknown spaces
        self.frontiers = []
  
            
    def update_map(self,proposed_updates):
        
        for (x, y), data in proposed_updates.items():
            # self.M[y][x] = data
            if self.M[y][x] == 0.5:
                self.M[y][x] = data
            
            elif self.M[y][x]==0.0:
                if data == 1:
                    self.M[y][x] = 1
            # else:
                # if data == 0.0:
                    # self.M[y][x] = 0.5
    def cost (self,x1, y1, x2,y2):
        return abs(x1-x2)+abs(y2-y1)
    def disp(self):
        print('-----------------------------------------------------')
        for y in range(len(self.M)-1,-1,-1):
            print('|| ',end='')
            for x in range (len(self.M[0])):
                agent_found = False
                for k in range(len(self.Agents)):
                    if x == self.Agents[k].x and y == self.Agents[k].y:
                        print(str(self.Agents[k].pose)+'A'+str(k),end = ' ')
                        agent_found = True
                        break
                if not agent_found:
                    print(self.M[y][x],end = ' ')
            print('||')
        print('-----------------------------------------------------')

    def has_unexplored_neighbor(self, x, y):
        # Check if a cell has at least one explored neighbor
        neigh = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip the current cell
                
                
                new_x, new_y = x + dx, y + dy
                if (0 <= new_x < len(self.M[0]) and
                    0 <= new_y < len(self.M) and
                    self.M[new_y][new_x] == 0.5):
                    neigh.append((new_x,new_y))
        return neigh
    
    def isValidMove(self, new_x, new_y):
        if (0 <= new_x < len(self.M[0]) and
                    0 <= new_y < len(self.M) and
                    self.M[new_y][new_x] == 0.0):
            return True
        return False

    def getNeighbors(self, row, col):
        possibleNeighbors = [
            (row + 1, col),
            (row - 1, col),
            (row, col + 1),
            (row, col - 1)
        ]
        neighbors = []
        for r, c in possibleNeighbors:
            if self.isValidMove(r,c):
                neighbors.append((r,c))
        
        return neighbors
        
    def identify_frontiers(self):
        self.frontiers = []
        # Iterate through each cell in the map
        for y in range(len(self.M)):
            for x in range(len(self.M[0])):
                
                if self.M[y][x] == 0.0:  # Unexplored cell
                    if self.has_unexplored_neighbor( x, y):
                        if (x,y) not in self.frontiers:
                           self.frontiers.append((x,y))
    
    
    def choose_best_frontiers(self, num_frontiers=3):
        # Calculate information gain for all possible combinations of num_frontiers frontiers
        self.identify_frontiers()
        best_frontier_combinations = []

        for combination in itertools.combinations(self.frontiers, num_frontiers):
            visited = []
            for i in range(len(combination)):
                for neigh in self.has_unexplored_neighbor(combination[i][0],combination[i][1]):
                    if neigh not in visited:
                        visited.append(neigh)
            best_frontier_combinations.append((combination, len(visited)))

        # Sort combinations by collective IG and choose the top num_frontiers
        best_frontier_combinations.sort(key=lambda x: x[1], reverse=True)

        # Extract and return the top frontiers
        best_frontiers = best_frontier_combinations[0]
        return best_frontiers[0]

    def assign_frontiers_to_robots(self):
        assignments = {}  # Dictionary to store frontier assignments
        frontiers = self.choose_best_frontiers()
        for frontier in frontiers:
            min_distance = float('inf')  # Initialize with a large value
            assigned_robot = None

            for robot in self.Agents:
                if robot in assignments:
                    continue
                #distance = len(self.astar((robot.x, robot.y), (frontier[0], frontier[1])))
                distance = self.cost(robot.x, robot.y, frontier[0], frontier[1])
                if distance < min_distance:
                    min_distance = distance
                    assigned_robot = robot

            
            assignments[assigned_robot] = frontier
            

        return assignments
    
    def up(self,robot):
        route = []
        if robot.pose == 0:
            pass
        elif robot.pose == 1:
            route.append("l")
        elif robot.pose == 2:
            route.append("l")
            route.append("l")
        else:
            route.append("r")
        #robot.pose = 0
        
        route.append("f")
        return route
    
    def down(self, robot):
        route = []
        if robot.pose == 0:
            route.append("l")
            route.append("l")
        elif robot.pose == 1:
            route.append("r")
        elif robot.pose == 2:
            pass
        else:
            route.append("l")
        #robot.pose = 2
        
        route.append("f")
        return route
    
    def left(self, robot):
        
        route = []
        if robot.pose == 0:
            route.append("l")
        elif robot.pose == 1:
            route.append("r")
            route.append("r")
        elif robot.pose == 2:
            route.append("r")
        else:
            pass
        #robot.pose = 3
        
        
        route.append("f")
        return route
    
    def right(self, robot):
        route = []
        if robot.pose == 0:
            route.append("r")
        elif robot.pose == 1:
            pass
        elif robot.pose == 2:
            route.append("l")
            
        else:
            route.append("r")
            route.append("r")
        robot.pose = 1
        
        route.append("f")
        return route
    
    def retrace(self,goal,start,mapp):
        answer = []
        curr = goal
        while curr != start:
            nex = mapp[curr]
            if nex[0]-curr[0] == 1:
                answer.append('L')
            elif nex[0]-curr[0] == -1:
                answer.append('R')
            elif nex[1]-curr[1] == 1:
                answer.append('D')
            else:
                answer.append('U')
            curr = mapp[curr]
        
        return answer[::-1]
    
    def ManhattenD(self,curr,target):
        
        return abs(curr[0]-target[0])+abs(curr[1]-target[1])
    
    def searchPos(self,target,Q):
        for i in range (len(Q)):
            if Q[i][2] == target:
                return i
        return None

        
    
    def astar(self, start, goal):
        arr = []
        
        #heap (bestGuessCost,costFromStart,(pos))
        pq.heappush(arr,(self.ManhattenD(start,goal),0,start))
        parent = dict()
        #dict{(pos),parent}
        
        while True:
            if len(arr)==0:
                return []
            curr = pq.heappop(arr)
            #curr  = (bestGuessCost,costFromStart,(pos))
            if curr[2] == goal:
                break
            
            for i in self.getNeighbors(curr[2][0],curr[2][1]):
                cost = curr[1]+1
                fn = cost+ self.ManhattenD(curr[2],goal) #heuristic
                foundPos= self.searchPos(i,arr)
                if i not in parent and not foundPos: #not visited
                    parent[i] = curr[2]
                    pq.heappush(arr,(fn,cost,i))
                
                elif foundPos:
                    if cost<arr[foundPos][1]:
                        arr = arr[:foundPos]+arr[foundPos+1:] #delete from heap
                        pq.heapify(arr)
                        pq.heappush(arr,(fn,cost,i))
                        parent[i] = curr[2]
                
        return self.retrace(goal,start,parent)
    
    def plan_astar(self):
        assignments = self.assign_frontiers_to_robots()
        routes = {}
        for robot, frontier in assignments.items():
            route = []  # Initialize an empty list to store the route
            robot_x, robot_y, robot_pose = robot.x, robot.y, robot.pose
            target_x, target_y = frontier[0], frontier[1]
            routes[robot] = self.astar((robot_x,robot_y),(target_x,target_y))
        return routes
    
    def plan_routes(self):
        final = {}
        assignments = self.plan_astar()
        #print(assignments)
        for robot, routes in assignments.items():
            route = []
            for r in routes:
                if r=="U":
                    for action in self.up(robot):
                        route.append(action)
                    robot.pose = 0
                    robot.y+=1
                elif r == "D":
                    for action in self.down(robot):
                        route.append(action)
                    robot.pose = 2
                    robot.y-=1
                elif r == "L":
                    for action in self.left(robot):
                        route.append(action)
                    robot.pose = 3
                    robot.x-=1
                else:
                    for action in self.right(robot):
                        route.append(action)
                    robot.pose = 1
                    robot.x+=1
            
            final[robot.name] = route
        return final



    def cont(self):
        for i in self.M:
            if 0.5 in i:
                return True
        return False
class Driver (Supervisor):
    timeStep = 128

    def __init__(self):
        super(Driver, self).__init__()
        self.emitter = self.getDevice('emitter')
        self.receiver = self.getDevice('receiver')
        self.receiver.enable(self.timeStep)
        robot = [self.getFromDef('ROBOT1'),self.getFromDef('ROBOT2'),self.getFromDef('ROBOT3')]
        agents = [Agent(0,0,4,1,'A1'),Agent(0,0.25,4,1,'A2'),Agent(0,0.5,4,1,'A3')]
        self.translationField = [robot[0].getField('translation'),robot[1].getField('translation'),robot[2].getField('translation')]
        self.rotationField  = [robot[0].getField('rotation'),robot[1].getField('rotation'),robot[2].getField('rotation')]
        self.keyboard.enable(Driver.timeStep)
        self.keyboard = self.getKeyboard()
        self.map = Map(3,4,4,agents)
        self.map.disp()
        for agent in agents:
            self.map.M[agent.y][agent.x] =0
        
        #always explore all unknown space at a new frontier
        #ie turn 360
        movesCompleted = False
        done = 0
        while not movesCompleted:
            msg = self.listen()
            if msg:
                if msg[0] == 'Done':#COMPLETED
                    done+=1
                    print('completed')
                else:

                    if msg[0] =='Driver':
                        update = msg[1]
                    
                    
                        self.map.update_map(update)
                        print(update)
                        self.map.disp()
                    
                self.step(self.timeStep)
            if done == 3:
                movesCompleted = True
            self.step(self.timeStep)
            

        
        
        self.map.identify_frontiers()
        self.map.disp()
        print('initialise driver complete')

        
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
            return message
    
    def cleanup(self):#for a little bit of position fixing
        for i in range(len( self.map.Agents)):
            curr = self.map.Agents[i]
            
            self.translationField[i].setSFVec3f([curr.x/4+0.125,curr.y/4+0.125,0])
            if curr.pose ==0:
                self.rotationField[i].setSFRotation([0,0,-1,-math.pi/2])
            elif curr.pose ==1:
                self.rotationField[i].setSFRotation([0,0,-1,0])
            elif curr.pose ==2:
                self.rotationField[i].setSFRotation([0,0,-1,math.pi/2])
            else:
                self.rotationField[i].setSFRotation([0,0,-1,math.pi])
            
            
            

    
    def run(self):
        previous_message = ''
        counter = 0
        while self.map.cont():
            counter+=1
            movesCompleted = True
            if movesCompleted:
                nextMoves = self.map.plan_routes()
                #print('astar')
                destination = {}
                for i in self.map.Agents:
                    destination[i.name] = (i.x,i.y,i.pose)
                #print(nextMoves)
                
                
                message = str(['Agent',nextMoves,destination])
                #print(message)
                self.talk(message,previous_message)
                previous_message = message
                movesCompleted = False
            done = 0
            while not movesCompleted:
                msg = self.listen()
                if msg:
                    if msg[0] == 'Done':#COMPLETED
                        done+=1
                    else:
    
                        if msg[0] =='Driver':
                            update = msg[1]
                            
                        
                            self.map.update_map(update)
                            
                            
                if done == 3:
                    movesCompleted = True
                    self.map.disp()
                    print('\n')
                    
                    self.cleanup()
                if self.step(self.timeStep) == -1:
                    break

            
controller = Driver()
common_print('driver')
controller.run()