
# from controller import Supervisor
# import numpy as np
# import csv

# class Driver (Supervisor):
#     timeStep = 128

#     def __init__(self):
#         super(Driver, self).__init__()
#         #self.emitter = self.getDevice('emitter')
#         #self.receiver = self.getDevice('receiver')
#         #self.receiver.enable(self.timeStep)
#         robot = [self.getFromDef('ROBOT1'),self.getFromDef('ROBOT2'),self.getFromDef('ROBOT3')]
#         self.translationField = [robot[0].getField('translation'),robot[1].getField('translation'),robot[2].getField('translation')]
#         self.keyboard.enable(Driver.timeStep)
#         self.keyboard = self.getKeyboard()
#         self.target = [1,1.5]
#         self.M = np.full((4*4,3*4),0)
#     def nearest(self, n):
#         # Get current position
#         curr = self.translationField[n].getSFVec3f()
        
#         posx, posy = curr[0], curr[1]
    
#         # Calculate the nearest grid cell indices
#         grid_cell_size = 0.25
#         offset_x, offset_y = 0.125, 0.125
#         compx, compy = 0, 0
        
#         # Adjust for the offset and compute the grid cell indices
#         grid_x = int(round(max(0, (posx - offset_x) / grid_cell_size))) + compx
#         grid_y = int(round(max(0, (posy - offset_y) / grid_cell_size))) + compy
    
#         return grid_x, grid_y
    
        
#     def run(self):
#         i=0
#         while True:
#             if i%2==0: #simstep 2
#                 for j in range (len(self.translationField)):
#                     pos = self.nearest(j)
#                     self.M[pos[1],pos[0]]+=1
#             if i==3000: #2000 steps
#                 with open('coverage.csv', 'w', newline='') as csvfile:
#                     spamwriter = csv.writer(csvfile)
                    
#                     spamwriter.writerows(reversed(self.M))
#                     print('CSV written')
                            
#                 break
            
#             if i%100==0:
#                 print("Count is",str(i))
                
#             i+=1
            
#             if self.step(self.timeStep) == -1:
#                 break

# controller = Driver()
# controller.run()


from controller import Supervisor
import numpy as np
import csv
split = 1
class Driver (Supervisor):
    timeStep = 128
    split = 1
    def __init__(self):
        super(Driver, self).__init__()
        #self.emitter = self.getDevice('emitter')
        #self.receiver = self.getDevice('receiver')
        #self.receiver.enable(self.timeStep)
        robot = [self.getFromDef('ROBOT1'),self.getFromDef('ROBOT2'),self.getFromDef('ROBOT3')]
        self.translationField = [robot[0].getField('translation'),robot[1].getField('translation'),robot[2].getField('translation')]
        #self.keyboard.enable(Driver.timeStep)
        self.keyboard = self.getKeyboard()
        self.target = [1,1.5]
        self.M = np.full((split*4*4,split*3*4),0)
    def nearest(self, n):
        # Get current position
        curr = self.translationField[n].getSFVec3f()
        
        posx, posy = curr[0], curr[1]
    
        # Calculate the nearest grid cell indices
        grid_cell_size = 0.25/split
        offset_x, offset_y = 0.125/split, 0.125/split
        compx, compy = 0, 0
        
        # Adjust for the offset and compute the grid cell indices
        grid_x = int(round(max(0, (posx - offset_x) / grid_cell_size))) + compx
        grid_y = int(round(max(0, (posy - offset_y) / grid_cell_size))) + compy
    
        return grid_x, grid_y
    
        
    def run(self):
        i=0
        while True:
            if i%2==0: #simstep 2
                for j in range (len(self.translationField)):
                    pos = self.nearest(j)
                    self.M[pos[1],pos[0]]+=1
            if i==3000: #2000 steps
                
                # Assuming self.M is the new data to be added
                # Assuming self.M is the new data to be added
                new_data = list(reversed(self.M))
                
                # Read existing data if the file exists
                # Read existing data if the file exists
                try:
                    with open('coverageFBE.csv', 'r', newline='') as csvfile:
                        existing_data = list(csv.reader(csvfile))
                        
                except FileNotFoundError:
                    existing_data = []
                
                # Combine existing data (if any) with the new data using summation
                if existing_data:
                    combined_data = []
                else:
                    combined_data = new_data
                
                for new_data_row, existing_row in zip(new_data, existing_data):
                    combined_row = []
                    for new_value, existing_value in zip(new_data_row, existing_row):
                        combined_row.append(str(int(existing_value) + int(new_value)))
                    combined_data.append(combined_row)
                
                
                with open('coverageFBE.csv', 'w', newline='') as csvfile:
                    wfile = csv.writer(csvfile)
                    for row in combined_data:
                        wfile.writerow(row)
                
                print('CSV written')
                            
                break
            
            if i%100==0:
                print("Count is",str(i))
                
            i+=1
            
            if self.step(self.timeStep) == -1:
                break

controller = Driver()
controller.run()
