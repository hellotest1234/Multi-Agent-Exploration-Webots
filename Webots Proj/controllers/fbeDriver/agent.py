        
class Agent ():
    def __init__(self,tx,ty,div,pose,name):
        self.xmax = 12
        self.ymax = 16
        self.x = int(tx*div)
        self.y = int(ty*div)
        self.pose =pose
        self.name = name
        

    def turnR(self):
        self.pose+=1
        self.pose %=4

    def turnL(self):
        self.pose -=1
        self.pose %=4

    def move(self):
         # Define coordinate changes for each direction (North, East, South, West)
        direction_changes = {
            0: (0, 1),  # North
            1: (1, 0),   # East
            2: (0, -1),   # South
            3: (-1, 0)   # West
            }

        dx, dy = direction_changes[self.pose]
        new_x, new_y = self.x + dx, self.y + dy

        # Check if the new coordinates are within bounds
        if (0 <= new_x < self.xmax and
            0 <= new_y < self.ymax):
            self.x = new_x
            self.y = new_y
        else:
            print('Out of bounds!')
    
    def view(self,left,right):
        #for now assume reading is clear
        left_sensor = left
        right_sensor = right
        l,r,f = 0,0,0

        def is_within_bounds(x, y):
            return 0 <= x < self.xmax and 0 <= y < self.ymax
        if left_sensor:
            l = 1
        if right_sensor:
            r = 1
        if left_sensor and right_sensor:
            f = 1
            l = 0.5
            r = 0.5
        
        x, y = self.x, self.y
        proposed_updates = {}
        update_dict = {}

        if self.pose == 0:
            proposed_updates = {
                (x, y - 1): f,
                (x - 1, y - 1): l,
                (x + 1, y - 1): r
            }
        elif self.pose == 1:
            proposed_updates = {
                (x + 1, y): f,
                (x + 1, y - 1): l,
                (x + 1, y + 1): r
            }
        elif self.pose == 2:
            proposed_updates = {
                (x, y + 1): f,
                (x - 1, y + 1): l,
                (x + 1, y + 1): r
            }
        else:
            proposed_updates = {
                (x - 1, y): f,
                (x - 1, y - 1): l,
                (x - 1, y + 1): r
            }
        for (x, y), data in proposed_updates.items():
            if is_within_bounds(x, y):
                update_dict[(x, y)] = data
        return update_dict
        #sendMapinfo

    def receive(self,route,Map):
        
        for i in route:
            if i == 'f':
                self.move()
            elif i =='r':
                self.turnR()
            else:
                self.turnL()
            
        for i in range(4):
            self.turnR()
            