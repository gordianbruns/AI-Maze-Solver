'''
Gordian Bruns, Dipesh Poudel
CS365 Lab A
'''

class Mouse:
    def __init__(self, start):
        self.visited = []
        self.goals_obtained = []
        self.start = start
        self.position = start
        self.cost = 0

    def get_cost(self):
        return self.cost

    def get_position(self):
        return self.position

    def goNorth(self):
        self.position = (self.position[0]-1, self.position[1])
        self.cost += 1

    def goEast(self):
        self.position = (self.position[0], self.position[1]+1)
        self.cost += 1

    def goSouth(self):
        self.position = (self.position[0]+1, self.position[1])
        self.cost += 1

    def goWest(self):
        self.position = (self.position[0], self.position[1]-1)
        self.cost += 1

    def get_goal(self, goal_location):
        self.goals_obtained.append(goal_location)

    def goal_test(self, goals):  # goal test function
        if len(self.goals_obtained) == len(goals):
            return True
        return False
