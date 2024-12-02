

class Point:

    def __init__(self, x=0, y=0, velocity=(0, 0), force=(0, 0)):
        self.x = x
        self.y = y
        self.velocity = velocity
        self.force = force


    def getPosition(self):
        return self.x, self.y

