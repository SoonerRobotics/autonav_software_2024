class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        #TODO also initialize polar
    
    def __init__(self, angle, length):
        self.angle = angle
        self.lenght = length
        #TODO initialize cartesian
    
    def getXY(self):
        return self.x, self.y
    
    def getPolar(self):
        return self.angle, self.length
    
    def setXY(self, x, y):
        self.x = x
        self.y = y
        #TODO update polar
    
    def setPolar(self, angle, length):
        self.angle = angle
        self.length = length
        #TODO update cartesian
    
    def setLength(self, length):
        self.length = length
        #TODO update cartesian
    
    def __add__(self, other):
        return self.x + other.x, self.y + other.y
    
    def __sub__(self, other):
        return self.x + other.x, self.y + other.y