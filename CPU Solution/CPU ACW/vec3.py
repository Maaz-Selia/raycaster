import math

class vec3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, vec):
        return vec3(self.x + vec.x, self.y + vec.y, self.z + vec.z)

    def __sub__(self, vec):
        return vec3(self.x - vec.x, self.y - vec.y, self.z - vec.z)
    
    def __mul__(self, multiplier):
        return vec3(self.x * multiplier, self.y * multiplier, self.z * multiplier)

    def __truediv__(self, divider):
        return vec3(self.x / divider, self.y / divider, self.z / divider)

    def mag(self):
        return math.sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z))

    def dot(self, vec):
        return (self.x * vec.x) + (self.y * vec.y) + (self.z * vec.z)

    def dotSelf(self):
        return (self.x * self.x) + (self.y * self.y) + (self.z * self.z)

    def unitVec(self):
        mag = self.mag()
        if mag == 0:
            raise Exception("Magnitude = 0")
        return self / mag
    
    def distance(self, vec):
        return math.sqrt(math.pow(vec.x - self.x, 2) + math.pow(vec.y - self.y, 2) + math.pow(vec.z - self.z, 2))
