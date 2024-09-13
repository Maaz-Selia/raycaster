import vec3

class Ray:

    def __init__(self, origin, direction):
        self.origin = origin
        self.direction = direction

    def position(self, t):
        origin = self.origin
        direction = self.direction
        return origin + (direction * t)