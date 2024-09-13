import math
import vec3

class Particle:

    def __init__(self, loc, vel):
        self.loc = loc
        self.vel = vel
        self.radius = 1
        self._center = self.loc
        self.force = 0

    def move(self):
        self.loc = self.loc + self.vel
        self._center = self.loc

    def reVel(self):
        xloc = self.loc.x + self.vel.x      
        yloc = self.loc.y + self.vel.y
        zloc = self.loc.z + self.vel.z
        if(xloc < (-1*49) or xloc > 49):
            self.vel.x = self.vel.x * -1
        if(yloc < (-1*49) or yloc > 49):
            self.vel.y = self.vel.y * -1
        if(zloc < (-1*99) or zloc > 0):
            self.vel.z = self.vel.z * -1

    def collisionReVel(self, particle):
        a = particle.vel - self.vel
        initialMomentum = particle.vel + self.vel
        v2After = (initialMomentum - a) / 2
        return v2After + a

    def hit(self, r):
        oc = r.origin - self._center
        a = r.direction.dotSelf()
        b = 2.0 * oc.dot(r.direction)
        c = oc.dotSelf() - (self.radius*self.radius)

        discriminant = (b*b) - (4*a*c)

        if (not discriminant < 0):
            numerator = (-1*b) - math.sqrt(discriminant)
            if numerator > 0:
                t = numerator / (2*a)
                return r.position(t)
            
            numerator = (-1*b) + math.sqrt(discriminant)
            if numerator > 0:
                t = numerator / (2*a)
                return r.position(t)
    
        return -1

    