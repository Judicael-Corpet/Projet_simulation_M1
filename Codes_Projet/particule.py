from vector3D import Vector3D as V3D
from math import pi,atan2

class Particule(object):
    def __init__(self, mass=1, p0=V3D(), v0=V3D(), a0=V3D(), fix=False, name="paf", color='red'):
        self.mass = mass
        self.position = p0  # Assurez-vous que position est un seul V3D, pas une liste
        self.speed = v0
        self.acceleration = a0
        self.name = name
        self.color = color
        self.forces = V3D()
        self.fix = fix

    def __str__(self):
        msg = 'Particule ('+str(self.mass)+', '+str(self.position)+', '+str(self.speed)+', '+str(self.acceleration)+', "'+self.name+'", "'+str(self.color)+'" )'
        return msg

    def __repr__(self):
        return str(self)

    def applyForce(self, *args):
        for f in args:
            self.forces += f

    def simulate(self, step):
        self.pfd(step)
        
    def pfd(self, step):
        if not self.speed:
            self.speed = V3D(0, 0, 0)
        if not self.acceleration:
            self.acceleration = V3D(0, 0, 0)

        if not(self.fix):
            a = self.forces * (1 / self.mass)
            v = self.speed + a * step
        else:
            a = V3D()
            v = V3D()

        p = self.position + 0.5 * a * step**2 + self.speed * step

        self.acceleration = a
        self.speed = v
        self.position = p
        self.forces = V3D()

    # Correction de la méthode `getPosition` pour retourner simplement `self.position`
    def getPosition(self):
        return self.position  # Retourne directement l'objet Vector3D (pas d'indexation)
    
    def getSpeed(self):
        return self.speed if self.speed else V3D(0, 0, 0)  # Retourne la vitesse ou une valeur par défaut

    def gameDraw(self, scale, screen):
        import pygame
        X = int(scale * self.position.x)
        Y = int(scale * self.position.y)
        VX = int(scale * self.speed.x)
        VY = int(scale * self.speed.y)
        size = 3
        
        if type(self.color) is tuple:
            color = (self.color[0] * 255, self.color[1] * 255, self.color[2] * 255)
        else:
            color = self.color

        pygame.draw.circle(screen, color, (X, Y), size * 2, size)
        pygame.draw.line(screen, color, (X, Y), (X + VX, (Y + VY)), size)

if __name__=='__main__':
    from pylab import figure, show, legend

    P0 =Particule(v0=V3D(10,10,0))
    print(P0)
    
    while P0.getPosition().y >= 0. :
        P0.applyForce(V3D(0, -9.81, 0))
        P0.pfd(0.01)
        
    
    
    
