# -*- coding: utf-8 -*-
"""
Created on Tue Mar 19 09:11:59 2024

@author: UserTP
"""

from vector3D import Vector3D as V3D
from math import pi,atan2

# un objet TurtleBot pour une simulation cinématique simple 
class Turtle(object):
    def __init__(self,P0=V3D(),R0=0,name='toto',color='red'):
        
        self.position=P0
        self.orientation=R0
        self.pose=[(P0,R0)]
        self.name=name
        self.color=color
        
        self.speedTrans = 0
        self.speedRot = 0
        
        self.speedTransMax = 1
        self.speedRotMax = pi/30
        
    def __str__(self):
        return "Turtle (%s, %g, %s, %s)" % (self.position, self.orientation, self.name, self.color)
    
    def __repr__(self):
        return str(self)
    
    def turn(self,angle):
        self.orientation = self.orientation + angle
        self.pose.append((self.position,self.orientation))
                
    def walk(self,dist):
        d = V3D(dist,0,0).rotZ(self.orientation)
        self.position = self.position + d
        self.pose.append((self.position,self.orientation))
    
    def move(self,step=0.1):
        # calcul du déplacement (rotation puis translation) en fct des vitesses
        angle = self.speedRot * step
        dist = self.speedTrans * step
        self.turn(angle)
        self.walk(dist)
        
    def controlGoTo(self,target=V3D(),Kp=10):
        # Correcteur proportionel: on calcule des vitesses pour aller vers target
        errorVect = target-self.position
        targetDirection = errorVect.norm()
        posError = errorVect.mod()

        turtleDirection = V3D(1,0).rotZ(self.orientation)       
        rotError = ( turtleDirection * targetDirection).z #sinus de l'angle entre les 2 vect, en conservant le signe

        if posError > 0.01:
            self.speedTrans = Kp * posError
            self.speedTrans = self.speedTrans * (targetDirection ** turtleDirection) # On diminue la vitesse désirée si on n'est pas vers la cible
            if self.speedTrans > self.speedTransMax:
                self.speedTrans = self.speedTransMax
            if self.speedTrans  < 0: 
                self.speedTrans  = 0 # pas de marche arrière

        else: 
            self.speedTrans = 0
        
        self.speedRot = Kp * rotError

        if self.speedRot > self.speedRotMax:
            self.speedRot = self.speedRotMax
        if self.speedRot < -self.speedRotMax:
            self.speedRot = - self.speedRotMax
        
    
    
    def plot(self):
        
        from pylab import plot
        X=[]
        Y=[]
        for p in self.pose:
            X.append(p[0].x)
            Y.append(p[0].y)
            
    
        return plot(X,Y,color=self.color,label=self.name)+plot(X[-1],Y[-1],'*',color=self.color)   
        
    def gameDraw(self,scale,screen):
        # on va calculer les position en pixel et dessiner eds objet pygame sur la fenetre
        import pygame
        
        X = int(scale*self.position.x)
        Y = int(scale*self.position.y)
        
        vit = V3D(self.speedTrans).rotZ(self.orientation)
        VX = int(scale*vit.x)
        VY = int(scale*vit.y) 
        size=5
        
        if type(self.color) is tuple:
            color = (self.color[0]*255,self.color[1]*255,self.color[2]*255)
        else:
            color=self.color
            
        pygame.draw.circle(screen,color,(X,Y),size*2,size)
        pygame.draw.line(screen,color,(X,Y),(X+VX,(Y+VY)),size)
  
 
class Univers(object):
    def __init__(self,name='ici',t0=0,step=0.1,dimensions=(100,100),game=False,gameDimensions=(1024,780),fps=60):
        self.name=name
        self.time=[t0]
        self.population=[]
        self.step = step
        
        self.dimensions = dimensions
        
        self.game = game
        self.gameDimensions = gameDimensions
        self.gameFPS = fps
        
        self.scale =  gameDimensions[0] / dimensions[0]
        
    
    def __str__(self):
        return 'Univers (%s,%g,%g)' % (self.name, self.time[0], self.step)
        
    def __repr__(self):
        return str(self)
        
    def addUnit(self,*members):
        for i in members:
            self.population.append(i)
        
    def stepAll(self):
        #On calcule le mouvement pur un pas pour chaque agent
        for p in self.population:
            p.move(self.step)
        self.time.append(self.time[-1]+self.step)

    def moveAll(self,duration):
        # On calcule autant de pas que nécessaire pendant duration
        while duration > 0:
            self.stepAll()
            duration -= self.step
    
    
    def plot(self):
        from pylab import figure,legend,show
        
        figure(self.name)
        
        for agent in self.population :
            agent.plot()
            
        legend()
        show()
       
    def gameInteraction(self,events,keys):
        # Fonctin qui sera surchargée par le client pour définir ses intéractions
        pass
    
    def simulateRealTime(self):
        # initilisation de l'environnement pygmae, création de la fenetre
        import pygame
        
        running = self.game
    
        successes, failures = pygame.init()
        W, H = self.gameDimensions
        screen = pygame.display.set_mode((W, H))        
        clock = pygame.time.Clock()
                
        # début simulation
        while running:
            screen.fill((240,240,240)) # effacer les images du pas précédent

            pygame.event.pump() # process event queue
            keys = pygame.key.get_pressed() # It gets the states of all keyboard keys.
            events = pygame.event.get()
            
            # gestion de la fermeture de la fenetre / touche Echap
            if keys[pygame.K_ESCAPE]:
                running = False
                
            for event in events:
                if event.type == pygame.QUIT:
                    running = False
            
            # Allons gérer les interactions ailleurs
            self.gameInteraction(events,keys) 
            
            # simuler les mouvement des chaque agent pendant la durée de ce pas
            self.moveAll(1/self.gameFPS)    
            
            # demander à chaque agent sondessin en pixels sur la fenêtre
            for t in self.population:
                t.gameDraw(self.scale,screen)
            
            
            # get y axis upwards, origin on bottom left : La fenetre pygame a l'axe y vers le bas. On le retourne.
            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            
            pygame.display.flip()  # envoie de la fenetre vers l'écran
            clock.tick(self.gameFPS) # attendre le prochain pas d'affichage
        
        pygame.quit()

if __name__ == "__main__":    
    from pylab import figure, show, legend
   
    step = 0.1
    
    bob = Turtle(name='bob')
    toto = Turtle(color='green')
    print(toto)
    print(bob)
    bob.turn(pi/4)
    bob.walk(.5)
    bob.turn(pi/4)
    bob.walk(.3)
    toto.turn(pi/2)
    toto.walk(1)
    toto.turn(-pi/4)
    toto.walk(.8)
    
    bob.speedTrans = 1
    bob.speedRot = pi/10
    
    for i in range(100):
        bob.controlGoTo(V3D(25,25))
        bob.move()
        
    for i in range(100):
        bob.controlGoTo(V3D(-25,25))
        bob.move()
        
    for i in range(100):
        bob.controlGoTo(V3D(-25,0),1)
        bob.move(step)
    
    figure('Turtles')
    bob.plot()
    toto.plot()
    legend()
    show()   
      
    plage = Univers(name='plage')
    
    plage.addUnit(toto,bob)
    
    print(plage.population)
    plage.moveAll(10)

    plage.plot()
  
