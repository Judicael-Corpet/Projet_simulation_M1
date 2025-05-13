from vector3D import Vector3D as V3D
from turtle import *
from random import random,randint
import pygame
from pygame.locals import *
from types import MethodType


# Création de l'environnement de simulation
plage = Univers(name='plage',dimensions=(20,20))

# On crée N robot aléatoires
N = 5
for i in range(N):
    x = random()*10
    y =  random()*10
    r = 2 * random() * pi
    p0 = V3D(x,y)
    name = 'Tortue'+str(i)
    color=(random(),random(),random())
    t = Turtle(p0,r,name,color=color)
    t.speedTrans = random()*.05    
    t.speedRot = random()*pi/20
    plage.addUnit(t)

#On désigne la tortue 0 comme leader
leader = plage.population[0]
leader.position=V3D(10,10)
leader.speedRot = 0
leader.speedTrans = 0.3
leader.orientation=pi/4


# On va surcharger la fonction gameInteraction pour créer les évenements interactif pendant SimulationRealTime
def myInteraction(self,events,keys):
    # controle de leader avec le clavier
    if keys[ord('z')] or keys[pygame.K_UP]: # And if the key is z or K_DOWN:
        leader.speedTrans += .05
    if keys[ord('s')] or keys[pygame.K_DOWN]: # And if the key is s or K_DOWN:
        leader.speedTrans -= .05
    if keys[ord('q')] or keys[pygame.K_LEFT]: # And if the key is q or K_DOWN:
        leader.orientation += pi/50
    if keys[ord('d')] or keys[pygame.K_RIGHT]: # And if the key is d K_DOWN:
        leader.orientation -= pi/50
    
    # Création des turtle au clic de souris 
    for event in events:
        if event.type == pygame.MOUSEBUTTONDOWN:
            x , y = event.pos
            p0 = V3D(x/self.scale,(plage.gameDimensions[1]-y)/self.scale) # il faut mettre l'axe y vers le haut!
            name='Turtle_'+str(len(plage.population))
            color=(random(),random(),random())
            t = Turtle(p0,r,name,color)
            plage.addUnit(t)
            
    for turtle in plage.population[1:]:
        turtle.controlGoTo(leader.position,.5)    
     
# Surcharge de la fonction ici
plage.gameInteraction = MethodType(myInteraction,plage)

# Lancement de la simulation
plage.game=True
plage.simulateRealTime()

plage.plot()
