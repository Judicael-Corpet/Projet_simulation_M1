from pylab import figure, plot, show, legend
from particule import *
from univers import *


monUnivers = Univers(game=True)

monUnivers.step=0.0001

P0 = Particule(p0=V3D(10,10,0))

P_fixe = Particule(p0=V3D(monUnivers.dimensions[0]/2,monUnivers.dimensions[1]/2),fix=True)
P_osc = Particule(p0=V3D(monUnivers.dimensions[0]/2,monUnivers.dimensions[1]/2 - 10))
P_double = Particule(p0=V3D(monUnivers.dimensions[0]/2-2,monUnivers.dimensions[1]/2 - 15),color=(0,0,1))

grav = Gravity(V3D(0,-10))

prism1 = Prism(P_fixe,P_osc,V3D(1,0))
tige = Link(P_double,P_osc)


forceX5 = ForceSelect(V3D(5,0),[P_osc])
forceXN5 = ForceSelect(V3D(-5,0),[P_osc])
forceY15 = ForceSelect(V3D(0,15),[P_osc])
forceYN15 = ForceSelect(V3D(0,-15),[P_osc]) 

monUnivers.addParticule(P_fixe,P_osc,P_double)
monUnivers.addGenerators(grav)
monUnivers.addGenerators(prism1,tige)
monUnivers.addGenerators(forceX5,forceXN5,forceY15,forceYN15)

monUnivers.outTime = []
monUnivers.out1 = []
monUnivers.out2 = []

def myInteraction(self,events,keys):
	# controle de leader avec le clavier
	forceX5.active = False
	forceXN5.active = False
	forceY15.active = False
	forceYN15.active = False
	if keys[ord('z')] or keys[pygame.K_UP]: # And if the key is z or K_DOWN:
		forceY15.active = True
	if keys[ord('s')] or keys[pygame.K_DOWN]: # And if the key is s or K_DOWN:
		forceYN15.active = True
	if keys[ord('q')] or keys[pygame.K_LEFT]: # And if the key is q or K_DOWN:
		forceXN5.active = True
	if keys[ord('d')] or keys[pygame.K_RIGHT]: # And if the key is d K_DOWN:
		forceX5.active = True
	
	if keys[pygame.K_SPACE]:
		grav.active = not force.active   
	
	# Création des particules au clic de souris 
	for event in events:
		if event.type == pygame.MOUSEBUTTONDOWN:
			x , y = event.pos # les coordonnées en pixel, y vers le bas !
			pos = V3D(x/self.scale,(monUnivers.gameDimensions[1]-y)/self.scale) # il faut mettre l'axe y vers le haut! 
			vit = V3D(random()*20-10,random()*20-10)
			name='P_'+str(len(monUnivers.population))
			color=(random(),random(),random())
			part = Particule(p0=pos,v0=vit,name=name,color=color,mass=1)
			monUnivers.addParticule(part)

	# post-process
	monUnivers.out1.append( P_osc.getPosition().x - P_osc.position[0].x)
	monUnivers.out2.append( P_double.getPosition().x - P_double.position[0].x +2)
	monUnivers.outTime.append(monUnivers.time[-1])
	 
# Surcharge de la fonction ici
monUnivers.gameInteraction = MethodType(myInteraction,monUnivers)

monUnivers.simulateRealTime()

monUnivers.plot()

print(len(monUnivers.out1), '\t', len(monUnivers.outTime))
figure("pendule double")
plot(monUnivers.outTime,monUnivers.out1,label="P1_x",color=P_osc.color)
plot(monUnivers.outTime,monUnivers.out2,label="P2_x",color=P_double.color)
legend()
show()
