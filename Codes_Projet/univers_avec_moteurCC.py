"""
Fichier pour le controleur PID du moteur centrifugeuse.
Pour changer les coefficients kp et kd (voire ki) il suffit de se rendre 
lignes 345 à 349.
La longueur du ressort cible peut également être réglée tout comme la vitesse 
de rotation maximale.
"""
from random import random,randint
from vector3D import Vector3D as V3D
from particule import Particule
import pygame
from pygame.locals import *
from Moteur_CC import MoteurCC, ControlPI_vitesse
from types import MethodType
import matplotlib.pyplot as plt

class Univers(object):
    def __init__(self,name='ici',t0=0,step=0.1,dimensions=(100,100),game=False,gameDimensions=(1024,780),fps=60):
        self.name=name
        self.time=[t0]
        self.population = []
        self.generators = []
        self.step = step
        self.drawOnly = []  # éléments dessinés mais pas simulés

        self.dimensions = dimensions
        
        self.game = game
        self.gameDimensions = gameDimensions
        self.gameFPS = fps
        
        self.scale =  gameDimensions[0] / dimensions[0]
        
    
    def __str__(self):
        return 'Univers (%s,%g,%g)' % (self.name, self.time[0], self.step)
        
    def __repr__(self):
        return str(self)
        
    def addParticule(self,*members):
        for i in members:
            self.population.append(i)
        
    def addGenerators(self,*members):
        for i in members:
            self.generators.append(i)
        
    def simulateAll(self):
        for p in self.population:
            for source in self.generators:
                source.setForce(p)
            p.simulate(self.step)

        # Ajout ici : mesurer omega et longueur du ressort
        pos1 = moteur_fixe.getPosition()
        pos2 = P_mobile.getPosition()
        longueur_ressort = (pos2 - pos1).mod()
        omega_history.append(moteur.omega)
        length_history.append(longueur_ressort)

        self.time.append(self.time[-1] + self.step)

    def simulateFor(self,duration):
        while duration > 0:
            self.simulateAll()
            duration -= self.step
        
    def plot(self):
        from pylab import figure,legend,show
        figure(self.name)
        for agent in self.population:
            agent.plot()
        legend()
        show()
       
    def gameInteraction(self,events,keys):
        pass
    
    def simulateRealTime(self):
        import pygame
        running = self.game
        successes, failures = pygame.init()
        W, H = self.gameDimensions
        screen = pygame.display.set_mode((W, H))        
        clock = pygame.time.Clock()
        while running:
            screen.fill((240,240,240))
            pygame.event.pump()
            keys = pygame.key.get_pressed()
            events = pygame.event.get()
            if keys[pygame.K_ESCAPE]:
                running = False
            for event in events:
                if event.type == pygame.QUIT:
                    running = False
            self.gameInteraction(events,keys) 
            self.simulateFor(1/self.gameFPS)    
            for t in self.population:
                t.gameDraw(self.scale,screen)
            for g in self.drawOnly:
                g.gameDraw(self.scale, screen)
            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            font_obj = pygame.font.Font('freesansbold.ttf', 22)
            text_surface_obj = font_obj.render(f'time: {self.time[-1]:.2f}', True, 'black', (240,240,240))
            text_rect_obj = text_surface_obj.get_rect()
            text_rect_obj.topleft = (10, 10)
            screen.blit(text_surface_obj, text_rect_obj)
            if hasattr(self, "drawExtra"):
                self.drawExtra(screen)
            pygame.display.flip()
            clock.tick(self.gameFPS)
        pygame.quit()
        plt.figure()
        plt.plot(omega_history, length_history, '.', markersize=3)
        plt.xlabel("Vitesse angulaire ω (rad/s)")
        plt.ylabel("Longueur du ressort L (cm)")
        plt.title("Allongement du ressort en fonction de ω")
        plt.grid(True)
        plt.show()

class Force(object):
    def __init__(self,force=V3D(),name='force',active=True):
        self.force = force
        self.name = name
        self.active = active
        
    def __str__(self):
        return "Force ("+str(self.force)+', '+self.name+")"
    
    def __repr__(self):
        return str(self)
    
    def setForce(self,particule):
        if self.active:
            particule.applyForce(self.force)

class ForceSelect(Force):
    def __init__(self,force=V3D(),subject=None,name='force',active=True):
        self.force = force
        self.name = name
        self.active = active
        self.subjects=subject

    def setForce(self,particule):
        if self.active and particule in self.subjects:
            particule.applyForce(self.force)

class Gravity(Force):
    def __init__(self,g=V3D(0,-9.8),name='gravity',active=True):
        self.g = g
        self.name = name
        self.active = active

    def setForce(self,particule):
        if self.active:
            particule.applyForce(self.g*particule.mass)

class Bounce_y(Force):
    def __init__(self,k=1,step=0.1,name="boing",active=True):
        self.name=name
        self.k = k
        self.step = step

    def setForce(self,particule):
        if particule.getPosition().y < 0 and particule.getSpeed().y <0:
            particule.applyForce(-2*(self.k/self.step)*V3D(0,particule.getSpeed().y * particule.mass ))

class Bounce_x(Force):
    def __init__(self,k=1,step=0.1,name="boing",active=True):
        self.name=name
        self.k = k
        self.step = step

    def setForce(self,particule):
        if particule.getPosition().x < 0 and particule.getSpeed().x <0:
            particule.applyForce(-2*(self.k/self.step)*V3D(particule.getSpeed().x * particule.mass))

class SpringDamper(Force):
    def __init__(self,P0,P1,k=0,c=0,l0=0, l_max = 15, active=True,name="spring_and_damper"):
        Force.__init__(self,V3D(),name,active)
        self.k = k
        self.c = c
        self.P0 = P0
        self.P1 = P1
        self.l0 = l0
        self.l_max = l_max

    def setForce(self, particule):
        vec_dir = self.P1.getPosition() - self.P0.getPosition()
        dist = vec_dir.mod()

        # Correction de position si la distance dépasse l_max
        if self.l_max is not None and dist > self.l_max:
            # Direction normalisée
            dir_norm = vec_dir.norm()
            # Nouvelle position corrigée
            new_position = self.P0.getPosition() + dir_norm * self.l_max
            self.P1.position[-1] = new_position  # Forcer la dernière position
            self.P1.speed[-1] = self.P1.speed[-1] * 0.5  # Optionnel : amortir un peu la vitesse
            dist = self.l_max
            vec_dir = new_position - self.P0.getPosition()

        # Appliquer la force de ressort
        v_n = vec_dir.norm()
        flex = dist - self.l0
        vit = self.P1.getSpeed() - self.P0.getSpeed()
        vit_n = vit ** v_n * self.c
        force = (self.k * flex + vit_n) * v_n

        if particule == self.P0:
            particule.applyForce(force)
        elif particule == self.P1:
            particule.applyForce(-force)


class Link(SpringDamper):
    def __init__(self,P0,P1,name="link"):
        l0 = (P0.getPosition()-P1.getPosition()).mod()
        SpringDamper.__init__(self,P0, P1,5000,100,l0,True,name)

class Prism(SpringDamper):
    def __init__(self,P0,P1,axis=V3D(),name="prism"):
        l0 = (P0.getPosition()-P1.getPosition()).mod()
        SpringDamper.__init__(self,P0, P1,1000,100,l0,True,name)
        self.axis=axis.norm()

    def setForce(self, particule):
        vec_dir = self.P1.getPosition() - self.P0.getPosition()
        vec_dir -= vec_dir ** self.axis * self.axis
        v_n = vec_dir.norm()
        flex = vec_dir.mod()-self.l0
        vit = self.P1.getSpeed() - self.P0.getSpeed()
        vit_n = vit ** v_n * self.c 
        force = (self.k * flex + vit_n)* v_n
        if particule == self.P0:
            particule.applyForce(force)
        elif particule == self.P1:
            particule.applyForce(-force)

class ForceCentrifuge:
    def __init__(self, moteur_fixe):
        self.moteur_fixe = moteur_fixe

    def setForce(self, particule):
        omega = self.moteur_fixe.moteur.omega
        if abs(omega) < 1e-5:
            return  # Pas de rotation, pas de force centrifuge

        pos1 = self.moteur_fixe.getPosition()
        pos2 = particule.getPosition()
        r_vec = pos2 - pos1
        r = r_vec.mod()
        if r == 0:
            return

        # Direction radiale et tangentielle
        radial_dir = r_vec.norm()
        tangent_dir = V3D(-radial_dir.y, radial_dir.x)

        # Force centrifuge radiale : m * ω² * r
        F_centrifuge = radial_dir * (particule.mass * omega**2 * r)

        # Vitesse attendue pour trajectoire circulaire
        v_attendue = tangent_dir * (omega * r)
        v_actuelle = particule.getSpeed()
        delta_v = v_attendue - v_actuelle

        # Correction tangentielle douce (comme un PID D)
        facteur_sync = 2.0  # Réglable
        F_sync = delta_v * particule.mass * facteur_sync

        # Appliquer les deux forces
        particule.applyForce(F_centrifuge)
        particule.applyForce(F_sync)


class MoteurFixe(Particule):
    def __init__(self, p0, moteur: MoteurCC, radius=1.0):
        super().__init__(p0=p0, fix=True)
        self.moteur = moteur
        self.radius = radius
        self.color = (0, 0, 255)
        
    def gameDraw(self, scale, screen):
        pos = self.getPosition()
        x = int(pos.x * scale)
        y = int(pos.y * scale)
        pygame.draw.circle(screen, self.color, (x, y), int(self.radius * scale))

class ControlPID_elongation:
    def __init__(self, moteur, l_cible, Kp=1.0, Ki=0.1, Kd=0.05, vmax=24.0):
        self.moteur = moteur
        self.l_cible = l_cible
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.vmax = vmax

        self.integral = 0.0
        self.last_error = 0.0

    def update(self, longueur_actuelle, step):
        # Calcul de l’erreur
        erreur = self.l_cible - longueur_actuelle

        # Intégrale et dérivée
        self.integral += erreur * step
        deriv = (erreur - self.last_error) / step if step > 0 else 0.0

        # PID
        tension = self.Kp * erreur + self.Ki * self.integral + self.Kd * deriv

        # Saturation
        tension = max(min(tension, self.vmax), -self.vmax)

        # Appliquer
        self.moteur.setVoltage(tension)
        self.moteur.simule(step)

        # Stocker erreur pour la prochaine dérivée
        self.last_error = erreur




if __name__ == '__main__':
    #valeurs pour tracer la longueur du ressort en fonction de la vitesse omega
    omega_history = []
    length_history = []
    
    monUnivers = Univers(game=True)
    monUnivers.step = 0.01

    moteur = MoteurCC()
    moteur.setVoltage(10)
    moteur_fixe = MoteurFixe(p0=V3D(50, 50), moteur=moteur, radius=0.5)

    P_mobile = Particule(
        p0=V3D(50, 45),  # plus proche du moteur
        v0=V3D(0, 0, 0),
        a0=V3D(0, 0, 0),
        mass=1.0,
        color=(139/255, 69/255, 19/255)
    )
    elongation_cible = 10 #en cm
    k_p = 20.0
    k_i = 0.5
    k_d = 5.0
    vitesse_max = 24.0 #rad/s
    # Contrôleur PI sur l'élongation (réglage de la vitesse cible)
    control_elong = ControlPID_elongation(moteur, l_cible=elongation_cible, Kp= k_p, Ki=k_i, Kd=k_d, vmax=vitesse_max)

    

    ressort = SpringDamper(P0=moteur_fixe, P1=P_mobile, k=10, c= 2, l0=5)
    gravite = Gravity(g=V3D(0, -9.81))
    centrifuge = ForceCentrifuge(moteur_fixe)

    monUnivers.addParticule(moteur_fixe, P_mobile)
    monUnivers.addGenerators(gravite, ressort, centrifuge)

    def myInteraction(self, events, keys):
        l_actuelle = (P_mobile.getPosition() - moteur_fixe.getPosition()).mod()
        control_elong.update(l_actuelle, self.step)




    monUnivers.gameInteraction = MethodType(myInteraction, monUnivers)

    def drawExtraInfo(self, screen):
        font = pygame.font.Font(None, 24)

        # Affichage d'omega
        omega_txt = font.render(f"\u03A9 = {moteur.omega:.2f}", True, (0, 0, 0))
        screen.blit(omega_txt, (screen.get_width() - 120, 10))

        # Affichage de la longueur du ressort
        pos1 = moteur_fixe.getPosition()
        pos2 = P_mobile.getPosition()
        longueur_ressort = (pos2 - pos1).mod()

        longueur_txt = font.render(f"L = {longueur_ressort:.2f}", True, (100, 0, 0))
        screen.blit(longueur_txt, (screen.get_width() - 120, 35))

        vitesse = P_mobile.getSpeed().mod()
        v_txt = font.render(f"v = {vitesse:.2f} m/s", True, (0, 100, 0))
        screen.blit(v_txt, (screen.get_width() - 120, 60))

        elong_txt = font.render(f"L_target = {control_elong.l_cible:.2f}", True, (0, 0, 150))
        screen.blit(elong_txt, (screen.get_width() - 160, 85))

        tension_txt = font.render(f"Um = {moteur.Um:.2f} V", True, (150, 0, 150))
        screen.blit(tension_txt, (screen.get_width() - 160, 110))



    monUnivers.drawExtra = MethodType(drawExtraInfo, monUnivers)

    monUnivers.simulateRealTime()
