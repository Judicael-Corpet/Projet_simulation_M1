from vector3D import Vector3D as V
from torseur import Torseur as T
import matplotlib.pyplot as plt
from numpy import *

import pygame
from pygame.locals import *
from types import MethodType

class Poutre:
    def __init__(self, name = 'poutre', color = (0,0,0), masse = 1, longueur = 1, pos_init = V(), vit_init = V(), theta_0 = 0, omega_0 = 0):
        self.name = name
        self.color = color
        self.masse = masse
        self.longueur = longueur
        self.torseur = []
        self.pos = [pos_init]   # Position du centre de masse (au milieu)
        self.vit = [vit_init]   # Vitesse du centre de masse (au milieu)
        self.orientation = [theta_0]   # Par rapport à l'axe x
        self.omega = [omega_0] # Vitesse de rotation

    def __str__(self):
        return f"Poutre(name = {self.name}, color = {self.color}, mass = {self.masse}, pos_init = {self.pos[0]}, vit_init = {self.vit[0]})"
    
    def __repr__(self):
        return str(self)
    
    def ajout_effort(self, *force):
        """Ajouter un effort"""
        for i in force:
            self.torseur.append(i)

    def move(self, dt):
        """Bouge d'un temps dt"""
        sum_torseur = T()
        for t in self.torseur:
            P_temp = t.P
            t.changePoint(self.pos[-1])
            sum_torseur += t
            t.changePoint(P_temp)

        # Équilibre dynamique en translation du CDM
        a_x = sum_torseur.R.x/self.masse
        a_y = sum_torseur.R.y/self.masse

        pos_init = self.pos[-1]
        vit_init = self.vit[-1]

        new_vit_x = vit_init.x + a_x*dt
        new_vit_y = vit_init.y + a_y*dt

        self.vit.append(V(new_vit_x, new_vit_y))

        new_pos_x = pos_init.x + vit_init.x*dt + a_x*dt**2/2
        new_pos_y = pos_init.y + vit_init.y*dt + a_y*dt**2/2

        self.pos.append(V(new_pos_x, new_pos_y))

        # Équilibre en rotation
        I = (1/12) * self.masse * self.longueur**2

        # Accélération angulaire
        alpha = sum_torseur.M.z / I
        # Vitesse angulaire (intégration)
        omega_new = self.omega[-1] + alpha * dt
        self.omega.append(omega_new)
        # Position angulaier (intégration)
        theta_new = self.orientation[-1] + omega_new * dt + 1/2 * alpha * dt**2
        self.orientation.append(theta_new)

    def plot(self):
        pos = self.pos[-1]
        theta = self.orientation[-1]
        L = self.longueur / 2

        # Coordonnées des extrémités de la poutre
        dx = L * cos(theta)
        dy = L * sin(theta)

        x_values = [pos.x - dx, pos.x + dx]
        y_values = [pos.y - dy, pos.y + dy]

        plt.plot(x_values, y_values, color = tuple(c/255 for c in self.color), linewidth=3, label = self.name)
        plt.scatter(pos.x, pos.y, color='red')  # Affiche le centre de masse

    def draw(self, surface, scale=100, center=(500, 400)):
        pos = self.pos[-1]
        theta = self.orientation[-1]
        L = self.longueur / 2

        dx = L * cos(theta)
        dy = L * sin(theta)

        x1 = int(center[0] + scale * (pos.x - dx))
        y1 = int(center[1] - scale * (pos.y - dy))
        x2 = int(center[0] + scale * (pos.x + dx))
        y2 = int(center[1] - scale * (pos.y + dy))

        pygame.draw.line(surface, self.color, (x1, y1), (x2, y2), 4)

        # Dessine le centre de masse
        x_cm = int(center[0] + scale * pos.x)
        y_cm = int(center[1] - scale * pos.y)
        pygame.draw.circle(surface, (255, 0, 0), (x_cm, y_cm), 4)

class Force:
    def __init__(self, name = 'force', T = T(), active = True, color = (0,0,0)):
        self.name = name
        self.T_list = [T]  # Liste des torseurs appliqués à chaque pas
        self.active = active
        self.color = color
    
    def set_force(self, poutre):
        if self.active:
            poutre.ajout_effort(self.T)

    def plot(self):
        if isinstance(self, Attractor):
                plt.plot(self.p0.x, self.p0.y, "x")
        for T in self.T_list:
            P = T.P
            R = T.R
            plt.arrow(P.x, P.y, R.x, R.y, head_width=0.2, head_length=0.3,
                      fc='orange', ec='black', length_includes_head=True)
            #plt.text(P.x + R.x, P.y + R.y, self.name, fontsize=8, color='orange')

    def draw(self, surface, scale=10, center=(500, 400)):
        if isinstance(self, Attractor):
            x = int(center[0] + scale * self.p0.x)
            y = int(center[1] - scale * self.p0.y)
            pygame.draw.circle(surface, (0, 0, 255), (x, y), int(self.radius * scale/10), 1)
        for T in self.T_list:
            P = T.P
            R = T.R
            x = int(center[0] + scale * P.x)
            y = int(center[1] - scale * P.y)
            dx = int(scale * R.x)
            dy = -int(scale * R.y)  # - car l'axe y est inversé en pygame
            pygame.draw.line(surface, (255, 140, 0), (x, y), (x + dx, y + dy), 2)
            pygame.draw.circle(surface, (255, 140, 0), (x, y), 3)

class Gravity(Force):
    def __init__(self, name = 'gravity', g = 9.81, active = True, color = (0,0,0)):
        super().__init__(name, [], active, color)
        self.g = g

    def set_force(self, poutre):
        if self.active:
            force = V(0, - self.g * poutre.masse, 0)

            T_g = T(poutre.pos[-1], force, V())
            poutre.ajout_effort(T_g)

            self.T_list.append(T_g)

class Damping(Force):
    def __init__(self, name="damping", c_trans = 0.2, c_rot = 0.1, active=True, color = (0,0,0)):
        super().__init__(name, [], active, color)
        self.c_trans = c_trans
        self.c_rot = c_rot

    def set_force(self, poutre):
        if self.active:
            v = poutre.vit[-1]
            damping_force =  - self.c_trans * v
            damping_moment = V(0, 0, - self.c_rot * poutre.omega[-1])

            T_c = T(poutre.pos[-1], damping_force, damping_moment)
            poutre.ajout_effort(T_c)

            self.T_list.append(T_c)

class Attractor(Force):
    def __init__(self, name = "attractor", color = (0,0,0), p0 = V(), radius = 1, intensity = 1, active = True):
        super().__init__(name = name, T = T(), active = active)
        self.color = color
        self.p0 = p0
        self.radius = radius
        self.intensity = intensity

    def set_force(self, poutre):
        if self.active and (poutre.pos[-1] - self.p0).mod() < self.radius:
            vec = (self.p0 - poutre.pos[-1]).norm()

            T_a = T(poutre.pos[-1], vec) * self.intensity
            poutre.ajout_effort(T_a)

            self.T_list.append(T_a)

class UniversPoutre:
    def __init__(self, name = "Versailles", t0 = 0, dt = 0.01, dimensions = (100, 100), game = False, gameFPS = 60, gameDimensions = (1024, 780)):
        self. name = name
        self.time = t0
        self.dt = dt
        
        self.poutres_list = []
        self.generators_list = []

        self.dimensions = dimensions
        
        self.game = game
        self.gameDimensions = gameDimensions
        self.gameFPS = gameFPS

        self.scale = gameDimensions[0] / dimensions[0]
    
    def __str__(self):
        return f"UniversPoutre(name = {self.name}, temps = {self.time}, dt = {self.dt})"
    
    def __repr__(self):
        return str(self)
    
    def ajout_poutre(self, *poutres):
        for p in poutres:
            self.poutres_list.append(p)

    def ajout_generators(self, *generators):
        for g in generators:
            self.generators_list.append(g)

    def step_all(self):
        # Vider les anciennes forces appliquées
        for g in self.generators_list:
            g.T_list = []

        for p in self.poutres_list:
            for g in self.generators_list:
                g.set_force(p)
            p.move(self.dt)
            p.torseur = []
        self.time += self.dt

    def move_all(self, duration):
        while duration >= 0 :
            self.step_all()
            duration -= self.dt

    def plot_all(self):
        cpt = 0
        for p in self.poutres_list:
            p.plot()
        for g in self.generators_list:
            g.plot()
        plt.title(f"Univers à t = {round(self.time,2)} s")
        plt.grid()
        plt.legend()

    def draw_all(self, surface):
        for p in self.poutres_list:
            p.draw(surface, self.scale)
        for g in self.generators_list:
            g.draw(surface, self.scale)

    def simulatedRealTime(self, duration=10, background_color=(255, 255, 255)):
        pygame.init()
        screen = pygame.display.set_mode(self.gameDimensions)
        clock = pygame.time.Clock()

        running = True
        elapsed_time = 0

        while running and elapsed_time < duration:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill(background_color)
            self.step_all()
            self.draw_all(screen)

            pygame.display.flip()
            clock.tick(self.gameFPS)

            elapsed_time += self.dt

    pygame.quit()

if __name__ == "__main__":

    # Définition des poutres
    P1 = Poutre("Poutre 1", longueur = 8)
    print(P1)

    P2 = Poutre("Poutre 2", (255, 255, 0), 2, 2, V(1,5), V(), pi/3)
    print(P2)

    P3 = Poutre("Poutre 3", (255, 0, 255), 20, 5, V(-2,3), V(), -pi/6, pi/3)
    print(P3)

    P4 = Poutre("Poutre 4", (0, 255, 255), 15, 3, V(-5,-2), V(), -pi/4)
    print(P4)

    # Définition des générateurs (= forces)
    G = Gravity()
    D = Damping()
    A = Attractor(p0 = V(-10, -10), intensity = 20, radius = 100)

    # Définition de l'Univers
    U = UniversPoutre(game = True)
    U.ajout_poutre(P1, P2, P3, P4)
    U.ajout_generators(G, D, A)

    # Tests
    plot = False
    draw = True
    if plot:
        plt.figure(1, figsize = (9,6))
        plt.subplot(131)
        U.move_all(0.5)
        U.plot_all()

        plt.subplot(132)
        U.move_all(1.0)
        U.plot_all()

        plt.subplot(133)
        U.move_all(1.5)
        U.plot_all()

        plt.tight_layout()
        plt.show()
    if draw:
        U.simulatedRealTime(duration=15)
    
    

