from vector3D import Vector3D as V
from torseur import Torseur as T
import matplotlib.pyplot as plt
import numpy as np
from numpy import *

import pygame
from pygame.locals import *
from types import MethodType

class Barre2D:
    def __init__(self, name = 'barre', color = (0,0,0), masse = 1, longueur = 1, pos_init = V(), vit_init = V(), theta_0 = 0, omega_0 = 0, point_fixe=None, fixed_end='left'):
        self.name = name
        self.color = color
        self.masse = masse
        self.longueur = longueur
        self.torseur = []
        self.pos = [pos_init]   # Position du centre de masse (au milieu)
        self.vit = [vit_init]   # Vitesse du centre de masse (au milieu)
        self.orientation = [theta_0]   # Par rapport à l'axe x
        self.omega = [omega_0] # Vitesse de rotation
        self.point_fixe = point_fixe  # Le point fixe pour le pendule
        self.fixed_end = fixed_end  # 'left' ou 'right' pour déterminer l'extrémité fixée

    def __str__(self):
        return f"Barre(name = {self.name}, color = {self.color}, mass = {self.masse}, pos_init = {self.pos[0]}, vit_init = {self.vit[0]}, theta_0 = {self.orientation[0]}, Omega_0 = {self.omega[0]} , Point_fixe = {self.point_fixe})"
    
    def __repr__(self):
        return str(self)
    
    def ajout_effort(self, *force):
        """Ajouter un effort"""
        for i in force:
            self.torseur.append(i)

    def applyEffort (self, Force=V(), Torque= V(), Point = 0) :
        if Point == -1 :
            point_appli = V(self.pos[-1].x - self.longueur/2*np.cos(self.orientation[-1]))


    def move(self, dt):
        """Bouge d'un temps dt"""
        g = 9.81  # Accélération due à la gravité
        sum_torseur = T()

        # Si la barre est fixée à un point fixe
        if self.point_fixe is not None:
            # Calcul de l'accélération angulaire pour un pendule
            alpha = - (3 * g / (2 * self.longueur)) * np.sin(self.orientation[-1])

            # Mise à jour de la vitesse angulaire (intégration)
            omega_new = self.omega[-1] + alpha * dt
            self.omega.append(omega_new)

            # Mise à jour de l'orientation de la barre (intégration)
            theta_new = self.orientation[-1] + omega_new * dt + 0.5 * alpha * dt**2
            self.orientation.append(theta_new)

            # Calcul de la position de l'extrémité libre en fonction de l'orientation
            L = self.longueur  # Longueur de la barre

            # Si l'extrémité gauche est fixée
            if self.fixed_end == 'left':
                # Position de l'extrémité libre (droite) calculée par rapport au point fixe
                x_libre = self.point_fixe.x + L * 0.5 * np.sin(self.orientation[-1])
                y_libre = self.point_fixe.y - L * 0.5 * np.cos(self.orientation[-1])

            # Si l'extrémité droite est fixée
            elif self.fixed_end == 'right':
                # Position de l'extrémité libre (gauche) calculée par rapport au point fixe
                x_libre = self.point_fixe.x - L * 0.5 * np.sin(self.orientation[-1])
                y_libre = self.point_fixe.y + L * 0.5 * np.cos(self.orientation[-1])

            # Ajoute la nouvelle position de l'extrémité libre
            self.pos.append(V(x_libre, y_libre))

        else:
            # Si la barre n'est pas fixée, le mouvement suit les calculs de forces génériques
            for t in self.torseur:
                P_temp = t.P
                t.changePoint(self.pos[-1])
                sum_torseur += t
                t.changePoint(P_temp)

            # Équilibre dynamique en translation du centre de masse
            a_x = sum_torseur.R.x / self.masse
            a_y = sum_torseur.R.y / self.masse

            pos_init = self.pos[-1]
            vit_init = self.vit[-1]

            new_vit_x = vit_init.x + a_x * dt
            new_vit_y = vit_init.y + a_y * dt
            self.vit.append(V(new_vit_x, new_vit_y))

            new_pos_x = pos_init.x + vit_init.x * dt + a_x * dt**2 / 2
            new_pos_y = pos_init.y + vit_init.y * dt + a_y * dt**2 / 2
            self.pos.append(V(new_pos_x, new_pos_y))

            # Calcul de l'inertie et de l'accélération angulaire pour la rotation
            I = (1 / 12) * self.masse * self.longueur**2
            alpha = sum_torseur.M.z / I

            # Mise à jour de la vitesse angulaire (intégration)
            omega_new = self.omega[-1] + alpha * dt
            self.omega.append(omega_new)

            # Mise à jour de l'orientation (intégration)
            theta_new = self.orientation[-1] + omega_new * dt + 0.5 * alpha * dt**2
            self.orientation.append(theta_new)



    def plot(self):
        pos = self.pos[-1]
        theta = self.orientation[-1]
        L = self.longueur / 2

        # Coordonnées des extrémités de la barre
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
    
    def set_force(self, barre):
        if self.active:
            barre.ajout_effort(self.T)

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

    def set_force(self, barre):
        if self.active:
            force = V(0, - self.g * barre.masse, 0)

            T_g = T(barre.pos[-1], force, V())
            barre.ajout_effort(T_g)

            self.T_list.append(T_g)

class Damping(Force):
    def __init__(self, name="damping", c_trans = 0.2, c_rot = 0.1, active=True, color = (0,0,0)):
        super().__init__(name, [], active, color)
        self.c_trans = c_trans
        self.c_rot = c_rot

    def set_force(self, barre):
        if self.active:
            v = barre.vit[-1]
            damping_force =  - self.c_trans * v
            damping_moment = V(0, 0, - self.c_rot * barre.omega[-1])

            T_c = T(barre.pos[-1], damping_force, damping_moment)
            barre.ajout_effort(T_c)

            self.T_list.append(T_c)

class Attractor(Force):
    def __init__(self, name = "attractor", color = (0,0,0), p0 = V(), radius = 1, intensity = 1, active = True):
        super().__init__(name = name, T = T(), active = active)
        self.color = color
        self.p0 = p0
        self.radius = radius
        self.intensity = intensity

    def set_force(self, barre):
        if self.active and (barre.pos[-1] - self.p0).mod() < self.radius:
            vec = (self.p0 - barre.pos[-1]).norm()

            T_a = T(barre.pos[-1], vec) * self.intensity
            barre.ajout_effort(T_a)

            self.T_list.append(T_a)

class UniversBarres:
    def __init__(self, name = "Versailles", t0 = 0, dt = 0.01, dimensions = (100, 100), game = False, gameFPS = 60, gameDimensions = (1024, 780)):
        self. name = name
        self.time = t0
        self.dt = dt
        
        self.barres_list = []
        self.generators_list = []

        self.dimensions = dimensions
        
        self.game = game
        self.gameDimensions = gameDimensions
        self.gameFPS = gameFPS

        self.scale = gameDimensions[0] / dimensions[0]
    
    def __str__(self):
        return f"UniversBarres(name = {self.name}, temps = {self.time}, dt = {self.dt})"
    
    def __repr__(self):
        return str(self)
    
    def ajout_Barre(self, *barres):
        for p in barres:
            self.barres_list.append(p)

    def ajout_generators(self, *generators):
        for g in generators:
            self.generators_list.append(g)

    def step_all(self):
        # Vider les anciennes forces appliquées
        for g in self.generators_list:
            g.T_list = []

        for p in self.barres_list:
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
        for p in self.barres_list:
            p.plot()
        for g in self.generators_list:
            g.plot()
        plt.title(f"Univers à t = {round(self.time,2)} s")
        plt.grid()
        plt.legend()

    def draw_all(self, surface):
        for p in self.barres_list:
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

    # Définir un point fixe pour attacher l'extrémité de la barre
    point_fixe = V(0, 0)  # Le point fixe à l'origine (0, 0)

    # Définir une barre (pendule) avec une extrémité fixée
    B1 = Barre2D(
        name="Pendule",
        longueur=20,  # Longueur de la barre (en unités arbitraires)
        point_fixe=point_fixe,  # L'extrémité de la barre est attachée au point fixe
        masse=2,  # Masse de la barre
        theta_0=pi / 4,  # Position initiale de la barre (angle initial)
        omega_0=0,  # Vitesse angulaire initiale
        fixed_end='left'  # L'extrémité gauche est fixée
    )

    print(B1)  # Affiche la description de la barre

    # Définir les forces appliquées à la barre
    G = Gravity()  # Force de gravité
    D = Damping()  # Force d'amortissement

    # Définir l'Univers de simulation
    U = UniversBarres(game=True)  # Initialise l'univers de simulation avec Pygame
    U.ajout_Barre(B1)  # Ajoute la barre à l'univers
    U.ajout_generators(G)  # Ajoute les générateurs de forces (gravité et amortissement)

    # Simuler en temps réel pendant 15 secondes
    U.simulatedRealTime(duration=15)

    
    

