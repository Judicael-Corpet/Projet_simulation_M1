from vector3D import Vector3D as V
from torseur import Torseur as T
import matplotlib.pyplot as plt
from univers import *
from numpy import *

import pygame
from pygame.locals import *
from types import MethodType

class Barre_2D :
    def __init__(self, name = "Barre", longueur = 1, masse = 1, color = (0,0,0),pos_init = V(), vit_init = V(), theta_0 = 0, omega_0 = 0):
        self.name = name
        self.longueur = longueur
        self.masse = masse
        self.color = color
        self.pos = [pos_init] #position du centre de masse
        self.vit = [vit_init] #vitesse du centre de masse
        self.orientation = [theta_0] # Par rapport à l'axe x
        self.omega= [omega_0] # Vitesse de rotation
        self.torseur_efforts =[]

    def __str__(self):
        return f" {self.name}, longueur = {self.longueur}, masse = {self.masse}, couleur =  {self.color}, position initiale = {self.pos}, vitesse initiale = {self.vit}"

    def __repr__(self):
        return(str(self))
    
    def applyEffort(self, force, pt_application):
        """Ajouter un effort sur un point d'application compris entre -1 (extrémité gauche de la barre) et +1 (extrémité droite de la barre). 
        0 représente le centre de la barre
        Le point d'application est donc un coefficient"""

        effort = T()
        if pt_application < 0 :
            effort.P.x = -pt_application*cos(self.orientation)
            effort.P.y = -pt_application*sin(self.orientation)

        

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


if __name__ == "__main__":

    # Définition des barres 2D
    B1 = Barre_2D("Barre 1", longueur = 8)
    print(B1)

    B2 = Barre_2D("Barre 2", (255, 255, 0), 2, 2, V(1,5), V(), pi/3)
    print(B2)

    B3 = Barre_2D("Barre 3", (255, 0, 255), 20, 5, V(-2,3), V(), -pi/6, pi/3)
    print(B3)

    B4 = Barre_2D("Barre 4", (0, 255, 255), 15, 3, V(-5,-2), V(), -pi/4)
    print(B4)

    # Définition de l'Univers
    U = Univers(game=True)

    # Définition des générateurs (= forces)
    grav = Gravity(V3D(0,-10))
    boing = Bounce_y(.9,U.step) 
    boing2 = Bounce_x(1,U.step)
    U.addBarre(B1, B2, B3, B4)
    U.addGenerators(grav, boing, boing2)

    U.simulatedRealTime(duration=15)

   