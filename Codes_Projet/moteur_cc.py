import numpy as np

class MoteurCC () :
    def __init__(self, R, L, E, Um, i, J, f, Gamma, Omega):
        #Caractéristiques physiques du moteur
        #Grandeurs électriques
        self.R = R #résistance de l'induit
        self.L = L #inductance de l'induit
        self.E = E #force contre-electromotrice
        

        #Grandeurs mécaniques
        self.J = J #inertie du rotor
        self.f = f #frottements visqueux
        

        #Entrées
        self.Um = Um #tension aux bornes du moteur

        #Sorties
        self.i = i #courant
        self.Gamma = Gamma #couple moteur
        self.Omega = Omega #vitesse du rotor

    def SetVoltage(V):