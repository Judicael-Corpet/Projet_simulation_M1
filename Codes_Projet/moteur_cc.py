import numpy as np
import matplotlib.pyplot as plt
import math

class MoteurCC () :
    def __init__(self, R, L, Um, i, kc, ke, J, f, step = 0.01):
        #Caractéristiques physiques du moteur
        #Grandeurs électriques
        self.R = R #résistance de l'induit
        self.L = L #inductance de l'induit
        self.Um = Um #tension aux bornes du moteur
        self.i = i #courant 

        #Grandeurs mécaniques
        self.f = f #frottements visqueux
        self.kc = kc
        self.ke = ke
        self.J = J #inertie du rotor

        self.tau = (self.R * self.J)/(self.ke*self.kc + self.R*self.f)
        self.K = self.kc/(self.ke*self.kc + self.R*self.f)
        self.Omega = [self.K*(1 - math.exp(-self.t[1]/self.tau))*self.Um]  #vitesse du rotor 
        
        self.Gamma = [self.kc*self.i] #couple moteur
        self.E = self.ke*self.Omega #force contre-electromotrice

        self.step = step

        self.theta = [0]

        #tableau de temps
        self.t = [0]
          

    def __str__(self):
        return f"MoteurCC avec : résistance R = {self.R}, inductance L = {self.L}, force contre-electromotrice E = {self.E}"    

    def __repr__(self):
        return self.__str__

    def SetVoltage(self, New_U):
        self.Um = New_U
        return self.Um
    
    def getPosition(self):
        return self.theta[-1]

    
    def getSpeed(self) :
        return self.Omega

        
    
    def getTorque(self) :
        return self.Gamma

    def getIntensity(self):
        return self.i
    
    def simule(self, step, duration):
        
    
    def plot(self, duree):
        """
        Trace la vitesse en fonction du temps.
        
        Paramètres :
            duree (float) : durée totale de la simulation en secondes
            type_vitesse (str) : 'constante', 'lineaire' ou 'sinusoïdale'
        """
        
        t = np.arange(0, duree, self.step)  # vecteur temps

        # Tracé
        plt.figure(figsize=(10, 4))
        plt.plot(t, self.Omega, label = "Viesse de rotation du rotor en fonction du temps")
        plt.title("Vitesse en fonction du temps")
        plt.xlabel("Temps (s)")
        plt.ylabel("Vitesse (m/s)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()


