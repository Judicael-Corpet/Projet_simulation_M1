import matplotlib.pyplot as plt

class MoteurCC:
    def __init__(self,
                 R=1.0,
                 L=0.001,
                 kc=0.01,
                 ke=0.01,
                 J=0.01,
                 f=0.1,
                 inertie_charge=0.0,
                 couple_ext=0.0,
                 viscosite_ext=0.0):
        # Constantes physiques
        self.R = R
        self.L = L
        self.kc = kc
        self.ke = ke
        self.J = J + inertie_charge  # Total inertia
        self.f = f + viscosite_ext   # Total friction
        self.couple_ext = couple_ext

        # Variables d'état
        self.Um = 0.0
        self.omega = 0.0
        self.i = 0.0
        self.torque = 0.0
        self.position = 0.0

        # Stockage pour tracé
        self.speed = []
        self.time = []

    def setVoltage(self, V):
        self.Um = V

    def simule(self, step):
        # Simplification L ≈ 0 (si L != 0, ajouter une dérivée ici)
        self.i = (self.Um - self.ke * self.omega) / self.R
        self.torque = self.kc * self.i

        # Équation mécanique enrichie
        domega = (self.torque - self.f * self.omega - self.couple_ext) / self.J
        self.omega += domega * step
        self.position += self.omega * step

        # Sauvegarde pour tracé
        if len(self.time) == 0:
            self.time.append(0)
        else:
            self.time.append(self.time[-1] + step)
        self.speed.append(self.omega)

    def getSpeed(self):
        return self.omega

    def getPosition(self):
        return self.position

    def getTorque(self):
        return self.torque

    def getIntensity(self):
        return self.i

    def plot(self):
        plt.figure()
        plt.plot(self.time, self.speed, label="Vitesse Ω(t)")
        plt.xlabel("Temps (s)")
        plt.ylabel("Vitesse (rad/s)")
        plt.title("Réponse du moteur CC enrichi")
        plt.grid(True)
        plt.legend()
        plt.show()

class ControlPI_vitesse:
    def __init__(self, moteur, Kp=1.0, Ki=0.0, vmax=24.0):
        self.moteur = moteur
        self.Kp = Kp
        self.Ki = Ki
        self.vmax = vmax

        self.target_speed = 0.0
        self.integral = 0.0
        self.last_error = 0.0

        self.voltages = []
        self.speeds = []

    def setTarget(self, vitesse):
        self.target_speed = vitesse

    def simule(self, step):
        error = self.target_speed - self.moteur.omega
        self.integral += error * step

        # Commande PI
        voltage = self.Kp * error + self.Ki * self.integral

        # Saturation de la tension
        voltage = max(min(voltage, self.vmax), -self.vmax)

        self.moteur.setVoltage(voltage)
        self.moteur.simule(step)

        self.voltages.append(voltage)
        self.speeds.append(self.moteur.omega)



    def plot(self):
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(self.moteur.time, self.speeds, label='Vitesse (Ω)')
        plt.plot(self.moteur.time, [self.target_speed]*len(self.speeds), '--', label='Consigne')
        plt.xlabel("Temps (s)")
        plt.ylabel("Vitesse (rad/s)")
        plt.title("Commande PID de vitesse")
        plt.grid(True)
        plt.legend()
        plt.show()

   