import numpy as np
import matplotlib.pyplot as plt

# Paramètres physiques
k = 10         # raideur du ressort
l0 = 5         # longueur à vide
m = 1.0        # masse de la particule

# Vecteur de vitesses angulaires (omega)
omega_vals = np.linspace(0, 2.5, 500)

# Longueur du ressort en fonction de omega
r_vals = []
for omega in omega_vals:
    denominator = k - m * omega**2
    if denominator <= 0:
        r_vals.append(np.nan)  # valeur non définie si dénominateur nul ou négatif
    else:
        r = (k * l0) / denominator
        r_vals.append(r)

# Tracé
plt.figure(figsize=(8, 5))
plt.plot(omega_vals, r_vals, label="r = k·l0 / (k - m·ω²)")
plt.axvline(np.sqrt(k / m), color='r', linestyle='--', label='Limite ω (instabilité)')
plt.xlabel("Vitesse angulaire ω (rad/s)")
plt.ylabel("Longueur du ressort r (m)")
plt.title("Élongation du ressort en fonction de la vitesse angulaire")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
