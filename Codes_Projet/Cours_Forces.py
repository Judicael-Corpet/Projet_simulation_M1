"""
Liaison cinématique
créer une liaison cinématique = interdire des mouvements = ajouter des contraintes.
2 solutions pour empêcher une vitesse dans une direction :
- annuler la composante dans la direction donnée
- ou créer une force qui s'oppose au mouvement

Comment créer une force de réaction ??
(Hertz) s'il y a contact, il y a déformation : on peut modéliser ce contact par un ressort-amortisseur
On remplace donc les contraintes par des ressort-amortisseur dans la direction de l'axe de la contrainte
le spring-damper est un peu la "pièce de lego de base"

Exercice :
Simuler un pendule double et pouvoir tracer la position en x en fonction du temps
On va également ajouter une liaison prismatique Prism(P0, P1, axe)

"""

