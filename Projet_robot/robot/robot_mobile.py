"""
robot_mobile.py — Modèle du robot (M du MVC)
---------------------------------------------
Encapsule la pose du robot (x, y, theta) avec des propriétés Python
et délègue la cinématique à un objet Moteur (pattern Strategy).
L'attribut lidar, optionnel, est utilisé par Environnement pour
révéler progressivement la carte en mode fog of war.
"""

import math


class RobotMobile:
    """
    Modèle cinématique d'un robot mobile 2D.

    Attributs publics (via @property) :
        x, y        : position en mètres dans le repère monde.
        orientation : cap en radians, normalisé dans [-π, π].
        a_objet     : True si l'item de mission a été collecté.
        lidar       : instance de Lidar ou None.
    """

    _nb_robots = 0

    def __init__(self, x=0.0, y=0.0, orientation=0.0,
                 moteur=None, rayon=0.3, lidar=None):
        self.__x = x
        self.__y = y
        self.__orientation = orientation
        self.moteur = moteur
        self.rayon = rayon
        self.a_objet = False
        self.lidar = lidar
        RobotMobile._nb_robots += 1

    @property
    def x(self): return self.__x
    @x.setter
    def x(self, value): self.__x = value

    @property
    def y(self): return self.__y
    @y.setter
    def y(self, value): self.__y = value

    @property
    def orientation(self): return self.__orientation
    @orientation.setter
    def orientation(self, value):
        self.__orientation = (value + math.pi) % (2 * math.pi) - math.pi

    def commander(self, **kwargs):
        """Transmet une commande au moteur."""
        if self.moteur:
            self.moteur.commander(**kwargs)

    def mettre_a_jour(self, dt):
        """Délègue l'intégration cinématique au moteur pour un pas dt."""
        if self.moteur:
            self.moteur.mettre_a_jour(self, dt)

    def __str__(self):
        return (f"Robot(x={self.x:.2f}, y={self.y:.2f}, "
                f"theta={self.orientation:.2f}, objet={self.a_objet})")

    @classmethod
    def nombre_robots(cls):
        """Retourne le nombre total de robots instanciés."""
        return cls._nb_robots