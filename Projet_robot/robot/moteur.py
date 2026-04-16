"""
moteur.py — Stratégies cinématiques (M du MVC)
-----------------------------------------------
Implémente le pattern Strategy pour la cinématique du robot.
Le robot délègue sa mise à jour de pose à un objet Moteur,
ce qui permet de changer de modèle cinématique sans modifier RobotMobile.

Classes :
  - Moteur : classe abstraite définissant l'interface commune.
  - MoteurDifferentiel : robot à deux roues (commandes v, omega).
  - MoteurOmnidirectionnel : robot holonome (commandes vx, vy, omega).
"""

from abc import ABC, abstractmethod
import math


class Moteur(ABC):
    """Interface abstraite : tout moteur doit accepter une commande et
    mettre à jour la pose du robot pour un pas de temps dt."""

    @abstractmethod
    def commander(self, **kwargs): pass

    @abstractmethod
    def mettre_a_jour(self, robot, dt): pass


class MoteurDifferentiel(Moteur):
    """
    Cinématique du robot à deux roues différentielles.

    Commandes :
        v     (float) : vitesse linéaire en m/s.
        omega (float) : vitesse angulaire en rad/s.
    """

    def __init__(self):
        self.v = 0.0
        self.omega = 0.0

    def commander(self, v=0.0, omega=0.0, **kwargs):
        self.v = v
        self.omega = omega

    def mettre_a_jour(self, robot, dt):
        robot.orientation = robot.orientation + self.omega * dt
        robot.x += self.v * math.cos(robot.orientation) * dt
        robot.y += self.v * math.sin(robot.orientation) * dt


class MoteurOmnidirectionnel(Moteur):
    """
    Cinématique du robot holonome (omnidirectionnel).

    Commandes exprimées dans le repère robot :
        vx    (float) : vitesse longitudinale en m/s.
        vy    (float) : vitesse latérale en m/s (strafe).
        omega (float) : vitesse angulaire en rad/s.
    """

    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

    def commander(self, vx=0.0, vy=0.0, omega=0.0, **kwargs):
        self.vx, self.vy, self.omega = vx, vy, omega

    def mettre_a_jour(self, robot, dt):
        theta = robot.orientation
        robot.orientation = theta + self.omega * dt
        # Projection des vitesses repère robot → repère monde
        robot.x += (self.vx * math.cos(theta) - self.vy * math.sin(theta)) * dt
        robot.y += (self.vx * math.sin(theta) + self.vy * math.cos(theta)) * dt