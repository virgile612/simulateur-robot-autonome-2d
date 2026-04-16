"""
controleur.py — Contrôleurs (C du MVC)
---------------------------------------
Contient :
  - Controleur : classe abstraite (ABC) définissant l'interface commune.
  - EtatMission : énumération des états de la machine d'états de mission.
  - StatsMission : chronomètre et compteur de détections évitées.
  - ControleurClavier / ControleurClavierOmni : pilotage manuel au clavier.
  - ControleurAutonome : navigation autonome par planification A* avec suivi
    cinématique de waypoints, exploration par frontières (fog of war) et
    replanification automatique si les zones de danger évoluent.
"""

from abc import ABC, abstractmethod
from enum import Enum
import math
import time
import pygame

from robot.environnement import Carte, INCONNU
from robot.planificateur import astar


# ══════════════════════════════════════════════════════════════════
# Interface commune
# ══════════════════════════════════════════════════════════════════
class Controleur(ABC):
    @abstractmethod
    def lire_commande(self): pass


# ══════════════════════════════════════════════════════════════════
# Machine d'états de mission
# ══════════════════════════════════════════════════════════════════
class EtatMission(Enum):
    MENU      = "MENU"
    SEARCHING = "SEARCHING"
    COLLECTED = "COLLECTED"
    VICTORY   = "VICTORY"
    DEFEAT    = "DEFEAT"
    PAUSED    = "PAUSED"


# ══════════════════════════════════════════════════════════════════
# Pilotage manuel
# ══════════════════════════════════════════════════════════════════
class ControleurClavier(Controleur):
    """Moteur différentiel : ↑/↓ avance/recule, ←/→ tourne."""

    def lire_commande(self):
        keys = pygame.key.get_pressed()
        v, omega = 0.0, 0.0
        if keys[pygame.K_UP]:    v = 1.5
        if keys[pygame.K_DOWN]:  v = -1.5
        if keys[pygame.K_LEFT]:  omega = 1.5
        if keys[pygame.K_RIGHT]: omega = -1.5
        return {"v": v, "omega": omega}


class ControleurClavierOmni(Controleur):
    """Moteur omnidirectionnel : ↑/↓ avant/arrière, ←/→ strafe, [A]/[E] rotation."""

    def lire_commande(self):
        keys = pygame.key.get_pressed()
        vx, vy, omega = 0.0, 0.0, 0.0
        if keys[pygame.K_UP]:    vx =  1.5
        if keys[pygame.K_DOWN]:  vx = -1.5
        if keys[pygame.K_LEFT]:  vy = -1.5
        if keys[pygame.K_RIGHT]: vy =  1.5
        if keys[pygame.K_a]:     omega =  1.5
        if keys[pygame.K_e]:     omega = -1.5
        return {"vx": vx, "vy": vy, "omega": omega}


# ══════════════════════════════════════════════════════════════════
# Statistiques de mission
# ══════════════════════════════════════════════════════════════════
class StatsMission:
    """Chronomètre de mission et compteur de détections caméra évitées."""

    def __init__(self):
        self._debut = time.monotonic()
        self._pause_debut = None
        self._temps_pause = 0.0
        self.detections_evitees = 0
        self._en_zone_camera = False

    def pause(self):
        if self._pause_debut is None:
            self._pause_debut = time.monotonic()

    def reprendre(self):
        if self._pause_debut is not None:
            self._temps_pause += time.monotonic() - self._pause_debut
            self._pause_debut = None

    @property
    def elapsed(self):
        if self._pause_debut is not None:
            return self._pause_debut - self._debut - self._temps_pause
        return time.monotonic() - self._debut - self._temps_pause

    def formater_chrono(self):
        t = int(self.elapsed)
        return f"{t // 60:02d}:{t % 60:02d}"

    def signaler_proximite_camera(self, en_zone):
        """
        Incrémente le compteur à chaque fois que le robot quitte une zone
        de détection sans avoir été capturé.
        """
        if self._en_zone_camera and not en_zone:
            self.detections_evitees += 1
        self._en_zone_camera = en_zone


# ══════════════════════════════════════════════════════════════════
# Pilotage autonome : A* + suivi cinématique + replanification
# ══════════════════════════════════════════════════════════════════
class ControleurAutonome(Controleur):
    """
    Contrôleur autonome basé sur la planification A*.

    Comportement :
      - En mode fog of war (lidar actif), planifie sur la carte_connue et
        explore les frontières si l'objectif courant est encore inconnu.
      - Suit les waypoints avec une loi de commande cinématique : alignement
        du cap puis avance proportionnelle à la distance.
      - Replanifie automatiquement si la grille de danger évolue (caméra
        déplacée), détecté par hachage de la grille.
      - Machine d'états : SEARCHING → COLLECTED → VICTORY (ou DEFEAT).
    """

    VITESSE_MAX = 2.0
    OMEGA_MAX   = 3.0
    GAIN_OMEGA  = 4.0
    SEUIL_WAYPOINT   = 0.15   # Distance pour considérer un waypoint atteint
    SEUIL_ALIGNEMENT = 0.4    # Erreur angulaire max pour avancer
    SEUIL_OBJECTIF   = 0.3    # Distance pour collecter/sortir

    def __init__(self, environnement):
        self.env = environnement
        self.etat = EtatMission.SEARCHING
        self.stats = StatsMission()
        self.waypoints = []
        self._hash_danger = None
        self._mode_exploration = False
        self._planifier_vers_item()

    # ── Fog of war actif ? ────────────────────────────────────────
    @property
    def _fog_actif(self):
        return getattr(self.env.robot, 'lidar', None) is not None

    # ── Grille de planification selon le mode ─────────────────────
    def _grille_planif(self):
        """Retourne la grille A* adaptée : carte complète ou carte connue."""
        if self._fog_actif:
            # Les cellules INCONNU sont supposées libres pour laisser passer A*
            connue = []
            for ligne in self.env.carte_connue:
                connue.append([0 if v == INCONNU else v for v in ligne])
            return self.env.grille_de_danger(carte_source=connue)
        return self.env.grille_de_danger()

    def _hash_grille(self, grille):
        return hash(tuple(tuple(r) for r in grille))

    # ── Cellule courante du robot ─────────────────────────────────
    def _cellule_robot(self):
        return Carte.monde_vers_cellule(self.env.robot.x, self.env.robot.y)

    # ── Planification générique ───────────────────────────────────
    def _planifier(self, cible_col, cible_row, force=False):
        """
        Lance A* du robot vers la cellule cible.
        Si force=False, ignore la demande si la grille est inchangée et
        des waypoints sont déjà disponibles.
        """
        depart  = self._cellule_robot()
        arrivee = (cible_col, cible_row)
        danger  = self._grille_planif()

        h = self._hash_grille(danger)
        if not force and h == self._hash_danger and self.waypoints:
            return
        self._hash_danger = h

        # Débloquer les cellules de départ et d'arrivée pour A*
        danger[depart[1]][depart[0]] = 0
        r_arr, c_arr = arrivee[1], arrivee[0]
        if 0 <= r_arr < len(danger) and 0 <= c_arr < len(danger[0]):
            danger[r_arr][c_arr] = 0

        chemin = astar(danger, depart, arrivee)
        if not chemin:
            # Repli sur la carte brute si la grille de danger bloque tout
            chemin = astar(self.env.carte.grille, depart, arrivee)

        self.waypoints = [Carte.cellule_vers_monde(c, r)
                          for (c, r) in chemin[1:]]

    def _planifier_vers_item(self):
        if self.env.item:
            self._planifier(self.env.item.col, self.env.item.row, force=True)

    def _planifier_vers_sortie(self):
        if self.env.sortie:
            self._planifier(self.env.sortie.col, self.env.sortie.row, force=True)

    def _planifier_vers_frontiere(self):
        """Navigue vers la frontière d'exploration la plus proche (fog of war)."""
        frontieres = self.env.frontiere_exploration()
        if not frontieres:
            return
        cr = self._cellule_robot()
        cible = min(frontieres,
                    key=lambda f: abs(f[0]-cr[0]) + abs(f[1]-cr[1]))
        self._planifier(cible[0], cible[1], force=True)
        self._mode_exploration = True

    # ── Replanification sur changement de grille ─────────────────
    def _verifier_replan(self):
        """Invalide les waypoints courants si la grille de danger a changé."""
        if not self.waypoints:
            return
        danger = self._grille_planif()
        h = self._hash_grille(danger)
        if h != self._hash_danger:
            self.waypoints = []
            self._hash_danger = h

    # ── Machine d'états de mission ────────────────────────────────
    def _maj_etat_mission(self):
        robot = self.env.robot
        self.stats.signaler_proximite_camera(self.env.test_detection())

        if self.env.test_detection():
            self.etat = EtatMission.DEFEAT
            self.waypoints = []
            return

        if self.etat == EtatMission.SEARCHING and self.env.item:
            ix, iy = self.env.item.position_monde
            if math.hypot(robot.x - ix, robot.y - iy) < self.SEUIL_OBJECTIF:
                self.env.item.collecte = True
                robot.a_objet = True
                self.etat = EtatMission.COLLECTED
                self._mode_exploration = False
                self._planifier_vers_sortie()

        elif self.etat == EtatMission.COLLECTED and self.env.sortie:
            sx, sy = self.env.sortie.position_monde
            if math.hypot(robot.x - sx, robot.y - sy) < self.SEUIL_OBJECTIF:
                self.etat = EtatMission.VICTORY
                self.waypoints = []

    # ── Loi de commande ───────────────────────────────────────────
    def lire_commande(self):
        self._maj_etat_mission()

        if self.etat in (EtatMission.VICTORY, EtatMission.DEFEAT):
            return {"v": 0.0, "omega": 0.0}

        self._verifier_replan()

        # Sélection de la prochaine cible si aucun waypoint disponible
        if not self.waypoints:
            if self.etat == EtatMission.SEARCHING:
                if self._fog_actif:
                    item = self.env.item
                    if item:
                        ic, ir = item.col, item.row
                        if self.env.carte_connue[ir][ic] != INCONNU:
                            self._mode_exploration = False
                            self._planifier_vers_item()
                        else:
                            self._planifier_vers_frontiere()
                    else:
                        self._planifier_vers_frontiere()
                else:
                    self._planifier_vers_item()
            elif self.etat == EtatMission.COLLECTED:
                self._planifier_vers_sortie()
            if not self.waypoints:
                return {"v": 0.0, "omega": 0.0}

        robot = self.env.robot
        wx, wy = self.waypoints[0]
        dx, dy = wx - robot.x, wy - robot.y
        distance = math.hypot(dx, dy)

        if distance < self.SEUIL_WAYPOINT:
            self.waypoints.pop(0)
            if not self.waypoints:
                return {"v": 0.0, "omega": 0.0}
            wx, wy = self.waypoints[0]
            dx, dy = wx - robot.x, wy - robot.y
            distance = math.hypot(dx, dy)

        cap_cible = math.atan2(dy, dx)
        erreur_angle = (cap_cible - robot.orientation + math.pi) % (2 * math.pi) - math.pi

        omega = max(-self.OMEGA_MAX,
                    min(self.OMEGA_MAX, self.GAIN_OMEGA * erreur_angle))

        if abs(erreur_angle) > self.SEUIL_ALIGNEMENT:
            v = 0.0   # Alignement du cap avant d'avancer
        else:
            v = min(self.VITESSE_MAX,
                    self.VITESSE_MAX * min(1.0, distance / 0.5))

        return {"v": v, "omega": omega}