"""
environnement.py — Monde simulé (M du MVC)
-------------------------------------------
Contient :
  - Les classes d'obstacles physiques (Obstacle ABC, ObstacleCirculaire,
    ObstacleRectangulaire) pour la détection de collision en espace continu.
  - La Carte (grille 20×15, conversions cellule ↔ monde).
  - Camera : cône de vision discret, patrouille optionnelle sur un trajet.
  - Item et Sortie : entités cibles de la mission.
  - Lidar : capteur de distance par lancer de rayons (DDA simplifié).
  - Environnement : agrège toutes les entités, gère la physique (rollback
    de collision), la mise à jour des caméras et la carte_connue révélée
    progressivement par le lidar.
"""

from abc import ABC, abstractmethod
import math

TAILLE_CELLULE = 1.0
INCONNU = -1   # Valeur dans carte_connue pour une cellule non encore observée


# ══════════════════════════════════════════════════════════════════
# Obstacles physiques (espace continu)
# ══════════════════════════════════════════════════════════════════
class Obstacle(ABC):
    """Interface abstraite : tout obstacle doit savoir tester une collision."""
    @abstractmethod
    def collision(self, x, y, rayon): pass


class ObstacleCirculaire(Obstacle):
    def __init__(self, x, y, rayon):
        self.x, self.y, self.rayon = x, y, rayon

    def collision(self, rx, ry, rr):
        return math.hypot(self.x - rx, self.y - ry) <= (self.rayon + rr)


class ObstacleRectangulaire(Obstacle):
    def __init__(self, x, y, largeur, hauteur):
        self.x, self.y = x, y
        self.largeur, self.hauteur = largeur, hauteur

    def collision(self, rx, ry, rr):
        # Point du rectangle le plus proche du centre du robot
        cx = max(self.x, min(rx, self.x + self.largeur))
        cy = max(self.y, min(ry, self.y + self.hauteur))
        return math.hypot(rx - cx, ry - cy) <= rr


# ══════════════════════════════════════════════════════════════════
# Carte
# ══════════════════════════════════════════════════════════════════
class Carte:
    """Grille 20×15 représentant le bâtiment (0 = libre, 1 = mur)."""

    def __init__(self):
        self.grille = [
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],  # 0
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],  # 1
            [1,0,1,1,1,0,1,1,1,1,0,1,1,1,1,0,1,1,0,1],  # 2
            [1,0,1,0,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1],  # 3
            [1,0,1,0,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1],  # 4
            [1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1],  # 5
            [1,0,1,1,1,0,1,1,1,1,0,1,1,0,1,1,1,1,0,1],  # 6
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],  # 7
            [1,0,1,1,1,1,0,1,1,0,1,1,1,0,1,1,1,1,0,1],  # 8
            [1,0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,1],  # 9
            [1,0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,1],  # 10
            [1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,1],  # 11
            [1,0,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,0,1],  # 12
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],  # 13
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],  # 14
        ]
        self.cols = len(self.grille[0])
        self.rows = len(self.grille)

    def est_mur(self, col, row):
        if 0 <= col < self.cols and 0 <= row < self.rows:
            return self.grille[row][col] == 1
        return True  # Hors-limites considéré comme mur

    @staticmethod
    def cellule_vers_monde(col, row):
        """Retourne le centre monde (x, y) d'une cellule grille."""
        return (col + 0.5) * TAILLE_CELLULE, (row + 0.5) * TAILLE_CELLULE

    @staticmethod
    def monde_vers_cellule(x, y):
        """Retourne la cellule (col, row) correspondant à une position monde."""
        return int(x // TAILLE_CELLULE), int(y // TAILLE_CELLULE)


# ══════════════════════════════════════════════════════════════════
# Caméra (statique ou patrouilleuse)
# ══════════════════════════════════════════════════════════════════
class Camera:
    """
    Caméra avec cône de vision discret.

    Paramètres :
        col, row        : position initiale dans la grille.
        direction       : (dx, dy) entiers indiquant l'orientation de visée.
        portee          : profondeur de surveillance en nombre de cellules.
        trajet          : liste [(col, row), ...] formant un circuit de patrouille
                          en boucle. Vide ou None = caméra statique.
        vitesse_patrol  : vitesse de déplacement en cellules par seconde.

    La direction est recalculée automatiquement lors des déplacements de patrouille.
    """

    def __init__(self, col, row, direction, portee=3,
                 trajet=None, vitesse_patrol=0.8):
        self.col = col
        self.row = row
        self.direction = direction
        self.portee = portee
        self.cellules_vues = []

        self.trajet = trajet or []
        self.vitesse_patrol = vitesse_patrol
        self._idx_trajet = 0
        self._x_cont = float(col)
        self._y_cont = float(row)

    # ── Vision ────────────────────────────────────────────────────
    def calculer_vision(self, carte):
        """Recalcule la liste des cellules visibles depuis la position courante."""
        self.cellules_vues = []
        dx, dy = self.direction
        for dist in range(1, self.portee + 1):
            demi = dist // 2
            for ecart in range(-demi, demi + 1):
                if dx != 0:
                    cx = self.col + dx * dist
                    cy = self.row + ecart
                else:
                    cx = self.col + ecart
                    cy = self.row + dy * dist
                if 0 <= cx < carte.cols and 0 <= cy < carte.rows:
                    if not carte.est_mur(cx, cy):
                        self.cellules_vues.append((cx, cy))

    def voit_position(self, x, y):
        """Retourne True si la position monde (x, y) est dans le cône de vision."""
        c, r = Carte.monde_vers_cellule(x, y)
        return (c, r) in self.cellules_vues

    # ── Patrouille ────────────────────────────────────────────────
    def avancer(self, dt, carte):
        """Déplace la caméra le long de son trajet et recalcule la vision."""
        if not self.trajet:
            return

        cible_col, cible_row = self.trajet[self._idx_trajet]
        dc = cible_col - self._x_cont
        dr = cible_row - self._y_cont
        dist = math.hypot(dc, dr)

        if dist < 0.05:
            # Waypoint atteint : ancrage sur la cellule entière
            self.col = cible_col
            self.row = cible_row
            self._x_cont = float(cible_col)
            self._y_cont = float(cible_row)
            self._idx_trajet = (self._idx_trajet + 1) % len(self.trajet)
            # Mise à jour de la direction vers le prochain waypoint
            nc, nr = self.trajet[self._idx_trajet]
            raw_dx = nc - self.col
            raw_dy = nr - self.row
            if raw_dx != 0 or raw_dy != 0:
                if abs(raw_dx) >= abs(raw_dy):
                    self.direction = (1 if raw_dx > 0 else -1, 0)
                else:
                    self.direction = (0, 1 if raw_dy > 0 else -1)
        else:
            pas = self.vitesse_patrol * dt
            self._x_cont += (dc / dist) * min(pas, dist)
            self._y_cont += (dr / dist) * min(pas, dist)
            new_col = int(round(self._x_cont))
            new_row = int(round(self._y_cont))
            if (new_col, new_row) != (self.col, self.row):
                self.col = new_col
                self.row = new_row

        self.calculer_vision(carte)


# ══════════════════════════════════════════════════════════════════
# Entités cibles
# ══════════════════════════════════════════════════════════════════
class Item:
    """Objet à collecter par le robot."""

    def __init__(self, col, row):
        self.col, self.row = col, row
        self.collecte = False

    @property
    def position_monde(self):
        return Carte.cellule_vers_monde(self.col, self.row)


class Sortie:
    """Zone de sortie à atteindre après la collecte de l'item."""

    def __init__(self, col, row):
        self.col, self.row = col, row

    @property
    def position_monde(self):
        return Carte.cellule_vers_monde(self.col, self.row)


# ══════════════════════════════════════════════════════════════════
# Lidar
# ══════════════════════════════════════════════════════════════════
class Lidar:
    """
    Capteur de distance par lancer de rayons continus (DDA simplifié).

    Retourne une liste de tuples (angle_absolu, distance, x_impact, y_impact)
    pour chaque rayon. Seuls les murs de la carte sont testés.
    """

    def __init__(self, portee=6.0, nb_rayons=36):
        self.portee = portee
        self.nb_rayons = nb_rayons

    def scanner(self, robot, carte):
        resultats = []
        for i in range(self.nb_rayons):
            angle = robot.orientation + (2 * math.pi * i / self.nb_rayons)
            dist, xi, yi = self._lancer_rayon(robot.x, robot.y, angle, carte)
            resultats.append((angle, dist, xi, yi))
        return resultats

    def _lancer_rayon(self, ox, oy, angle, carte):
        pas = 0.15
        dx = math.cos(angle) * pas
        dy = math.sin(angle) * pas
        x, y = ox, oy
        steps = int(self.portee / pas)
        for _ in range(steps):
            x += dx
            y += dy
            c, r = Carte.monde_vers_cellule(x, y)
            if carte.est_mur(c, r):
                return math.hypot(x - ox, y - oy), x, y
        return (self.portee,
                ox + math.cos(angle) * self.portee,
                oy + math.sin(angle) * self.portee)


# ══════════════════════════════════════════════════════════════════
# Environnement
# ══════════════════════════════════════════════════════════════════
class Environnement:
    """
    Conteneur du monde simulé.

    Responsabilités :
      - Agrège le robot, les obstacles, les caméras, l'item et la sortie.
      - Génère automatiquement des ObstacleRectangulaires pour chaque mur.
      - Applique un rollback de pose si une collision est détectée après
        le déplacement du robot.
      - Maintient une carte_connue révélée progressivement par le lidar.
      - Fournit grille_de_danger() pour la planification A*, traitant les
        zones de caméras comme des murs virtuels.
      - Fournit frontiere_exploration() pour guider l'exploration autonome.
    """

    def __init__(self, carte=None):
        self.carte = carte if carte is not None else Carte()
        self.largeur = self.carte.cols * TAILLE_CELLULE
        self.hauteur = self.carte.rows * TAILLE_CELLULE
        self.robot = None
        self.obstacles = []
        self.cameras = []
        self.item = None
        self.sortie = None

        # Grille de connaissance initialisée à INCONNU, révélée par le lidar
        self.carte_connue = [[INCONNU] * self.carte.cols
                             for _ in range(self.carte.rows)]
        self._generer_murs_depuis_grille()

    def _generer_murs_depuis_grille(self):
        """Crée un ObstacleRectangulaire 1×1 pour chaque cellule mur de la grille."""
        for r in range(self.carte.rows):
            for c in range(self.carte.cols):
                if self.carte.grille[r][c] == 1:
                    self.obstacles.append(ObstacleRectangulaire(
                        x=c * TAILLE_CELLULE, y=r * TAILLE_CELLULE,
                        largeur=TAILLE_CELLULE, hauteur=TAILLE_CELLULE,
                    ))

    def ajouter_robot(self, robot):    self.robot = robot
    def ajouter_obstacle(self, obs):   self.obstacles.append(obs)
    def definir_item(self, item):      self.item = item
    def definir_sortie(self, sortie):  self.sortie = sortie

    def ajouter_camera(self, camera):
        camera.calculer_vision(self.carte)
        self.cameras.append(camera)

    # ── Tests ─────────────────────────────────────────────────────
    def test_collision(self):
        if not self.robot:
            return False
        return any(obs.collision(self.robot.x, self.robot.y, self.robot.rayon)
                   for obs in self.obstacles)

    def test_detection(self):
        """Retourne True si le robot est dans le cône d'au moins une caméra."""
        if not self.robot:
            return False
        return any(cam.voit_position(self.robot.x, self.robot.y)
                   for cam in self.cameras)

    # ── Mise à jour de la carte connue via le lidar ───────────────
    def _mettre_a_jour_carte_connue(self):
        """Révèle les cellules traversées par chaque rayon du lidar."""
        robot = self.robot
        if not robot or not getattr(robot, 'lidar', None):
            return
        resultats = robot.lidar.scanner(robot, self.carte)
        for angle, dist, xi, yi in resultats:
            pas = 0.15
            nb = int(dist / pas) + 1
            ox, oy = robot.x, robot.y
            dx = math.cos(angle) * pas
            dy = math.sin(angle) * pas
            x, y = ox, oy
            for _ in range(nb):
                c, r = Carte.monde_vers_cellule(x, y)
                if 0 <= c < self.carte.cols and 0 <= r < self.carte.rows:
                    self.carte_connue[r][c] = self.carte.grille[r][c]
                x += dx
                y += dy

    # ── Boucle principale ─────────────────────────────────────────
    def mettre_a_jour(self, dt):
        """Avance la simulation d'un pas dt : cinématique, collision, caméras, lidar."""
        if not self.robot:
            return
        ax, ay, ao = self.robot.x, self.robot.y, self.robot.orientation
        self.robot.mettre_a_jour(dt)
        if self.test_collision():
            self.robot.x, self.robot.y, self.robot.orientation = ax, ay, ao

        for cam in self.cameras:
            cam.avancer(dt, self.carte)

        self._mettre_a_jour_carte_connue()

    # ── Grille de planification ───────────────────────────────────
    def grille_de_danger(self, carte_source=None):
        """
        Retourne une grille 2D où les positions et cônes de caméras sont bloqués
        (valeur 1), utilisée par l'algorithme A* pour éviter les zones surveillées.

        carte_source : grille de base optionnelle (défaut = carte complète).
                       Permet de planifier sur la carte_connue en fog of war.
        """
        base = carte_source if carte_source is not None else self.carte.grille
        danger = [ligne[:] for ligne in base]
        for cam in self.cameras:
            if 0 <= cam.row < self.carte.rows and 0 <= cam.col < self.carte.cols:
                danger[cam.row][cam.col] = 1
            for (cx, cy) in cam.cellules_vues:
                if 0 <= cy < len(danger) and 0 <= cx < len(danger[0]):
                    danger[cy][cx] = 1
        return danger

    # ── Frontières d'exploration ──────────────────────────────────
    def frontiere_exploration(self):
        """
        Retourne les cellules libres connues adjacentes à au moins une cellule INCONNU.
        Utilisées par le contrôleur autonome pour naviguer vers les zones inexplorées.
        """
        frontieres = []
        for r in range(self.carte.rows):
            for c in range(self.carte.cols):
                if self.carte_connue[r][c] == 0:
                    for dc, dr in [(1,0),(-1,0),(0,1),(0,-1)]:
                        nc, nr = c + dc, r + dr
                        if (0 <= nc < self.carte.cols and
                                0 <= nr < self.carte.rows and
                                self.carte_connue[nr][nc] == INCONNU):
                            frontieres.append((c, r))
                            break
        return frontieres