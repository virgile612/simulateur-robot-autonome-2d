"""
vue.py — Vue (V du MVC)
------------------------
Rendu Pygame top-down de la simulation d'infiltration.

Responsabilités :
  - Dessin de la carte (murs biseautés, pièces colorées par flood-fill, portes).
  - Affichage des caméras (corps, cône de vision, trajet de patrouille).
  - Rendu du robot, de l'item et de la sortie avec effets de halo animés.
  - Overlay fog of war (cellules inconnues masquées) quand le lidar est actif.
  - Rayons lidar en mode debug (activé/désactivé par [F1]).
  - HUD : état de mission, chronomètre, détections évitées, touches.
  - Menus de sélection du mode de contrôle et du type de moteur.
"""

import pygame
import math

from robot.environnement import Carte, TAILLE_CELLULE, INCONNU
from robot.controleur import EtatMission, ControleurClavierOmni


# ── Palette ──────────────────────────────────────────────────────
BG           = (18, 18, 28)
FLOOR_BASE   = (38, 40, 55)
WALL_FILL    = (75, 78, 95)
WALL_TOP     = (90, 93, 110)
WALL_SHADOW  = (30, 30, 45)
GRID_LINE    = (32, 34, 48)
DOOR_FILL    = (48, 52, 68)
DOOR_MARK    = (70, 72, 88)
ROBOT_CORE   = (70, 210, 255)
ROBOT_EDGE   = (40, 140, 200)
ROBOT_GLOW   = (70, 210, 255, 35)
ITEM_CORE    = (255, 215, 40)
ITEM_EDGE    = (220, 180, 20)
ITEM_GLOW_C  = (255, 220, 50, 40)
EXIT_FILL    = (60, 230, 110)
CAM_BODY     = (240, 60, 60)
CAM_LENS     = (180, 40, 40)
CAM_CONE     = (255, 50, 50, 35)
CAM_CONE_EDG = (255, 80, 80, 60)
TEXT_COL     = (210, 215, 230)
HUD_BG       = (22, 22, 36)
PATH_COL     = (70, 200, 255)
FOG_COL      = (10, 10, 18, 210)
LIDAR_COL    = (80, 255, 160, 60)
PATROL_TRAIL = (255, 180, 60, 40)

ROOM_TINTS = [
    (42, 44, 62), (44, 42, 58), (40, 46, 58), (46, 44, 56),
    (40, 44, 64), (44, 40, 60), (42, 48, 58), (48, 44, 58),
]

BAR_HEIGHT = 48


class VuePygame:
    """Vue Pygame top-down pour la simulation d'infiltration."""

    def __init__(self, environnement, scale=32):
        pygame.init()
        self.env = environnement
        self.scale = scale
        self.cell = int(scale * TAILLE_CELLULE)

        self.largeur = self.env.carte.cols * self.cell
        self.hauteur = self.env.carte.rows * self.cell

        self.screen = pygame.display.set_mode(
            (self.largeur, self.hauteur + BAR_HEIGHT))
        pygame.display.set_caption("Robot Infiltration — Mission de nuit")
        self.clock = pygame.time.Clock()

        self.font       = pygame.font.SysFont("consolas", 17, bold=True)
        self.big_font   = pygame.font.SysFont("consolas", 42, bold=True)
        self.small_font = pygame.font.SysFont("consolas", 11)
        self.menu_font  = pygame.font.SysFont("consolas", 26, bold=True)

        self._rooms = self._detecter_pieces()
        self._show_lidar = False   # Basculé par la touche [F1]

    # ── Conversion monde → pixels ─────────────────────────────────
    def monde_vers_pixel(self, x, y):
        return int(x * self.scale), int(y * self.scale)

    def toggle_lidar(self):
        """Active ou désactive l'affichage des rayons lidar."""
        self._show_lidar = not self._show_lidar

    # ── Détection des pièces par flood-fill ───────────────────────
    def _detecter_pieces(self):
        """
        Attribue une teinte de fond à chaque pièce fermée de la carte par
        flood-fill sur les cellules libres non-portes.
        """
        carte = self.env.carte
        est_porte = [[self._est_porte(c, r)
                      for c in range(carte.cols)]
                     for r in range(carte.rows)]
        vu = [[False] * carte.cols for _ in range(carte.rows)]
        pieces = {}
        idx = 0
        for r in range(carte.rows):
            for c in range(carte.cols):
                if carte.grille[r][c] == 0 and not vu[r][c] and not est_porte[r][c]:
                    region, file = [], [(c, r)]
                    vu[r][c] = True
                    while file:
                        cx, cy = file.pop(0)
                        region.append((cx, cy))
                        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                            nx, ny = cx+dx, cy+dy
                            if (0 <= nx < carte.cols and 0 <= ny < carte.rows
                                    and not vu[ny][nx]
                                    and carte.grille[ny][nx] == 0
                                    and not est_porte[ny][nx]):
                                vu[ny][nx] = True
                                file.append((nx, ny))
                    teinte = ROOM_TINTS[idx % len(ROOM_TINTS)]
                    for cell in region:
                        pieces[cell] = teinte
                    idx += 1
        return pieces

    def _est_porte(self, c, r):
        """Retourne True si la cellule est encadrée par des murs sur deux côtés opposés."""
        carte = self.env.carte
        if carte.est_mur(c, r):
            return False
        h = carte.est_mur(c-1, r) and carte.est_mur(c+1, r)
        v = carte.est_mur(c, r-1) and carte.est_mur(c, r+1)
        return h or v

    def _voisins_murs(self, c, r):
        """Retourne l'ensemble des directions (N/S/E/W) ayant un mur voisin."""
        carte = self.env.carte
        d = set()
        if r > 0 and carte.est_mur(c, r-1):              d.add("N")
        if r < carte.rows-1 and carte.est_mur(c, r+1):   d.add("S")
        if c > 0 and carte.est_mur(c-1, r):              d.add("W")
        if c < carte.cols-1 and carte.est_mur(c+1, r):   d.add("E")
        return d

    # ══════════════════════════════════════════════════════════════
    # Menus
    # ══════════════════════════════════════════════════════════════
    def dessiner_menu(self, mode_selectionne="auto"):
        """Affiche l'écran de sélection du mode de contrôle."""
        self.screen.fill(BG)

        titre = self.big_font.render("INFILTRATION", True, ROBOT_CORE)
        self.screen.blit(titre,
            ((self.largeur - titre.get_width()) // 2, self.hauteur // 4))

        sous = self.menu_font.render("Choisir le mode de contrôle :", True, TEXT_COL)
        self.screen.blit(sous,
            ((self.largeur - sous.get_width()) // 2, self.hauteur // 4 + 70))

        modes = [("auto",   "A  —  Autonome (A* + suivi)"),
                 ("manuel", "M  —  Manuel   (flèches)")]
        y0 = self.hauteur // 4 + 130
        for key, label in modes:
            col = ITEM_CORE if key == mode_selectionne else TEXT_COL
            surf = self.menu_font.render(label, True, col)
            if key == mode_selectionne:
                pad = 10
                bg = pygame.Surface((surf.get_width() + pad*2,
                                     surf.get_height() + pad*2), pygame.SRCALPHA)
                bg.fill((40, 40, 60, 180))
                pygame.draw.rect(bg, ITEM_CORE,
                                 (0, 0, bg.get_width(), bg.get_height()), 2)
                self.screen.blit(bg,
                    ((self.largeur - bg.get_width()) // 2, y0 - pad))
            self.screen.blit(surf,
                ((self.largeur - surf.get_width()) // 2, y0))
            y0 += 55

        hint = self.small_font.render(
            "[↑↓] naviguer   [Entrée] lancer   [Q] quitter", True, (100, 100, 120))
        self.screen.blit(hint,
            ((self.largeur - hint.get_width()) // 2,
             self.hauteur - 40 + BAR_HEIGHT // 2))

        pygame.display.flip()

    def dessiner_menu_moteur(self, moteur_selectionne="differentiel"):
        """Affiche le sous-menu de sélection du type de moteur (mode manuel)."""
        self.screen.fill(BG)

        titre = self.big_font.render("MODE MANUEL", True, ITEM_CORE)
        self.screen.blit(titre,
            ((self.largeur - titre.get_width()) // 2, self.hauteur // 4))

        sous = self.menu_font.render("Choisir le type de moteur :", True, TEXT_COL)
        self.screen.blit(sous,
            ((self.largeur - sous.get_width()) // 2, self.hauteur // 4 + 70))

        moteurs = [
            ("differentiel", "D  —  Différentiel  (↑↓ avance, ←→ tourne)"),
            ("omni",         "O  —  Omni          (↑↓ avance, ←→ strafe, A/E rot.)"),
        ]
        y0 = self.hauteur // 4 + 130
        for key, label in moteurs:
            col = ITEM_CORE if key == moteur_selectionne else TEXT_COL
            surf = self.menu_font.render(label, True, col)
            if key == moteur_selectionne:
                pad = 10
                bg = pygame.Surface((surf.get_width() + pad*2,
                                     surf.get_height() + pad*2), pygame.SRCALPHA)
                bg.fill((40, 40, 60, 180))
                pygame.draw.rect(bg, ITEM_CORE,
                                 (0, 0, bg.get_width(), bg.get_height()), 2)
                self.screen.blit(bg,
                    ((self.largeur - bg.get_width()) // 2, y0 - pad))
            self.screen.blit(surf,
                ((self.largeur - surf.get_width()) // 2, y0))
            y0 += 55

        hint = self.small_font.render(
            "[↑↓] naviguer   [Entrée] confirmer   [Q] quitter", True, (100, 100, 120))
        self.screen.blit(hint,
            ((self.largeur - hint.get_width()) // 2,
             self.hauteur - 40 + BAR_HEIGHT // 2))

        pygame.display.flip()

    # ══════════════════════════════════════════════════════════════
    # Dessin de la carte
    # ══════════════════════════════════════════════════════════════
    def _dessiner_carte(self):
        carte = self.env.carte
        cell = self.cell
        for r in range(carte.rows):
            for c in range(carte.cols):
                x, y = c * cell, r * cell
                rect = pygame.Rect(x, y, cell, cell)
                if carte.grille[r][c] == 1:
                    pygame.draw.rect(self.screen, WALL_FILL, rect)
                    nb = self._voisins_murs(c, r)
                    if "N" not in nb or r == 0:
                        pygame.draw.line(self.screen, WALL_TOP,
                                         (x, y), (x+cell-1, y), 2)
                    if "W" not in nb or c == 0:
                        pygame.draw.line(self.screen, WALL_TOP,
                                         (x, y), (x, y+cell-1), 2)
                    if "S" not in nb or r == carte.rows-1:
                        pygame.draw.line(self.screen, WALL_SHADOW,
                                         (x, y+cell-1), (x+cell-1, y+cell-1), 2)
                    if "E" not in nb or c == carte.cols-1:
                        pygame.draw.line(self.screen, WALL_SHADOW,
                                         (x+cell-1, y), (x+cell-1, y+cell-1), 2)
                else:
                    teinte = self._rooms.get((c, r), FLOOR_BASE)
                    pygame.draw.rect(self.screen, teinte, rect)
                    if self._est_porte(c, r):
                        pygame.draw.rect(self.screen, DOOR_FILL, rect)
                        if carte.est_mur(c-1, r) and carte.est_mur(c+1, r):
                            pygame.draw.line(self.screen, DOOR_MARK,
                                             (x+2, y), (x+2, y+cell), 2)
                            pygame.draw.line(self.screen, DOOR_MARK,
                                             (x+cell-3, y), (x+cell-3, y+cell), 2)
                        else:
                            pygame.draw.line(self.screen, DOOR_MARK,
                                             (x, y+2), (x+cell, y+2), 2)
                            pygame.draw.line(self.screen, DOOR_MARK,
                                             (x, y+cell-3), (x+cell, y+cell-3), 2)
                    else:
                        pygame.draw.rect(self.screen, GRID_LINE, rect, 1)

    # ── Fog of war ────────────────────────────────────────────────
    def _dessiner_fog(self):
        """Masque les cellules non encore observées par le lidar."""
        fog = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)
        carte_connue = self.env.carte_connue
        cell = self.cell
        for r in range(self.env.carte.rows):
            for c in range(self.env.carte.cols):
                if carte_connue[r][c] == INCONNU:
                    fog.fill(FOG_COL, pygame.Rect(c*cell, r*cell, cell, cell))
        self.screen.blit(fog, (0, 0))

    # ── Lidar debug ───────────────────────────────────────────────
    def _dessiner_lidar(self):
        """Affiche les rayons lidar (mode debug, activé par [F1])."""
        robot = self.env.robot
        if not robot or not getattr(robot, 'lidar', None):
            return
        resultats = robot.lidar.scanner(robot, self.env.carte)
        overlay = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)
        cx, cy = self.monde_vers_pixel(robot.x, robot.y)
        for angle, dist, xi, yi in resultats:
            ex, ey = self.monde_vers_pixel(xi, yi)
            pygame.draw.line(overlay, LIDAR_COL, (cx, cy), (ex, ey), 1)
        self.screen.blit(overlay, (0, 0))

    # ── Caméras ───────────────────────────────────────────────────
    def _dessiner_cameras(self):
        """Dessine les cônes de vision, trajets de patrouille et corps des caméras."""
        cell = self.cell
        overlay = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)

        for cam in self.env.cameras:
            # Trajet de patrouille
            if len(cam.trajet) >= 2:
                for i in range(len(cam.trajet)):
                    a = cam.trajet[i]
                    b = cam.trajet[(i+1) % len(cam.trajet)]
                    ax = a[0]*cell + cell//2
                    ay = a[1]*cell + cell//2
                    bx = b[0]*cell + cell//2
                    by = b[1]*cell + cell//2
                    pygame.draw.line(overlay, PATROL_TRAIL, (ax, ay), (bx, by), 2)

            # Cellules du cône
            for (vc, vr) in cam.cellules_vues:
                pygame.draw.rect(overlay, CAM_CONE,
                                 pygame.Rect(vc*cell, vr*cell, cell, cell))

            # Contour du cône
            cx = cam.col * cell + cell // 2
            cy = cam.row * cell + cell // 2
            if cam.cellules_vues:
                pts = [(vc*cell + cell//2, vr*cell + cell//2)
                       for (vc, vr) in cam.cellules_vues]
                pts.sort(key=lambda p: math.atan2(p[1]-cy, p[0]-cx))
                if len(pts) >= 2:
                    hull = [(cx, cy)] + pts + [(cx, cy)]
                    pygame.draw.lines(overlay, CAM_CONE_EDG, False, hull, 2)

        self.screen.blit(overlay, (0, 0))

        # Corps des caméras (par-dessus le cône pour rester lisibles)
        for cam in self.env.cameras:
            cx = cam.col * cell + cell // 2
            cy = cam.row * cell + cell // 2
            dx, dy = cam.direction
            pygame.draw.circle(self.screen, CAM_LENS, (cx, cy), cell//3 + 1)
            pygame.draw.circle(self.screen, CAM_BODY, (cx, cy), cell//3 - 1)
            pygame.draw.line(self.screen, CAM_BODY, (cx, cy),
                             (cx + int(dx*cell*0.5),
                              cy + int(dy*cell*0.5)), 3)

    # ── Chemin planifié ───────────────────────────────────────────
    def _dessiner_chemin(self, controleur):
        if not hasattr(controleur, "waypoints") or not controleur.waypoints:
            return
        overlay = pygame.Surface((self.largeur, self.hauteur), pygame.SRCALPHA)
        total = len(controleur.waypoints)
        for i, (wx, wy) in enumerate(controleur.waypoints):
            progress = 1.0 - (i / max(total, 1))
            alpha = int(30 + 80 * progress)
            taille = max(2, int(4 * progress))
            px, py = self.monde_vers_pixel(wx, wy)
            pygame.draw.circle(overlay, (70, 200, 255, alpha), (px, py), taille)
        self.screen.blit(overlay, (0, 0))

    # ── Item ──────────────────────────────────────────────────────
    def _dessiner_item(self):
        item = self.env.item
        if not item or item.collecte:
            return
        wx, wy = item.position_monde
        cx, cy = self.monde_vers_pixel(wx, wy)
        t = pygame.time.get_ticks() / 500.0
        glow_r = int(self.cell * 0.8 + math.sin(t) * 4)
        glow = pygame.Surface((glow_r*2, glow_r*2), pygame.SRCALPHA)
        pygame.draw.circle(glow, ITEM_GLOW_C, (glow_r, glow_r), glow_r)
        self.screen.blit(glow, (cx-glow_r, cy-glow_r))
        h = self.cell // 4
        diamant = [(cx, cy-h), (cx+h, cy), (cx, cy+h), (cx-h, cy)]
        pygame.draw.polygon(self.screen, ITEM_CORE, diamant)
        pygame.draw.polygon(self.screen, ITEM_EDGE, diamant, 2)

    # ── Sortie ────────────────────────────────────────────────────
    def _dessiner_sortie(self):
        sortie = self.env.sortie
        if not sortie:
            return
        x = sortie.col * self.cell
        y = sortie.row * self.cell
        inner = pygame.Rect(x+2, y+2, self.cell-4, self.cell-4)
        t = pygame.time.get_ticks() / 400.0
        alpha = int(140 + math.sin(t) * 60)
        pygame.draw.rect(self.screen, EXIT_FILL, inner)
        surf = pygame.Surface((self.cell, self.cell), pygame.SRCALPHA)
        pygame.draw.rect(surf, (60, 230, 110, min(255, alpha)),
                         (0, 0, self.cell, self.cell), 3)
        self.screen.blit(surf, (x, y))
        label = self.small_font.render("EXIT", True, (20, 20, 20))
        self.screen.blit(label,
                         (x + (self.cell - label.get_width())//2,
                          y + (self.cell - label.get_height())//2))

    # ── Robot ─────────────────────────────────────────────────────
    def _dessiner_robot(self):
        robot = self.env.robot
        if not robot:
            return
        cx, cy = self.monde_vers_pixel(robot.x, robot.y)
        r = int(robot.rayon * self.scale)
        glow = pygame.Surface((r*6, r*6), pygame.SRCALPHA)
        pygame.draw.circle(glow, ROBOT_GLOW, (r*3, r*3), r*3)
        self.screen.blit(glow, (cx-r*3, cy-r*3))
        pygame.draw.circle(self.screen, ROBOT_EDGE, (cx, cy), r+2)
        pygame.draw.circle(self.screen, ROBOT_CORE, (cx, cy), r)
        x_dir = cx + int(r * 1.4 * math.cos(robot.orientation))
        y_dir = cy + int(r * 1.4 * math.sin(robot.orientation))
        pygame.draw.line(self.screen, (255, 80, 80), (cx, cy), (x_dir, y_dir), 3)

    # ── HUD ───────────────────────────────────────────────────────
    def _dessiner_hud(self, etat, controleur=None):
        """Affiche la barre d'état : mission, chronomètre, détections, touches."""
        bar_y = self.hauteur
        pygame.draw.rect(self.screen, HUD_BG, (0, bar_y, self.largeur, BAR_HEIGHT))
        pygame.draw.line(self.screen, WALL_TOP, (0, bar_y), (self.largeur, bar_y), 1)

        couleurs = {
            EtatMission.SEARCHING: ITEM_CORE,
            EtatMission.COLLECTED: EXIT_FILL,
            EtatMission.VICTORY:   EXIT_FILL,
            EtatMission.DEFEAT:    CAM_BODY,
            EtatMission.PAUSED:    (150, 150, 200),
        }
        messages = {
            EtatMission.SEARCHING: "RECHERCHE — trouver l'objet",
            EtatMission.COLLECTED: "OBJET RECUPERE — atteindre la sortie !",
            EtatMission.VICTORY:   "MISSION ACCOMPLIE",
            EtatMission.DEFEAT:    "DETECTE — GAME OVER",
            EtatMission.PAUSED:    "PAUSE",
        }
        col = couleurs.get(etat, TEXT_COL)
        pygame.draw.circle(self.screen, col, (16, bar_y + BAR_HEIGHT // 2), 5)
        self.screen.blit(self.font.render(messages[etat], True, TEXT_COL),
                         (30, bar_y + 6))

        # Chronomètre et compteur de détections évitées
        if controleur and hasattr(controleur, 'stats'):
            stats = controleur.stats
            chrono = stats.formater_chrono()
            det = stats.detections_evitees
            stats_txt = self.small_font.render(
                f"⏱ {chrono}   👁 évités : {det}", True, (160, 160, 200))
            self.screen.blit(stats_txt, (30, bar_y + 28))

        # Indicateur fog of war
        fog_actif = getattr(self.env.robot, 'lidar', None) is not None
        if fog_actif:
            fog_lbl = self.small_font.render(
                "FOG  [F1] lidar debug", True, (80, 180, 120))
            self.screen.blit(fog_lbl,
                             (self.largeur//2 - fog_lbl.get_width()//2,
                              bar_y + 28))

        # Aide contextuelle selon le type de moteur
        hint_txt = "[R] restart  [P] pause  [Q] quit"
        moteur_type = getattr(controleur, '_moteur_type', None)
        if moteur_type == "omni":
            hint_txt = "↑↓ avance  ←→ strafe  A/E rot  [R][P][Q]"
        elif moteur_type == "differentiel":
            hint_txt = "↑↓ avance  ←→ tourne  [R] restart  [P] pause  [Q] quit"
        hint = self.small_font.render(hint_txt, True, (100, 100, 120))
        self.screen.blit(hint,
                         (self.largeur - hint.get_width() - 10, bar_y + 14))

        # Panneau central de fin de jeu / pause
        if etat in (EtatMission.VICTORY, EtatMission.DEFEAT, EtatMission.PAUSED):
            color = (EXIT_FILL if etat == EtatMission.VICTORY
                     else (150, 150, 220) if etat == EtatMission.PAUSED
                     else CAM_BODY)
            label = ("MISSION ACCOMPLIE" if etat == EtatMission.VICTORY
                     else "PAUSE" if etat == EtatMission.PAUSED
                     else "DETECTE")
            big = self.big_font.render(label, True, color)
            pw, ph = big.get_width() + 60, big.get_height() + 30
            panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
            panel.fill((10, 10, 18, 200))
            pygame.draw.rect(panel, color, (0, 0, pw, ph), 2)
            px_pos = (self.largeur - pw) // 2
            py_pos = (self.hauteur - ph) // 2
            self.screen.blit(panel, (px_pos, py_pos))
            self.screen.blit(big, (px_pos + 30, py_pos + 15))

    # ══════════════════════════════════════════════════════════════
    # Point d'entrée principal
    # ══════════════════════════════════════════════════════════════
    def dessiner(self, controleur=None, etat=EtatMission.SEARCHING):
        """Compose et affiche une frame complète."""
        self.screen.fill(BG)
        self._dessiner_carte()
        self._dessiner_cameras()
        if controleur is not None:
            self._dessiner_chemin(controleur)
        self._dessiner_item()
        self._dessiner_sortie()
        if self._show_lidar:
            self._dessiner_lidar()
        if getattr(self.env.robot, 'lidar', None) is not None:
            self._dessiner_fog()
        self._dessiner_robot()
        self._dessiner_hud(etat, controleur=controleur)
        pygame.display.flip()

    def tick(self, fps=60):
        self.clock.tick(fps)