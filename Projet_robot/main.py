"""
main.py — Point d'entrée
-------------------------
Assemble le Modèle, la Vue et le Contrôleur, puis lance la boucle de simulation.

Touches (mode Différentiel) :
    ↑ / ↓       avancer / reculer
    ← / →       tourner gauche / droite

Touches (mode Omnidirectionnel) :
    ↑ / ↓       avant / arrière
    ← / →       strafe gauche / droite
    [A] / [E]   rotation gauche / droite

Touches globales :
    [R]   redémarrer (retour au menu)
    [P]   pause / reprendre
    [F1]  afficher / masquer les rayons lidar
    [Q]   quitter
"""

import pygame

from robot.robot_mobile import RobotMobile
from robot.moteur import MoteurDifferentiel, MoteurOmnidirectionnel
from robot.environnement import (Environnement, Carte, Camera, Item, Sortie, Lidar)
from robot.vue import VuePygame
from robot.controleur import (ControleurAutonome, ControleurClavier,
                               ControleurClavierOmni, Controleur,
                               EtatMission, StatsMission)


# ══════════════════════════════════════════════════════════════════
# Construction de la mission
# ══════════════════════════════════════════════════════════════════
def construire_mission(mode="auto", moteur_type="differentiel"):
    """
    Instancie et connecte tous les composants du monde :
    carte, robot, moteur, lidar, caméras, item et sortie.
    """
    carte = Carte()
    env   = Environnement(carte)

    # Le lidar est réservé au mode autonome (fog of war)
    lidar = Lidar(portee=6.0, nb_rayons=36) if mode == "auto" else None

    if mode == "manuel" and moteur_type == "omni":
        moteur = MoteurOmnidirectionnel()
    else:
        moteur = MoteurDifferentiel()

    sx, sy = Carte.cellule_vers_monde(1, 1)
    robot  = RobotMobile(x=sx, y=sy, orientation=0.0,
                         moteur=moteur, rayon=0.3, lidar=lidar)
    env.ajouter_robot(robot)

    env.definir_item(Item(col=16, row=4))
    env.definir_sortie(Sortie(col=1, row=13))

    # Caméras statiques
    env.ajouter_camera(Camera(col=10, row=1,  direction=(1, 0),  portee=3))
    env.ajouter_camera(Camera(col=6,  row=7,  direction=(-1, 0), portee=3))
    env.ajouter_camera(Camera(col=14, row=13, direction=(-1, 0), portee=4))

    # Caméra patrouilleuse
    env.ajouter_camera(Camera(
        col=3, row=7,
        direction=(1, 0),
        portee=3,
        trajet=[(3, 7), (8, 7), (14, 7), (8, 7)],
        vitesse_patrol=1.0,
    ))

    return env


# ══════════════════════════════════════════════════════════════════
# Contrôleur clavier avec machine d'états de mission
# ══════════════════════════════════════════════════════════════════
class ControleurClavierMission(Controleur):
    """
    Enveloppe un ControleurClavier ou ControleurClavierOmni.
    Gère la machine d'états SEARCHING → COLLECTED → VICTORY / DEFEAT
    en complément du pilotage manuel.
    """

    def __init__(self, env, clavier=None):
        self._clavier  = clavier if clavier is not None else ControleurClavier()
        self.env       = env
        self.etat      = EtatMission.SEARCHING
        self.stats     = StatsMission()
        self.waypoints = []
        self._etat_avant_pause = EtatMission.SEARCHING
        self._moteur_type = "differentiel"
        import math
        self._math = math

    def lire_commande(self):
        robot = self.env.robot

        self.stats.signaler_proximite_camera(self.env.test_detection())

        if self.env.test_detection():
            self.etat = EtatMission.DEFEAT
            return {"v": 0.0, "omega": 0.0}

        if self.etat == EtatMission.SEARCHING and self.env.item:
            ix, iy = self.env.item.position_monde
            if self._math.hypot(robot.x - ix, robot.y - iy) < 0.3:
                self.env.item.collecte = True
                robot.a_objet = True
                self.etat = EtatMission.COLLECTED

        elif self.etat == EtatMission.COLLECTED and self.env.sortie:
            sx, sy = self.env.sortie.position_monde
            if self._math.hypot(robot.x - sx, robot.y - sy) < 0.3:
                self.etat = EtatMission.VICTORY

        if self.etat in (EtatMission.VICTORY, EtatMission.DEFEAT):
            return {"v": 0.0, "omega": 0.0}

        return self._clavier.lire_commande()


# ══════════════════════════════════════════════════════════════════
# Menus
# ══════════════════════════════════════════════════════════════════
def boucle_menu(vue):
    """Affiche le menu de sélection du mode. Retourne 'auto', 'manuel' ou None."""
    modes = ["auto", "manuel"]
    idx   = 0
    while True:
        vue.dessiner_menu(mode_selectionne=modes[idx])
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_UP, pygame.K_DOWN):
                    idx = 1 - idx
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    return modes[idx]
                elif event.key == pygame.K_a:
                    return "auto"
                elif event.key == pygame.K_m:
                    return "manuel"
                elif event.key == pygame.K_q:
                    return None


def boucle_menu_moteur(vue):
    """Affiche le menu de sélection du moteur. Retourne 'differentiel', 'omni' ou None."""
    moteurs = ["differentiel", "omni"]
    idx = 0
    while True:
        vue.dessiner_menu_moteur(moteur_selectionne=moteurs[idx])
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_UP, pygame.K_DOWN):
                    idx = 1 - idx
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    return moteurs[idx]
                elif event.key == pygame.K_d:
                    return "differentiel"
                elif event.key == pygame.K_o:
                    return "omni"
                elif event.key == pygame.K_q:
                    return None


# ══════════════════════════════════════════════════════════════════
# Boucle principale
# ══════════════════════════════════════════════════════════════════
def main():
    env_tmp = construire_mission("auto")
    vue     = VuePygame(env_tmp, scale=32)

    running = True
    while running:
        # Sélection du mode
        mode = boucle_menu(vue)
        if mode is None:
            break

        # Sélection du moteur (mode manuel uniquement)
        moteur_type = "differentiel"
        if mode == "manuel":
            moteur_type = boucle_menu_moteur(vue)
            if moteur_type is None:
                break

        env        = construire_mission(mode, moteur_type)
        vue.env    = env
        vue._rooms = vue._detecter_pieces()

        # Instanciation du contrôleur adapté
        if mode == "auto":
            controleur = ControleurAutonome(env)
        else:
            clavier = (ControleurClavierOmni()
                       if moteur_type == "omni"
                       else ControleurClavier())
            controleur = ControleurClavierMission(env, clavier=clavier)
            controleur._moteur_type = moteur_type

        dt  = 0.05
        fps = 60
        restart = False

        # Boucle de simulation
        while running and not restart:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        running = False

                    elif event.key == pygame.K_r:
                        restart = True

                    elif event.key == pygame.K_p:
                        if hasattr(controleur, 'etat'):
                            if controleur.etat == EtatMission.PAUSED:
                                controleur.etat = controleur._etat_avant_pause
                                if hasattr(controleur, 'stats'):
                                    controleur.stats.reprendre()
                            elif controleur.etat not in (EtatMission.VICTORY,
                                                          EtatMission.DEFEAT):
                                controleur._etat_avant_pause = controleur.etat
                                controleur.etat = EtatMission.PAUSED
                                if hasattr(controleur, 'stats'):
                                    controleur.stats.pause()

                    elif event.key == pygame.K_F1:
                        vue.toggle_lidar()

            if not running or restart:
                break

            etat = getattr(controleur, 'etat', EtatMission.SEARCHING)

            if etat != EtatMission.PAUSED:
                cmd = controleur.lire_commande()
                env.robot.commander(**cmd)
                if etat not in (EtatMission.VICTORY, EtatMission.DEFEAT):
                    env.mettre_a_jour(dt)

            vue.dessiner(controleur=controleur, etat=etat)
            vue.tick(fps)

    pygame.quit()


if __name__ == "__main__":
    main()