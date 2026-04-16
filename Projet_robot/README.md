# Robot Infiltration — Simulation 2D

Simulation 2D d'un robot mobile autonome chargé de s'introduire dans un bâtiment surveillé par des caméras, de récupérer un objet cible, puis de rejoindre une sortie d'extraction sans se faire détecter. Le projet est structuré autour du patron MVC, avec une architecture orientée objet (classes abstraites, pattern Strategy, encapsulation via `@property`) et une planification de trajectoire A* évitant dynamiquement les zones surveillées.

---

## Architecture

```
robot-infiltration/
├── main.py                 # Point d'entrée — assemble MVC et lance la boucle
└── robot/
    ├── __init__.py
    ├── robot_mobile.py     # Modèle (M) — pose continue, encapsulation
    ├── moteur.py           # Moteur ABC + Différentiel + Omnidirectionnel
    ├── environnement.py    # Carte, Obstacles, Camera, Item, Sortie, Lidar
    ├── planificateur.py    # Algorithme A* sur grille d'occupation
    ├── controleur.py       # Controleur ABC + Clavier + Autonome
    └── vue.py              # VuePygame — rendu top-down
```

---

## Diagramme de classes (UML simplifié)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                             MODÈLE (M)                                  │
│                                                                         │
│  ┌────────────┐   moteur   ┌──────────────┐                            │
│  │RobotMobile │──────────▶ │   Moteur     │ <<abstract>>               │
│  │  x, y,     │           └──────┬───────┘                             │
│  │  orientation│                  ├── MoteurDifferentiel  (v, omega)    │
│  │  lidar      │                  └── MoteurOmnidirectionnel (vx,vy,ω) │
│  └─────┬──────┘                                                         │
│        │                                                                 │
│  ┌─────▼──────────────────────────────────────────────────┐            │
│  │                  Environnement                          │            │
│  │  carte : Carte         ◀── grille 20×15                │            │
│  │  obstacles : [Obstacle]    Obstacle <<abstract>>        │            │
│  │  cameras : [Camera]         ├── ObstacleRectangulaire  │            │
│  │  item : Item                └── ObstacleCirculaire     │            │
│  │  sortie : Sortie                                        │            │
│  │  carte_connue[][]      ◀── révélée par Lidar           │            │
│  │  grille_de_danger()    ──▶ grille pour A*              │            │
│  │  frontiere_exploration()──▶ cellules inexplorées        │            │
│  └────────────────────────────────────────────────────────┘            │
│         │            │           │                                       │
│      Camera        Item        Sortie                                    │
│   (trajet patrol) (collecte)  (extraction)                              │
│         │                                                                │
│       Lidar  (lancer de rayons, DDA)                                    │
└─────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────┐
│                   CONTRÔLEUR (C)                     │
│                                                      │
│   Controleur  <<abstract>>                           │
│       ├── ControleurClavier       (différentiel)     │
│       ├── ControleurClavierOmni   (omnidirectionnel) │
│       ├── ControleurClavierMission  (clavier + états)│
│       └── ControleurAutonome                         │
│               A* + suivi waypoints                   │
│               Exploration par frontières (fog)       │
│               Replanification sur changement         │
│               EtatMission : SEARCHING → COLLECTED    │
│                             → VICTORY / DEFEAT       │
│               StatsMission : chrono, détections      │
└──────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────┐
│                      VUE (V)                         │
│                                                      │
│   VuePygame                                          │
│       dessiner()     ← frame complète                │
│       dessiner_menu() / dessiner_menu_moteur()       │
│       _dessiner_carte / _cameras / _robot / _hud     │
│       _dessiner_fog() / _dessiner_lidar() [F1]       │
└──────────────────────────────────────────────────────┘

┌──────────────────────────┐
│     PLANIFICATEUR        │
│   astar(grille, dep, arr)│
│   heuristique Manhattan  │
└──────────────────────────┘
```

---

## Lancer le projet

### Prérequis

- Python 3.9 ou supérieur
- Pygame 2.x

```bash
pip install pygame
```

### Exécution

Depuis la racine du projet :

```bash
python main.py
```

Le fichier `main.py` doit se trouver à la racine, et le dossier `robot/` doit être dans le même répertoire.

---

## Modes de jeu

| Mode | Description |
|------|-------------|
| **Autonome** | Le robot planifie seul avec A*, explore en fog of war grâce au lidar, et replanifie si une caméra se déplace. |
| **Manuel différentiel** | Pilotage clavier : `↑↓` avance/recule, `←→` tourne. |
| **Manuel omnidirectionnel** | Pilotage clavier : `↑↓` avance/recule, `←→` strafe, `A/E` rotation. |

### Touches globales

| Touche | Action |
|--------|--------|
| `R`    | Redémarrer la mission |
| `P`    | Pause / Reprendre |
| `F1`   | Afficher / masquer les rayons lidar (mode autonome) |
| `Q`    | Quitter |

---

## Fonctionnalités techniques

- **Planification A** sur grille de danger (zones de caméras = murs virtuels), avec replanification automatique si la grille évolue.
- **Fog of war** : la carte est révélée progressivement par un lidar à 36 rayons (DDA simplifié) ; le contrôleur autonome explore les frontières jusqu'à localiser l'objectif.
- **Caméras patrouilleuses** : déplacement continu sur un trajet en boucle, direction recalculée dynamiquement.
- **Rollback de collision** : la pose du robot est restaurée à chaque collision physique détectée.
- **Pattern Strategy** : le moteur est interchangeable (`MoteurDifferentiel` / `MoteurOmnidirectionnel`) sans modifier `RobotMobile`.
