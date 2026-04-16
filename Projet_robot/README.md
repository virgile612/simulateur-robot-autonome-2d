# Robot Infiltration — Simulation 2D

> Simulation 2D d'un robot mobile autonome chargé de s'introduire dans un bâtiment surveillé par des caméras, de récupérer un objet cible, puis de rejoindre une sortie d'extraction **sans se faire détecter**.

Le projet est structuré autour du patron **MVC**, avec une architecture orientée objet (classes abstraites, pattern Strategy, encapsulation via `@property`) et une planification de trajectoire **A\*** évitant dynamiquement les zones surveillées.

---

## Aperçu

| Mode | Description |
|------|-------------|
| **Autonome** | Le robot planifie seul avec A\*, explore en fog of war grâce au lidar, et replanifie si une caméra se déplace. |
| **Manuel différentiel** | Pilotage clavier : `↑↓` avance/recule, `←→` tourne. |
| **Manuel omnidirectionnel** | Pilotage clavier : `↑↓` avant/arrière, `←→` strafe, `A/E` rotation. |

**Objectif :** collecter l'item, rejoindre la sortie sans croiser le cône de vision d'une caméra.

---

## Prérequis & Installation

- Python **3.9+**
- Pygame **2.x**

```bash
# Créer un environnement virtuel (recommandé)
python -m venv .venv
source .venv/bin/activate      # Linux / macOS
.venv\Scripts\activate         # Windows

# Installer la dépendance
pip install pygame
```

---

## Lancer le projet

La structure attendue est la suivante :

```
Projet_robot/
├── main.py
└── robot/
    ├── __init__.py
    ├── robot_mobile.py
    ├── moteur.py
    ├── environnement.py
    ├── planificateur.py
    ├── controleur.py
    └── vue.py
```

Depuis le dossier `Projet_robot/` :

```bash
python main.py
```

---

## Contrôles

### Touches globales

| Touche | Action |
|--------|--------|
| `R` | Redémarrer (retour au menu) |
| `P` | Pause / Reprendre |
| `F1` | Afficher / masquer les rayons lidar (mode autonome) |
| `Q` | Quitter |

### Mode différentiel

| Touche | Action |
|--------|--------|
| `↑` / `↓` | Avancer / Reculer |
| `←` / `→` | Tourner gauche / droite |

### Mode omnidirectionnel

| Touche | Action |
|--------|--------|
| `↑` / `↓` | Avant / Arrière |
| `←` / `→` | Strafe gauche / droite |
| `A` / `E` | Rotation gauche / droite |

---

## Architecture

```
Projet_robot/
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

## Diagramme de classes

```
┌─────────────────────────────────────────────────────────────────────────┐
│                             MODÈLE (M)                                  │
│                                                                         │
│  ┌────────────┐   moteur   ┌──────────────┐                            │
│  │RobotMobile │──────────▶ │   Moteur     │ <<abstract>>               │
│  │  x, y,     │            └──────┬───────┘                             │
│  │  orientation│                  ├── MoteurDifferentiel  (v, omega)    │
│  │  lidar      │                  └── MoteurOmnidirectionnel (vx,vy,ω) │
│  └─────┬──────┘                                                         │
│        │                                                                 │
│  ┌─────▼──────────────────────────────────────────────────┐            │
│  │                  Environnement                         │            │
│  │  carte : Carte         ◀── grille 20×15                │            │
│  │  obstacles : [Obstacle]    Obstacle <<abstract>>       │            │
│  │  cameras : [Camera]         ├── ObstacleRectangulaire  │            │
│  │  item : Item                └── ObstacleCirculaire     │            │
│  │  sortie : Sortie                                       │            │
│  │  carte_connue[][]      ◀── révélée par Lidar           │            │
│  │  grille_de_danger()    ──▶ grille pour A*              │            │
│  │  frontiere_exploration()──▶ cellules inexplorées       │            │
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

## Fonctionnalités techniques

- **Planification A\*** sur grille de danger (zones de caméras = murs virtuels), avec replanification automatique si la grille évolue.
- **Fog of war** : la carte est révélée progressivement par un lidar à 36 rayons (DDA simplifié) ; le contrôleur autonome explore les frontières jusqu'à localiser l'objectif.
- **Caméras patrouilleuses** : déplacement continu sur un trajet en boucle, direction recalculée dynamiquement.
- **Rollback de collision** : la pose du robot est restaurée à chaque collision physique détectée.
- **Pattern Strategy** : le moteur est interchangeable (`MoteurDifferentiel` / `MoteurOmnidirectionnel`) sans modifier `RobotMobile`.
- **Machine d'états de mission** : `SEARCHING → COLLECTED → VICTORY / DEFEAT / PAUSED`, avec chronomètre et compteur de détections évitées.

---

## Choix de conception

**Pourquoi MVC ?**
Séparer le modèle (pose du robot, physique) de la vue (Pygame) et du contrôleur (logique de navigation) permet de tester chaque couche indépendamment et de brancher facilement un nouveau mode de pilotage sans toucher au rendu.

**Pourquoi A\* ?**
L'environnement est une grille finie et les obstacles sont connus (ou partiellement connus en fog of war). A\* avec heuristique de Manhattan est optimal en 4-connexité et suffisamment rapide pour une replanification en temps réel à chaque changement de grille de danger.

**Limites actuelles**
- La grille est discrète (20×15) : les trajectoires peuvent sembler anguleuses en bordure d'obstacle.
- Le lidar utilise un DDA simplifié sans modèle de bruit : la détection est parfaite dans sa portée.
- Pas de gestion multi-robots.

---

## Dépendances

| Bibliothèque | Version | Usage |
|---|---|---|
| Python | ≥ 3.9 | Langage principal |
| Pygame | 2.x | Rendu graphique et événements clavier |
