"""
planificateur.py — Planificateur global A*
-------------------------------------------
Implémentation de l'algorithme A* sur une grille 2D d'occupation.

Convention :
    grille[r][c] = 0  →  cellule libre
    grille[r][c] = 1  →  cellule bloquée (mur ou zone surveillée)

Coût :
    g(n) = nombre de pas depuis le départ (4-connexité, coût unitaire)
    h(n) = distance de Manhattan (heuristique admissible en 4-connexité)
    f(n) = g(n) + h(n)
"""

import heapq


def heuristique(a, b):
    """Distance de Manhattan entre deux cellules (col, row)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grille, depart, arrivee):
    """
    Recherche un chemin de depart à arrivee sur la grille.

    Paramètres :
        grille  : liste 2D d'entiers (0 = libre, 1 = bloqué).
        depart  : tuple (col, row) de la cellule de départ.
        arrivee : tuple (col, row) de la cellule d'arrivée.

    Retourne :
        Liste de cellules (col, row) formant le chemin, départ inclus.
        Liste vide si aucun chemin n'existe.
    """
    rows = len(grille)
    cols = len(grille[0])

    open_set = []
    heapq.heappush(open_set, (0, depart))

    came_from = {}
    g_score = {depart: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == arrivee:
            chemin = [current]
            while current in came_from:
                current = came_from[current]
                chemin.append(current)
            chemin.reverse()
            return chemin

        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = current[0] + dx, current[1] + dy
            if not (0 <= nx < cols and 0 <= ny < rows):
                continue
            if grille[ny][nx] == 1:
                continue

            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get((nx, ny), float("inf")):
                came_from[(nx, ny)] = current
                g_score[(nx, ny)] = tentative_g
                f = tentative_g + heuristique((nx, ny), arrivee)
                heapq.heappush(open_set, (f, (nx, ny)))

    return []