# EXPERIMENTS — catalogue des séries RL

> **Squelette, à remplir avec Léo-Paul.** Les résultats viennent de `docs/rl/PROJECT_CONTEXT.md` (mis à jour le 11/11/2026). Ce sont des résultats **en simulation uniquement**, sauf mention contraire.

Règle pour les artefacts (reprise de `docs/experiments/README.md`) : le code et les petites métriques sont versionnés ; les traces brutes et les checkpoints restent sur SCITAS (`/home/rizkalla/running_robot_experiments/`). Les fichiers de plus de 1 Mo et les galeries sont dans `dash-reports`.

## Vue d'ensemble

| Série | Dates | Question | Résultat connu | Testé sur robot |
| --- | --- | --- | --- | --- |
| STAND-05 pose-return | 11/11 | revenir à la posture de repos après récupération | implémenté, 23 tests ; **pas entraîné** | non |
| STAND-04 | 10–11/11 | tenir debout, plus un pas de récupération après poussée, 3 plants | V1 59/80, V2 52/80, X 72/80 (rejeu de 80 essais) ; `standing04_xshape_best` | **oui, déployé (runs 1–9, 17–20/11)** |
| STAND-03 | 09/11 | tenir debout, 3 plants | best 78/80 (V1), 69/80 (V2), 70/80 (X) | non |
| CLEARANCE-01/02 | 25–27/10 | lever le pied en marche | archivé 12/12 essais de 30 s à 1,50 m/s ; rejeu frais 9/12, écart **non expliqué** | non |
| SPEED-01, WALK-01 | 19/10 | vitesse et marche sur le plant CAD-v2 | à résumer | non |
| GAIT-01 à 11 | 14–19/10 | marche sur le nouveau plant CAD ; shaping bilatéral, drag de contact | vidéos de 0,10 à 0,35 m/s | non |
| STOP-01 à 06 | 02–12/10 | s'arrêter et tenir : PPO, imitation, rehearsal, arrêt en mouvement | STOP-02 v3 : 0/32 holds ; STOP-03 (imitation) : 21/32 | non |
| Avant Léo-Paul (Nathann) | 2026 | sprint m1→m6, Fourier gait, `walk_mit`, `leg2d` | 3 m/s en simulation ; mur historique à m3 | non |

## Les trois plants « debout » (STAND-03/04)

| Fichier (`BalanceRL/model/`) | Pied | Référence au repos | STAND-04, rejeu de 80 essais |
| --- | --- | --- | --- |
| `stand_v1.xml` | semelle continue (V1 d'origine) | `quiet_v1.json` | 59/80 ; échoue l'essai de poussée fixe à 6,90 s |
| `stand_v2.xml` | deux patins | `quiet_v2.json` | 52/80 ; échoue à 6,80 s |
| `stand_xshape.xml` | deux patins, masses X **provisoires** | `quiet_xshape.json` | 72/80 ; tient les 20 s |

Les checkpoints `best.msgpack` de STAND-03/04 sont dans `checkpoints/standing0{3,4}/{v1,v2,xshape}/` (parents pour un warm-start). Une copie figée des trois plants est dans `archive/models/standing/`.

## Fiches

Modèle d'une fiche, une par série :

```text
### <SÉRIE>
- Objectif et hypothèse :
- Plant (.xml et son hash) :
- Config (preset dans config.py) :
- Nombre de steps :
- Critère de succès (fixé avant) :
- Résultat chiffré :
- Ce qui a été appris :
- Chemin SCITAS des artefacts :
- Policy exportée :
- Statut :
```

### STAND-05 pose-return
_à remplir_

### STAND-04
_à remplir_

### STAND-03
_à remplir_

### CLEARANCE-01/02
_à remplir_

### SPEED-01, WALK-01
_à remplir_

### GAIT-01 à 11
_à remplir_

### STOP-01 à 06
_à remplir_
