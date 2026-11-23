# DASH-01 — run 5 de standing04_xshape (2026-11-20), premier run après le plan C

Code sur le Pi : `cfd33c9`, c'est-à-dire la socket CAN brute (`raw backend`) et l'IMU à 200 Hz.
Le worker de politique est actif (`"worker": true`). Le zéro a été repris debout juste avant
(`zero_epoch` 43).

Fichiers :
- `raw/` contient la black box brute :
  - segments A de la session `5e9165ae` ;
  - dumps B à 200 Hz, dont `B_..._0005_mode_MANUAL_to_POLICY`, qui couvre tout le run : les
    dumps 0006 et 0007 ont été déclenchés pendant sa fenêtre et y ont été fusionnés ;
  - événements de la session ;
  - fichier du run `policyrun_20260822_081716_...npz` ;
  - `session_calibration.json` ;
  - journal systemd de 08:16 à 08:19 (heure du Pi).
- Les `.csv` sont les exports du dump 0005 et du fichier du run.

## Résultat

| | run 3 | run 4 | **run 5** |
|---|---|---|---|
| rythme réel de la politique | 60 Hz | 76 Hz | **83 Hz** |
| ticks en retard | 88 / 100 | 72 / 100 | 43 / 100 (`policy.done`) |
| durée avant l'arrêt pour le rythme | 1,69 s | 1,32 s | 1,21 s |
| tangage au départ, puis à l'arrêt | −7,0° puis +4,8° | −8,7° puis −3,0° | **−6,7° puis +0,9°** |
| fin | chute dans le harnais | reste debout | **reste debout** (`SET_POS`, inclinaison 2,4°) |

- **Comportement : bon.**
  - La politique redresse le robot de −6,7° à +0,9° en 1,2 s, sans dépassement.
  - Le roulis reste entre −1,4° et −2,6°, et les vitesses angulaires sous 0,52 rad/s.
  - Aucune saturation du governor (`clamp_counts` vide).
  - Workspace : marge minimale 5,9° sur la pose mesurée et 1,74° sur la consigne, au genou droit.
  - Couples maximaux : 41 N·m (cam droit) et 34,9 N·m (cam gauche). Bobinages : 41 °C au maximum.
- **Arrêt** : par la garde de rythme (`POLICY_MIN_RATE_FRAC` = 90 %), comme prévu. Le run tient
  83 Hz, sous le seuil de 90 Hz.

## Où sont passées les 205 ms perdues

Le run dure 1 215 ms pour 100 ticks, au lieu de 1 000 ms. Les ticks lents arrivent **en
rafales**, et ces rafales coïncident **à la milliseconde** avec les requêtes HTTP du journal :

| cause | ticks | temps perdu | preuve |
|---|---|---|---|
| **`GET /api/state_cold` relu par le daemon**, ~2 fois par seconde | 19 | **104 ms** | rafales de 13 à 23 ms à 14,23–14,31 / 14,70–14,79 / 15,21–15,31 ; `state_cold` est servi à 14,279 / 14,777 / 15,273 |
| **`POST /api/policy/keepalive`**, 5 fois par seconde | 5 | **31 ms** | ticks de 17 à 23 ms à 14,508 / 14,705 / 14,898 / 15,101 ; keepalives à 14,506 / 14,697 / 14,895 / 15,097 |
| **un tick de 72 ms** (14,42) | 1 | **62 ms** | aucune requête HTTP à cet instant. Non expliqué : ramasse-miettes ? |
| autres | 2 | 4 ms | |

En dehors de ces moments, les ticks sont à 10 ms pile.
- **Sans `state_cold` ni `keepalive`, le run aurait tenu 92,6 Hz.**
- **Sans le tick de 72 ms en plus, 98,7 Hz.**

**Le mécanisme de `state_cold`.** uiproc garde en cache la moitié « froide » de `/api/state`. Elle
coûte au daemon ~44 lectures de fichiers sur la carte SD : mesures, trajectoires, listes de
répertoires. Mesuré à l'instant en LIMP : **35 à 68 ms par requête**, sous le GIL du daemon.
uiproc **vide ce cache à chaque écriture relayée** (`proxy()` → `_invalidate_cold()`). Pendant un
run, le `keepalive` est un POST envoyé 5 fois par seconde : le cache n'est jamais valide, et chaque
`/api/state` de l'interface (2 fois par seconde) refait la lecture complète dans le daemon. Le
`POST /api/thermal/identify/plan`, que l'interface envoie toutes les 15–20 s, fait de même.

## Coût par étape (`timing_ms`, en ms, p50 / p90 / max)

| étape | run 5 | référence hors contention (20/11, calcul pur, cœur libre) |
|---|---|---|
| `measure` | 0,39 / 0,46 / 1,50 | |
| `ctrl` (attente du worker) | 3,75 / 5,56 / 8,44 | `wstep` dans le worker : 2,83 / 3,37 ; aller-retour (`rtt`) 3,95 / 5,85 |
| `gov` | 1,42 / 2,35 / 4,32 | 0,64 / 0,94 : **×2,2** |
| `ws` | 0,59 / 1,23 / 2,35 | |
| `send` (6 trames) | 1,17 / 2,92 / 5,17 | 0,21 (CAN virtuel) : **×5,6** |
| `log` | 0,16 / 0,19 / 0,25 | |
| **`total`** | **7,62 / 12,17 / 18,81** | |

- Le worker : 1 réponse en retard sur 100, aucune panne. Sonde à l'armement : 3,18 ms (p90 4,73).
- En p50, la politique laisse ~2,4 ms de marge sur 10 ms : **le régime normal tient**.
- `gov` et `send` coûtent 2 à 6 fois leur valeur isolée. Chaque `send()` relâche le GIL ; pour le
  reprendre, la boucle doit attendre qu'un autre thread (IMU, Flask) le rende.

## Ce qu'il faut corriger, dans l'ordre

1. **uiproc : ne plus vider le cache de `state_cold` sur les POST qui ne le changent pas**
   (`/api/policy/keepalive`, `/api/thermal/identify/plan`, ou n'importe quel POST pendant
   POLICY). Seul l'affichage est concerné. Gain attendu : ~104 ms sur 1,2 s, soit 83 → ~92 Hz.
   Risque faible.
2. **Alléger le `keepalive`** : ~6 ms par appel dans le daemon. Soit le transmettre par le canal
   existant d'uiproc plutôt que par une requête HTTP complète, soit le ralentir si le délai de
   l'homme mort le permet.
3. **Trouver le tick de 72 ms** : instrumenter le ramasse-miettes (`gc.callbacks`) et horodater
   ce qui tient le GIL. Si c'est le GC : `gc.freeze()` à l'armement, ou désactiver le GC pendant
   le run.
4. Ensuite seulement : envois CAN sans relâcher le GIL, governor en calcul scalaire, IMU lue en
   phase avec le tick (journal §3.7).

Les points 1 et 2 suffiraient probablement à passer la garde des 90 Hz. Le point 3 est nécessaire
pour viser 99 Hz.

Un run avec `"worker": false` n'a pas été fait. À refaire après la correction 1, pour comparer.
