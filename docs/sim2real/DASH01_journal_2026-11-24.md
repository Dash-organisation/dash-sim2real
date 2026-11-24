# DASH-01 — Journal de travail du 24 novembre 2026 : le sifflement vient du force control des drives

Suite du journal du 20/11 et de `DASH01_plan_test_Kd_2026-11-24.md`. Branche `hw/Kd_validation`.
Sujet de départ : le kd des cames (collé à 5 par la politique) est-il la cause du sifflement et du
problème sim-to-real ? **Réponse : non. Le sifflement est une oscillation du mode force control
(« MIT ») des drives CubeMars AKE90-8. Elle ne dépend ni de kp ni de kd, grandit avec le couple, et
s'entretient une fois lancée.**

Analyse chiffrée des runs : `Logs/DASH01_kd_2026-11-24/ANALYSE.md` (+ `summary.md`, figures).

---

## 1. Consigne de sécurité (à lire avant tout essai en MIT)

- **En MIT, une fois lancée, l'oscillation ne s'arrête pas d'elle-même**, même quand on enlève la
  charge. **Seul LIMP (ou l'E-STOP) l'arrête.** Au premier gros sifflement : LIMP.
- **Pas de runs MIT sous charge** (politique, bundles `kdval`, MIT hold debout) tant que la cause
  n'est pas corrigée. Pour les manipulations sous charge, rester en SET_POS.
- Une jambe qui oscille chauffe sans bouger : surveiller les températures (60 °C maximum).

## 2. Résumé

| Hypothèse | Verdict | Preuve |
|---|---|---|
| H1 : kd = 5 (le maximum) fait siffler les cames | **réfutée** | même bruit de kd 5 à kd 0,5 en run figé, et jusqu'à kd 0,1 au panneau |
| le kp | **réfutée** | même bruit à kp 200, 100 et 50, et jusqu'à kp 0,1 au panneau |
| la dynamique de la politique (sauts de kp, consignes qui bougent) | **réfutée** | une trame figée et le MIT hold du panneau sifflent comme la politique |
| H2 : la lutte came/cuisse à travers le pied | **pas la cause** | ça siffle aussi suspendu, pieds libres ; la charge ne fait qu'amplifier |
| **le mode force control (MIT) des drives** | **retenue** | SET_POS silencieux sous la même charge ; MIT siffle à tous les gains ; note fixe à 2,5 kHz ; oscillation qui s'entretient |

Autres résultats :
- **La raideur statique réelle vaut 1,7 à 1,9 fois le kp commandé** en MIT, sur les quatre
  articulations de jambe (§5.2). À confirmer avec une masse connue : c'est un écart sim-to-real
  direct.
- **Le premier run (A) a coupé l'alimentation de tout le robot** à 0,87 s (§4.1). La cause n'est pas
  prouvée. Pistes : le robot en charge pendant le run, ou un pic de courant dû à l'oscillation.

## 3. Décisions prises et pourquoi

| Décision | Alternatives écartées | Raison |
|---|---|---|
| Après la coupure du run A, abandonner le pas figé de la politique (bundles v1) au profit de la trame « home » : l'action 0 du bundle (`nominal_ctrl`, kp 200/120, kd 5/4) | garder le pas de la politique | rester simple : on veut juste comprendre kp et kd. C'est la trame que le daemon envoie déjà au passage SET_POS → politique. Choix de l'opérateur |
| Variantes : kd 5 / 2 / 1 / 0,5 à kp 200, kd 5 / 1 à kp 100 (`kdval_B` … `G`) ; `kdval_A_step` supprimé | garder A | A serait identique à B dans cette trame |
| Ordre des runs par kd croissant, E (kd 0,5) en premier | A, B, C… | le premier run est celui qui tire le moins de courant ; si ça coupe encore, c'est l'alimentation |
| Ne pas faire de tests pendant la charge du robot | | un chargeur n'est pas une alimentation : risque de surtension ou de surintensité du BMS pendant les pics et le freinage |
| Ne pas contourner la garde de rythme (run E à 76 Hz) | `allow_slow_loop` | à 76 Hz, les vitesses observées sont gonflées de 1,3 fois, ce qui fausse justement le kd |
| Passer aux tests au panneau (MIT hold) après les runs | continuer la série §6.1 (C, F, poussées) | les runs E, B, D, G sifflant tous pareil, la question devenait « MIT contre SET_POS », pas « quel kd » |

## 4. Ce qui s'est passé

Les horodatages du Pi sont faux (pas de RTC, pas de NTP) : les fichiers s'appellent `20260822_*`.

### 4.1 Run A (pas figé, bundles v1) : coupure d'alimentation

- Avant le run, le premier `pose` a été refusé : `left.thigh` était 21° loin de la dernière pose
  enregistrée (2 articulations à plus de 10°). Le message envisage un cycle d'alimentation des
  moteurs pendant que l'UI était arrêtée. Après `limp`, le `pose` est passé.
- `run kdval_A_step.npz` : `hold 0.00 s`, `run 0.87 s 100.0 Hz`, puis le SSH a coupé
  (`Operation timed out`). **Tout le robot s'est éteint.**
- `blackbox_read.py --postmortem` : la session `c08e49d1` (11,3 min) se termine **sans arrêt
  propre**, donc le Pi a été tué ou a perdu l'alimentation. 31 sessions se terminent pareil, en
  général parce que le robot est éteint à la batterie : la boîte noire seule ne distingue pas une
  panne d'un arrêt normal. Ici, le moment (0,87 s après le début du run) et la perte du SSH en même
  temps suffisent.
- **Question ouverte** : le robot était-il en charge pendant le run A ? Les runs suivants, sur
  batterie, n'ont pas coupé.
- Rappel du 20/11 (§3.10) : les drives `left.abd` (104) et `left.thigh` (106) s'étaient tus ensemble
  pendant 120 ms. L'alimentation et les connecteurs de la jambe gauche restent à vérifier.

### 4.2 Bundles v2 et runs debout

Bundles régénérés (`make_frozen_bundles.py --frame home`, commit `4151774`) et copiés sur le Pi.

| run | kp jambes | kd jambes | rythme | sifflement (oreille) | fin |
|---|---|---|---|---|---|
| REF SET_POS (debout, immobile) | servo | — | — | très léger, plus aigu | — |
| E (1er essai) | 200 | 0,5 | 76 Hz | — | garde de rythme après 1,3 s |
| E | 200 | 0,5 | 95,9 Hz | **atroce** | 10 s |
| B | 200 | 5 | 92,7 Hz | **énorme** | 10 s |
| D | 200 | 1 | 95 Hz | **énorme** | 10 s |
| G | 100 | 1 | 96 Hz | **très fort** | 10 s |

- La boucle tient 93–96 Hz avec `total` p90 de 8,9 à 9,3 ms : c'est utilisable, mais ça reste sous le
  critère (99 Hz). Le premier essai de E, à 76 Hz, était un ralentissement passager.
- Le D suspendu n'a pas de log.
- Après un run, le robot tient la pose **mesurée** en SET_POS : il faut refaire `pose` avant chaque
  `run`, sinon l'armement refuse (« cam_R is 1.7 deg from the policy's start pose »).
- Une fois, un run s'est terminé sur « mode now POLICY » au lieu de MANUAL. Non expliqué.

### 4.3 Tests au panneau (MIT hold, sans la politique)

Le MIT hold remplace le SET_POS par une trame force control `tau = kp·(cible − pos) − kd·vel`, avec
les gains du panneau. C'est désactivé par défaut et enregistré dans `data/balance.json` (`hold.mit`).
Pendant la référence SET_POS, `hold.mit` valait bien `False`. Les trames MIT ont été vérifiées
(`manual.hold_law` dans `events.jsonl`, et un changement de kp/kd se sent au toucher).

Observations, dans l'ordre :
1. **Debout, MIT hold : même bruit que pendant la politique.** En SET_POS, avec la même pose et la
   même charge : presque silencieux.
2. **Suspendu, MIT hold : ça siffle aussi**, moins fort que debout.
3. **kp 50 / kd 3, puis kp 50 / kd 0,3, puis kp = kd = 0,1 : ça siffle toujours.** Le bruit ne vient
   pas de la loi kp/kd.
4. **Fréquence : 2,5 kHz** (spectrogramme, suspendu, kp 50 / kd 0,1). Le léger bruit en SET_POS est
   **plus aigu**, donc c'est une autre source (probablement le PWM).
5. **Le bruit grandit avec la force appliquée à la main sur la jambe.** Au-delà d'un seuil, il
   **s'emballe en une forte oscillation qui ne s'arrête plus**, même quand on relâche : **il faut
   passer en LIMP**.
6. **Toute la jambe entre en oscillation** (came et cuisse), pas seulement la came.
7. Impossible de dire à l'oreille si la gauche ou la droite part en premier.

## 5. Analyse

### 5.1 Ondulation du courant (runs debout)

Écart-type du courant autour de sa moyenne glissante sur 90 ms, en A (`analyze_runs.py`) :

| run | came G | cuisse G | came D | cuisse D | abductions |
|---|---|---|---|---|---|
| E (kp 200, kd 0,5) | 6,1 | 0,15 | 6,5 | 0,39 | 0,1 |
| D (kp 200, kd 1) | 6,3 | 0,16 | 6,7 | 0,85 | 0,1 |
| B (kp 200, kd 5) | 3,0 | 0,32 | 6,3 | 1,65 | 0,1 |
| G (kp 100, kd 1) | 6,0 | 0,31 | 5,1 | 0,29 | 0,1 |
| *SET_POS immobile (run 4, 17/11)* | *0,07* | *0,02* | *0,01* | *0,02* | |

- Ce qu'on mesure ici est la trace **repliée** d'une oscillation à 2,5 kHz, puisque le log est à
  100 Hz. Les « pics » à 15–50 Hz des spectres sont des artefacts du repliement.
- La loi commandée n'explique rien du courant des cames (`law_R2` ≈ 0). La vitesse des cames change
  de signe sur 17 à 38 % des ticks.
- Les cames portent environ 20 N·m debout, les cuisses de 10 à 17 N·m. Les cames ondulent le plus
  dans les logs, mais au panneau, toute la jambe part en oscillation.
- La « lutte » came/cuisse (puissances de signe opposé : 49 à 63 % à droite, 6 à 30 % à gauche) est
  gonflée par la vibration elle-même. On ne peut pas la lire comme un vrai combat des deux boucles.

### 5.2 Raideur réelle ≈ 1,8 × la commande

`kp_static` = couple moyen / erreur moyenne : 340 à 380 N·m/rad pour kp 200, et 160 à 190 pour
kp 100, sur les quatre articulations de jambe. En ajustant `couple = k·kp·(err + δ)` sur les deux
niveaux de kp, on obtient **δ ≈ 0** et **k = 1,7 (came G) à 1,9 (cuisse D)**. C'est donc un facteur
d'échelle, pas un décalage du zéro.

La conversion courant → couple (Kt effectif = 0,85 × datasheet) a été validée par Nathann avec une
masse connue (`controller/deploy/jointmap.py`, identification du 2026-08-05). Il est donc
probable que **le driver applique en MIT environ 1,8 fois le kp demandé**. `controller/deploy/mit.py`
signale déjà que les plages de kp/kd de l'AKE90-8 ne sont pas dans le manuel (supposées 0–500 /
0–5). Si c'est confirmé :
- le robot réel est bien plus raide que la simulation (écart sim-to-real, peut-être lié à la dérive
  vers l'avant du 20/11) ;
- le correctif est simple : envoyer kp/1,8, ou corriger la plage dans `mit.py`.

**Pas encore prouvé** : il faut un test statique avec une masse connue en MIT hold (§7).

### 5.3 Interprétation : une boucle de courant instable en force control

Ce qu'on observe correspond au comportement d'une **boucle de courant (ou boucle interne du
driver) à la limite de la stabilité**, qui devient instable à fort courant :
- la note est fixe (2,5 kHz) et ne dépend pas de kp ni de kd : sa fréquence vient de la boucle de
  courant et de l'électrique du moteur, pas de la loi d'impédance ;
- elle grandit avec le couple ; au-delà d'un seuil, elle passe à un **état d'oscillation stable qui
  persiste** même quand le couple redescend (hystérésis) ;
- c'est absent en servo mode (SET_POS) au même couple : le firmware n'utilise donc pas le même
  chemin ou les mêmes réglages de courant dans les deux modes.

Une explication classique, **non prouvée ici** : à fort courant, le fer du moteur sature,
l'inductance baisse, et une boucle de courant réglée juste devient trop agressive.

Les gains de courant des deux jambes diffèrent (relevés sur les cartes avec l'outil CubeMars le
2026-08-04, `controller/fixed_gait/webui/dynstore.py` `DRIVE_GAINS` : jambe D cur kp 0,207 / ki 2545,
jambe G cur kp 0,126 / ki 1705). D'après `dynstore.py`, c'est la signature d'un autotuning par carte
sur le R/L réel du bobinage, pas une erreur de réglage. Comme les deux côtés partent à peu près au
même effort, **cet écart n'est pas la cause principale**. On ne sait pas si le force control utilise
ces gains-là (ce sont ceux de la cascade du servo mode).

Conséquences probables, à vérifier :
- le bruit « atroce » des runs debout : les cames chargées à environ 20 N·m déclenchent
  l'oscillation, qui se maintient ensuite ;
- l'échauffement : 55 à 59 °C en fin de série (cuisse D à 59 °C au run D) ;
- peut-être la coupure du run A (pics de courant d'une oscillation sous kp 211/218), **non prouvé**.

## 6. Pourquoi on est en MIT (thèse de Nathann Morand)

Source : `Doc/Master_Thesis_Nathann_Morand_Dash-01-1.pdf`, §2.4.1, §3.4.7, §4.6.2–4.6.3, §7.

Bande passante mesurée sur les cuisses AKE90-8, jambes à vide (robot sur son stand) :

| | SET_POS (servo) | MIT kp 200 / kd 5 | MIT kp 500 |
|---|---|---|---|
| −3 dB | **1,9 Hz** | 6,1–6,5 Hz | 18 Hz |
| retard | 19–22 ms | 11–13 ms | — |

Raisons du choix du MIT :
1. **La vitesse.** Le servo est 3 à 10 fois plus lent, alors qu'un contact au sol en course dure
   quelques dizaines de ms.
2. **La simulation.** La loi MIT est exactement l'actionneur de position de MuJoCo. Le servo est une
   cascade de PID inconnus, « une liability pour le sim2real ».
3. **L'impédance.** La politique module kp et kd à chaque pas.

Le MIT n'a **jamais été mesuré sous charge** dans la thèse, et la politique finale n'a pas été
déployée sur le robot : l'oscillation n'était pas connue.

**SET_POS peut-il marcher ?**
- **Pour tenir debout, probablement oui.** La chute du torse prend environ 300 ms, et 2 Hz avec
  20 ms de retard est limite mais plausible. Il faut réentraîner avec un modèle de l'actionneur
  servo (second ordre plus retard, à partir des Bode de Nathann). On perd les canaux kp et kd, et
  le servo est rigide.
- **Pour courir, non** : la bande passante ne suffit pas.
- **Hybride possible** : le force control fonctionne en parallèle du servo sans changer de mode
  (`mit.py`). On pourrait mettre certaines articulations en SET_POS, au prix d'un modèle de
  simulation mixte.

## 7. Suite

| # | Action | Ce que ça dit |
|---|---|---|
| 1 | **Abductions (AK60-39) au même test d'appui** en MIT hold (suspendu, LIMP prêt) | si elles n'oscillent jamais, c'est propre à l'AKE90-8 ou à son firmware |
| 2 | **Version du firmware** des six drives (outil CubeMars) et **sauvegarde de leur configuration** | indispensable avant tout réglage et pour le support |
| 3 | **Écrire à CubeMars** (texte ci-dessous) | savoir quelle boucle tourne à 2,5 kHz en force control et comment la régler |
| 4 | **Sur une seule came, après sauvegarde** : diviser par deux les gains de la boucle de courant, puis refaire le test d'appui | si le seuil d'emballement monte, la boucle de courant est trop agressive |
| 5 | **Test de la masse connue** en MIT hold (suspendu, kp 100 / kd 1) : déflexion sous un couple connu | confirme ou non le facteur 1,8 sur le kp |
| 6 | **Garde logicielle** dans le daemon : passer en LIMP quand l'ondulation du courant ou l'alternance de signe de la vitesse dépasse un seuil | protège tant que le firmware n'est pas corrigé. Touche `controller/` : checklist du README §8 |
| 7 | Vérifier l'alimentation et les connecteurs (jambe gauche, drives 104 et 106) ; savoir si le robot était en charge au run A | la coupure du run A |
| 8 | En dernier recours : standing en SET_POS, avec un modèle servo et un réentraînement | si l'oscillation ne se corrige pas |

Message à CubeMars (à compléter avec la version du firmware et le résultat du point 1) :

> AKE90-8, force control mode (extended id, control mode 8). From a certain torque on, the drive
> enters an oscillation at about 2.5 kHz, and the whole leg vibrates. It is independent of kp and kd
> (it persists at kp = kd = 0.1), grows with the load torque, and once started it keeps going even
> when the load is removed: only releasing the motor stops it. In servo (position) mode, at the
> same torque, it does not happen. It behaves the same on two drives whose current-loop gains
> differ (kp 0.207 / ki 2545 and kp 0.126 / ki 1705). Which loop runs at this frequency in force
> control mode, which current-loop gains does force control use, and how can they be tuned?

## 8. Recettes utiles

```bash
# Sur le Pi (lecture seule)
cd ~/running_robot/robot
python3 fixed_gait/webui/blackbox_read.py --postmortem         # coupures, zéro, trips
python3 -c "import json; print(json.load(open('fixed_gait/webui/data/balance.json')).get('hold'))"
grep '"manual.hold_law"' fixed_gait/webui/data/blackbox/events.jsonl | tail -2   # MIT hold actif ?

# Un run figé (pose avant chaque run)
python3 tools/policy_cli.py pose kdval_E_kd05.npz
python3 tools/policy_cli.py run  kdval_E_kd05.npz --allow-uncalibrated-thermal --seconds 10

# Depuis le Mac (code/dash-sim2real)
scp controller/deploy/bundles/kdval_*.npz nemo@192.168.4.1:running_robot/robot/deploy/bundles/
scp "nemo@192.168.4.1:running_robot/robot/fixed_gait/webui/data/policyruns/policyrun_*kdval*" \
    docs/sim2real/Logs/DASH01_kd_2026-11-24/raw/
python3 tools/kd_validation/analyze_runs.py docs/sim2real/Logs/DASH01_kd_2026-11-24/raw/policyrun_*.npz \
    --labels ... --out docs/sim2real/Logs/DASH01_kd_2026-11-24
```
