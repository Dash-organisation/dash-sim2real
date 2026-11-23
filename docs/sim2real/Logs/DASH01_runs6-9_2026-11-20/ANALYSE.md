# DASH-01 — runs 6 à 9 de standing04_xshape (2026-11-20)

Code sur le Pi : `cfd33c9` (CAN brut, IMU à 200 Hz), session de black box `5e9165ae`, worker actif
sur tous les runs.
- Runs 6 à 8 : lancés depuis l'interface web.
- Run 9 : lancé depuis le terminal (`tools/policy_cli.py`, `db57ccc`).

Fichiers :
- `raw/` : fichiers des runs, dumps B à 200 Hz (run 9 : `0025`, qui couvre le run, et `0027`,
  jusqu'au retour en LIMP), segments A, événements de la session, journal systemd de 08:28 à
  09:58 (heure du Pi).
- `.csv` : exports.

## Résumé

| run (heure du Pi) | lancé depuis | rythme | durée | fin | `total` p50 / p90 | `dt` p90 / max | tangage début → fin |
|---|---|---|---|---|---|---|---|
| 5 (08:17) | web | 83,0 Hz | 1,21 s | garde de rythme | 7,62 / 12,17 ms | 15,9 / 72 ms | −6,7° → +0,9° |
| 6 (08:30) | web | 88,5 Hz | 2,25 s | garde de rythme | 7,22 / 11,64 ms | 14,8 / 32 ms | −6,4° → **+7,4°** |
| 7 (08:40) | web | 88,5 Hz | 2,55 s | garde de rythme | 7,28 / 11,81 ms | 15,0 / 25 ms | −6,1° → −6,6° |
| 8 (08:40) | web | 88,8 Hz | 1,47 s | garde de rythme | 7,39 / 12,75 ms | 15,3 / 29 ms | −6,7° → −6,4° |
| **9 (09:55)** | **terminal** | **100,0 Hz** | 0,88 s | **arrêt dur : « telemetry stale by 50 ms »** | **7,26 / 8,88 ms** | **11,6 / 16 ms** | **+0,1° → +14,1°** |

## 1. Le rythme est réglé : sans la page web, la boucle tient 100 Hz

- Run 9 : quasiment tous les ticks à 10 ms pile, rythme réel de 100,0 Hz, `total` p90 à 8,9 ms,
  aucune réponse du worker en retard. Le temps de calcul de la politique n'a pas changé par rapport
  aux runs 5 à 8 (`ctrl` 3,6, `gov` 1,3, `send` 1,1 ms) ; seules les interruptions ont disparu.
- Pendant le run 9, le journal ne montre que l'armement et la sauvegarde. L'onglet web était
  pourtant resté ouvert (`state_cold` toutes les 5 s, `thermal/identify/plan`). Sans `keepalive`
  venant du navigateur, le cache d'uiproc tient et le daemon n'est presque plus sollicité.
- Le critère (≥ 99 Hz, `total` p90 < 10 ms) **ne peut pas être déclaré tenu**, parce que le run
  n'a duré que 0,88 s. La première version du script affichait « MET » à tort ; corrigé dans
  `39612bf`.

## 2. L'arrêt du run 9 : deux drives de la jambe gauche se taisent 120 ms

- **`left.abd` (104) et `left.thigh` (106)**, sur `can1`, cessent d'envoyer leur état **au même
  instant**, à 16124,79 s, soit 0,83 s après le début du run. **`left.cam` (105)**, sur le même bus,
  continue sans interruption.
- 50 ms plus tard, le governor fait un arrêt dur (`HARD-STOP`). Le robot passe en LIMP et retombe
  dans le harnais.
- Les deux drives reprennent seuls à 16124,91 s : **le silence a duré ~120 ms**.
- Côté Pi, rien d'anormal :
  - `can1` reste ERROR-ACTIVE, avec 0 erreur de bus, 0 bus-off et des compteurs d'erreurs à 0 ;
  - aucune erreur d'envoi côté daemon ;
  - aucun code d'erreur des drives ;
  - positions continues, sans saut d'origine (`continuity_ok`) ;
  - aujourd'hui, les trois drives émettent de nouveau à 200 Hz.
- **Interprétation.**
  - Le contrôleur CAN du Pi recevait encore les trames du 105 : le bus et le câblage commun
    fonctionnaient.
  - Ce sont donc les deux drives 104 et 106 qui ont **cessé d'émettre** : réinitialisation,
    baisse de tension de leur alimentation, ou protection interne.
  - Ils ont en commun la jambe gauche, et peut-être une branche d'alimentation ou un connecteur.
  - Ce n'est pas le code du plan C : le 105 du même bus, servi par la même socket, n'a rien perdu.
- **Ce n'est pas la première fois.** Des sessions précédentes contiennent déjà des
  « telemetry stale by 92–245 ms » (session `f212677d`) et un « drive error flag on thigh_L,
  cam_R » (`52c93016`).
- **À vérifier** : les connecteurs d'alimentation et de CAN des drives 104 et 106 de la jambe
  gauche, et s'ils partagent un fil d'alimentation.
- **`can1` a 5 727 trames reçues perdues** (`rx_dropped`), alors que `can0` en a 0. Le compteur ne
  bouge pas au repos. On ignore quand il est monté : aucune sonde ne tournait pendant les runs. Une
  perte de ce type serait aléatoire, alors qu'ici deux identifiants précis se taisent pendant que le
  troisième passe. Ce n'est donc probablement pas la cause, mais c'est à suivre.

## 3. Plus important : la politique ne retient pas le robot quand il penche en avant

| run | tangage au fil du run |
|---|---|
| 5 | −6,7 → −3,4 → −0,9 → +0,1 → **+0,9°** (encore +4,5°/s à la fin) |
| 6 | −6,4 → −2,7 → +0,5 → +2,7 → **+7,4°** (en accélération) |
| 7 et 8 | reste vers −6°. Le robot semble retenu (harnais ?) |
| 9 | +0,1 → +3,1 → +5,4 → +11,0 → **+14,1°** (+24°/s à la fin ; vitesse en tangage jusqu'à 0,6 rad/s) |

- Les « redressements » des runs 3 à 5 n'en étaient peut-être pas : **le robot penche vers l'avant
  à chaque fois, et rien ne le freine au passage à la verticale**.
- Au run 9, parti droit, il bascule tout de suite en avant, de plus en plus vite. Le silence des
  drives arrive à +12°, **pendant** la chute : il n'en est pas la cause.
- Hypothèses, non tranchées :
  1. **le centre de masse réel est plus en avant que dans la simulation**, à la verticale ;
  2. **la politique pousse vers l'avant**, par un biais appris ou par une observation décalée
     (par exemple, le zéro debout repris aujourd'hui, `zero_epoch` 43) ;
  3. **une inversion de signe sur l'axe de tangage** entre le robot et la simulation. Peu
     probable : la vitesse en tangage et l'inclinaison sont cohérentes entre elles, et le run 4 se
     comportait différemment.
- Pour trancher :
  - rejouer hors ligne les observations enregistrées dans la politique, et dans la simulation à
    l'état initial du run 9, pour voir ce qu'elle commanderait si on la penchait de +5° ;
  - vérifier que le tangage « debout immobile » du robot correspond au 0 de la simulation.

## Ce qu'il faut retenir

1. **Le problème de rythme est résolu** par le lancement depuis le terminal : 100 Hz tenus.
2. **Un problème matériel ou d'alimentation sur la jambe gauche** : les drives 104 et 106 se
   taisent ensemble pendant 120 ms. Le daemon a réagi comme prévu, par un arrêt dur.
3. **Un problème de contrôle** : la politique ne stabilise pas le tangage vers l'avant. C'est la
   prochaine question à traiter, avant de chercher à faire durer les runs.

## 4. Approfondissement : pourquoi le robot part en avant (analyse hors ligne)

Script : `replay_policy_offline.py`. Il charge le vrai bundle (md5 `282a0f31…`, copié depuis le Pi)
dans le vrai contrôleur `PolicyControllerBalance`.

- **Le rejeu est fidèle.** Les observations du run 9, rejouées dans la politique, redonnent les
  consignes enregistrées à 0,003 rad (0,17°) près.
- **Le réflexe de tangage est désactivé** dans ce bundle (`reflex.enable = false`). Tout le
  redressement vient du réseau.
- **La politique corrige dans le bon sens, mais peu.** Au run 9, quand le tangage passe de 0 à
  +14°, les consignes des cuisses bougent de −1° et +2° seulement, dans le sens stabilisant.
- **Surtout, elle assouplit les cuisses quand le robot penche.** Au run 9, la raideur des cuisses
  passe de 105 / 207 à **29–62 N·m/rad**. Au run 6 à +7°, elle est à 47 / 68.
- Résultat : **les hanches cèdent**. Les cuisses mesurées s'écartent de leur consigne de 5 à 6°, à
  l'opposé de la correction. Le robot se plie aux hanches et bascule. Les moteurs ne sont pas en
  limite (couples de 8 à 13 N·m, aucune saturation du governor) : c'est la consigne de raideur qui
  est basse.
- **Runs 7 et 8 : couples presque nuls**, entre 0 et 2 N·m sur la plupart des articulations. Le
  robot ne portait pas son poids et pendait dans le harnais, d'où le tangage resté vers −6°. Ces
  runs ne disent rien sur l'équilibre.
- Face à une inclinaison **constante** (joints fixes, vitesses nulles), la réponse de la politique
  est faible (~0,5° de cuisse pour 6°) et plutôt de mauvais signe. Mais cet état n'existe pas à
  l'entraînement, où le bassin fait ±1° : ce test n'est pas probant.

**Pourquoi la politique ferait-elle ça ?** Elle a été entraînée sur le robot simulé, dont le bassin
d'équilibre fait ±0,02 rad, soit ±1,1° (`BalanceRL/README.md`). Le robot réel est loin de cette
situation :
1. **Il ne part pas de la verticale.** Aux runs 5 et 6, il démarre à −6,5°, posé sur les talons :
   six fois la largeur du bassin.
2. **Le 0° de l'IMU n'est pas forcément la vraie verticale.** La référence « level » a été prise
   robot suspendu au banc (`sensehat_mount.json`, `reference: hung on the test rig`). En LIMP, le robot
   lisait −6,6° aujourd'hui (mesure de bruit de 07:03 ; on ne sait pas s'il était suspendu ou
   posé sur les talons). Le README rappelle
   qu'une erreur de montage de 1° suffit déjà à remplir le bassin. Si le 0° du robot est décalé de
   plusieurs degrés, la politique « voit » une inclinaison qui n'existe pas, et inversement.
3. **Les couples observés diffèrent de la simulation** : 17 N·m au cam à la reprise de main du
   run 5, presque rien au run 9. La politique lit aussi ces couples.

**Deux vérifications, à faire avant tout nouveau run :**
- **Inclinomètre** (téléphone ou niveau à bulle) posé sur le torse : robot tenu vraiment vertical,
  quel tangage le robot affiche-t-il (`policy_cli.py status`, ou le panneau capteurs) ? S'il ne
  lit pas ~0°, il faut refaire la référence « level » robot vertical, puis les quatre inclinaisons.
- **Simulation** : faire tourner le même bundle dans BalanceRL. Partir de −6,5°, ou ajouter un
  décalage de montage de quelques degrés, et regarder si la politique y assouplit aussi les
  cuisses et tombe en avant. Il faut mujoco et jax sur le Mac, ou le cluster.
