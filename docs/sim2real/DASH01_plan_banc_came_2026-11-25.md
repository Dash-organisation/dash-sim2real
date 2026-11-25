# DASH-01 — Plan pour Claude Code : banc « came libre » pour diagnostiquer l'oscillation MIT des AKE90-8

Rédigé le 25/11/2026. Contexte : `DASH01_journal_2026-11-24.md` (§1 sécurité, §4.3, §5.3, §7) et la
session du 25/11 (manuel CubeMars « V3 Drive Software User Manual » V1.0.1, enregistrement audio
`B.m4a`). Branche à créer : `hw/mit_bench` (depuis `hw/Kd_validation`).

> **À Claude Code** : ce plan te demande d'**écrire** du code et des tests hors ligne. Tu ne lances
> **rien** qui fasse bouger un moteur, et tu ne te connectes pas au Pi (`CLAUDE.md`, « Robot safety »).
> C'est l'opérateur (Tim) qui lance le banc. Code, commentaires et messages de commit en anglais ;
> ce document et le README du banc peuvent rester en français.

---

## 1. Le problème, en bref

- En force control (« MIT », trame étendue mode 8, `controller/deploy/mit.py`), les AKE90-8 (cames et
  cuisses) produisent un bruit tonal et, au-delà d'un certain effort, une oscillation qui
  **s'entretient** et que **seul LIMP arrête**. En servo (SET_POS), même charge, même pose : presque
  silencieux. Les AK60-39 (abductions) : rien.
- Indépendant de kp et kd (persiste à kp = kd = 0,1), grandit avec l'effort.
- Fréquences : « 2,5 kHz » noté à l'oreille/spectrogramme le 24/11 (suspendu, une jambe) ; le 25/11,
  robot debout + politique, raies fixes à **734 Hz** (harmoniques impaires 2203, 3690 Hz),
  **1444 Hz**, et un groupe de 4 raies à **7037–7111 Hz** (≈ 25 Hz d'écart). Début brutal, régime
  stable, arrêt net au LIMP.
- Manuel CubeMars, schéma bloc du force control (p. 31) : `kp·(θ_ref−θ) + kd·(V_ref−V) + T_ref` →
  limite de couple → `iq_ref` → boucle de courant FOC. En servo, la commande passe par la boucle de
  vitesse avant `iq_ref`. Aucun réglage propre au MIT dans l'outil.

## 2. Les hypothèses et ce que chacune prédit

| | H1 : chemin MIT du firmware | H2 : boucle de courant peu stable | H3 : mécanique (jambe/liaison) | H4 : saturation à fort courant |
|---|---|---|---|---|
| Idée | le calcul MIT (dérivée de θ, cadence) oscille de lui-même | la boucle de courant est à la limite et le MIT l'excite | une résonance de la jambe est entretenue par la boucle MIT | au-delà d'un courant, tension/duty saturent → cycle limite |
| Came libre, MIT, sans toucher | **oscille** (au moins la raie) | possible, faible | **n'oscille pas** | n'oscille pas (pas de courant) |
| Came libre, MIT kp = kd = 0 | **oscille** | non | non | non |
| Fréquence vs config | fixe | fixe | **change** avec la mécanique | fixe |
| SET_POS / SET_RPM au même effort | silencieux | silencieux ou faible | silencieux | silencieux |
| Appui à la main en MIT | amplifie | amplifie | — | **seuil net** en courant |

Le banc est conçu pour que chaque phase élimine au moins une ligne.

## 3. Le montage (opérateur)

- **Came gauche** (bus `can1`, id `105`, voir `controller/fixed_gait/webui/paths.py`) : la came est
  vissée sur le moteur mais **non reliée à la jambe**. Elle a une petite inertie, tourne librement, et
  on peut appuyer dessus à la main. Un emballement en vitesse ne casse rien.
- Robot posé sur son support ou suspendu ; la jambe gauche ne peut plus porter.
- Les autres drives : LIMP via l'interface web, **puis arrêter le daemon** (un seul programme sur le
  bus). Services : `controller/fixed_gait/webui/systemd/` (`runningrobot-webui.service`,
  `runningrobot-ui.service`) — Claude Code vérifie lequel ouvre le CAN et écrit la commande exacte
  dans le README du banc.
- Batterie 50–51 V, chargeur débranché, E-STOP à portée.
- **Téléphone en enregistrement continu** à 20 cm de la came, toujours au même endroit, pendant tout
  le banc.
- Au remontage : refaire la calibration du zéro de cette articulation (`session_calibration.json`).

## 4. Ce que Claude Code doit écrire

### 4.1 `controller/tools/mit_bench.py` (tourne sur le Pi)

Style : `controller/tools/ak_servo_torque.py` et `mit_identify.py` (argparse, python-can, numpy).
**Ne pas réimplémenter le format MIT** : importer `controller/deploy/mit.py` (`pack`, `limp_payload`)
et les fonctions de `controller/fixed_gait/webui/canio.py` (`set_pos`, `set_current`,
`force_control`, `parse_status`, `open_buses` dont le mode `mock=True`). Regarder comment les autres
outils résolvent ces imports sur le Pi (`~/running_robot/robot/` = `controller/`).

**Interface :**
```
python3 tools/mit_bench.py --channel can1 --id 105 --phase A [--steps A2,A3] [--dry-run] [--mock] [--out <npz>]
```
- `--dry-run` : affiche le plan des étapes et quitte, sans ouvrir le bus.
- `--mock` : `canio.open_buses(..., mock=True)` pour les tests hors ligne.
- Chaque étape : annonce à l'écran (label, durée, consigne opérateur), **bip/pause de 3 s** entre
  étapes (marqueur audio et courant), puis exécution. Les étapes « appui » affichent un compte à rebours
  « APPUIE PROGRESSIVEMENT » / « RELÂCHE ».
- Au démarrage : une étape `SYNC` de 5 s à 0 A où l'opérateur **tape une fois** sur la came (pic
  visible dans l'audio et dans Iq → alignement).
- Journal horodaté (temps monotone + heure murale) de chaque début/fin d'étape, imprimé et sauvegardé.

**Commande et mesure :**
- Envoi à **100 Hz** (comme le daemon) ; lecture de toutes les trames d'état de l'id visé (filtre
  `arbitration_id & 0xFF == id`, trames étendues) ; enregistrer pour chaque trame : t, pos (deg),
  vitesse (ERPM), Iq (A), température, code d'erreur, et la commande en cours.
- Mesurer et rapporter la **cadence réelle du retour** (Hz). Ne pas modifier la configuration du drive.
- **Position cible MIT = position mesurée au début de l'étape** (jamais une constante). Refuser l'étape
  si `|raw| ≥ mit.P_MAX` (le champ position ne couvre que ±720°, `daemon._send_hold`).
- Si une étape utilise `v_des ≠ 0`, passer `v_range=(-20.0, 20.0)` à `mit.pack` (plage de l'AKE90-8
  dans le manuel V1.0.1, p. 31), et le commenter comme « manual value, not yet verified on DASH-01 ».
  **Pas de `tau_ff` en phase A.**

**Sécurités (toutes actives, testées en mock) :**
1. kp ≤ 200, kd ≤ 5 (clamp + refus si une étape en demande plus).
2. Coupure si |vitesse| > seuil de l'étape (par défaut 3 rad/s en sortie ≈ 4 800 ERPM ; pour les
   étapes en vitesse : v_des + marge). Conversion ERPM ↔ rad/s : 21 paires × 8 ≈ 1 604 ERPM par rad/s
   de sortie (datasheet ; `jointmap.py` dit que ce facteur n'a jamais été mesuré : l'outil affiche
   aussi la vitesse dérivée de la position pour recouper).
3. Coupure si |Iq| > 15 A, température ≥ 60 °C, code d'erreur ≠ 0, ou aucune trame d'état depuis
   50 ms.
4. **Détecteur d'emballement** : écart-type d'Iq autour de sa moyenne glissante (fenêtre 90 ms) >
   seuil (par défaut 3 A) pendant > 0,3 s, **ou** changement de signe de la vitesse sur > 30 % des
   trames sur 0,5 s → l'étape est arrêtée, marquée `runaway`, avec l'Iq moyen et le temps au moment du
   déclenchement. On passe à l'étape suivante après la pause.
5. **Relâche systématique** (fin d'étape, coupure, Ctrl+C, exception) : 1 s de `mit.limp_payload()`
   en continu **puis** `set_current(0)`, comme `daemon._stream_limp` (un `SET_CURRENT 0` seul ne relâche
   pas une trame MIT). Ctrl+C une fois = fin de l'étape en cours ; deux fois = arrêt de tout.

**Sorties :** un `.npz` par session (toutes les trames + table des étapes + journal), et un tableau
récapitulatif imprimé : par étape, cadence de retour, Iq moyen, `I_ripple_A`, `v_flip`, Iq max,
température max, issue (`ok` / `runaway` / `abort:<raison>`).

### 4.2 Les étapes de la phase A (came libre)

Durées indicatives : 5 s « sans toucher » + 10 s « appui » quand indiqué.

| id | commande | réglage | consigne opérateur | ce qu'elle teste |
|---|---|---|---|---|
| A0 | `set_current(0)` | — | ne pas toucher | bruit de fond de la mesure |
| A1 | `set_pos` à la position mesurée | servo | 5 s sans toucher, 10 s appui | référence servo (doit être calme) |
| A2a | MIT | kp 0, kd 0 | sans toucher | **MIT engagé, loi nulle** : oscillation sans courant ? (H1) |
| A2b | MIT | kp 0,1, kd 0,1 | sans toucher | reproduit le 24/11 |
| A2c | MIT | kp 5, kd 0,5 | sans toucher + appui | |
| A2d | MIT | kp 50, kd 1 | sans toucher + appui | réglage du test d'appui du 24/11 |
| A2e | MIT | kp 50, kd 0,1 | sans toucher + appui | effet de kd |
| A2f | MIT | kp 50, kd 5 | sans toucher + appui | effet de kd |
| A2g | MIT | kp 200, kd 1 | sans toucher + appui | effort max de la politique |
| A3a | MIT amortisseur | kp 0, kd 1, v_des 0 | faire tourner la came à la main, lentement | terme kd seul, en mouvement |
| A3b | MIT amortisseur | kp 0, kd 5, v_des 0 | idem | |
| A4a | MIT vitesse | kp 0, kd 1, v_des +1 rad/s | ne pas toucher | MIT en rotation lente |
| A4b | servo `SET_RPM` | ≈ +1 600 ERPM (≈ 1 rad/s sortie) | ne pas toucher | même vitesse en servo (référence) |

`SET_RPM` (paquet servo 3) n'existe pas dans `canio` : l'ajouter **dans l'outil** (pas dans `canio`),
format du manuel §2.1.4 (int32 big-endian, ERPM).

Ordre d'exécution par défaut : A0, A1, A2a…A2g, A3a, A3b, A4a, A4b. Option `--steps` pour en rejouer.

### 4.3 Phase B (optionnelle, seulement si l'opérateur installe une butée rigide)

Sortie bloquée contre une butée solide (pas la main). Paliers de 5 s, Iq visé 2, 4, 6, 8, 10, 12 A :
- **B1** servo `set_current(I)` ;
- **B2** MIT position, cible décalée de la butée pour obtenir le même Iq (asservir le décalage sur
  l'Iq mesuré, ou balayer le décalage par petits pas) ;
- **B3** MIT couple seul (kp = kd = 0, `tau_ff`) : **exige** `t_range=(-200.0, 200.0)` (manuel) et une
  confirmation `--allow-tau-ff`. L'unité de T n'est pas confirmée (N·m ou A) : on règle sur l'Iq lu.

Écrire la phase B derrière `--phase B` avec une question de confirmation (« butée rigide en place ? »)
et un plafond de 12 A. **Ne la livrer qu'après la phase A** (commit séparé).

### 4.4 `tools/mit_bench/analyze.py` (tourne sur le Mac)

À la manière de `tools/kd_validation/analyze_runs.py` :
- lit le `.npz`, refait le tableau récapitulatif, trace par étape Iq, vitesse, température ;
- **option `--audio <fichier>`** (m4a/wav, conversion ffmpeg) : aligne l'audio sur le `SYNC` (pic du
  tapotement) et donne, par étape, le spectre moyen et les **raies dominantes** (fréquence, niveau en
  dB au-dessus de A0) entre 200 Hz et 10 kHz, plus un spectrogramme global annoté par étape ;
- écrit `summary.md`, `summary.json` et les figures dans un dossier de sortie.

### 4.5 Tests hors ligne

`controller/tools/tests/` ou à côté des tests existants de la webui, au choix de Claude Code (suivre
l'existant) :
- en `--mock` : le plan d'étapes s'exécute ; chaque coupure (vitesse, courant, température, erreur,
  silence, emballement) déclenche bien la relâche (trames `limp_payload` pendant 1 s puis
  `set_current(0)`) ;
- `mit.pack` est appelé avec une position égale à la position mesurée ; aucune trame de longueur ≠ 8 ;
- `--dry-run` n'ouvre pas le bus.
Puis `python -m pytest controller/fixed_gait/webui/tests controller/deploy/tests` : aucun nouvel échec.

## 5. Déroulé côté opérateur (à recopier dans `tools/mit_bench/README.md`)

1. Montage §3. Interface web : LIMP, puis arrêt du daemon.
2. Copier l'outil sur le Pi (`deploy_to_pi.sh` ou `scp` du seul fichier, selon le README).
3. `python3 tools/mit_bench.py --channel can1 --id 105 --phase A --dry-run` : relire le plan.
4. Lancer l'enregistrement du téléphone, puis `--phase A` sans `--dry-run`. Suivre les consignes.
5. Rapatrier le `.npz` et l'audio dans `docs/sim2real/Logs/DASH01_banc_came_<date>/` (fichiers > 1 MiB
   → `dash-reports`, voir `CLAUDE.md`), lancer `analyze.py`.
6. Relancer le daemon, vérifier que les 6 moteurs répondent.

## 6. Lecture des résultats

| Résultat | Conclusion | Suite |
|---|---|---|
| A2a (kp = kd = 0) oscille | H1 : le mode MIT oscille sans loi ni courant | dossier pour CubeMars ; pas de réglage possible chez nous |
| A2a calme, A2b–A2g oscillent sans appui | H1 ou H2 : la loi MIT déclenche, même sans charge | phase B (MIT vs `SET_CURRENT`) |
| Rien sans appui, oscillation avec appui, seuil en Iq net | H4 (saturation) ou H2 à fort courant | phase B ; relever Duty/Vq dans l'outil CubeMars au seuil |
| Aucune oscillation en came libre, même en appui | **H3** : la mécanique de la jambe est nécessaire | remonter la liaison, refaire le banc pièce par pièce |
| Fréquences en came libre ≠ 734 Hz / 7 kHz du 25/11 | la fréquence dépend de la mécanique (H3 participe) | comparer au même enregistrement avec jambe montée |
| A1, A4b (servo) calmes et A3/A4a (MIT) oscillent | confirme : propre au chemin MIT | |

Ce qu'il faut me renvoyer : `summary.md`, les figures, l'audio, et les remarques de l'opérateur.

## 7. Hors sujet ici (liste de côté)

Le facteur ≈ 1,85 (jambes) / ≈ 4 (hanches) entre raideur réelle et kp commandé ; la correction de
`DRIVE_GAINS` (gains lus drive par drive le 25/11) ; les paires de pôles instables à l'identification ;
la coupure d'alimentation du run A. À traiter après le diagnostic.

## 8. Livraison

- Branche `hw/mit_bench`. Commits proposés :
  1. `tools: mit_bench phase A — free-cam MIT/servo diagnostic with safety guards`
  2. `tools: mit_bench analyzer with audio alignment`
  3. (plus tard) `tools: mit_bench phase B — blocked-output MIT vs SET_CURRENT`
- `controller/` est touché (un nouvel outil, pas le daemon) : le signaler à Tim ; pas de merge dans
  `main` sans la checklist du README §8.
- Mettre à jour `docs/sim2real/README.md` (lien vers ce plan) et, après le banc, écrire le journal.
