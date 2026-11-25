# tools/mit_bench — banc « came libre » : l'oscillation MIT des AKE90-8

Plan : [`docs/sim2real/DASH01_plan_banc_came_2026-11-25.md`](../../docs/sim2real/DASH01_plan_banc_came_2026-11-25.md)
(hypothèses H1–H4 au §2, lecture des résultats au §6).

> **English summary.** `controller/tools/mit_bench.py` runs on the Pi, on one drive (left cam, can1
> id 105, unlinked from the leg): SET_CURRENT 0, SET_POS, MIT at a ladder of kp/kd, MIT as a damper,
> MIT in slow rotation and SET_RPM, with speed / current / temperature / error / silence / runaway
> guards. Every stop releases with 1 s of `mit.limp_payload()` then SET_CURRENT 0. It writes one
> `.npz` per session. `tools/mit_bench/analyze.py` runs on the Mac: per-step table and figures, and,
> with the phone audio, alignment on the SYNC tap, dominant lines above A0 and an annotated
> spectrogram. Tests: `controller/fixed_gait/webui/tests/test_mit_bench.py` (bench, fake bus) and
> `tools/mit_bench/test_analyze.py` (analyzer, synthetic session). Phase B (blocked output) is not
> written yet.

| Fichier | Tourne sur | Rôle |
| --- | --- | --- |
| `controller/tools/mit_bench.py` | Pi (python-can, numpy) | le banc : étapes, sécurités, enregistrement `.npz` |
| `tools/mit_bench/analyze.py` | Mac (numpy, scipy, matplotlib ; ffmpeg ou afconvert pour le m4a) | tableau, figures, audio |
| `controller/fixed_gait/webui/tests/test_mit_bench.py` | partout, pytest | chaque coupure relâche (limp 1 s puis 0 A), cible MIT = position mesurée, trames MIT de 8 octets, `--dry-run` n'ouvre pas le bus |
| `tools/mit_bench/test_analyze.py` | Mac, pytest | alignement sur le tapotement, raie à 734 Hz retrouvée, mêmes métriques que le banc |

## Les étapes (phase A)

Toujours en premier : **SYNC**, 5 s à 0 A ; tape **une fois**, sec, sur la came quand l'écran dit
« TAPE MAINTENANT » (pic dans l'audio et dans Iq/vitesse → alignement).

| id | commande | réglage | consigne | durée |
|---|---|---|---|---|
| A0 | `SET_CURRENT 0` | — | ne pas toucher | 5 s |
| A1 | `SET_POS` à la position mesurée | servo | 5 s sans toucher + 10 s appui | 15 s |
| A2a | MIT | kp 0, kd 0 | sans toucher | 5 s |
| A2b | MIT | kp 0,1, kd 0,1 | sans toucher | 5 s |
| A2c | MIT | kp 5, kd 0,5 | sans toucher + appui | 15 s |
| A2d | MIT | kp 50, kd 1 | sans toucher + appui | 15 s |
| A2e | MIT | kp 50, kd 0,1 | sans toucher + appui | 15 s |
| A2f | MIT | kp 50, kd 5 | sans toucher + appui | 15 s |
| A2g | MIT | kp 200, kd 1 | sans toucher + appui | 15 s |
| A3a | MIT amortisseur | kp 0, kd 1 | faire tourner la came à la main, lentement | 10 s |
| A3b | MIT amortisseur | kp 0, kd 5 | idem | 10 s |
| A4a | MIT vitesse | kp 0, kd 1, v_des +1 rad/s | ne pas toucher | 5 s |
| A4b | `SET_RPM` | +1604 ERPM (≈ 1 rad/s sortie) | ne pas toucher | 5 s |

Entre deux étapes : annonce, bip (cloche du terminal, donc sur le Mac si tu es en ssh), 3 s à 0 A.
**Dis l'id de l'étape à voix haute** pendant la pause : c'est l'annotation de l'audio.
Étape « appui » : « APPUIE PROGRESSIVEMENT » pendant 7 s, puis « RELÂCHE » pendant 3 s.
Total ≈ 3 min 20 s pour toute la phase A.

La cible MIT est toujours la position mesurée au début de l'étape. A4a est refusée si la came est
trop près du bord du champ position MIT (±720° brut) : la ramener vers 0 à la main.

**Sécurités** (toutes testées) : kp ≤ 200, kd ≤ 5 ; |vitesse| > 3 rad/s sortie (A4 : 3 rad/s
aussi, v_des + 2) ; |Iq| > 15 A ; T ≥ 60 °C ; code d'erreur ≠ 0 ; aucune trame d'état depuis 50 ms ;
emballement (écart-type d'Iq sur 90 ms > 3 A pendant > 0,3 s, ou vitesse qui change de signe sur
> 30 % des trames en 0,5 s, au-delà de ±0,05 rad/s). Vitesse, courant, emballement : l'étape s'arrête,
le banc passe à la suivante. Température, erreur, silence, commande d'un autre programme sur le bus,
exception : tout s'arrête. **Ctrl+C une fois = fin de l'étape, deux fois = arrêt de tout.** Chaque
arrêt relâche : 1 s de `limp_payload` (trame MIT kp = kd = 0) puis `SET_CURRENT 0`.

À savoir en lisant les résultats :
- la trame d'A2a (kp = kd = 0) ne diffère du `limp_payload` de la relâche que par le champ position.
  Si A2a oscille, la relâche peut osciller aussi pendant 1 s : c'est le `SET_CURRENT 0` qui suit
  qui coupe ;
- le facteur 1604 ERPM par rad/s (21 paires × 8) vient de la datasheet, jamais mesuré : l'outil
  affiche aussi la vitesse dérivée de la position (`v_pos`), compare les deux en A4a/A4b ;
- la plage vitesse MIT ±20 rad/s (A4a) vient du manuel V1.0.1 p. 31, pas encore vérifiée sur DASH-01 :
  si A4a ne tourne pas à ≈ 1 rad/s, c'est la plage qui est fausse ;
- l'horloge du Pi est fausse (pas de NTP) : l'heure murale du journal ne sert pas à aligner, le
  SYNC si.

## Déroulé opérateur

1. **Montage** (plan §3). Came gauche dévissée de la liaison (vissée sur le moteur, libre). Robot sur
   son support ou suspendu. Batterie 50–51 V, chargeur débranché, E-STOP à portée. Téléphone à 20 cm
   de la came, toujours au même endroit.
2. **Interface web : LIMP.** Puis arrêter le daemon, en ssh sur le Pi. C'est `runningrobot-webui`
   (`server.py`, le daemon) qui ouvre le CAN ; `runningrobot-ui` ne touche pas au matériel, mais il
   a `Wants=runningrobot-webui` et redémarrerait le daemon s'il redémarre lui-même : arrête les deux.
   ```bash
   ssh nemo@192.168.4.1
   sudo systemctl stop runningrobot-ui runningrobot-webui
   systemctl is-active runningrobot-ui runningrobot-webui     # inactive inactive
   pgrep -af server.py                                         # rien
   ```
3. **Copier l'outil** (depuis le Mac, à la racine du repo). Un seul fichier : `canio.py` et `mit.py`
   n'ont pas changé depuis la version du Pi (cfd33c9 = deef483).
   ```bash
   scp controller/tools/mit_bench.py nemo@192.168.4.1:running_robot/robot/tools/
   ```
   `deploy_to_pi.sh` n'est pas utilisé ici : il déploierait tout `controller/` depuis le commit du
   Pi. Avant le prochain `deploy_to_pi.sh`, supprimer ce fichier sur le Pi
   (`rm ~/running_robot/robot/tools/mit_bench.py`) ou le laisser faire avec `--force` (sauvegarde).
4. **Relire le plan et vérifier les imports**, sur le Pi :
   ```bash
   cd ~/running_robot/robot
   ../.venv/bin/python tools/mit_bench.py --channel can1 --id 105 --phase A --dry-run
   ../.venv/bin/python tools/mit_bench.py --channel can1 --id 105 --phase A --mock --yes --steps A0 --out /tmp/mock.npz
   ```
   (le `--mock` n'ouvre pas le CAN ; ≈ 20 s.)
5. **Lancer l'enregistrement du téléphone**, puis :
   ```bash
   ../.venv/bin/python tools/mit_bench.py --channel can1 --id 105 --phase A
   ```
   Taper `go`. Suivre les consignes. Le préflight écoute 0,5 s sans rien envoyer : il refuse de
   démarrer s'il voit un autre programme commander le bus, si l'id 105 ne répond pas, si le drive
   est en erreur ou à ≥ 60 °C. Pour rejouer : `--steps A2d,A3` (un id ou un préfixe).
   Sortie : `mit_bench_105_<date>.npz` dans le dossier courant (ou `--out`).
6. **Rapatrier** le `.npz` et l'audio dans `docs/sim2real/Logs/DASH01_banc_came_<date>/` (fichiers
   > 1 MiB → `dash-reports`, voir `CLAUDE.md`) :
   ```bash
   scp nemo@192.168.4.1:running_robot/robot/mit_bench_105_*.npz docs/sim2real/Logs/DASH01_banc_came_<date>/
   python tools/mit_bench/analyze.py docs/sim2real/Logs/DASH01_banc_came_<date>/mit_bench_105_<ts>.npz \
       --audio <audio>.m4a --out docs/sim2real/Logs/DASH01_banc_came_<date>/analyse
   ```
   Si le tapotement est mal repéré dans l'audio, `summary.md` liste les candidats : relancer avec
   `--sync-audio-t <t>` (et `--sync-data-t <t>` si besoin).
7. **Relancer le daemon** et vérifier que les 6 moteurs répondent (interface web) :
   ```bash
   sudo systemctl start runningrobot-webui runningrobot-ui
   journalctl -u runningrobot-webui -n 30 --no-pager
   ```
8. Au remontage de la came sur la jambe : refaire la calibration du zéro de cette articulation
   (`session_calibration.json`).

**À renvoyer** : `summary.md`, les figures, l'audio, et tes remarques (ce que tu as entendu, à quelle
étape).
